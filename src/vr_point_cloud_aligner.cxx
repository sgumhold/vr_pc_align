#include "vr_point_cloud_aligner.h"
#include <cgv/base/register.h>
#include <cgv/math/inv.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/mouse_event.h>
#include "../sparseicp/ICP.h"
#include "kabsch.h"
#include <random>
#include <thread>
#include <cgv/gui/animate.h>
#include <cgv/base/find_action.h>
#include <fstream>
#include <cgv/utils/file.h>
#include <cgv/gui/file_dialog.h>

#include <cgv/base/node.h>
#include <cgv/signal/rebind.h>
#include <cgv/base/register.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/provider.h>
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv_gl/box_renderer.h>
#include <cg_gamepad/gamepad_server.h>

#include <libs/cgv_gl/gl/gl.h>


#define FILE_OPEN_TITLE "Open Transformation Project"
#define FILE_OPEN_FILTER "Transformation Projects (tpj):*.tpj;|all Files:*.*"

#define FILE_SAVE_TITLE "Tramsformation Projects"
#define FILE_SAVE_FILTER "Transformation Projects (tpj):*.tpj;|all Files:*.*"

#define EMPTY_CONSTRUCTED_SET constructed_set(std::vector<int>(), -1)

#define ANIMATION_DURATION 2

#define TABLE_HEIGT 1
#define x_room_min -1.75
#define y_room_min 0
#define z_room_min -1.75
#define x_room_max 1.75
#define y_room_max 7
#define z_room_max 1.75

#define MAIN_CONTROLLER 1
#define THROTTLE_THRESHOLD 0.4

#ifdef _DEBUG
#define SAMPLE_COUNT 100
#else 
#define SAMPLE_COUNT 1000
#endif
///Module 1 Startup methods

///Constructor
vr_point_cloud_aligner::vr_point_cloud_aligner()
{
	set_name("VR Point Cloud Aligner");
	vr_view_ptr = 0;
	picked_box_extent = 0.1f;
	ray_length = 3.0f;
	picked_box_color = Clr(float_to_color_component(1.0f), 0, 0);
	view_ptr = 0;
	have_picked_point = false;
	picked_point = Pnt(0, 0, 0);
	last_view_point = Pnt(0, 0, 0);
	have_view_ray = false;
	icp_executing = false;
	pending_unite = false;
	seperation_in_process = false;
	subsample_changed = true;
	global_placeholder_set = EMPTY_CONSTRUCTED_SET;
	picked_group = &global_placeholder_set;
	previous_picked_group = &global_placeholder_set;
	program_state_stack = std::vector<program_state>();
	pss_count = 0;
	time_blink_counter = double(0);
	generate_room_boxes();
	construct_environment(0.2f, 3 * 5, 3 * 7, 3, 3.5, 3.5, 7);
	box_render_style.map_color_to_material = cgv::render::MS_FRONT_AND_BACK;
	box_render_style.culling_mode = cgv::render::CM_BACKFACE;
	box_render_style.illumination_mode = cgv::render::IM_TWO_SIDED;
	picked_component = -1;
	picked_component_valid = false;
	oldColor = RGBA(1, 1, 1, 1);
	even_older_color = RGBA(1, 1, 1, 1);
	defaultFacing = cgv::math::quaternion<float>(1, 0, 0, 0);
	projectLoading_in_process = false;
	current_scaling_factor = 1.0;

	connect(cgv::gui::get_animation_trigger().shoot, this, &vr_point_cloud_aligner::timer_event);
	last_kit_handle = 0;
	connect(cgv::gui::ref_vr_server().on_device_change, this, &vr_point_cloud_aligner::on_device_change);
	cgv::gui::connect_gamepad_server();

	drag_active = false;
}

void vr_point_cloud_aligner::timer_event(double t, double dt)
{
	if(pending_unite)
	{
		if (t > time_blink_counter + 0.5)
		{
			time_blink_counter = t + 0.5;
			animate_pending_unite_blink();
		}
	}
}

///Setup for the working room
void vr_point_cloud_aligner::generate_room_boxes()
{
	room_boxes.clear();
	room_colors.clear();
	Box room;
	room.add_point(Pnt(Crd(x_room_min), Crd(y_room_min - 0.2), Crd(z_room_min)));
	room.add_point(Pnt(Crd(x_room_max), Crd(y_room_min), Crd(z_room_max)));
	room_boxes.push_back(room);
	room_colors.push_back(generate_a_valid_color(3));

	Box wall1;
	wall1.add_point(Pnt(Crd(x_room_min - 0.2), Crd(y_room_min), Crd(z_room_min)));
	wall1.add_point(Pnt(Crd(x_room_min), Crd(y_room_max), Crd(z_room_max)));
	room_boxes.push_back(wall1);
	room_colors.push_back(generate_a_valid_color(3));

	//The table in the middle of the room
	Box tableTop;
	Box tableLeg1;
	Box tableLeg2;
	Box tableLeg3;
	Box tableLeg4;

	float tableTopCorner1 = x_room_min+1, tableTopCorner2 = x_room_max-1;
	tableTop.add_point(Pnt(Crd(tableTopCorner1), Crd(TABLE_HEIGT - 0.2), Crd(tableTopCorner1) ));
	tableTop.add_point(Pnt(Crd(tableTopCorner2), Crd(TABLE_HEIGT), Crd(tableTopCorner2)));
	room_boxes.push_back(tableTop);
	room_colors.push_back(generate_a_valid_color(1));

	//Maybe instanciate the leg with different coordinates?
	tableLeg1.add_point(Pnt(Crd(tableTopCorner1), Crd(TABLE_HEIGT - 0.2), Crd(tableTopCorner1)));
	tableLeg1.add_point(Pnt(Crd(tableTopCorner1 + .2), Crd(0), Crd(tableTopCorner1 + .2)));
	room_boxes.push_back(tableLeg1);
	room_colors.push_back(generate_a_valid_color(2));

	tableLeg2.add_point(Pnt(Crd(tableTopCorner1 + 1.3), Crd(TABLE_HEIGT - 0.2), Crd(tableTopCorner1)));
	tableLeg2.add_point(Pnt(Crd(tableTopCorner1 + 1.5), Crd(0), Crd(tableTopCorner1 + 0.2)));
	room_boxes.push_back(tableLeg2);
	room_colors.push_back(generate_a_valid_color(2));

	tableLeg3.add_point(Pnt(Crd(tableTopCorner1), Crd(TABLE_HEIGT - 0.2), Crd(tableTopCorner1 + 1.3) ));
	tableLeg3.add_point(Pnt(Crd(tableTopCorner1 + .2), Crd(0), Crd(tableTopCorner1 + 1.5) ));
	room_boxes.push_back(tableLeg3);
	room_colors.push_back(generate_a_valid_color(2));

	tableLeg4.add_point(Pnt(Crd(tableTopCorner2), Crd(TABLE_HEIGT - 0.2), Crd(tableTopCorner2)));
	tableLeg4.add_point(Pnt(Crd(tableTopCorner2 - 0.2), Crd(0), Crd(tableTopCorner2 - .2)));
	room_boxes.push_back(tableLeg4);
	room_colors.push_back(generate_a_valid_color(2));

	Box line;
	line.add_point(Pnt(Crd(x_room_min), Crd(1.8), z_room_min));
	line.add_point(Pnt(Crd(x_room_min + .05), Crd(2), z_room_max));
	room_boxes.push_back(line);
	room_colors.push_back(generate_a_valid_color(1));
}

/// construct boxes for environment
void vr_point_cloud_aligner::construct_environment(float s, float ew, float ed, float eh, float w, float d, float h)
{
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	unsigned n = unsigned(ew / s);
	unsigned m = unsigned(ed / s);
	for (unsigned i = 0; i < n; ++i) {
		float x = i * s - 0.5f*ew;
		for (unsigned j = 0; j < m; ++j) {
			float z = j * s - 0.5f*ed;
			if ((x + s > -0.5f*w && x < 0.5f*w) && (z + s > -0.5f*d && z < 0.5f*d))
				continue;
			float h = 0.2f*(std::max(abs(x) - 0.5f*w, 0.0f) + std::max(abs(z) - 0.5f*d, 0.0f))*distribution(generator) + 0.1f;
			room_boxes.push_back(box3(vec3(x, 0, z), vec3(x + s, h, z + s)));
			room_colors.push_back(rgb(0.2f*distribution(generator) + 0.1f, 0.4f*distribution(generator) + 0.2f, 0.2f*distribution(generator) + 0.1f));
		}
	}
}

std::string vr_point_cloud_aligner::get_type_name() const
{
	return "vr_point_cloud_aligner";
}

bool vr_point_cloud_aligner::self_reflect(cgv::reflect::reflection_handler& rh)
{
	return
		rh.reflect_member("project_file", project_file) &&
		point_cloud_interactable::self_reflect(rh);
}
void vr_point_cloud_aligner::stream_help(std::ostream& os)
{
	point_cloud_interactable::stream_help(os);

	// stream out more help here or comment out call to point_cloud_interactable::stream_help and stream out all relevant help here
	os << "   <R|Shift-R> increase|decrease nr rows, <C|Shift-C> increase|decrease nr cols" << std::endl;
}
void vr_point_cloud_aligner::stream_stats(std::ostream& os)
{
	point_cloud_interactable::stream_stats(os);

	// stream out more statistics here or comment out call to point_cloud_interactable::stream_stats and stream out all relevant stats here
	//os << "   rows x cols: " << sample_member_rows << " x " << sample_member_cols << std::endl;
}

void vr_point_cloud_aligner::init_frame(cgv::render::context& ctx)
{
	point_cloud_interactable::init_frame(ctx);

	// do your per frame initialization here
}

void vr_point_cloud_aligner::create_gui()
{
	// start with gui for based class
	point_cloud_interactable::create_gui();

	if (begin_tree_node("picking", have_picked_point, true, "level=3")) {
		align("\a"); // increase identation
		add_gui("position", picked_point, "options='active=false'");
		add_member_control(this, "box extent", picked_box_extent, "value_slider", "min=0.0001;max=10;ticks=true;log=true");
		add_member_control(this, "box color", picked_box_color);
		align("\b"); // decrease identation
		end_tree_node(room_boxes);
	}

	//GUI for saving Transformations from scans
	if (begin_tree_node("Save transformations", true, true, "level=3")) {
		align("\a");
		add_gui("file_name", project_file, "file_name",
			"w=130;"
			"open=true;open_title='" FILE_OPEN_TITLE "';open_filter='" FILE_OPEN_FILTER "';"
		);
		add_gui("file_name", write_project_file, "file_name",
			"w=130;"
			"save=true;save_title='" FILE_SAVE_TITLE "';save_filter='" FILE_SAVE_FILTER "'"
		);
		align("\b");
		end_tree_node(true);
	}
	add_decorator("vr_test", "heading", "level=3");
	add_member_control(this, "ray_length", ray_length, "value_slider", "min=0.1;max=10;log=true;ticks=true");
	if (last_kit_handle) {
		vr::vr_kit* kit_ptr = vr::get_vr_kit(last_kit_handle);
		const std::vector<std::pair<int, int> >* t_and_s_ptr = 0;
		if (kit_ptr)
			t_and_s_ptr = &kit_ptr->get_controller_throttles_and_sticks(0);
		add_decorator("deadzone and precisions", "heading", "level=3");
		int ti = 0;
		int si = 0;
		for (unsigned i = 0; i < left_deadzone_and_precision.size(); ++i) {
			std::string prefix = std::string("unknown[") + cgv::utils::to_string(i) + "]";
			if (t_and_s_ptr) {
				if (t_and_s_ptr->at(i).second == -1)
					prefix = std::string("throttle[") + cgv::utils::to_string(ti++) + "]";
				else
					prefix = std::string("stick[") + cgv::utils::to_string(si++) + "]";
			}
			add_member_control(this, prefix + ".deadzone", left_deadzone_and_precision[i].first, "value_slider", "min=0;max=1;ticks=true;log=true");
			add_member_control(this, prefix + ".precision", left_deadzone_and_precision[i].second, "value_slider", "min=0;max=1;ticks=true;log=true");
		}
	}
}

///Module 2 Helper functions

void vr_point_cloud_aligner::position_scans()
{
	// a new component has been added and should be moved to the line. Therefore all pointclouds that are already there should be moved, but only if they are not already usermodified
	int nr = 0;
	for (unsigned int a = 0; a < user_modified.size(); a++) {
		if (user_modified.at(a)) {
			nr++;
		}
	}
	if (nr > 0)
	{
		for (unsigned int i = 0; i < pc.get_nr_components(); i++)
		{
			float x = float(x_room_max) / nr;
			if (!user_modified.at(i))
			{
				pc.component_translation(i).set(Crd(x_room_min + 0.2), Crd(1.8), Crd(i * x));
				pc.component_rotation(i) = (defaultFacing);
			}

		}
	}
	else
	{
		reset_componets_transformations();
	}
}

/// small helper function to get a valid color.
point_cloud_types::Clr vr_point_cloud_aligner::generate_a_valid_color(int color)
{
	double color_values[] =
	{
		1.0, 0.0, 0.5, // my red
		1.0, 0.7, 0.5, // my yellow
		0.5, 0.5, 0.5, // my grey
		0.2, 0.2, 0.2 // dark grey
	};
	int idx = color - 1;
	if (idx < 0)
		idx = 0;
	if (idx > 3)
		idx = 3;
	return Clr(
		float_to_color_component(color_values[3 * idx]),
		float_to_color_component(color_values[3 * idx + 1]),
		float_to_color_component(color_values[3 * idx + 2])
	);
}

void vr_point_cloud_aligner::animate_pending_unite_blink()
{
	std::vector<int> tempIds = picked_group->get_component_IDs();
	cgv::media::color<float,cgv::media::RGB,cgv::media::OPACITY> changeTo;
	if (blink) 
	{
		changeTo = RGBA(1, 0, 0, 1);
		blink = false;
	}
	else
	{
		changeTo = pc.component_color(previous_picked_group->get_component_IDs().at(0));
		blink = true;
	}
	for (unsigned int i = 0; i < tempIds.size(); ++i)
	{
		pc.component_color(tempIds.at(i)) = changeTo;
	}
	post_redraw();
}

void vr_point_cloud_aligner::unite(bool unite) 
{
	int uniting_Pos = -1;
	int other_pos = -1;
	int count = 0;
	if (unite)
	{
		for (auto a : sets)
		{
			if (a.get_ID() == previous_picked_group->get_ID())
				uniting_Pos = count;
			if (a.get_ID() == picked_group->get_ID())
				other_pos = count;
			count++;
		}

		//now unite and delete
		//use iterator for deletion
		std::list<constructed_set>::iterator uniting_iter_pos = sets.begin(), other_iter_pos = sets.begin();
		std::advance(uniting_iter_pos, uniting_Pos);
		std::advance(other_iter_pos, other_pos);
		uniting_iter_pos._Ptr->_Myval.unite(other_iter_pos._Ptr->_Myval);
		sets.erase(other_iter_pos);
		previous_picked_group = &global_placeholder_set;
		//Direct pointer to list object
		picked_group = &uniting_iter_pos._Ptr->_Myval;
		oldColor = even_older_color;	//pc.component_color(sets.at(unitingPos).get_component_IDs().at(0));
		//set the colors equal
		std::vector<int> tempIDs = picked_group->get_component_IDs();
		Clr tosetTo = RGBA(1, 0, 0, 1);
		for (unsigned int b = 0; b < tempIDs.size(); ++b)
		{
			pc.component_color(tempIDs.at(b)) = tosetTo;
		}
		//save state and make the changes permanent
		push_back_state();
		post_redraw();
	}
	// if not reset to pre icp state (which is the actual state)
	else
	{
		restore_state(pss_count-1);
		post_redraw();
	}
}

void vr_point_cloud_aligner::push_back_state()
{
	if (pss_count < int(program_state_stack.size()))
	{
		program_state_stack.erase(program_state_stack.begin() + pss_count, program_state_stack.end());
	}
	std::vector<Dir> translations(pc.get_nr_components());
	std::vector<Qat> rotations(pc.get_nr_components());
	std::vector<RGBA> colors(pc.get_nr_components());
	for (unsigned int i = 0; i < pc.get_nr_components(); ++i) 
	{
		translations[i] = pc.component_translation(i);
		rotations[i] = pc.component_rotation(i);
		colors[i] = pc.component_color(i);
	}
	program_state state = program_state(translations, rotations, *picked_group, *previous_picked_group, sets,colors,oldColor,even_older_color);
	program_state_stack.push_back(state);
	printf("saved state %d!\n", pss_count);
	pss_count++;
}

void vr_point_cloud_aligner::restore_state(int i)
{
	if(i > int(program_state_stack.size())-1)
	{
		printf("Wiederherstellen nicht möglich, keine Aktionen wurden weiter ausgeführt als diese!\n");
		return;
	}
	if (i < 0)
	{
		printf("Keine Vorherige Aktion wiederherstellbar!");
		return;
	}
	program_state_stack.at(i).put_back_state(pc, picked_group, previous_picked_group, sets,oldColor,even_older_color);
	pss_count = i;
	printf("restored state %d!\n", pss_count);
	pss_count++;
	post_redraw();
}

void vr_point_cloud_aligner::subsample(Eigen::Matrix<double, 3, Eigen::Dynamic> &vertices_source, 
										Eigen::Matrix<double, 3, Eigen::Dynamic> &vertices_source_copy, 
											Eigen::Matrix<double, 3,Eigen::Dynamic> &vertices_target, int subsampling_range)
{
	std::vector<int> Ids_source = picked_group->get_component_IDs();
	std::vector<component_info> component_info_stack_source;
	int nr_of_all_points_source=0;
	bool subsample_condition = false;
	for (unsigned int i = 0; i < Ids_source.size(); ++i) 
	{	
		component_info a = pc.component_point_range(Ids_source.at(i));
		component_info_stack_source.push_back(a);
		nr_of_all_points_source += a.nr_points;
	}
	int subsampled_nr = int(round(nr_of_all_points_source / subsampling_range));
	int p = 0;
	int range_counter = 1;
	int picker = 1;
	bool subsampling_OFF = false;
	if (subsampling_range == 1)
	{
		subsampling_OFF = true;
	}
	vertices_source.resize(Eigen::NoChange, subsampled_nr);
	vertices_source_copy.resize(Eigen::NoChange, subsampled_nr);
	for (unsigned int current_component = 0; current_component < component_info_stack_source.size(); ++current_component)
	{
		component_info a= component_info_stack_source.at(current_component);
		
		for (unsigned int current_adress = a.index_of_first_point; current_adress < a.index_of_first_point + a.nr_points; ++current_adress)
		{
			if (!subsampling_OFF)
			{
				if (range_counter > subsampling_range)
				{
					range_counter = 1;
					picker = rand() % subsampling_range;
				}
				if (picker != range_counter) 
				{
					range_counter++;
					continue;
				}
			}
			if (p >= int(a.nr_points) / subsampling_range)
				break;
			Pnt tr_pnt = pc.transformed_pnt(current_adress);
			vertices_source(0, p) = tr_pnt.x();
			vertices_source(1, p) = tr_pnt.y();
			vertices_source(2, p) = tr_pnt.z();

			Pnt _pnt = pc.pnt(current_adress);
			vertices_source_copy(0, p) = _pnt.x();
			vertices_source_copy(1, p) = _pnt.y();
			vertices_source_copy(2, p) = _pnt.z();
			++p;
			++range_counter;
		}
	}
	if (p < subsampled_nr) {

		Pnt tr_pnt = pc.transformed_pnt(component_info_stack_source.at(0).index_of_first_point);
		vertices_source(0, p) = tr_pnt.x();
		vertices_source(1, p) = tr_pnt.y();
		vertices_source(2, p) = tr_pnt.z();

		Pnt _pnt = pc.pnt(component_info_stack_source.at(0).index_of_first_point);
		vertices_source_copy(0, p) = _pnt.x();
		vertices_source_copy(1, p) = _pnt.y();
		vertices_source_copy(2, p) = _pnt.z();
		++p;
	}
	std::vector<int> Ids_target = previous_picked_group->get_component_IDs();
	int sample_nr_target = Ids_target.size();
	std::vector<component_info> component_info_stack_target;
	int nr_of_all_points_target = 0;
	subsample_condition = false;
	for (int i = 0; i < int(Ids_target.size()); ++i)
	{
		component_info a = pc.component_point_range(Ids_target.at(i));
		component_info_stack_target.push_back(a);
		nr_of_all_points_target += a.nr_points;
	}
	p = 0;

	vertices_target.resize(Eigen::NoChange, nr_of_all_points_target);
	for (int current_component = 0; current_component < int(component_info_stack_target.size()); ++current_component)
	{
		component_info a = component_info_stack_target.at(current_component);

		for (int current_adress = int(a.index_of_first_point); current_adress < int(a.index_of_first_point + a.nr_points); current_adress++)
		{
			Pnt tr_pnt = pc.transformed_pnt(current_adress);
			vertices_target(0, p) = tr_pnt.x();
			vertices_target(1, p) = tr_pnt.y();
			vertices_target(2, p) = tr_pnt.z();
			++p;
		}
	}
	printf("Subsampled source set!");
}

float vr_point_cloud_aligner::find_deepest_BB_point()
{
	std::vector<Pnt> transformed_Pnts;
	std::vector<int> searched_ids = picked_group->get_component_IDs();
	for (int x = 0; x < int(searched_ids.size()); ++x)
	{
		Box current_box = pc.box(searched_ids[x]);
		for (int y = 0; y < 8; ++y)
		{
			if(current_box.is_valid())
			transformed_Pnts.push_back(transform_to_global(current_box.get_corner(y), pc.component_translation(x), pc.component_rotation(x)));
		}
	}
	float lowest_y = INFINITY;
	for(int x = 0; x < int(transformed_Pnts.size()); ++x)
	{
		if (lowest_y > transformed_Pnts.at(x).y())
		{
			lowest_y = transformed_Pnts.at(x).y();
		}
	}
	if (lowest_y == INFINITY)
	{
		printf("BB not valid!\n");
		return TABLE_HEIGT;
	}
	return lowest_y;
}

void vr_point_cloud_aligner::repostion_above_table()
{
	Pnt average_middle(0, 0, 0);
	int nr_off_all_points = 0;
	std::vector<component_info> component_info_stack;
	std::vector<int> search_ids = picked_group->get_component_IDs();
	for (int x = 0; x<int(search_ids.size()); ++x)
	{
		component_info_stack.push_back(pc.component_point_range(search_ids.at(x)));
	}
	for (unsigned int current_component = 0; current_component < component_info_stack.size(); ++current_component)
	{
		component_info a = component_info_stack.at(current_component);
		Dir current_normal(0, 0, 0);
		for (unsigned int current_adress = a.index_of_first_point; current_adress < a.index_of_first_point + a.nr_points; ++current_adress)
		{
			average_middle += pc.transformed_pnt(current_adress);
		}
		nr_off_all_points += a.nr_points;
	}
	//needed for a middle position of the scan
	average_middle /= nr_off_all_points;
	Dir xz_addition = Pnt(0,0,0) - average_middle;
	//needed for the height, so that no point is below the tables surface
	float deepest_y = find_deepest_BB_point();
	float y_addition = 0;
	if (deepest_y < TABLE_HEIGT)
	{
		y_addition = TABLE_HEIGT - deepest_y;
	}
	for (int x = 0; x < int(search_ids.size()); ++x)
	{
		cgv::gui::animate_with_linear_blend(pc.component_translation(search_ids[x]), pc.component_translation(search_ids[x]) + Dir(xz_addition.x(), y_addition, xz_addition.z()),ANIMATION_DURATION,0,false);
	}
	push_back_state();
}

void vr_point_cloud_aligner::start_ICP()
{
	//int nr_of_all_points_source = 0;
	//std::vector<int> Ids_source = picked_group->get_component_IDs();
	//for (int i = 0; i < int(Ids_source.size()); ++i)
	//{
	//	component_info a = pc.component_point_range(Ids_source.at(i));
	//	nr_of_all_points_source += a.nr_points;
	//}
	//int subsampling_range = int(round(nr_of_all_points_source / SAMPLE_COUNT));
	//if (subsampling_range < 1)
	//{
	//	subsampling_range = 1;
	//}
	//if (subsample_changed)
	//{
	//	subsample(vertices_source, vertices_source_copy, vertices_target,subsampling_range);
	//	subsample_changed = false;
	//}
	//
	//ICP::Parameters par;
	//par.p = .5;
	//par.max_icp = 100;
	//auto tic = std::chrono::steady_clock::now();
	//ICP::point_to_point(vertices_source, vertices_target, par);
	//auto toc = std::chrono::steady_clock::now();
	//double time_ms = std::chrono::duration <double, std::milli>(toc - tic).count();
	//printf("ICP finished, runtime: %f \n",time_ms);
	/////Now the vertices_source is recalculated and overwritten. we now need to calculate back the actual rotation and translation.
	/////This should be now possible through using some points in the source cloud that have been transformed. Because of correspondencies there is now a closed solution form
	/////Using this method the first 4 points should be enough to solve this. The test case now uses all points.
	//Eigen::Affine3d A = kabsch::Find3DAffineTransform(vertices_source_copy, vertices_source);
	///*for (int m = 0; m < 5; ++m) 
	//{
	//	printf("First 5 Points of ICP:\n");
	//	printf("ICPresult %d: %f, %f, %f; Original: %f, %f, %f; Target: %f, %f, %f \n", m, vertices_source(0, m), vertices_source(1, m), vertices_source(2, m), vertices_source_copy(0, m), vertices_source_copy(1, m), vertices_source_copy(2, m), vertices_target(0, m), vertices_target(1, m), vertices_target(2, m));
	//}*/
	//Eigen::Vector3d translation_vec = A.translation();
	//cgv::math::mat<float> casted_mat;
	//casted_mat.resize(3, 3);
	//Eigen::Matrix<double, 3, 3> R = A.rotation();
	//
	//double m00 = R(0, 0),m01=R(0,1),m02 =R(0,2), m10=R(1,0), m11 = R(1, 1), m12 = R(1,2), m20 = R(2,0), m21 = R(2,1), m22 = R(2, 2);
	//double qx, qy, qz, qw;
	//double tr = m00 + m11 + m22;

	//	if (tr > 0)
	//	{
	//		double S = sqrt(tr + 1.0) * 2; // S=4*qw 
	//		qw = 0.25 * S;
	//		qx = (m21 - m12) / S;
	//		qy = (m02 - m20) / S;
	//		qz = (m10 - m01) / S;
	//	}
	//	else if ((m00 > m11)&(m00 > m22))
	//	{
	//		double S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
	//		qw = (m21 - m12) / S;
	//		qx = 0.25 * S;
	//		qy = (m01 + m10) / S;
	//		qz = (m02 + m20) / S;
	//	}
	//	else if (m11 > m22)
	//	{
	//		double S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
	//		qw = (m02 - m20) / S;
	//		qx = (m01 + m10) / S;
	//		qy = 0.25 * S;
	//		qz = (m12 + m21) / S;
	//	}
	//	else
	//	{
	//		double S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
	//		qw = (m10 - m01) / S;
	//		qx = (m02 + m20) / S;
	//		qy = (m12 + m21) / S;
	//		qz = 0.25 * S;
	//	}
	//	
	//std::vector<int> temp_IDs = picked_group->get_component_IDs();
	//if (!translation_vec.allFinite())
	//{
	//	printf("Kabsch or ICP Failure!");
	//}
	//else 
	//{
	//	//cgv::gui::animate_with_axis_rotation(,,,2,2,false)
	//	for (int i = 0; i < int(temp_IDs.size()); ++i)
	//	{
	//		pc.component_rotation(temp_IDs.at(i)).set(qw, qx, qy, qz);
	//		//pc.component_translation(temp_IDs.at(i)).set(translation_vec.x(), translation_vec.y(), translation_vec.z());
	//		cgv::gui::animate_with_linear_blend(pc.component_translation(temp_IDs.at(i)), Dir(translation_vec.x(), translation_vec.y(), translation_vec.z()), ANIMATION_DURATION, 0, false);

	//	}
	//}
	//Set important flags when finished!
	transformation_lock = false;
	icp_executing = false;
	pending_unite = true;

	post_redraw();
}


bool vr_point_cloud_aligner::ensure_view_pointer()
{
	cgv::base::base_ptr bp(dynamic_cast<cgv::base::base*>(this));
	if (bp) {
		std::vector<cgv::render::view*> views;
		cgv::base::find_interface<cgv::render::view>(bp, views);
		if (!views.empty()) {
			view_ptr = views[0];
			return true;
		}
	}
	return false;
}

void vr_point_cloud_aligner::try_component_pick()
{
	if (!seperation_in_process) 
	{
		return;
	}
	//check for box intersections
	if (have_picked_point && !pending_unite)
	{
		std::vector<Crd> intersectedPoints;
		std::vector<int> component_NR;
		std::vector<int> picked_group_ids = picked_group->get_component_IDs();
		for (int i = 0; i < int(picked_group_ids.size()); i++)
		{
			Crd temp = box_ray_intersection(last_view_point, picked_point, pc.box(picked_group_ids.at(i)), pc.component_translation(picked_group_ids.at(i)), pc.component_rotation(picked_group_ids.at(i)));
			if (temp > 0)
			{
				intersectedPoints.push_back(temp);
				component_NR.push_back(picked_group_ids.at(i));
			}
		}

		int toPrint = intersectedPoints.size();
		printf("%d \n", toPrint);
		if (intersectedPoints.size() != 0)
		{

			Dir ray_dir = picked_point - last_view_point;
			float z_factor_min = 1000.0f;
			int min_component = -1;
			for (int i = 0; i < int(intersectedPoints.size()); ++i)
			{
				if (z_factor_min > intersectedPoints.at(i))
				{
					min_component = component_NR.at(i);
					z_factor_min = intersectedPoints.at(i);
				}
			}
			//If there was a previously picked component restore its color before picking the new one
			if (picked_component_valid)
			{
				pc.component_color(picked_component) = picked_component_color;
			}
			picked_component = min_component;
			picked_component_valid = true;
			picked_component_color = pc.component_color(picked_component);
			pc.component_color(picked_component) = RGBA(1, 0, 0, 1);
		}
		else
		{
			//Do nothing
		}
	}
}

void vr_point_cloud_aligner::deselect_groups()
{
	printf("deselected groups\n");
	std::vector<int> toChange = picked_group->get_component_IDs();
	for (int x = 0; x < int(toChange.size()); ++x)
	{
		pc.component_color(toChange.at(x)) = oldColor;
	}
	toChange = previous_picked_group->get_component_IDs();
	for (int x = 0; x < int(toChange.size()); ++x)
	{
		pc.component_color(toChange.at(x)) = even_older_color;
	}
	picked_group = &global_placeholder_set;
	previous_picked_group = &global_placeholder_set;
}


bool vr_point_cloud_aligner::try_group_pick()
{
	if (transformation_lock || pending_unite)
	{
		return false;
	}

	//check for box intersections
	if (have_picked_point)
	{
		std::vector<Crd> intersectedPoints;
		std::vector<int> component_NR;
		for (int i = 0; i < int(pc.get_nr_components()); i++)
		{
			Crd temp = box_ray_intersection(last_view_point, picked_point, pc.box(i), pc.component_translation(i), pc.component_rotation(i));
			if (temp != INFINITY)
			{
				intersectedPoints.push_back(temp);
				component_NR.push_back(i);
			}
		}

		int toPrint = intersectedPoints.size();
		printf("%d \n", toPrint);
		if (intersectedPoints.size() != 0)
		{

			Dir ray_dir = picked_point - last_view_point;
			float z_factor_min = INFINITE;
			int min_component = -1;
			for (int i = 0; i < int(intersectedPoints.size()); ++i)
			{
				if (z_factor_min > intersectedPoints.at(i))
				{
					min_component = i;
					z_factor_min = intersectedPoints.at(i);
				}
			}
			int a = component_NR.at(min_component);
			current_picked_distance = z_factor_min;
			constructed_set* new_pick;
			int count = 0;
			//Search for possible picked groups and update if nothing is found return
			for (auto s : sets)
			{
				if (s.find_component_ID(a))
				{
					new_pick = &s;
					//only update if new pick is really new!
					if (new_pick->get_ID() >= 0 && new_pick->get_ID() != picked_group->get_ID())
					{
						printf("Picked a group\n");
						//reset colors of older group
						if (previous_picked_group->get_ID() >= 0)
						{
							std::vector<int> toChange = previous_picked_group->get_component_IDs();
							for (int x = 0; x < int(toChange.size()); ++x)
							{
								pc.component_color(toChange.at(x)) = even_older_color;
							}
						}
						//reset colors of old group
						if (picked_group->get_ID() >= 0)
						{
							std::vector<int> toChange = picked_group->get_component_IDs();
							for (int x = 0; x < int(toChange.size()); ++x)
							{
								pc.component_color(toChange.at(x)) = RGBA(0.5, 0, 0, 1);
							}
						}
						//set picked to previous picked
						previous_picked_group = picked_group;
						//save back older color
						even_older_color = oldColor;
						//Save back old color
						oldColor = pc.component_color(a);
						//change active group to red
						std::vector<int> toChange = new_pick->get_component_IDs();
						for (int x = 0; x < int(toChange.size()); ++x)
						{
							pc.component_color(toChange.at(x)) = RGBA(1, 0, 0, 1);
						}
						//Save pick by iterator advancing
						std::list<constructed_set>::iterator iter = sets.begin();
						std::advance(iter, count);
						picked_group = &iter._Ptr->_Myval;
						push_back_state();

					}
					subsample_changed = true;
					post_redraw();
				}
				count++;
			}
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
	return true;
}



void vr_point_cloud_aligner::draw(cgv::render::context& ctx)
{
	// store current transformation matrix from world to device coordinates
	DPV = ctx.get_modelview_projection_device_matrix();

	point_cloud_interactable::draw(ctx);

	// draw array of boxes with renderer from gl_point_cloud_drawable and my box_render_style
	b_renderer.set_render_style(box_render_style);
	///This is now not needed, the viewpoint update is now from controller pose!
	if (have_view_ray) {
		glLineWidth(5);
		glColor3f(1, 1, 0);
		glBegin(GL_LINES);
		glVertex3fv(&last_view_point[0]);
		glVertex3fv(&last_target_point[0]);
		glEnd();
		glLineWidth(1);
	}

	std::vector<vec3> P;
	std::vector<rgb> C;

	if (vr_view_ptr) {

		const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
		if (state_ptr) {
			for (int i = 0; i < 2; ++i) {
				vec3 ray_origin, ray_direction;
				state_ptr->controller[i].put_ray(&ray_origin(0), &ray_direction(0));
				P.push_back(ray_origin);
				P.push_back(ray_origin + ray_length * ray_direction);
				rgb c(float(1 - i), 0, float(i));
				C.push_back(c);
				C.push_back(c);
			}
		}

	}
	else 
	{
		vec3 ray_origin, ray_direction;
		ray_origin = last_view_point;
		ray_direction = picked_point - last_view_point;
		P.push_back(ray_origin);
		P.push_back(ray_origin + ray_length * ray_direction);
		rgb c(float(1), 0, float(1));
		C.push_back(c);
		C.push_back(c);
	}
	if (P.size() > 0) {
		cgv::render::shader_program& prog = ctx.ref_default_shader_program();
		int pi = prog.get_position_index();
		int ci = prog.get_color_index();
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
		cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, C);
		cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
		glLineWidth(3);
		prog.enable(ctx);
		glDrawArrays(GL_LINES, 0, P.size());
		prog.disable(ctx);
		cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
		glLineWidth(1);
	}

	// this is actually already set to false in gl_point_cloud_drawable but repeated here to make sure that you notice that when specifying boxes with min and max points, position_is_center must be false in renderer
	b_renderer.set_position_is_center(false);
	b_renderer.set_position_array(ctx, &room_boxes[0].get_min_pnt(), room_boxes.size(), sizeof(Box));
	// max points are passed to extent array in case of position_is_center is false
	b_renderer.set_extent_array(ctx, &room_boxes[0].get_max_pnt(), room_boxes.size(), sizeof(Box));
	b_renderer.set_color_array(ctx, &room_colors[0], room_colors.size());
	b_renderer.validate_and_enable(ctx);
	glDrawArrays(GL_POINTS, 0, room_boxes.size());
	b_renderer.disable(ctx);
}

bool vr_point_cloud_aligner::box_ray_intersection(const Pnt& ray_start, const Pnt& ray_dir, const Box& box, point_cloud_types::Pnt& result)
{
	//Boxes are axis aligned defined by 2 coordinates. This means one can use the techniques of ECG raytracing slides s22
	//Calculate for each axis its intersection intervall. Then intersect all intervalls to get the target intervall and its parameters
	interval xintersection = calculate_intersectionintervall(ray_start.x(), box.get_min_pnt().x(), box.get_max_pnt().x(), ray_dir.x());
	//printf("Local xIntersection %f %f\n", xintersection.get_min(), xintersection.get_max());
	interval yintersection = calculate_intersectionintervall(ray_start.y(), box.get_min_pnt().y(), box.get_max_pnt().y(), ray_dir.y());
	//printf("Local yIntersection %f %f\n", yintersection.get_min(), yintersection.get_max());
	interval zintersection = calculate_intersectionintervall(ray_start.z(), box.get_min_pnt().z(), box.get_max_pnt().z(), ray_dir.z());
	//printf("Local zIntersection %f %f\n", zintersection.get_min(), zintersection.get_max());
	if (xintersection.isInvalid() || yintersection.isInvalid() || zintersection.isInvalid())
	{
		return false;
	}
	interval solution = (xintersection.intersectIntervals(yintersection)).intersectIntervals(zintersection);
	if (solution.isInvalid()) {
		return false;
	}
	//I am not certain about this solution, somehow there should be a number € Tq that is the solution. Anyway the minimum should not be that far away from the solution
	result = ray_start + ray_dir * solution.get_min();
	return true;
}

point_cloud_types::Crd vr_point_cloud_aligner::box_ray_intersection(const Pnt& ray_start, const Pnt& ray_end, const Box& box, const Dir& box_translation, const Qat& box_rotation)
{
	//Implementation plan: apply inverse transformations to rays to transform them to the local coordinate system of the box. Calculate intersection there, then transfrom back to global
	Pnt local_ray_start = transform_to_local(ray_start,box_translation,box_rotation);
	Pnt local_ray_direction = transform_to_local(ray_end, box_translation, box_rotation) - local_ray_start;
	Pnt local_result;
	if (!box_ray_intersection(local_ray_start, local_ray_direction, box, local_result))
		return INFINITY;
	printf("Intersection, local Coordinates: %f %f %f\n", local_result.x(), local_result.y(), local_result.z());
	//Pnt local_intersection_point = local_ray_start + local_result * local_ray_direction;
	Pnt global_intersection_point = box_translation + box_rotation.apply(local_result);
	printf("Intersection, global Coordinates: %f %f %f \n", global_intersection_point.x(), global_intersection_point.y(), global_intersection_point.z());
	Pnt degub_box_min__pnt = box_translation + box_rotation.apply(box.get_min_pnt());
	Pnt degub_box_max__pnt = box_translation + box_rotation.apply(box.get_max_pnt());
	printf("AABB min: %f %f %f \n", degub_box_min__pnt.x(), degub_box_min__pnt.y(), degub_box_min__pnt.z());
	printf("AABB max: %f %f %f \n", degub_box_max__pnt.x(), degub_box_max__pnt.y(), degub_box_max__pnt.z());

	Dir ray_dir = ray_end - ray_start;
	Crd global_result = ((global_intersection_point.x() - ray_start.x()) / ray_dir.x());
	return global_result;
}

void vr_point_cloud_aligner::load_project_file(std::string projectFile)
{
	//Important step!
	sets.clear();
	///using filestream to read
	std::ifstream inFile;
	inFile.open(projectFile);
	if (!inFile) {
		printf("Unable to open file");
		return;
	}
	std::string line;
	bool first_read = true;
	projectLoading_in_process = true;
	std::vector<int> alignment_IDs;
	user_modified.clear();
	file_paths.clear();
	while (std::getline(inFile, line))
	{
		std::istringstream iss(line);
		//pc.create_components();
		//File Format looks like this:
		//path of file			Translation	Quarternion(rotation)	UserFlag		alignment ID
		//componentpath.ply		x y z		re qx qy qz				isUserModified	id
		std::string fileName;
		int alignment_ID;
		float x, y, z;
		float re, xi, yi, zi;
		bool isUserModified;
		if (!(iss >> fileName >> x >> y >> z >> re >> xi >> yi >> zi >> isUserModified >> alignment_ID)) {
			//If reading fails, continue next
			continue;
		}
		if (fileName.empty()) {
			continue;
		}
		///This is necessary so that the create componets functions does not create empty components
		if (first_read) {
			if (!pc.read(fileName)) {
				continue;
			}
			transformation_lock = true;
			pc.create_components();
			pc.create_component_tranformations();
			pc.create_component_colors();
			pc.component_translation(0).set(x, y, z);
			pc.component_rotation(0).set(re, xi, yi, zi);
			pc.component_color(0) = RGBA(float(pc.get_nr_components()) / float(20),0.5f, 0.5f,1.0f);
			user_modified.push_back(isUserModified);
			file_paths.push_back(fileName);
			transformation_lock = false;
			first_read = false;
			alignment_IDs.push_back(alignment_ID);
			on_point_cloud_change_callback(PointCloudChangeEvent::PCC_NEW_POINT_CLOUD);
		}
		else {
			point_cloud pc_to_append;
			if (pc_to_append.read(fileName)) {
				transformation_lock = true;
				pc_to_append.create_components();
				pc_to_append.create_component_tranformations();
				pc_to_append.create_component_colors();
				pc_to_append.component_translation(0).set(x, y, z);
				pc_to_append.component_rotation(0).set(re, xi, yi, zi);
				pc_to_append.component_color(0) = cgv::media::color<float, cgv::media::HLS, cgv::media::OPACITY>(float(pc.get_nr_components()+1) / float(20), 0.5f, 0.5f, 1.0f);
				user_modified.push_back(isUserModified);
				file_paths.push_back(fileName);
				pc.append(pc_to_append);
				transformation_lock = false;
				alignment_IDs.push_back(alignment_ID);
				printf("\n");
				on_point_cloud_change_callback(PointCloudChangeEvent(PCC_POINTS_RESIZE + PCC_COMPONENTS_RESIZE));
			}
		}
	}

	// Algorithm to reconstruct the unite set structure
	std::vector<constructed_set> copySet;
	for (int l = 0; l < int(alignment_IDs.size()); ++l)
	{
		std::vector<int> a;
		a.push_back(l);
		copySet.push_back(constructed_set(a, alignment_IDs.at(l)));
	}
	std::vector<int> usedIDs;
	for(int l = 0; l < int(alignment_IDs.size()); ++l)
	{
		int currentminimizer = alignment_IDs.at(l);
		bool skip = false;
		for (int k = 0; k < int(usedIDs.size()); ++k)
		{
			if (currentminimizer == usedIDs.at(k))
				skip = true;
		}
		if (skip)
			continue;
		std::vector<int> b;
		constructed_set currentSet(b, currentminimizer);
		for (int j = 0; j < int(copySet.size()); ++j)
		{
			if (currentminimizer == copySet.at(j).get_ID())
			{
				currentSet.unite(copySet.at(j));
				std::vector<int> colorchange = currentSet.get_component_IDs();
				for (int t = 0; t < int(colorchange.size()); t++)
				{
					pc.component_color(colorchange.at(t)) = pc.component_color(colorchange.at(0));
				}
			}

		}
		sets.push_back(currentSet);
		usedIDs.push_back(currentminimizer);
	}
	projectLoading_in_process = false;
	//delete current stack and start a new one
	program_state_stack.erase(program_state_stack.begin(), program_state_stack.end());
	pss_count = 0;
	//deleting the stack resets some variables too!
	
	push_back_state();
	post_redraw();
}

void vr_point_cloud_aligner::save_project_file(std::string projectFile)
{
	std::ofstream outFile; 

	outFile.open(projectFile);
	if (0 == pc.get_nr_components()) {
		printf("Keine Komponenten zum speichern!");
	}
	for (int i = 0; i < int(pc.get_nr_components()); i++) {
		int alignmentID = -1;
		for (auto s : sets) 
		{
			if (s.find_component_ID(i))
				alignmentID = s.get_ID();
		}
		outFile << ' ' << file_paths.at(i) << ' ' << pc.component_translation(i) << ' ' << pc.component_rotation(i).re() << ' ' << pc.component_rotation(i).im() << ' ' << user_modified.at(i) << ' ' << alignmentID <<'\n';
	}
	outFile.close();
}

point_cloud_types::Pnt vr_point_cloud_aligner::transform_to_local(const Pnt& in, const Pnt& local_translation, const Qat& local_rotation)
{
	Pnt result = in - local_translation;
	local_rotation.inverse_rotate(result);
	return result;
}

point_cloud_types::Pnt vr_point_cloud_aligner::transform_to_global(const Pnt& in, const Pnt& local_translation, const Qat& local_rotation)
{
	//	Pnt global_intersection_point = box_translation + box_rotation.apply(local_result);
	Pnt result = local_translation + local_rotation.apply(in);
	return result;
}

interval vr_point_cloud_aligner::calculate_intersectionintervall(float rayStart, float maxBoxCoord1, float maxBoxCoord2, float raydir)
{
	//X0 = px + t* x vx
	//First we need the parameter vx especially its sign and if its bigger 0
	// vx = abl(x)/abl(t) = vx
	float px = rayStart;
	float x0 = maxBoxCoord1;
	float x1 = maxBoxCoord2;
	//delta x / delta t is the same as ray_dir (1 step of it)
	float vx = raydir;
	float t0 = 0, t1 = 0;
	if (vx == 0)
	{
		// Does not interesect the given axis of the box -> no intersection
		interval a(1.0,0.0);
		return a;
	}
	else if (vx > 0)
	{
		t0 = (x0 - px) / vx;
		t1 = (x1 - px) / vx;
	}
	else
	{
		t0 = (x1 - px) / vx;
		t1 = (x0 - px) / vx;
	}
	return interval(t0, t1);
}

void vr_point_cloud_aligner::update_picked_point(cgv::render::context& ctx, int x, int y)
{
	have_picked_point = true;
	//const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
	//Pnt origin(state_ptr->controller[0].pose[9], state_ptr->controller[0].pose[10], state_ptr->controller[0].pose[11]);
	//picked_point = origin + Pnt(state_ptr->controller[0].pose[9], 0, 0);
	
	//double z = ctx.get_z_D(x, y);
	//// check for background
	//if (z > 0.99999999) {
	//	if (have_picked_point)
	//		post_redraw();
	//	have_picked_point = false;
	//}
	//else
	//{

	//	have_picked_point = true;
	//	picked_point = Pnt((&ctx.get_point_W(x, y, DPV))[0]);
	//	post_redraw();
	//}
}

void vr_point_cloud_aligner::display_reverse_seperation() 
{
	restore_state(pss_count - 1);
	post_redraw();
}

void vr_point_cloud_aligner::seperate_component() 
{
	/// WORK IN PROGRESS -> KNOWN BUG: Colors do not update as expected after updating. (use pointers for picked components?)
	if (!picked_group->find_component_ID(picked_component))
	{
		printf("Critical error in seperation process!\n");
		picked_component_valid = false;
	}
	else 
	{
		display_reverse_seperation();
		int count = 0;
		for (auto s : sets)
		{
			if (s.get_ID() == picked_group->get_ID())
			{
				break;
			}
			count++;
		}
		std::list<constructed_set>::iterator iter = sets.begin();
		std::advance(iter, count);
		iter._Ptr->_Myval.seperate_component(picked_component);

		int lowest_available_ID = 0;
		for (auto s : sets)
		{
			for (auto d : sets)
			{
				if (d.get_ID() == lowest_available_ID)
				{
					++lowest_available_ID;
					break;
				}
			}
		}
		std::vector<int> newly_vec;
		newly_vec.push_back(picked_component);
		constructed_set newly_seprated_single_component_set(newly_vec, lowest_available_ID);
		sets.push_back(newly_seprated_single_component_set);
		///Push it up to see it better and dont start before old animation is finished
		cgv::gui::animate_with_linear_blend(pc.component_translation(picked_component), pc.component_translation(picked_component) + Dir(0,0,1), ANIMATION_DURATION, ANIMATION_DURATION, false);
		//Autoselect new component mostly code from the picking algorithm
		printf("Picked a new created group\n");
		//reset colors of older group
		if (previous_picked_group->get_ID() >= 0)
		{
			std::vector<int> toChange = previous_picked_group->get_component_IDs();
			for (int x = 0; x < int(toChange.size()); ++x)
			{
				pc.component_color(toChange.at(x)) = even_older_color;
			}
		}
		//reset colors of old group
		if (picked_group->get_ID() >= 0)
		{
			std::vector<int> toChange = picked_group->get_component_IDs();
			for (int x = 0; x < int(toChange.size()); ++x)
			{
				pc.component_color(toChange.at(x)) = RGBA(0.5, 0, 0, 1);
			}
		}
		//set picked to previous picked
		previous_picked_group = picked_group;
		//save back older color
		even_older_color = oldColor;
		//Save back old color (which is a new created one hence the component has been created new by seperation
		oldColor = cgv::media::color<float, cgv::media::HLS, cgv::media::OPACITY>(float(newly_seprated_single_component_set.get_ID()) / float(sets.size()), 0.5f, 0.5f, 1.0f);
		//change active group to red
		std::vector<int> toChange = newly_seprated_single_component_set.get_component_IDs();
		for (int x = 0; x < int(toChange.size()); ++x)
		{
			pc.component_color(toChange.at(x)) = RGBA(1, 0, 0, 1);
		}
		//Save pick
		picked_group = &sets.back();
		push_back_state();
		subsample_changed = true;
		picked_component_valid = false;
		transformation_lock = false;
		post_redraw();
	}
}

void vr_point_cloud_aligner::display_seperation_selection()
{
	//Start by calculating BB diagonals
	std::vector<int> ids_of_group = picked_group->get_component_IDs();
	double max_bb_diagonal = double(0);
	for (int i = 0; i < int(ids_of_group.size()); ++i)
	{
		Dir BB_diag = pc.box(ids_of_group.at(i)).get_max_pnt() - pc.box(ids_of_group.at(i)).get_min_pnt();
		if (max_bb_diagonal < double(BB_diag.length()))
		{
			max_bb_diagonal = BB_diag.length();
		}
	}
	if (!pc.has_normals())
	{	
		printf("Normals need to be calculated first!");
		seperation_in_process = false;
		transformation_lock = false;
		return;
	}
	//calculate per component average normal to estimate optimal splitting direction
	std::vector<component_info> component_info_stack_source;
	int nr_off_all_points = 0;
	for (unsigned int i = 0; i < ids_of_group.size(); ++i)
	{
		component_info a = pc.component_point_range(ids_of_group.at(i));
		component_info_stack_source.push_back(a);
		nr_off_all_points += a.nr_points;
	}

	std::vector<Dir> averaged_normal_direction;
	Pnt average_middle(0, 0, 0);
	for (unsigned int current_component = 0; current_component < component_info_stack_source.size(); ++current_component)
	{
		component_info a = component_info_stack_source.at(current_component);
		Dir current_normal(0, 0, 0);
		for (unsigned int current_adress = a.index_of_first_point; current_adress < a.index_of_first_point + a.nr_points; ++current_adress)
		{
			current_normal += pc.nml(current_adress);
			average_middle += pc.transformed_pnt(current_adress);
		}
		current_normal /= a.nr_points;
		current_normal.normalize();
		averaged_normal_direction.push_back(current_normal);
	}
	average_middle /= nr_off_all_points;
	// Problems -> scans with similiar normals have similiar directions
	// Solution: use fixed directions. For every averaged normal, the best direction is choosen via crossproduct with fixed direction options
	// This way no scan overlaps
	// Problem -> the number of directions has to be dynamic and in best case the directions adjust to the number
	// Solution: Most scans components are taken in an 30° intervall. use these 30° intervals as ground directions. calculate the crossproduct to all directions.
	// Find the smallest product and use this direction. Repeat for all scans. Normally the scans should all go to different directions.
	// Solution 2: Use a formula to devide the 360° circle of the xy area to equal parts and use the biggest numbers as top scan if the number is not even(the one going updward towards z)
	// step 1 obtain rotation matrix formula
	double rotation_rad = 0;
	int rotation_steps = 0;
	bool uneven_comp_number = false;
	if (ids_of_group.size() % 2 == 0)
	{
		rotation_steps = int(ids_of_group.size());
		uneven_comp_number = false;
	}
	else
	{
		rotation_steps = int(ids_of_group.size()-1);
		uneven_comp_number = true;
	}
	rotation_rad = 2 * std::_Pi / rotation_steps;

	double needed_lentgth = 1.1*max_bb_diagonal / abs(2 * sin(rotation_rad / 2));
	
	//A rotation matrix that rotates for x degrees. Do not forget to multiply PI/180!
	cgv::math::fmat<float,3,3> rotation_mat;
	rotation_mat(0, 0) = cos(rotation_rad);
	rotation_mat(0, 1) = 0;
	rotation_mat(0, 2) = sin(rotation_rad);
	rotation_mat(1, 0) = 0;
	rotation_mat(1, 1) = 1;
	rotation_mat(1, 2) = 0;
	rotation_mat(2, 0) = -sin(rotation_rad);
	rotation_mat(2, 1) = 0;
	rotation_mat(2, 2) = cos(rotation_rad);;
	std::vector<Dir> rotated_directions;
	std::set<int> matched_ids;
	std::vector<int> ids;
	Dir basic_vec(1, 0, 0);
	for (int i = 0; i < rotation_steps; ++i)
	{
		//Dir basic_vec(cos(i*rotation_rad), sin(i*rotation_rad), 0);
		rotated_directions.push_back(basic_vec);
		double temp_cross_product = 1000;
		int lowest_ID = -1;
		for (int x = 0; x < int(averaged_normal_direction.size());++x)
		{
			bool debug = !(matched_ids.find(ids_of_group.at(x)) == matched_ids.end());
			if (debug)
			{
				continue;
			}
			if ((cross(averaged_normal_direction.at(x), basic_vec).length()) < temp_cross_product)
			{
				lowest_ID = ids_of_group.at(x);
				temp_cross_product = cross(averaged_normal_direction.at(x), basic_vec).length();
			}
		}
		matched_ids.insert(lowest_ID);
		ids.push_back(lowest_ID);
		basic_vec = (rotation_mat * basic_vec);
	}
	//step 2 apply all the changes by animation and if the number is uneven find the uneven scan and lift it
	for (int x = 0; x < int(ids.size()); ++x)
	{
		//animate with transition to rotated direction
		rotated_directions.at(x).normalize();
		Dir translation = (rotated_directions.at(x) * needed_lentgth) + pc.component_translation(ids.at(x));
		printf("%d iteration, translated compNR: %d in direction: %f %f %f \n", x,ids.at(x),rotated_directions.at(x).x(), rotated_directions.at(x).y(), rotated_directions.at(x).z());
		cgv::gui::animate_with_linear_blend(pc.component_translation(ids.at(x)), translation, ANIMATION_DURATION, 0, false)->set_base_ptr(base_ptr(this));
		RGBA color_cast = cgv::media::color<float, cgv::media::HLS, cgv::media::OPACITY>(float(x) / float(ids.size()), 0.5f, 0.5f, 1.0f);
		cgv::gui::animate_with_linear_blend(pc.component_color(ids.at(x)), color_cast, ANIMATION_DURATION, 0, false)->set_base_ptr(base_ptr(this));
		post_redraw();		
	}
	if (uneven_comp_number)
	{
		for (int x = 0; x < ids_of_group.size(); ++x)
		{
			if (matched_ids.find(ids_of_group.at(x)) == matched_ids.cend())
			{
				//animate with transition to top direction
				Dir translation = (Dir(0,1,0) * needed_lentgth) + average_middle;
				cgv::gui::animate_with_linear_blend(pc.component_translation(ids_of_group.at(x)), Dir(translation.x(), translation.y(), translation.z()), ANIMATION_DURATION, 0, false);
				cgv::gui::animate_with_linear_blend(pc.component_color(ids_of_group.at(x)),RGBA(0.5, 0.5, 0.5, 0.5), ANIMATION_DURATION,0,false);
				post_redraw();
			}
		}
	}
	//step 3 cleanup and post redraw
	post_redraw();
}

bool vr_point_cloud_aligner::scale(float scaling_difference)
{
	//Note: does scaling affect the ground truth? Has the ground truth to be scaled along with the scans to be used?

	if(pc.get_nr_components() < 1)
		return false;
	// If the user already modified his scans, rescaling is not posible, because it would destroy current alignments.
	for (int x = 0; x < int(user_modified.size()); ++x)
	{
		if (user_modified[x])
			return false;
	}
	//Reverse old scaling
	mat4 scaling_mat;
	scaling_mat.identity();

	scaling_mat(0, 0) = 1/current_scaling_factor;
	scaling_mat(1, 1) = 1/current_scaling_factor;
	scaling_mat(2, 2) = 1/current_scaling_factor;
	scaling_mat(3, 3) = 1;
	pc.transform(scaling_mat);
	
	//now apply new scaling
	current_scaling_factor += scaling_difference;
	scaling_mat.identity();
	
	scaling_mat(0, 0) = current_scaling_factor;
	scaling_mat(1, 1) = current_scaling_factor;
	scaling_mat(2, 2) = current_scaling_factor;
	scaling_mat(3, 3) = 1;
	pc.transform(scaling_mat);
	//Scalierung mit transform -> Scalierungsfaktor speichern!
	//UpdateBoxes!
}

bool vr_point_cloud_aligner::handle(cgv::gui::event& e)
{
	if ((e.get_flags() & cgv::gui::EF_VR) != 0)
	{
		switch (e.get_kind())
		{
			case cgv::gui::EID_KEY:
			{
				cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
				if (vrke.get_action() == cgv::gui::KA_RELEASE || vrke.get_action() == cgv::gui::KA_REPEAT)
				{
					switch (vrke.get_key())
					{
						//right controller actions
						case vr::VR_RIGHT_BUTTON0:
							if (!seperation_in_process && picked_group->get_ID() != -1 && picked_group->get_component_IDs().size() > 1)
							{
								seperation_in_process = true;
								transformation_lock = true;
								std::thread sep_thread(&vr_point_cloud_aligner::display_seperation_selection, this);
								sep_thread.detach();
								post_redraw();
							}
							return true;
						case vr::VR_RIGHT_STICK_UP:
							if (!transformation_lock)
							{
								repostion_above_table();
							}
							return true;
						case vr::VR_RIGHT_STICK_LEFT:
							if (pending_unite) {
								unite(false);
								pending_unite = false;
								transformation_lock = false;
							}
							else if (seperation_in_process)
							{
								seperation_in_process = false;
								transformation_lock = false;
								display_reverse_seperation();
							}
							else
								restore_state(pss_count - 2);
							return true;
						case vr::VR_RIGHT_STICK_RIGHT:
							if(!transformation_lock)
								restore_state(pss_count);
							return true;
						case vr::VR_RIGHT_STICK_DOWN:
						{
							icp_executing = true;
							transformation_lock = true;
							printf("ICP pressed\n");
							//std::thread icp_thread(&vr_point_cloud_aligner::start_ICP, this);
							//icp_thread.detach();
							start_ICP();
							return true;
						}
						case vr::VR_RIGHT_MENU:
							if (pending_unite) {
								unite(true);
								pending_unite = false;
							}
							else if (seperation_in_process)
							{
								if (picked_component_valid)
								{
									seperation_in_process = false;
									seperate_component();
								}
							}
							transformation_lock = false;
							return true;

							//Left controlller actions
						case vr::VR_LEFT_BUTTON0:
							if (!transformation_lock)
							{
								transformation_lock = true;
								reset_componets_transformations();
								transformation_lock = false;
							}
							return true;
						case vr::VR_LEFT_MENU:
							if (!transformation_lock)
							{
								deselect_groups();
							}
							return true;
					}
				}
			}
			case cgv::gui::EID_STICK:
			{
				cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
				switch (vrse.get_action()) {
				case cgv::gui::SA_TOUCH:
				case cgv::gui::SA_PRESS:
				case cgv::gui::SA_UNPRESS:
				case cgv::gui::SA_RELEASE:
					break;
				case cgv::gui::SA_MOVE:
				case cgv::gui::SA_DRAG:
					if (vrse.get_controller_index() == MAIN_CONTROLLER)
					{
						if (drag_active && !local_pose_mat_stack.empty()) {
							for (auto& M : local_pose_mat_stack) {
								M(2, 3) -= vrse.get_dy();
							}
							return true;
					}
					}
					else {
						if (scale(vrse.get_dy()/5))
							printf("Scaled scans to %f \n", current_scaling_factor);
						
						return true;
					}
				}
				break;
				return true;
			}
			case cgv::gui::EID_POSE:
			{
				cgv::gui::vr_pose_event& vrpo = static_cast<cgv::gui::vr_pose_event&>(e);
				if (vrpo.get_trackable_index() == MAIN_CONTROLLER)
				{
					last_view_point = vrpo.get_position();
					Dir pick_dir(float(vrpo.get_state().controller[1].pose[6]),float(vrpo.get_state().controller[1].pose[7]), float(vrpo.get_state().controller[1].pose[8]));
					picked_point = last_view_point + (-pick_dir * 5);
					have_picked_point = true;
				}
				if (drag_active && !transformation_lock) {
					drag_scan(vrpo.get_orientation(),vrpo.get_position());
				}
				return true;
			}
			case cgv::gui::EID_THROTTLE:
			{
				if (transformation_lock)
					return true;
				cgv::gui::vr_throttle_event& vrth = static_cast<cgv::gui::vr_throttle_event&>(e);
				if (vrth.get_controller_index() == MAIN_CONTROLLER)
				{
					if (vrth.get_value() > THROTTLE_THRESHOLD)
					{
						if (drag_active)
						{
							return true;
						}
						else if (seperation_in_process)
						{
							try_component_pick();
							return true;
						}
						else if (try_group_pick())
						{
							//Dragging blocks the picker so that the picker is only called once
							drag_active = true;
							calculate_local_pose(vrth.get_state().controller[MAIN_CONTROLLER].pose);
							return true;
						}
					}
					///This means the throttle is now below the dragging threshold -> Dragging ends and the state should be pushed back if the user dragged something
					else 
					{
						if(drag_active)
							push_back_state();
						drag_active = false;
						return true;
					}
				}
			}
		}
	}
	else if (e.get_kind() == cgv::gui::EID_KEY) {
		cgv::gui::key_event& ke = static_cast<cgv::gui::key_event&>(e);
		if (ke.get_action() == cgv::gui::KA_PRESS || ke.get_action() == cgv::gui::KA_REPEAT) {
			switch (ke.get_key()) {
			case 'R':
				reset_componets_transformations();
				return true;
			case 'I':
				icp_executing = true;
				return true;
			case 'C':
				if (pending_unite) {
					unite(false);
					pending_unite = false;
				}
				else if (seperation_in_process)
				{
					seperation_in_process = false;
					transformation_lock = false;
					display_reverse_seperation();
				}
				return true;			
			case 'U':
				if (pending_unite) {
					unite(true);
					pending_unite = false;
				}
				else if (seperation_in_process)
				{
					if (picked_component_valid)
					{
						seperation_in_process = false;
						seperate_component();
					}
				}
				return true;
			case 'S':
				if (!seperation_in_process && picked_group->get_ID() != -1 && picked_group->get_component_IDs().size() > 1)
				{
					seperation_in_process = true;
					transformation_lock = true;
					std::thread sep_thread(&vr_point_cloud_aligner::display_seperation_selection, this);
					sep_thread.detach();
					post_redraw();
				}
				return true;
			case 'T':
				if (!transformation_lock)
				{
					repostion_above_table();
				}
				return true;
			}

		}
		else if (ke.get_action() == cgv::gui::KA_RELEASE) 
		{
			switch (ke.get_key()) {
			case 'I':
				icp_executing = false;
				pending_unite = true;
				return true;
			case 'Z': 
				restore_state(pss_count - 2);
				return true;
			case 'Y':
				restore_state(pss_count);
			}
		}
	}
	else if (e.get_kind() == cgv::gui::EID_MOUSE) {
		// convert event to mouse event
		cgv::gui::mouse_event& me = static_cast<cgv::gui::mouse_event&>(e);
		// check if context is available
		cgv::render::context* ctx_ptr = get_context();
		if (me.get_modifiers() == cgv::gui::EM_CTRL && ctx_ptr && ctx_ptr->make_current()) {
			update_picked_point(*ctx_ptr, me.get_x(), me.get_y());
			// handle mouse move action
			switch (me.get_action()) {
			case cgv::gui::MA_MOVE:
				return true;
			case cgv::gui::MA_PRESS:
				if (me.get_button() == cgv::gui::MB_RIGHT_BUTTON)
					return true;
				break;
			case cgv::gui::MA_RELEASE:
				if (me.get_button() == cgv::gui::MB_RIGHT_BUTTON) {
					ensure_view_pointer();
					if (view_ptr) {
						last_view_point = view_ptr->get_eye() - 0.5*view_ptr->get_view_up_dir();
						last_target_point = picked_point;
						have_view_ray = have_picked_point;
						if (seperation_in_process)
							try_component_pick();
						else
							try_group_pick();
					}
					return true;
				}
				break;
			case cgv::gui::MA_WHEEL:
				if (me.get_button_state() == cgv::gui::MB_RIGHT_BUTTON && !transformation_lock)
				{
					if (picked_group->get_ID() > -1) 
					{
						Dir vec;// = last_target_point - last_view_point;
						if (me.get_dy() > 0)
							vec = (last_target_point - last_view_point) * float(-0.1);
						else
							vec = (last_target_point - last_view_point) * float(0.1);

						std::vector<int> temp_ids = picked_group->get_component_IDs();
						for (int i = 0; i < int(temp_ids.size()); ++i) {
							Dir temp = vec + pc.component_translation(temp_ids.at(i));
							pc.component_translation(temp_ids.at(i)).set(temp.x(), temp.y(), temp.z());
						}						
						push_back_state();
						return true;
					}
				}
				break;
			}
		}
	}


	// pass on remaining events to base class
	return point_cloud_interactable::handle(e);
}
void vr_point_cloud_aligner::on_point_cloud_change_callback(PointCloudChangeEvent pcc_event)
{
	point_cloud_interactable::on_point_cloud_change_callback(pcc_event);
	if (pcc_event == 11 && !projectLoading_in_process) {
		
		int i = pc.get_nr_components();
		if (i <= 0) {
			return;
		}

		transformation_lock = true;
		
		position_scans();
		
		transformation_lock = false;

		//The newest added component is pushed back as a new single unit set
		std::vector<int> a;
		a.push_back(pc.get_nr_components() - 1);
		sets.push_back(constructed_set(a, sets.size()));
	}
	if (pcc_event == PCC_NEW_POINT_CLOUD && !projectLoading_in_process)
	{
		if (pc.get_nr_components() != user_modified.size())
		{
			//Problem: how to tell if append or deleteionloading is applied
			//Solution: check num of components first and flush and rewrite
			//A non append operation has been made, reset stacks and reiterate them
			file_paths.clear();
			user_modified.clear();

			void* handle = cgv::utils::file::find_first(directory_name + "/*.*");
			while (handle) {
				if (!cgv::utils::file::find_directory(handle))
				{
					file_paths.push_back(directory_name + "/" + cgv::utils::file::find_name(handle));
					user_modified.push_back(false);
				}
				handle = cgv::utils::file::find_next(handle);
			}
			//The newest added component is pushed back as a new single unit set Warning: Components may not exist yet
			std::vector<int> a;
			a.push_back(pc.get_nr_components());
			sets.push_back(constructed_set(a, sets.size()));

		}
	}
	// do more handling of point clout change events here
	post_redraw();
}

void vr_point_cloud_aligner::calculate_local_pose(const float* controller_pose)
{
	//First of all obtain the transformation from global to local by using the contoller pose
	mat4 controller_pose_mat;
	controller_pose_mat.set_col(0, vec4(reinterpret_cast<const vec3&>(controller_pose[0]), 0));
	controller_pose_mat.set_col(1, vec4(reinterpret_cast<const vec3&>(controller_pose[3]), 0));
	controller_pose_mat.set_col(2, vec4(reinterpret_cast<const vec3&>(controller_pose[6]), 0));
	controller_pose_mat.set_col(3, vec4(reinterpret_cast<const vec3&>(controller_pose[9]), 1));

	mat4 inv_controller_pose_mat = inv(controller_pose_mat);

	//now assemble a global pose matrices for the picked scan group
	local_pose_mat_stack.clear();
	std::vector<int> ids = picked_group->get_component_IDs();
	for (int x = 0; x < int(ids.size()); ++x)
	{
		mat3 rot;
		pc.component_rotation(ids.at(x)).put_matrix(rot);
		rot.transpose();
		mat4 temp;
		temp.set_col(0, vec4(rot.col(0), 0));
		temp.set_col(1, vec4(rot.col(1), 0));
		temp.set_col(2, vec4(rot.col(2), 0));
		temp.set_col(3, vec4(pc.component_translation(ids.at(x)), 1));
		local_pose_mat_stack.push_back(inv_controller_pose_mat*temp);
	}
}

void vr_point_cloud_aligner::reset_componets_transformations() {
	int nr = pc.get_nr_components();
	if (nr > 0)
	{
		for (int i = 0; i < nr; i++)
		{
			float x = float(x_room_max / nr);
			pc.component_translation(i).set(x_room_min + 0.2, Crd(1.8),Crd(i * x));
			pc.component_rotation(i) = defaultFacing;
			user_modified[i] = false;
		}
	}
	push_back_state();
	post_redraw();
}

void vr_point_cloud_aligner::on_set(void* member_ptr)
{
	if (member_ptr == &project_file) {
		load_project_file(project_file);
	}
	if (member_ptr == &directory_name) {
		//Problem: how to match filenames to components?
		//Solution: use the exact same method to get the same order as like in point_cloud_interactable.cxx

		void* handle = cgv::utils::file::find_first(directory_name + "/*.*");
		while (handle) {
			if (!cgv::utils::file::find_directory(handle)) 
			{
				file_paths.push_back(directory_name + "/" + cgv::utils::file::find_name(handle));
				user_modified.push_back(false);
			}
			handle = cgv::utils::file::find_next(handle);
		}
		//Continue with internal method
	}
	if (member_ptr == &write_project_file) {
		save_project_file(write_project_file);
	}
	if(!transformation_lock && pc.get_nr_components()>0)
	for (int i = 0; i < int(pc.get_nr_components()); i++) {
		if (member_ptr == &pc.component_rotation(i) || member_ptr == &pc.component_translation(i))
		{
			user_modified.at(i) = true;
			break;
		}	
	}

	// always call base class implementation to post_redraw and update_member in gui
	point_cloud_interactable::on_set(member_ptr);
}

void vr_point_cloud_aligner::drag_scan(cgv::math::fmat<float,3,3> rotation, Dir translation)
{
	mat4 controller_pose_mat;
	controller_pose_mat.set_col(0, vec4(rotation.col(0), 0));
	controller_pose_mat.set_col(1, vec4(rotation.col(1), 0));
	controller_pose_mat.set_col(2, vec4(rotation.col(2), 0));
	controller_pose_mat.set_col(3, vec4(translation, 1));

	std::vector<int> picked_ids = picked_group->get_component_IDs();
	for (int x = 0; x < picked_ids.size(); ++x)
	{
		mat4 temp = controller_pose_mat*local_pose_mat_stack[x];
		mat3 new_rot;
		for (int i = 0; i < 3; ++i)
			new_rot.set_col(i, reinterpret_cast<const vec3&>(temp.col(i)));
		vec3 new_trans = reinterpret_cast<const vec3&>(temp.col(3));
		pc.component_rotation(picked_ids[x]).set(transpose(new_rot));
		pc.component_translation(picked_ids[x]) = new_trans;
	}
}

/// register on device change events
void vr_point_cloud_aligner::on_device_change(void* kit_handle, bool attach)
{
	if (attach) {
		if (last_kit_handle == 0) {
			vr::vr_kit* kit_ptr = vr::get_vr_kit(kit_handle);
			if (kit_ptr) {
				last_kit_handle = kit_handle;
				left_deadzone_and_precision = kit_ptr->get_controller_throttles_and_sticks_deadzone_and_precision(0);
				cgv::gui::ref_vr_server().provide_controller_throttles_and_sticks_deadzone_and_precision(kit_handle, 0, &left_deadzone_and_precision);
				post_recreate_gui();
			}
		}
	}
	else {
		if (kit_handle == last_kit_handle) {
			last_kit_handle = 0;
			post_recreate_gui();
		}
	}
}

bool vr_point_cloud_aligner::init(cgv::render::context& ctx)
{
	bool success = point_cloud_interactable::init(ctx);
	printf("called init method\n");
	cgv::gui::connect_vr_server(true);

	auto view_ptr = find_view_as_node();
	if (view_ptr) {
		view_ptr->set_eye_keep_view_angle(dvec3(0, 4, -4));
		// if the view points to a vr_view_interactor
		vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
		if (vr_view_ptr) {
			// configure vr event processing
			vr_view_ptr->set_event_type_flags(
				cgv::gui::VREventTypeFlags(
					cgv::gui::VRE_KEY +
					cgv::gui::VRE_THROTTLE +
					cgv::gui::VRE_STICK +
					cgv::gui::VRE_STICK_KEY +
					cgv::gui::VRE_POSE
				));
			vr_view_ptr->enable_vr_event_debugging(false);
			// configure vr rendering
			vr_view_ptr->draw_action_zone(false);
			vr_view_ptr->draw_vr_kits(true);
			vr_view_ptr->enable_blit_vr_views(true);
			vr_view_ptr->set_blit_vr_view_width(200);

		}
	}
	// ensure that the box renderer is initialized
	return success && b_renderer.init(ctx);
}

#include <cgv/base/register.h>

/// register a newly created cube with the name "cube1" as constructor argument
extern cgv::base::object_registration<vr_point_cloud_aligner> point_cloud_viewer_reg("");

#ifdef CGV_FORCE_STATIC
extern cgv::base::registration_order_definition dro("stereo_view_interactor;vr_point_cloud_aligner");
#endif


