#include "vr_point_cloud_aligner.h"
#include <cgv/base/register.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/mouse_event.h>
#include <libs/cgv_gl/gl/gl.h>

void vr_point_cloud_aligner::generate_sample_boxes()
{
	sample_boxes.clear();
	sample_box_colors.clear();
	for (size_t i = 0; i < sample_member_rows; ++i) {
		for (size_t j = 0; j < sample_member_cols; ++j) {
			Box B;
			B.add_point(Pnt(Crd(i), Crd(j), 0));
			B.add_point(Pnt(i + 0.5f, j + 0.5f, 1));
			sample_boxes.push_back(B);
			sample_box_colors.push_back(
				Clr( // to support float and uint8 color components, use to float_to_color_component function to convert from a double value
					float_to_color_component(double(i) / (sample_member_rows - 1)),
					float_to_color_component(double(j) / (sample_member_cols - 1)),
					float_to_color_component(0.5)
				)
			);
		}
	}
}

void vr_point_cloud_aligner::generate_room_boxes()
{
	room_boxes.clear();
	room_colors.clear();
	Box room;
	float x_room_min= 0, y_room_min= 0, z_room_min= -.2;
	float lineCount = 20; 
	room.add_point(Pnt(Crd(x_room_min), Crd(y_room_min), Crd(z_room_min)));
	room.add_point(Pnt(Crd(5), Crd(5), Crd(0)));
	room_boxes.push_back(room);
	room_colors.push_back(generate_a_valid_color(0));

	Box wall1;
	wall1.add_point(Pnt(Crd(x_room_min - 0.2), Crd(y_room_min), Crd(z_room_min)));
	wall1.add_point(Pnt(Crd(x_room_min), Crd(5), Crd(5)));
	room_boxes.push_back(wall1);
	room_colors.push_back(generate_a_valid_color(3));

	//The table in the middle of the room
	Box tableTop;
	Box tableLeg1;
	Box tableLeg2;
	Box tableLeg3;
	Box tableLeg4;

	float tableTopCorner1 = 1.5, tableTopCorner2 = 3;
	tableTop.add_point(Pnt(Crd(tableTopCorner1), Crd(tableTopCorner1), Crd(1.3)));
	tableTop.add_point(Pnt(Crd(tableTopCorner2), Crd(tableTopCorner2), Crd(1.5)));
	room_boxes.push_back(tableTop);
	room_colors.push_back(generate_a_valid_color(1));
	
	//Maybe instanciate the leg with different coordinates?
	tableLeg1.add_point(Pnt(Crd(tableTopCorner1), Crd(tableTopCorner1), Crd(1.3)));
	tableLeg1.add_point(Pnt(Crd(tableTopCorner1 + .2), Crd(tableTopCorner1 + .2), Crd(0)));
	room_boxes.push_back(tableLeg1);
	room_colors.push_back(generate_a_valid_color(2));

	tableLeg2.add_point(Pnt(Crd(tableTopCorner1 + 1.3), Crd(tableTopCorner1), Crd(1.3)));
	tableLeg2.add_point(Pnt(Crd(tableTopCorner1 + 1.5), Crd(tableTopCorner1 + 0.2), Crd(0)));
	room_boxes.push_back(tableLeg2);
	room_colors.push_back(generate_a_valid_color(2));

	tableLeg3.add_point(Pnt(Crd(tableTopCorner1), Crd(tableTopCorner1 + 1.3), Crd(1.3)));
	tableLeg3.add_point(Pnt(Crd(tableTopCorner1 + .2), Crd(tableTopCorner1 + 1.5), Crd(0)));
	room_boxes.push_back(tableLeg3);
	room_colors.push_back(generate_a_valid_color(2));

	tableLeg4.add_point(Pnt(Crd(tableTopCorner2), Crd(tableTopCorner2), Crd(1.3)));
	tableLeg4.add_point(Pnt(Crd(tableTopCorner2 - 0.2), Crd(tableTopCorner2 - .2), Crd(0)));
	room_boxes.push_back(tableLeg4);
	room_colors.push_back(generate_a_valid_color(2));

	//Wäscheleine an der Wand  hälfte 1
	for (size_t i = 1; i < lineCount; ++i) {
			Box B;
			B.add_point(Pnt(Crd(x_room_min), Crd((2.5/lineCount * (i - 1))+2.5), Crd((i / lineCount) +4)));
			B.add_point(Pnt(Crd(x_room_min + 0.2),Crd((2.5/lineCount) * i + 2.5), Crd((i / lineCount) + 4.1)));
			room_boxes.push_back(B);
			room_colors.push_back(generate_a_valid_color(1));
	}	
	//Wäscheleine an der Wand  hälfte 2
	for (size_t i = 1; i < lineCount; ++i) {
		Box B;
		B.add_point(Pnt(Crd(x_room_min), Crd((2.5 / lineCount * (i - 1))), Crd(5 - (i / lineCount))));
		B.add_point(Pnt(Crd(x_room_min + 0.2), Crd((2.5 / lineCount) * i), Crd(5.1 - (i / lineCount))));
		room_boxes.push_back(B);
		room_colors.push_back(generate_a_valid_color(1));
	}
}

// small helper function to get a valid color. Not working as expected atm
// 0 = black
// 1 = red
// 2 = orange
// 3 = yellow
// 4 = green
// 5 = blue-green
// 6 = blue
// 7 = violett

point_cloud_types::Clr vr_point_cloud_aligner::generate_a_valid_color(int color)
{
	int i=0, j= 0;
	int k = 0.5;
	switch (color) 
	{
	case 1:
		i = 1;
		break;
	case 2:
		i = 1;
		j = 0.7;
		k = 0.5;
		break;
	case 3:
		i = j = k = 0.5;
		break;
	case 4:
		break;
	case 5:
		break;
	case 6:
		break;
	case 7:
		break;
	default:
		break;
	}
	return Clr( // to support float and uint8 color components, use to float_to_color_component function to convert from a double value
		float_to_color_component(double(i)),float_to_color_component(double(j)),float_to_color_component(k));
}

vr_point_cloud_aligner::vr_point_cloud_aligner()
{
	set_name("VR Point Cloud Aligner");

	sample_member_rows = 5;
	sample_member_cols = 5;
	generate_room_boxes();
	//generate_sample_boxes();
	box_render_style.map_color_to_material = cgv::render::MS_FRONT_AND_BACK;
	box_render_style.culling_mode = cgv::render::CM_BACKFACE;
	box_render_style.illumination_mode = cgv::render::IM_TWO_SIDED;
}


std::string vr_point_cloud_aligner::get_type_name() const
{
	return "vr_point_cloud_aligner";
}

bool vr_point_cloud_aligner::self_reflect(cgv::reflect::reflection_handler& rh)
{
	return 
		rh.reflect_member("sample_member_rows", sample_member_rows) &&
		rh.reflect_member("sample_member_cols", sample_member_cols) &&
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
	os << "   rows x cols: " << sample_member_rows << " x " << sample_member_cols << std::endl;
}


bool vr_point_cloud_aligner::init(cgv::render::context& ctx)
{
	bool success = point_cloud_interactable::init(ctx);

	// do your one time init (e.g. shader loading, texture creation, ...) here and update the success member


	return success;

}
void vr_point_cloud_aligner::init_frame(cgv::render::context& ctx)
{
	point_cloud_interactable::init_frame(ctx);

	// do your per frame initialization here
}
void vr_point_cloud_aligner::draw(cgv::render::context& ctx)
{
	point_cloud_interactable::draw(ctx);
	
	// draw array of boxes with renderer from gl_point_cloud_drawable and my box_render_style
	b_renderer.set_render_style(box_render_style);

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


point_cloud_types::Crd vr_point_cloud_aligner::box_ray_intersection(const Pnt& ray_start, const Dir& ray_dir, const Box& box)
{
	//To Implement
	return Crd(0);
}

point_cloud_types::Crd vr_point_cloud_aligner::box_ray_intersection(const Pnt& ray_start, const Dir& ray_dir, const Box& box, const Dir& box_translationm, const Qat& box_rotation)
{
	//To Implement
	return Crd(0);
}

bool vr_point_cloud_aligner::handle(cgv::gui::event& e)
{
	if (e.get_kind() == cgv::gui::EID_KEY) {
		cgv::gui::key_event& ke = static_cast<cgv::gui::key_event&>(e);
		if (ke.get_action() == cgv::gui::KA_PRESS || ke.get_action() == cgv::gui::KA_REPEAT) {
			switch (ke.get_key()) {
			case 'R':
				switch (e.get_modifiers()) {
				case 0:
					++sample_member_rows;
					on_set(&sample_member_rows);
					return true;
				case cgv::gui::EM_SHIFT:
					if (sample_member_rows > 0) {
						--sample_member_rows;
						on_set(&sample_member_rows);
					}
					return true;
				default:
					break;
				}
				break;
			case 'C':
				switch (e.get_modifiers()) {
				case 0:
					++sample_member_cols;
					on_set(&sample_member_cols);
					return true;
				case cgv::gui::EM_SHIFT:
					if (sample_member_cols > 0) {
						--sample_member_cols;
						on_set(&sample_member_cols);
					}
					return true;
				default:
					break;
				}
				break;
			}
		}
	}
	else if (e.get_kind() == cgv::gui::EID_MOUSE) {
		cgv::gui::mouse_event& me = static_cast<cgv::gui::mouse_event&>(e);

		// handle mouse events and return true in case event was handles
	}

	// pass on remaining events to base class
	return point_cloud_interactable::handle(e);
}
void vr_point_cloud_aligner::on_point_cloud_change_callback(PointCloudChangeEvent pcc_event)
{
	point_cloud_interactable::on_point_cloud_change_callback(pcc_event);

	// do more handling of point clout change events here
}
void vr_point_cloud_aligner::on_set(void* member_ptr)
{
	if (member_ptr == &sample_member_rows || member_ptr == &sample_member_cols) {
		generate_sample_boxes();
	}
	// always call base class implementation to post_redraw and update_member in gui
	point_cloud_interactable::on_set(member_ptr);
}
void vr_point_cloud_aligner::create_gui()
{
	// start with gui for based class
	point_cloud_interactable::create_gui();

	// add own gui
	if (begin_tree_node("sample gui", sample_boxes, true, "level=3")) {
		align("\a"); // increase identation
		add_member_control(this, "rows", sample_member_rows, "value_slider", "min=1;max=20;ticks=true");
		add_member_control(this, "cols", sample_member_cols, "value_slider", "min=1;max=20;ticks=true");
		if (begin_tree_node("box style", box_render_style, false, "level=3")) {
			align("\a"); // increase identation
			add_gui("box render style", box_render_style);
			align("\b"); // decrease identation
			end_tree_node(box_render_style);
		}
		align("\b"); // decrease identation
		end_tree_node(sample_boxes);
	}
}

#include <cgv/base/register.h>

/// register a newly created cube with the name "cube1" as constructor argument
extern cgv::base::object_registration<vr_point_cloud_aligner> point_cloud_viewer_reg("");

#ifdef CGV_FORCE_STATIC
extern cgv::base::registration_order_definition dro("stereo_view_interactor;vr_point_cloud_aligner");
#endif


