#include "vr_point_cloud_aligner.h"
#include <cgv/base/register.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/mouse_event.h>
#include "../sparseicp/ICP.h"
#include "kabsch.h"
#include <cgv/base/find_action.h>
#include <fstream>
#include <cgv/utils/file.h>
#include <cgv/gui/file_dialog.h>
#include <libs/cgv_gl/gl/gl.h>


#define FILE_OPEN_TITLE "Open Transformation Project"
#define FILE_OPEN_FILTER "Transformation Projects (tpj):*.tpj;|all Files:*.*"

#define FILE_SAVE_TITLE "Tramsformation Projects"
#define FILE_SAVE_FILTER "Transformation Projects (tpj):*.tpj;|all Files:*.*"


///Module 1 Startup methods

///Constructor
vr_point_cloud_aligner::vr_point_cloud_aligner()
{
	set_name("VR Point Cloud Aligner");

	picked_box_extent = 0.1f;
	picked_box_color = Clr(float_to_color_component(1.0f), 0, 0);
	view_ptr = 0;
	have_picked_point = false;
	picked_point = Pnt(0, 0, 0);
	last_view_point = Pnt(0, 0, 0);
	have_view_ray = false;
	icp_executing = false;

	generate_room_boxes();
	box_render_style.map_color_to_material = cgv::render::MS_FRONT_AND_BACK;
	box_render_style.culling_mode = cgv::render::CM_BACKFACE;
	box_render_style.illumination_mode = cgv::render::IM_TWO_SIDED;

	pickedComponent = -1;
	previous_picked_component = -1;
	oldColor = RGBA(1, 1, 1, 1);
	defaultFacing = cgv::math::quaternion<float>(1, 0, 0, 0);
	projectLoading_in_process = false;
	connect(cgv::gui::get_animation_trigger().shoot, this, &vr_point_cloud_aligner::timer_event);
}

void vr_point_cloud_aligner::timer_event(double t, double dt)
{
	if (icp_executing) {
		start_ICP();
	}
}

///Setup for the working room
void vr_point_cloud_aligner::generate_room_boxes()
{
	room_boxes.clear();
	room_colors.clear();
	Box room;
	float x_room_min = 0, y_room_min = 0, z_room_min = -.2;
	float lineCount = 20;
	room.add_point(Pnt(Crd(x_room_min), Crd(y_room_min), Crd(z_room_min)));
	room.add_point(Pnt(Crd(5), Crd(5), Crd(0)));
	room_boxes.push_back(room);
	room_colors.push_back(generate_a_valid_color(3));

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

	Box line;
	line.add_point(Pnt(Crd(x_room_min), Crd(0), Crd(4)));
	line.add_point(Pnt(Crd(x_room_min + .2), Crd(5), Crd(4.1)));
	room_boxes.push_back(line);
	room_colors.push_back(generate_a_valid_color(1));
}

std::string vr_point_cloud_aligner::get_type_name() const
{
	return "vr_point_cloud_aligner";
}

bool vr_point_cloud_aligner::self_reflect(cgv::reflect::reflection_handler& rh)
{
	return
		//rh.reflect_member("sample_member_rows", sample_member_rows) &&
		//rh.reflect_member("sample_member_cols", sample_member_cols) &&
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
}

///Module 2 Helper functions

void vr_point_cloud_aligner::position_scans()
{
	// a new component has been added and should be moved to the line. Therefore all pointclouds that are already there should be moved, but only if they are not already usermodified
	int nr = 0;
	for (int a = 0; a < user_modified.size(); a++) {
		if (user_modified.at(a)) {
			nr++;
		}
	}
	if (nr > 0)
	{
		for (int i = 0; i < pc.get_nr_components(); i++)
		{
			float x = 5.1 / nr;
			if (!user_modified.at(i))
			{
				pc.component_translation(i).set(Crd(0.2), Crd(i * x), Crd(3.8));
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

void vr_point_cloud_aligner::display_unite_question()
{
	printf("Scans vereinigen?");
	bool unite = false;
	//IMPLEMENT GUI RESPONSES

	int unitingPos = -1;
	int otherPos = -1;
	if (unite)
	{
		for (int a = 0; a < sets.size(); ++a) 
		{
			if (sets.at(a).find_component_ID(previous_picked_component)) 
				unitingPos = a;
			if (sets.at(a).find_component_ID(pickedComponent))
				otherPos = a;

			///set the colors equal
			constructed_set temp = sets.at(otherPos);
			std::vector<int> tempIDs = temp.get_component_IDs();
			Clr tosetTo = pc.component_color(sets.at(unitingPos).get_component_IDs().at(0));
			for (int b = 0; b < tempIDs.size(); ++b)
			{
				pc.component_color(tempIDs.at(b)) = tosetTo;
			}
			//now unite and delete
			sets.at(unitingPos).unite(temp);
			sets.erase(sets.begin() + otherPos);
		}
	}
	/// if not reset to old state
	else
	{
		pc.component_translation(pickedComponent) = latest_overwritten_translation;
		pc.component_rotation(pickedComponent) = latest_overwritten_rotation;
		post_redraw();
	}

}


void vr_point_cloud_aligner::save_back_origin_state()
{
	if (pickedComponent < 0 || previous_picked_component < 0)
			return;
	latest_overwritten_translation = pc.component_translation(pickedComponent);
	latest_overwritten_rotation = pc.component_rotation(pickedComponent);
}

void vr_point_cloud_aligner::start_ICP()
{
	if (pickedComponent < 0 || previous_picked_component < 0) {
		return;
	}
	Eigen::Matrix<double, 3, Eigen::Dynamic> vertices_source;
	Eigen::Matrix<double, 3, Eigen::Dynamic> vertices_source_copy;
	component_info a  = pc.component_point_range(pickedComponent);
	int p = 0;
	//vertices , vertices_target; -> Downsampling integration here!
	vertices_source.resize(Eigen::NoChange, a.nr_points);
	vertices_source_copy.resize(Eigen::NoChange, a.nr_points);

	for (int current_adress = a.index_of_first_point; current_adress < a.index_of_first_point + a.nr_points; current_adress++) 
	{
		Pnt tr_pnt= pc.transformed_pnt(current_adress);
		vertices_source(0, p) = tr_pnt.x();
		vertices_source(1, p) = tr_pnt.y();
		vertices_source(2, p) = tr_pnt.z();

		Pnt _pnt = pc.pnt(current_adress);
		vertices_source_copy(0, p) = _pnt.x();
		vertices_source_copy(1, p) = _pnt.y();
		vertices_source_copy(2, p) = _pnt.z();
		++p;
	}

	Eigen::Matrix<double, 3, Eigen::Dynamic> vertices_target;
	a = pc.component_point_range(previous_picked_component);
	p = 0;
	vertices_target.resize(Eigen::NoChange, a.nr_points);
	for (int current_adress = a.index_of_first_point; current_adress < a.index_of_first_point + a.nr_points; current_adress++)
	{
		Pnt tr_pnt = pc.transformed_pnt(current_adress);
		vertices_target(0, p) = tr_pnt.x();
		vertices_target(1, p) = tr_pnt.y();
		vertices_target(2, p) = tr_pnt.z();
		++p;
	}
	ICP::Parameters par;
	par.p = .5;
	par.max_icp = 10;
	ICP::point_to_point(vertices_source, vertices_target, par);
	printf("ICP finished\n");
	///Now the vertices_source is recalculated and overwritten. we now need to calculate back the actual rotation and translation.
	///This should be now possible through using some points in the source cloud that have been transformed. Because of correspondencies there is now a closed solution form
	///Using this method the first 4 points should be enough to solve this. The test case now uses all points.
	Eigen::Affine3d A = kabsch::Find3DAffineTransform(vertices_source_copy, vertices_source);
	/*for (int m = 0; m < 5; ++m) 
	{
		printf("First 5 Points of ICP:\n");
		printf("ICPresult %d: %f, %f, %f; Original: %f, %f, %f; Target: %f, %f, %f \n", m, vertices_source(0, m), vertices_source(1, m), vertices_source(2, m), vertices_source_copy(0, m), vertices_source_copy(1, m), vertices_source_copy(2, m), vertices_target(0, m), vertices_target(1, m), vertices_target(2, m));
	}*/
	Eigen::Vector3d translation_vec = A.translation();
	cgv::math::mat<float> casted_mat;
	casted_mat.resize(3, 3);
	Eigen::Matrix<double, 3, 3> R = A.rotation();
	
	float m00 = R(0, 0),m01=R(0,1),m02 =R(0,2), m10=R(1,0), m11 = R(1, 1), m12 = R(1,2), m20 = R(2,0), m21 = R(2,1), m22 = R(2, 2);
	float qx, qy, qz, qw;
	float tr = m00 + m11 + m22;

		if (tr > 0) {
			float S = sqrt(tr + 1.0) * 2; // S=4*qw 
			qw = 0.25 * S;
			qx = (m21 - m12) / S;
			qy = (m02 - m20) / S;
			qz = (m10 - m01) / S;
		}
		else if ((m00 > m11)&(m00 > m22)) {
			float S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
			qw = (m21 - m12) / S;
			qx = 0.25 * S;
			qy = (m01 + m10) / S;
			qz = (m02 + m20) / S;
		}
		else if (m11 > m22) {
			float S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
			qw = (m02 - m20) / S;
			qx = (m01 + m10) / S;
			qy = 0.25 * S;
			qz = (m12 + m21) / S;
		}
		else {
			float S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
			qw = (m10 - m01) / S;
			qx = (m02 + m20) / S;
			qy = (m12 + m21) / S;
			qz = 0.25 * S;
		}
	pc.component_translation(pickedComponent).set(translation_vec.x(),translation_vec.y(),translation_vec.z());
	pc.component_rotation(pickedComponent).set(qw,qx,qy,qz);
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



void vr_point_cloud_aligner::draw(cgv::render::context& ctx)
{
	// store current transformation matrix from world to device coordinates
	DPV = ctx.get_DPV();

	point_cloud_interactable::draw(ctx);

	// draw array of boxes with renderer from gl_point_cloud_drawable and my box_render_style
	b_renderer.set_render_style(box_render_style);

	//check for box intersections
	if (have_picked_point && pick_active)
	{
		std::vector<Crd> intersectedPoints;
		std::vector<int> component_NR;
		for (int i = 0; i < pc.get_nr_components(); i++)
		{
			Crd temp = box_ray_intersection(last_view_point, picked_point, pc.box(i), pc.component_translation(i), pc.component_rotation(i));
			if (temp > 0)
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
			float z_factor_min = 1000.0f;
			int min_component = -1;
			for (int i = 0; i < intersectedPoints.size(); ++i)
			{
				if (z_factor_min > intersectedPoints.at(i)) 
				{
					min_component = i;
					z_factor_min = intersectedPoints.at(i);
				}
			}
			int a = component_NR.at(min_component);

			if (a == pickedComponent && pickedComponent != -1) 
			{
				printf("deselected scan\n");
				pc.component_color(pickedComponent) = oldColor;
				previous_picked_component = -1;
				pickedComponent = -1;
			}
			else if (a >= 0)
			{
				printf("Picked a scan\n");
				if(pickedComponent > 0)
					pc.component_color(pickedComponent) = oldColor;
				previous_picked_component = pickedComponent;
				//Change color of box
				oldColor = pc.component_color(a);
				pc.component_color(a) = RGBA(1, 0, 0, 1);
				pickedComponent = a;
			}
			post_redraw();
		}

		pick_active = false;
	}
	if (have_view_ray) {
		glLineWidth(5);
		glColor3f(1, 1, 0);
		glBegin(GL_LINES);
		glVertex3fv(&last_view_point[0]);
		glVertex3fv(&last_target_point[0]);
		glEnd();
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
	printf("Local xIntersection %f %f\n", xintersection.get_min(), xintersection.get_max());
	interval yintersection = calculate_intersectionintervall(ray_start.y(), box.get_min_pnt().y(), box.get_max_pnt().y(), ray_dir.y());
	printf("Local yIntersection %f %f\n", yintersection.get_min(), yintersection.get_max());
	interval zintersection = calculate_intersectionintervall(ray_start.z(), box.get_min_pnt().z(), box.get_max_pnt().z(), ray_dir.z());
	printf("Local zIntersection %f %f\n", zintersection.get_min(), zintersection.get_max());
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
		return -1;
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
			pc.component_color(0) = RGBA(1,0,0,1);
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
				pc_to_append.component_color(0) = cgv::media::color<float, cgv::media::HLS, cgv::media::OPACITY>(float(pc.get_nr_components()) / float(10), 0.5f, 1.0f, 1.0f);
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
	for (int l = 0; l < alignment_IDs.size(); ++l)
	{
		std::vector<int> a;
		a.push_back(l);
		copySet.push_back(constructed_set(a, alignment_IDs.at(l)));
	}
	std::vector<int> usedIDs;
	for(int l = 0; l < alignment_IDs.size(); ++l)
	{
		int currentminimizer = alignment_IDs.at(l);
		bool skip = false;
		for (int k = 0; k < usedIDs.size(); ++k)
		{
			if (currentminimizer == usedIDs.at(k))
				skip = true;
		}
		if (skip)
			continue;
		std::vector<int> b;
		constructed_set currentSet(b, currentminimizer);
		for (int j = 0; j < copySet.size(); ++j)
		{
			if (currentminimizer == copySet.at(j).get_ID())
				currentSet.unite(copySet.at(j));
		}
		sets.push_back(currentSet);
		usedIDs.push_back(currentminimizer);
	}
	projectLoading_in_process = false;
}

void vr_point_cloud_aligner::save_project_file(std::string projectFile)
{
	std::ofstream outFile; 

	outFile.open(projectFile);
	if (0 == pc.get_nr_components()) {
		printf("Keine Komponenten zum speichern!");
	}
	for (int i = 0; i < pc.get_nr_components(); i++) {
		int alignmentID = -1;
		for (int c = 0; c < sets.size(); ++c) 
		{
			if (sets.at(c).find_component_ID(i))
				alignmentID = sets.at(c).get_ID();
		}
		outFile << ' ' << file_paths.at(i) << ' ' << pc.component_translation(i) << ' ' << pc.component_rotation(i).re() << ' ' << pc.component_rotation(i).im() << ' ' << user_modified.at(i) << sets.at(alignmentID).get_ID() <<'\n';
	}
	outFile.close();
}

point_cloud_types::Pnt vr_point_cloud_aligner::transform_to_local(const Pnt& in, const Pnt& local_translation, const Qat& local_rotation)
{
	Pnt result = in - local_translation;
	local_rotation.inverse_rotate(result);
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
	// check if a matrix for unprojection is available
	if (DPV.size() == 0)
		return;
	
	double z = ctx.get_z_D(x, y);
	// check for background
	if (z > 0.99999999) {
		if (have_picked_point)
			post_redraw();
		have_picked_point = false;
	}
	else {
		have_picked_point = true;
		picked_point = Pnt(&ctx.get_point_W(x, y, DPV)[0]);
		post_redraw();
	}
}

bool vr_point_cloud_aligner::handle(cgv::gui::event& e)
{
	if (e.get_kind() == cgv::gui::EID_KEY) {
		cgv::gui::key_event& ke = static_cast<cgv::gui::key_event&>(e);
		if (ke.get_action() == cgv::gui::KA_PRESS || ke.get_action() == cgv::gui::KA_REPEAT) {
			switch (ke.get_key()) {
			case 'Y':
				reset_componets_transformations();
				return true;
			case 'I':
				if(!icp_executing)
					save_back_origin_state();
				icp_executing = true;
				start_ICP();
				return true;
			}
		}
		else if (ke.get_action() == cgv::gui::KA_RELEASE) 
		{
			switch (ke.get_key()) {
			case 'I':
				icp_executing = false;
				display_unite_question();
				return true;
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
						pick_active=true;
					}
					return true;
				}
				break;
			case cgv::gui::MA_WHEEL:
				if (me.get_button_state() == cgv::gui::MB_RIGHT_BUTTON)
				{
					if (pickedComponent > -1) 
					{
						Dir vec;// = last_target_point - last_view_point;
						if (me.get_dy() > 0)
							vec = (last_target_point - last_view_point) * -0.1;
						else
							vec = (last_target_point - last_view_point) * 0.1;

						vec += pc.component_translation(pickedComponent);
						pc.component_translation(pickedComponent).set(vec.x(),vec.y(),vec.z());
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
		sets.push_back(constructed_set(std::vector<int>(pc.get_nr_components() - 1), sets.size()));
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

		}
	}
	// do more handling of point clout change events here
	post_redraw();
}

void vr_point_cloud_aligner::reset_componets_transformations() {
	int nr = pc.get_nr_components();
	if (nr > 0)
	{
		for (int i = 0; i < nr; i++)
		{
			float x = 5.1 / nr;
			pc.component_translation(i).set(Crd(0.2), Crd(i * x), Crd(3.8));				
			pc.component_rotation(i) = defaultFacing;
		}
	}
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
	for (int i = 0; i < pc.get_nr_components(); i++) {
		if (member_ptr == &pc.component_rotation(i) || member_ptr == &pc.component_translation(i))
		{
			user_modified.at(i) = true;
			break;
		}	
	}

	// always call base class implementation to post_redraw and update_member in gui
	point_cloud_interactable::on_set(member_ptr);
}


#include <cgv/base/register.h>

/// register a newly created cube with the name "cube1" as constructor argument
extern cgv::base::object_registration<vr_point_cloud_aligner> point_cloud_viewer_reg("");

#ifdef CGV_FORCE_STATIC
extern cgv::base::registration_order_definition dro("stereo_view_interactor;vr_point_cloud_aligner");
#endif


