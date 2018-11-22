#include "vr_point_cloud_aligner.h"
#include <cgv/base/register.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/mouse_event.h>
#include "../sparseicp/ICP.h"
#include <cgv/gui/file_dialog.h>
#include <libs/cgv_gl/gl/gl.h>


#define FILE_OPEN_TITLE "Open Transformation Project"
#define FILE_OPEN_FILTER "Transformation Projects (tpj):*.tpj;|all Files:*.*"

#define FILE_SAVE_TITLE "Tramsformation Projects"
#define FILE_SAVE_FILTER "Transformation Projects (tpj):*.tpj;|all Files:*.*"

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

// small helper function to get a valid color. Not working as expected atm
// 0 = black
// 1 = red
// 2 = orange
// 3 = yellow
// 4 = green
// 5 = blue-green
// 6 = blue
// 7 = violett

void vr_point_cloud_aligner::position_scans() 
{
	int i = 0;
}

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
		float_to_color_component(color_values[3*idx]), 
		float_to_color_component(color_values[3*idx+1]), 
		float_to_color_component(color_values[3*idx+2])
	);
}

#include <cgv/base/find_action.h>

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

	sample_member_rows = 5;
	sample_member_cols = 5;
	generate_room_boxes();
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
	// store current transformation matrix from world to device coordinates
	DPV = ctx.get_DPV();

	point_cloud_interactable::draw(ctx);
	
	// draw array of boxes with renderer from gl_point_cloud_drawable and my box_render_style
	b_renderer.set_render_style(box_render_style);

	// draw box around picked point
	/*
	if (have_picked_point) {
		b_renderer.set_position_is_center(true);
		b_renderer.set_position_array(ctx, &picked_point, 1);
		Pnt extent(picked_box_extent, picked_box_extent, picked_box_extent);
		b_renderer.set_extent_array(ctx, &extent, 1);
		b_renderer.set_color_array(ctx, &picked_box_color, 1);
		b_renderer.validate_and_enable(ctx);
		glDrawArrays(GL_POINTS, 0, 1);
		b_renderer.disable(ctx);
	}*/

	//check for box intersections
	if(have_picked_point && pick_active)
	//for (int i = 0; i < pc.get_nr_components(); i++)
	{
		//Problem: There is no way to access the boxes of the components!
		
		if (Pnt(-1000, -1000, -1000) != box_ray_intersection(last_view_point, picked_point - last_view_point, pc.box())) //pc.component_translation(i), pc.component_rotation(i)));
		printf("Picked a scan");
		//Change color of box
	}
	pick_active = false;

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


point_cloud_types::Pnt vr_point_cloud_aligner::box_ray_intersection(const Pnt& ray_start, const Dir& ray_dir, const Box& box)
{
	//Boxes are axis aligned defined by 2 coordinates. This means one can use the techniques of ECG raytracing slides s22
	//Calculate for each axis its intersection intervall. Then intersect all intervalls to get the target intervall and its parameters
	interval xintersection = calculate_intersectionintervall(ray_start.x(), box.get_min_pnt().x(), box.get_max_pnt().x(), ray_dir.x());
	interval yintersection = calculate_intersectionintervall(ray_start.y(), box.get_min_pnt().y(), box.get_max_pnt().y(), ray_dir.y());
	interval zintersection = calculate_intersectionintervall(ray_start.z(), box.get_min_pnt().z(), box.get_max_pnt().z(), ray_dir.z());
	if (xintersection.isInvalid() || yintersection.isInvalid() || zintersection.isInvalid())
	{
		return Pnt(-1000,-1000,-1000);
	}
	interval solution = xintersection.intersectIntervals(yintersection).intersectIntervals(zintersection);
	if (solution.isInvalid() || solution.isNullInterval()) {
		return Pnt(-1000, -1000, -1000);
	}
	//I am not certain about this solution, somehow there should be a number € Tq that is the solution. Anyway the minimum should not be that far away from the solution
	Pnt result = ray_start + ray_dir * solution.get_min();
	return result;
}

point_cloud_types::Pnt vr_point_cloud_aligner::box_ray_intersection(const Pnt& ray_start, const Dir& ray_dir, const Box& box, const Dir& box_translation, const Qat& box_rotation)
{
	//Implementation plan: apply inverse transformations to rays to transform them to the local coordinate system of the box. Calculate intersection there, then transfrom back to global
	Pnt local_ray_start = transform_to_local(ray_start,box_translation,box_rotation);
	Dir local_ray_direction = transform_to_local(ray_dir, box_translation, box_rotation);
	Pnt local_result = box_ray_intersection(local_ray_start, local_ray_direction, box);
	//There is no intersection
	if(local_result == Pnt(-1000, -1000, -1000))
		return Pnt(-1000, -1000, -1000);
	Pnt local_intersection_point = local_ray_start + local_result * local_ray_direction;
	Pnt global_intersection_point = box_rotation.apply( local_intersection_point + box_translation);
	Crd global_result = ((global_intersection_point - ray_start).x() / ray_dir.x());
	return global_result;
}

point_cloud_types::Pnt vr_point_cloud_aligner::transform_to_local(const Pnt& in, const Dir& local_translation, const Qat& local_rotation) 
{
	Pnt result = Pnt((in - local_translation));
	local_rotation.inverse_rotate(result);
	return result;
}

interval vr_point_cloud_aligner::calculate_intersectionintervall(double rayStart, double maxBoxCoord1,double maxBoxCoord2,double raydir)
{
	//X0 = px + t* x vx
	//First we need the parameter vx especially its sign and if its bigger 0
	// vx = abl(x)/abl(t) = vx
	double px = rayStart;
	double x0 = maxBoxCoord1;
	double x1 = maxBoxCoord2;
	//delta x / delta t is the same as ray_dir (1 step of it)
	double vx = raydir;
	double t0 = 0, t1 = 0;
	if (vx == 0)
	{
		// Does not interesect the given axis of the box -> no intersection
		interval a(0,0);
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
	GLdouble objx, objy, objz,objx2,objy2,objz2;
	//Use graphics libary to unproject
	//gluUnProject(x, y, 0, , , , &objx, &objy, &objz);
	//gluUnProject(x, y, 1, , , , &objx2, &objy2, &objz2);


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
			case 'Y':
				reset_componets_transformations();
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
			}
		}
	}

	// pass on remaining events to base class
	return point_cloud_interactable::handle(e);
}
void vr_point_cloud_aligner::on_point_cloud_change_callback(PointCloudChangeEvent pcc_event)
{
	point_cloud_interactable::on_point_cloud_change_callback(pcc_event);
	/*if (pcc_event == PCC_NEW_POINT_CLOUD) {
		int i = pc.get_nr_components();
		
		if (pc.component_translation(i) == NULL)// find a way to check if this pointcloud was already aligned somehow by the user. If not use default translation to line
		{
			// a new Pointcloud has been added and should be moved to the line. Therefore all pointclouds should be put there, but only if they are not already moved
			int nr = pc.get_nr_components();
			if (nr > 0)
			{
				for (int i = 0; i < nr; i++)
				{
					float x = 5.1 / nr;
					pc.component_translation(i).set(Crd(0.2), Crd(i * x), Crd(3.8));
				}
			}
		}
	}*/
	reset_componets_transformations();
	// do more handling of point clout change events here
}

void vr_point_cloud_aligner::reset_componets_transformations() {
	int nr = pc.get_nr_components();
	if (nr > 0)
	{
		for (int i = 0; i < nr; i++)
		{
			float x = 5.1 / nr;
			pc.component_translation(i).set(Crd(0.2), Crd(i * x), Crd(3.8));				
			pc.component_rotation(i).set(cgv::math::quaternion<float>::AxisEnum::X_AXIS,0);
		}
	}
	post_redraw();
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
	if (begin_tree_node("picking", have_picked_point, true, "level=3")) {
		align("\a"); // increase identation
		add_gui("position", picked_point, "options='active=false'");
		add_member_control(this, "box extent", picked_box_extent, "value_slider", "min=0.0001;max=10;ticks=true;log=true");
		add_member_control(this, "box color", picked_box_color);
		align("\b"); // decrease identation
		end_tree_node(sample_boxes);
	}

	//GUI for saving Transformations from scans
	if (begin_tree_node("Save transformations", true, true, "level=3")) {
		align("\a");
		add_gui("file_name", project_file, "file_name",
			"w=130;"
			"open=true;open_title='" FILE_OPEN_TITLE "';open_filter='" FILE_OPEN_FILTER "';"
			"save=true;save_title='" FILE_SAVE_TITLE "';save_filter='" FILE_SAVE_FILTER "'"
		);
		align("\b");
		end_tree_node(true);
	}
}

#include <cgv/base/register.h>

/// register a newly created cube with the name "cube1" as constructor argument
extern cgv::base::object_registration<vr_point_cloud_aligner> point_cloud_viewer_reg("");

#ifdef CGV_FORCE_STATIC
extern cgv::base::registration_order_definition dro("stereo_view_interactor;vr_point_cloud_aligner");
#endif


