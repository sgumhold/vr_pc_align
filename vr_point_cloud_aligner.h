#pragma once

#include <libs/point_cloud/point_cloud_interactable.h>
#include <libs/cgv_gl/box_renderer.h>

#include "lib_begin.h"


/** the point cloud view adds a gui to the gl_point_cloud_drawable_base and adds
    some basic processing like normal computation as well as some debug rendering
	of the neighbor graph*/
class CGV_API vr_point_cloud_aligner : public point_cloud_interactable  
{
protected:
	/// examples of how to control a member variables (see contructor, self_reflect, stream_stats, stream_help, create_gui, handle)
	size_t sample_member_rows;
	size_t sample_member_cols;
private:
	std::vector<Box> sample_boxes;
	std::vector<Clr> sample_box_colors;
	cgv::render::surface_render_style box_render_style;
	void generate_sample_boxes();
public:
	/// construct viewer with default configuration
	vr_point_cloud_aligner();

	//**@name self reflection of class */
	//@{
	/// return type name of point_cloud_viewer
	std::string get_type_name() const;
	/// describe members
	bool self_reflect(cgv::reflect::reflection_handler& rh);
	/// stream out textual statistical information shown with F8
	void stream_stats(std::ostream&);
	/// stream out textual help information shown with F1
	void stream_help(std::ostream& os);
	//@}

	/// initialization on creation
	bool init(cgv::render::context& ctx);
	/// per frame initialization
	void init_frame(cgv::render::context& ctx);
	/// main rendering method
	void draw(cgv::render::context& ctx);
	//@}

	/**@name user interface */
	//@{
	/// process key and mouse events
	bool handle(cgv::gui::event& e);
	/// used to update all dependent variables in case of changes to the point cloud
	void on_point_cloud_change_callback(PointCloudChangeEvent pcc_event);
	/// used to update all dependent variables in case of changes to member variables
	void on_set(void* member_ptr);
	/// user interface creation
	void create_gui();
	//@}
};

#include <cgv/config/lib_end.h>