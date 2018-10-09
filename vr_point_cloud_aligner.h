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
	std::vector<Box> room_boxes;
	std::vector<Clr> sample_box_colors;
	std::vector<Clr> room_colors;
	cgv::render::surface_render_style box_render_style;
	void generate_sample_boxes();

	//Generates a room Box, Table and pointcloud holders
	void generate_room_boxes();
	//Helper function
	Clr generate_a_valid_color(int color);

protected:
	/**@name access to point cloud; always use these functions to access the point cloud data structure; if you need more access add more functions here*/
	//@{
	/// read access to the i-th point in the point cloud 
	const Pnt& pnt(Idx i) const { return pc.pnt(i); }
	/// read access to the i-th point normal in the point cloud 
	const Nml& nml(Idx i) const { return pc.nml(i); }
	/// read access to the i-th point in the point cloud 
	Pnt transformed_pnt(Idx i) const { return pc.transformed_pnt(i); }
	/// read access to component index of point 
	unsigned pnt_comp_idx(Idx i) const { return pc.component_index(i); }
	/// read access to component rotation
	const Qat& comp_rot(Idx i) const { return pc.component_rotation(i); }
	/// write access to component rotation
	Qat& comp_rot(Idx i) { return pc.component_rotation(i); }
	/// read access to component translation
	const Dir& comp_trans(Idx i) const { return pc.component_translation(i); }
	/// write access to component tranlation
	Dir& comp_trans(Idx i) { return pc.component_translation(i); }
	/// read access to component color
	const RGBA& comp_color(Idx i) const { return pc.component_color(i); }
	/// write access to component color
	RGBA& comp_color(Idx i) { return pc.component_color(i); }
	//@}	

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