#pragma once

#include <libs/point_cloud/point_cloud_interactable.h>
#include <libs/cgv_gl/box_renderer.h>
#include <cgv/render/view.h>

#include <cgv/gui/file_dialog.h>
#include "interval.h"
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
	std::vector<bool> user_modified;
	std::vector<std::string> file_paths;
	cgv::render::surface_render_style box_render_style;
	void generate_sample_boxes();

	/// homogeneous matrix used to unproject mouse locations
	cgv::render::context::mat_type DPV;
	/// whether a picked point is available
	bool have_picked_point;
	/// position of picked point
	Pnt picked_point;
	/// point is picked right now
	bool pick_active;
	/// extent of box rendered around picked point
	float picked_box_extent;
	/// color of picked box
	Clr picked_box_color;
	/// update the picked point from the given mouse position
	void update_picked_point(cgv::render::context& ctx, int x, int y);
	/// store view pointer
	cgv::render::view* view_ptr;
	/// ensure that view pointer has been extracted
	bool ensure_view_pointer();
	/// store last view point to draw eye ray
	Pnt last_view_point, last_target_point;
	/// whether a view ray has been defined
	bool have_view_ray;

	///Event when Transformations are stored to file
	bool saved_transformations_event;
	///ProjectFilepath
	std::string project_file;
	///ProjectFilepath write access
	std::string write_project_file;

	//Generates a room Box, Table and pointcloud holders
	void generate_room_boxes();
	//aligns the scans along the line
	void position_scans();
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
	const Qat& comp_rot(Idx ci) const { return pc.component_rotation(ci); }
	/// write access to component rotation
	Qat& comp_rot(Idx ci) { return pc.component_rotation(ci); }
	/// read access to component translation
	const Dir& comp_trans(Idx ci) const { return pc.component_translation(ci); }
	/// write access to component tranlation
	Dir& comp_trans(Idx ci) { return pc.component_translation(ci); }
	/// read access to component color
	const RGBA& comp_color(Idx ci) const { return pc.component_color(ci); }
	/// write access to component color
	RGBA& comp_color(Idx ci) { return pc.component_color(ci); }
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
	Pnt box_ray_intersection(const Pnt & ray_start, const Dir & ray_dir, const Box & box);
	Pnt box_ray_intersection(const Pnt & ray_start, const Dir & ray_dir, const Box & box, const Dir & box_translationm, const Qat & box_rotation);

	/// loads given Transformations and flags from File
	void load_project_file(std::string projectFile);
	/// saves usermodified Transformations and flags to File
	void save_project_file(std::string projectFile);
	/// This flag shows if the reset function is working on the transformations at this given point of time
	bool transformation_lock;

	/// transforms a rotation and translation from global to local coordinates of a given Point
	point_cloud_types::Pnt transform_to_local(const Pnt & in, const Dir & local_translation, const Qat & local_rotation);
	
	//@}
	interval calculate_intersectionintervall(double rayStart,double maxBoxCoord1, double maxBoxCoord2, double raydir);

	/**@name user interface */
	//@{
	/// process key and mouse events
	bool handle(cgv::gui::event& e);
	/// used to update all dependent variables in case of changes to the point cloud
	void on_point_cloud_change_callback(PointCloudChangeEvent pcc_event);
	/// used to reset all loaded componets transformations and put them back on the line
	void reset_componets_transformations();
	/// used to update all dependent variables in case of changes to member variables
	void on_set(void* member_ptr);
	/// user interface creation
	void create_gui();
	//@}
};

#include <cgv/config/lib_end.h>