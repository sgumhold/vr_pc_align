#pragma once

#include <libs/point_cloud/point_cloud_interactable.h>
#include <libs/cgv_gl/box_renderer.h>
#include <cgv/render/view.h>

#include <cgv/gui/file_dialog.h>
#include "interval.h"
#include "constructed_set.h"
#include "program_state.h"
#include "../sparseicp/ICP.h"
#include <list>
// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>

#include "lib_begin.h"


/** the point cloud view adds a gui to the gl_point_cloud_drawable_base and adds
    some basic processing like normal computation as well as some debug rendering
	of the neighbor graph*/
class CGV_API vr_point_cloud_aligner : public point_cloud_interactable
{
protected:

	///ProjectFilepath
	std::string project_file;
	///The default facing where scans should look at
	cgv::math::quaternion<float> defaultFacing;
	///Indicates, that a project file is loaded and the process is not finished yet
	bool projectLoading_in_process;

	///flag for the icp is in process
	bool icp_executing;

	// keep deadzone and precision vector for left controller
	cgv::gui::vr_server::vec_flt_flt left_deadzone_and_precision;
	// store handle to vr kit of which left deadzone and precision is configured
	void* last_kit_handle;

	// length of to be rendered rays
	float ray_length;

	// keep reference to vr_view_interactor
	vr_view_interactor* vr_view_ptr;

private:
	std::vector<Box> room_boxes;
	std::vector<Clr> sample_box_colors;
	std::vector<Clr> room_colors;
	std::vector<std::string> file_paths;
	std::vector<program_state> program_state_stack;
	int pss_count;

	cgv::render::surface_render_style box_render_style;
	RGBA oldColor;
	RGBA even_older_color;

	///a constant variable to hold a empty constructed set for pointer resetting
	constructed_set global_placeholder_set;
	///This shows the group the picked component belongs to, thus applieng all transformations to the group
	constructed_set* picked_group;
	///This is the same as previous picked component
	constructed_set* previous_picked_group;
	///This is a single component picked to seperate it from its group
	int picked_component;
	///This is the flag showing its valid
	bool picked_component_valid;
	///This variable stores the last color of a picked component
	RGBA picked_component_color;
	///safes latest programstackstate
	void push_back_state();
	///restores programstaskstate at point i
	void restore_state(int i);

	/// Checks for intersections and updates picked groups
	bool try_group_pick();
	/// Checks for intersections and updates picked components to seperate
	void try_component_pick();
	/// Deselects all groups and restores their colors
	void deselect_groups();
	/// Flag to show that a seperation is in process
	bool seperation_in_process;
	/// a chosen component to seperate
	int seperation_id;
	/// reverses the animation of scan seperation
	void display_reverse_seperation();
	/// seperates a previously chosen scan
	void seperate_component();
	/// animates a group to split
	void display_seperation_selection();
	/// scales all scans by the scaling_factor
	bool scale(float scaling_factor);
	///This flag shows if the subsampling variables are changed
	bool subsample_changed;
	Eigen::Matrix<double, 3, Eigen::Dynamic> vertices_source;
	Eigen::Matrix<double, 3, Eigen::Dynamic> vertices_source_copy;
	Eigen::Matrix<double, 3, Eigen::Dynamic> vertices_target;

	///Subsamples point clouds to given Resolution, can unite multiple pointclouds and draft a subsample from their points
	void subsample(Eigen::Matrix<double, 3, Eigen::Dynamic> &vertices_source, Eigen::Matrix<double, 3, Eigen::Dynamic> &vertices_source_copy, Eigen::Matrix<double, 3, Eigen::Dynamic> &target, int subsampling_range);
	
	///A helper function that finds the lowest transformed BB edge of all scans in their current position
	float find_deepest_BB_point();

	///This function repositions the scans above the centre table
	void reposition_above_table();

	/// stores information about aligned sets of scans
	std::list<constructed_set> sets;

	/// homogeneous matrix used to unproject mouse locations
	dmat4 DPV;
	/// whether a picked point is available
	bool have_picked_point;
	/// position of picked point
	Pnt picked_point;
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

	///ProjectFilepath write access
	std::string write_project_file;

	///Simple flag to show the user a pending unite
	bool pending_unite;
	///Simple flag for blinking animation
	bool blink;
	///Iteratet counter to get full seconds
	double time_blink_counter;

	//Generates a room Box, Table and pointcloud holders
	void generate_room_boxes();
	//Helper function
	Clr generate_a_valid_color(int color);
	///GUI for uniting components
	void animate_pending_unite_blink();
	///The actual uniting function
	void unite(bool unite);

	///indicates that a scan is dragged by the controller at the moment
	bool drag_active;

	///Saves the draging distance for an optimal point rotation
	float current_picked_distance;
	///Saves the local pose of a group/component for dragging
	std::vector<cgv::math::fmat<float, 4, 4>> local_pose_mat_stack;
	///calculates the local pose of the picked group/component
	void calculate_local_pose(const float*);

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


	void timer_event(double t, double dt);

	void construct_environment(float s, float ew, float ed, float eh, float w, float d, float h);

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
	bool box_ray_intersection(const Pnt & ray_start, const Pnt & ray_dir, const Box & box, Pnt& result);
	point_cloud_types::Crd box_ray_intersection(const Pnt & ray_start, const Pnt & ray_dir, const Box & box, const Dir & box_translation, const Qat & box_rotation);

	/// loads given Transformations and flags from File
	void load_project_file(std::string projectFile);
	/// saves usermodified Transformations and flags to File
	void save_project_file(std::string projectFile);
	/// This flag shows if the reset function is working on the transformations at this given point of time
	bool transformation_lock;

	float current_scaling_factor;
	/// transforms a rotation and translation from global to local coordinates of a given Point
	point_cloud_types::Pnt transform_to_local(const Pnt & in, const Pnt & local_translation, const Qat & local_rotation);
	/// transforms a rotation and translation from local to global coordinates of a given Point
	point_cloud_types::Pnt transform_to_global(const Pnt & in, const Pnt & local_translation, const Qat & local_rotation);

	//@}
	interval calculate_intersectionintervall(float rayStart, float maxBoxCoord1, float maxBoxCoord2, float raydir);
	
	/// Starts the ICP algorithm with the last 2 picked scans. the older one is the target aligned to
	void start_ICP();
	/// Animation Vector
	Dir animation_vector;
	/// Animated Rotation
	


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
	void drag_scan(cgv::math::fmat<float,3,3> rotation, Dir translation);
	void on_device_change(void * kit_handle, bool attach);
	/// user interface creation
	void create_gui();
	//@}
};

#include <cgv/config/lib_end.h>