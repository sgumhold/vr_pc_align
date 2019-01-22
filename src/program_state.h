#pragma once


#include <libs/point_cloud/point_cloud_interactable.h>
#include <libs/point_cloud/point_cloud.h>
#include "constructed_set.h"

#include "lib_begin.h"


class program_state 
{
private:
	std::vector<point_cloud_types::Dir> translations;
	std::vector<point_cloud_types::Qat> rotations;
	std::vector<constructed_set> group_informations;
	std::vector<cgv::media::color<float,cgv::media::RGB,cgv::media::OPACITY>> component_colors;
	cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY> old_color;
	constructed_set picked_Group;
	constructed_set previous_picked_Group;	

public:
	program_state();
	program_state(std::vector<point_cloud_types::Dir> translations_, std::vector<point_cloud_types::Qat> rotations_, constructed_set picked, constructed_set previous_picked, std::vector<constructed_set> group_informations_, std::vector<cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY>> component_colors_, cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY> old_color_);
	void pop_back_latest_state(point_cloud &pcs, constructed_set &picked, constructed_set &previous_picked, std::vector<constructed_set> &group_informations_, cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY> &old_color);
};


#include <cgv/config/lib_end.h>