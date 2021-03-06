#pragma once


#include <libs/point_cloud/point_cloud_interactable.h>
#include <libs/point_cloud/point_cloud.h>
#include "constructed_set.h"
#include <list>

#include "lib_begin.h"


class program_state 
{
private:
	std::vector<point_cloud_types::Dir> translations;
	std::vector<point_cloud_types::Qat> rotations;
	std::list<constructed_set> group_informations;
	std::vector<cgv::media::color<float,cgv::media::RGB,cgv::media::OPACITY>> component_colors;
	cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY> old_color;
	cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY> even_older_color;
	constructed_set picked_Group;
	constructed_set previous_picked_Group;	

public:
	program_state();
	program_state(const std::vector<point_cloud_types::Dir>& translations_, const std::vector<point_cloud_types::Qat>& rotations_, const constructed_set& picked, const constructed_set& previous_picked, const std::list<constructed_set>& group_informations_, const std::vector<cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY>>& component_colors_, const cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY>& old_color_, const cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY>& even_older_color_);
	void put_back_state(point_cloud &pcs, constructed_set* picked, constructed_set* previous_picked, std::list<constructed_set> &group_informations_, cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY> &old_color, cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY> &even_older_color_);
};


#include <cgv/config/lib_end.h>