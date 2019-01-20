#include "program_state.h"

#include <cgv/base/register.h>
#include <libs/cgv_gl/gl/gl.h>

program_state::program_state()
{
}

program_state::program_state(std::vector<point_cloud_types::Dir> translations_, std::vector<point_cloud_types::Qat> rotations_, constructed_set picked, constructed_set previous_picked, std::vector<constructed_set> group_informations_, std::vector<cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY>> component_colors_)
{
	translations = translations_;
	rotations = rotations_;
	group_informations = group_informations_;
	component_colors = component_colors_;
	picked_Group = picked;
	previous_picked_Group = previous_picked;
}

void program_state::pop_back_latest_state(point_cloud & pcs, constructed_set &picked, constructed_set &previous_picked, std::vector<constructed_set> &group_informations_)
{
	int nr_Components = pcs.get_nr_components();
	for (int i = 0; i < nr_Components; ++i)
	{ 
		pcs.component_rotation(i) = rotations.at(i);
		pcs.component_translation(i) = translations.at(i);
		pcs.component_color(i) = component_colors.at(i);
	}
	picked = picked_Group;
	previous_picked = previous_picked_Group;
	group_informations_ = group_informations;
}

