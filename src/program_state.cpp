#include "program_state.h"

#include <cgv/base/register.h>
#include <libs/cgv_gl/gl/gl.h>
#include "cgv/gui/animate.h"


constexpr auto ANIMATION_DURATION = 2;

program_state::program_state()
{
}

program_state::program_state(const std::vector<point_cloud_types::Dir>& translations_, const std::vector<point_cloud_types::Qat>& rotations_, const constructed_set& picked, const constructed_set& previous_picked, const std::list<constructed_set>& group_informations_, const std::vector<cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY>>& component_colors_, const cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY>& old_color_, const cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY>& even_older_color_)
{
	translations = translations_;
	rotations = rotations_;
	group_informations = group_informations_;
	component_colors = component_colors_;
	picked_Group = picked;
	previous_picked_Group = previous_picked;
	old_color = old_color_;
	even_older_color = even_older_color_;
}
#ifdef _DEBUG
void program_state::put_back_state(point_cloud & pcs, constructed_set* picked, constructed_set* previous_picked, std::list<constructed_set> &group_informations_, cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY> &old_color_, cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY> &even_older_color_)
{
	int nr_Components = pcs.get_nr_components();
	for (int i = 0; i < nr_Components; ++i)
	{ 
		pcs.component_rotation(i) = rotations.at(i);
		pcs.component_translation(i) = translations.at(i);
		pcs.component_color(i) = component_colors.at(i);
	}
	*picked = picked_Group;
	*previous_picked = previous_picked_Group;
	group_informations_ = group_informations;
	old_color_ = old_color;
	even_older_color_ = even_older_color;
}
#else
void program_state::put_back_state(point_cloud & pcs, constructed_set* picked, constructed_set* previous_picked, std::list<constructed_set> &group_informations_, cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY> &old_color_, cgv::media::color<float, cgv::media::RGB, cgv::media::OPACITY> &even_older_color_)
{
	int nr_Components = pcs.get_nr_components();
	for (int i = 0; i < nr_Components; ++i)
	{
		cgv::gui::animate_with_linear_blend(pcs.component_translation(i), translations.at(i), ANIMATION_DURATION, 0, false);
		pcs.component_rotation(i) = rotations.at(i);
		cgv::gui::animate_with_linear_blend(pcs.component_color(i), component_colors.at(i), ANIMATION_DURATION, 0, false);
	}
	*picked = picked_Group;
	*previous_picked = previous_picked_Group;
	group_informations_ = group_informations;
	old_color_ = old_color;
	even_older_color_ = even_older_color;
}
#endif // DEBUG

