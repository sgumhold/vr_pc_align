#include "user_action_counter.h"



user_action_counter::user_action_counter()
{
	actions = std::vector<user_action>();
	scan_drag_count = 0;
	scan_pick_count = 0;
	scan_deselect_count = 0;
	scan_group_seperate_display_count = 0;
	group_seperate_confirm_count = 0;
	reverse_count = 0;
	restore_count = 0;
	position_count = 0;
	reset_count = 0;
	unite_count = 0;
}


user_action_counter::~user_action_counter()
{
}

void user_action_counter::push_back_action(user_action u_a)
{
	switch (u_a)
	{
	case SCAN_DRAG:
		scan_drag_count++;
		break;
	case SCAN_PICK:
		scan_pick_count++;
		break;
	case SCAN_DESELECT:
		scan_deselect_count++;
		break;
	case GROUP_SEPERATE_DISPLAY:
		scan_group_seperate_display_count++;
		break;
	case GROUP_SEPERATE_CONFIRM:
		group_seperate_confirm_count++;
		break;
	case REVERSE:
		reverse_count++;
		break;
	case RESTORE:
		restore_count++;
		break;
	case POSITION_ABOVE:
		position_count++;
		break;
	case RESET:
		reset_count++;
		break;
	case UNITE:
		unite_count++;
		break;
	default:
		break;
	}
	actions.push_back(u_a);
}

std::stringstream user_action_counter::print_results_to_printable_string()
{
	std::string line="";
	std::stringstream iss;
	iss << "Total actions: " << (scan_drag_count + scan_pick_count + scan_deselect_count + scan_group_seperate_display_count + group_seperate_confirm_count + reverse_count + restore_count + position_count + reset_count + unite_count) + "\n";
	iss << "SCAN_DRAG: " << scan_drag_count << "\n";
	iss << "SCAN_PICK: " << scan_pick_count << "\n";
	iss << "SCAN_DESELECT: " << scan_deselect_count << "\n";
	iss << "GROUP_SEPERATE_DISPLAY: " << scan_group_seperate_display_count << "\n";
	iss << "GROUP_SEPERATE_CONFIRM: " << group_seperate_confirm_count << "\n";
	iss << "REVERSE: " << reverse_count << "\n";
	iss << "RESTORE: " << restore_count << "\n";
	iss << "POSITION_ABOVE: " << position_count << "\n";
	iss << "RESET: " << reset_count << "\n";
	iss << "UNITE: " << unite_count << "\n";
	return iss;
}