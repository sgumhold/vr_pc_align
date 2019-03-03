#pragma once
#include <libs/point_cloud/point_cloud_interactable.h>
#include "lib_begin.h"

enum user_action
{
	SCAN_DRAG, SCAN_PICK, SCAN_DESELECT, GROUP_SEPERATE_DISPLAY, GROUP_SEPERATE_CONFIRM, REVERSE, RESTORE, POSITION_ABOVE, RESET, UNITE
};


class user_action_counter
{
public:
	user_action_counter();
	~user_action_counter();

	void push_back_action(user_action u_a);
	std::stringstream print_results_to_printable_string();

private :
	
	std::vector<user_action> actions;
	int scan_drag_count;
	int scan_pick_count;
	int scan_deselect_count;
	int scan_group_seperate_display_count;
	int group_seperate_confirm_count;
	int reverse_count;
	int restore_count;
	int position_count;
	int reset_count;
	int unite_count;
};


#include <cgv/config/lib_end.h>