#pragma once

#include <libs/point_cloud/point_cloud_interactable.h>
#include "lib_begin.h"

class constructed_set
{
public:
	constructed_set();
	constructed_set(std::vector<int> init, int ID);
	~constructed_set();
	void unite(constructed_set other);
	void seperate_component(int to_Seperate);
	bool find_component_ID(int toFind);
	std::vector<int> get_component_IDs();
	int get_ID();
private:
	std::vector<int> scan_IDs;
	int ID;
};

#include <cgv/config/lib_end.h>
