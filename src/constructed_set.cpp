#include "constructed_set.h"



constructed_set::constructed_set()
{
}

constructed_set::constructed_set(std::vector<int> init, int ID) 
{
	this->scan_IDs = init;
	this->ID = ID;
}

constructed_set::~constructed_set()
{
}

void constructed_set::unite(constructed_set other)
{
	std::vector<int> otherIDs = other.get_component_IDs();
	for (int x = 0; x < otherIDs.size(); ++x) {
		this->scan_IDs.push_back(otherIDs.at(x));
	}
	//other.~constructed_set;
}

void constructed_set::seperate_component(int to_Seperate)
{
	if (!this->find_component_ID(to_Seperate))
		printf("Internal Error\n");
	std::vector<int> new_Ids;
	for (int x = 0; x < int(this->scan_IDs.size()); ++x)
	{
		if (to_Seperate != scan_IDs.at(x))
			new_Ids.push_back(scan_IDs.at(x));
	}
	this->scan_IDs = new_Ids;
}

bool constructed_set::find_component_ID(int toFind)
{
	for (int x = 0; x < scan_IDs.size(); ++x) 
	{
		if (toFind == scan_IDs.at(x)) 
		{
			return true;
		}
	}
	return false;
}

std::vector<int> constructed_set::get_component_IDs()
{
	return this->scan_IDs;
}

int constructed_set::get_ID()
{
	return ID;
}
