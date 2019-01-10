#pragma once


#include <libs/point_cloud/point_cloud_interactable.h>
#include "lib_begin.h"

class interval
{
public:
	interval(float a, float b);
	~interval();
	interval intersectIntervals(interval other);
	float get_min();
	float get_max();
	bool isInvalid();
private:
	float min;
	float max;
	bool invalid;

};

#include <cgv/config/lib_end.h>

