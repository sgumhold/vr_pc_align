#pragma once


#include <libs/point_cloud/point_cloud_interactable.h>
#include "lib_begin.h"

class interval
{
public:
	interval(double a, double b);
	~interval();
	interval intersectIntervals(interval other);
	double get_min();
	double get_max();
	bool isNullInterval();
	bool isInvalid();
	void setInvalid();
private:
	double min;
	double max;
	bool invalid;

};

#include <cgv/config/lib_end.h>

