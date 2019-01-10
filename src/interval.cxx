#include "interval.h"
#include <cgv/base/register.h>
#include <libs/cgv_gl/gl/gl.h>

interval::interval(float a, float b)
{
	if (a < b) {
		min = a;
		max = b;
		invalid = false;
	}
	else {
		a = 0;
		b = 0;
		invalid = true;
	}
}

interval interval::intersectIntervals(interval other) 
{
	if (cgv::math::maximum(min, other.get_min()) > cgv::math::minimum(max,other.get_max()))
	{
		interval a = interval(float(1.0),float(0.0));
		return a;
	}
	else {
		return interval(cgv::math::maximum(min, other.get_min()), cgv::math::minimum(max, other.get_max()));
	}
}

float interval::get_min()
{
	return min;
}

float interval::get_max()
{
	return max;
}

bool interval::isInvalid()
{
	return invalid;
}

interval::~interval()
{
}
