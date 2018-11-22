#include "interval.h"
#include <cgv/base/register.h>
#include <libs/cgv_gl/gl/gl.h>

interval::interval(double a, double b)
{
	if (a < b) {
		min = a;
		max = b;
		invalid = false;
	}
	else {
		a = 0;
		b = 0;
		this->setInvalid();
	}
}

interval interval::intersectIntervals(interval other) 
{
	if (cgv::math::maximum(min, other.get_min()) > cgv::math::minimum(max,other.get_max()))
	{
		interval a = interval(0,0);
		a.setInvalid();
		return a;
	}
	else {
		return interval(cgv::math::maximum(min, other.get_min()), cgv::math::minimum(max, other.get_max()));
	}
}

double interval::get_min()
{
	return min;
}

double interval::get_max()
{
	return max;
}

bool interval::isNullInterval()
{
	if(min==0 && max == 0)
		return true;
	return false;
}

bool interval::isInvalid()
{
	return invalid;
}

void interval::setInvalid()
{
	invalid = true;
}

interval::~interval()
{
}
