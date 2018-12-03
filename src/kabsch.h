#pragma once

#include <Eigen/Geometry>
#include "lib_begin.h"

class kabsch {
public:
	static Eigen::Affine3d Find3DAffineTransform(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out);
	static void TestFind3DAffineTransform();
};


#include <cgv/config/lib_end.h>