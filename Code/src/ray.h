#pragma once
#include "cg_math.h"

struct Ray {
	Vec3 origin;
	Vec3 dir; // normalized
	double tmin = 1e-4;
	double tmax = kInfinity;
};


