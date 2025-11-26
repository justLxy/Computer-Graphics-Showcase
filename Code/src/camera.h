#pragma once
#include "cg_math.h"
#include "ray.h"

struct Camera {
	Vec3 eye;
	Vec3 target;
	Vec3 up;
	double fov_y_deg;
	double aspect;

	Ray generate_ray(double px, double py, int width, int height) const;
};


