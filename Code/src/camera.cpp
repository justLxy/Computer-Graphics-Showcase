#include "camera.h"

Ray Camera::generate_ray(double px, double py, int width, int height) const{
	Vec3 forward = normalize(target - eye);
	Vec3 right = normalize(cross(forward, up));
	Vec3 trueUp = cross(right, forward);

	double tanHalfFov = std::tan(0.5 * fov_y_deg * PI / 180.0);
	double ndc_x = ( (px + 0.5) / static_cast<double>(width)  ) * 2.0 - 1.0;
	double ndc_y = ( (py + 0.5) / static_cast<double>(height) ) * 2.0 - 1.0;
	ndc_y = -ndc_y; // flip so +y is up

	Vec3 dir_cam = normalize(forward + ndc_x*aspect*tanHalfFov*right + ndc_y*tanHalfFov*trueUp);
	Ray r;
	r.origin = eye;
	r.dir = dir_cam;
	return r;
}


