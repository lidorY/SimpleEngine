#ifndef DIRECTIONAL_LIGHT_H
#define DIRECTIONAL_LIGHT_H



#include <Eigen\Core>
#include <Eigen\Geometry>

#include "common.h"


struct DirectionalLight {
	Eigen::Vector3f diffuse_coef;
	float intensity;
	Eigen::Vector3f direction;

	void Rotate(float angle_x, float angle_y, float angle_z) {
		float rad_x = (angle_x)*M_PI / 180;
		float rad_y = (angle_y)*M_PI / 180;
		float rad_z = (angle_z)*M_PI / 180;

		Eigen::Matrix3f m;
		m = Eigen::AngleAxisf(rad_x, Eigen::Vector3f::UnitX())
			* Eigen::AngleAxisf(rad_y, Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(rad_z, Eigen::Vector3f::UnitZ());

		direction = m * direction;
		direction.normalize();
	}

};

#endif // !DIRECTIONAL_LIGHT_H
