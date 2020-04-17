#ifndef PROJECTION_H
#define PROJECTION_H

#include <Eigen/Core>
#include <Eigen/Geometry>

struct Projection {
	virtual void operator()(Eigen::Vector3f& p) = 0;
};

struct PerspectiveProjection : public Projection {

	void operator()(Eigen::Vector3f& p) override {
		p = (projection_mat * p.homogeneous()).hnormalized();

		// Translate to on-screen pixels
		p.x() = static_cast<long>(((p.x() + 1.0f) / 2.0f) * screen_width_);
		p.y() = static_cast<long>(((p.y() + 1.0f) / 2.0f) * screen_height_);
		p.z() = static_cast<long>(((p.z() - 1.0f) / 2.0f) * screen_depth_);
	}

	PerspectiveProjection(
		std::size_t screen_width, std::size_t screen_height, std::size_t screen_depth,
		float clipping_plane_near, float clipping_plane_far,
		float view_angle) :
		screen_width_(screen_width), screen_height_(screen_height), screen_depth_(screen_depth),
		clipping_plane_near_(clipping_plane_near), clipping_plane_far_(clipping_plane_far)
	{

		float aspect_ratio = screen_width_ / screen_height_;
		float t = -clipping_plane_near_ * std::tan((view_angle / 2) * M_PI / 180);
		float b = clipping_plane_near_ * std::tan((view_angle / 2) * M_PI / 180);
		float r = aspect_ratio * t;
		float l = aspect_ratio * b;

		// Set the Projection Matrix
		projection_mat <<
			(2 * clipping_plane_near_) / (r - l), 0, (r + l) / (r - l), 0,
			0, 2 * clipping_plane_near_ / (t - b), (t + b) / (t - b), 0,
			0, 0, -(clipping_plane_far_ + clipping_plane_near_) /
			(clipping_plane_far_ - clipping_plane_near_),
			(-2 * clipping_plane_far_ * clipping_plane_near_) /
			(clipping_plane_far_ - clipping_plane_near_),
			0, 0, -1, 0;
	}

private:
	float clipping_plane_near_;
	float clipping_plane_far_;

	std::size_t screen_width_;
	std::size_t screen_height_;
	std::size_t screen_depth_;

	Eigen::Matrix<float, 4, 4> projection_mat;
};

struct OrthographicProjection : public Projection {

};


#endif // !PROJECTION_H


