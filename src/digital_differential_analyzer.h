#ifndef DIGITAL_DIFFERENTIAL_ANALYZER_H
#define DIGITAL_DIFFERENTIAL_ANALYZER_H

#include <Eigen/Core>

// TODO: make generic?
struct DDA {

	DDA(const Eigen::Vector3f& source, const Eigen::Vector3f& destination) {
		source_ = source;
		max_step_ = 0;
		for (int i = 0; i < source.size(); i++) {
			deltas_[i] = destination[i] - source[i];
			if (std::abs(deltas_[i]) > max_step_) {
				max_step_ = std::abs(deltas_[i]);
			}
		}
		deltas_ /= max_step_;
	}
	Eigen::Vector3f GetValueByStep(float step_value) {
		float actual_step = std::clamp(step_value, 0.f, max_step_);
		return source_ + (deltas_ * actual_step);
	}

	float GetMaxStep() {
		return max_step_;
	}
private:
	Eigen::Vector3f source_;
	Eigen::Vector3f deltas_;
	float max_step_;
};
#endif // !DIGITAL_DIFFERENTIAL_ANALYZER_H
