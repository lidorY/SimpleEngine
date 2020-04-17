#ifndef COMMON_H
#define COMMON_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//struct Point3D {
//	float x;
//	float y;
//
//	Point3D(float xi, float yi, float zi) : x(xi), y(yi), z(zi) {}
//	Point3D(Point3D&&) = default;
//	Point3D(const Point3D&) = default;
//
//	// TODO: complete constructors and assignments
//
//
//	bool operator<(const Point3D & rhs) const {
//		return std::tie(y, x) < std::tie(rhs.y, rhs.x);
//	}
//
//	bool operator>(const Point3D & rhs) const {
//		return std::tie(y, x) > std::tie(rhs.y, rhs.x);
//	}
//
//	//Point3D& operator=(const Point3D&) = default;
//	Point3D& operator=(const Point3D& other) {
//		if (&other == this)
//			return *this;
//		// reuse storage when possible
//		x = other.x;
//		y = other.y;
//		z = other.z;
//		return *this;
//	};
//
//	float z;
//};
#endif