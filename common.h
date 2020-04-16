#ifndef COMMON_H
#define COMMON_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <vector>
#include <array>
#include <windows.h>

// Math helper
#include <Eigen/Core>
#include <Eigen/Geometry>

struct Point3D {
	float x;
	float y;

	Point3D(float xi, float yi, float zi) : x(xi), y(yi), z(zi) {}
	Point3D(Point3D&&) = default;
	Point3D(const Point3D&) = default;

	// TODO: complete constructors and assignments


	bool operator<(const Point3D & rhs) const {
		return std::tie(y, x) < std::tie(rhs.y, rhs.x);
	}

	bool operator>(const Point3D & rhs) const {
		return std::tie(y, x) > std::tie(rhs.y, rhs.x);
	}

	//Point3D& operator=(const Point3D&) = default;
	Point3D& operator=(const Point3D& other) {
		if (&other == this)
			return *this;
		// reuse storage when possible
		x = other.x;
		y = other.y;
		z = other.z;
		return *this;
	};

	float z;
};

class Mesh {
public:
	typedef std::vector<std::array<Eigen::Vector3f, 3>> normals_collection_t;
	typedef std::vector<std::array<unsigned int, 3>> triangles_collection_t;
	typedef std::array<std::array<Eigen::Vector3f, 3>, 2>  triangle_copy_t;

	Mesh(std::vector<Eigen::Vector3f>& points,
		triangles_collection_t triangle_indices,
		Eigen::Vector3f pivot, std::unique_ptr<normals_collection_t> normals = nullptr)
		: pivot_(pivot){
		// Deep copy
		std::copy(points.begin(), points.end(), std::back_inserter(points_));
		std::copy(triangle_indices.begin(), triangle_indices.end(), std::back_inserter(triangles_));

		if(normals != nullptr) {
			normals_ = std::move(normals);
		} 
		else {
			normals_ = std::make_unique<normals_collection_t>();
			CalculateNormals();
		}
	}

	triangle_copy_t operator[](unsigned int i) const {
		return { 
			std::array<Eigen::Vector3f, 3>{
				points_[triangles_[i][0]],
				points_[triangles_[i][1]],
				points_[triangles_[i][2]]
			} ,
			std::array<Eigen::Vector3f, 3>{
				(*normals_)[i][0],
				(*normals_)[i][1],
				(*normals_)[i][2]
			}
		};
	}

	void RotateLocal(float angle_x, float angle_y, float angle_z) {
		float rad_x = (angle_x) * M_PI / 180;
		float rad_y = (angle_y) * M_PI / 180;
		float rad_z = (angle_z) * M_PI / 180;

		Eigen::Matrix3f m;
		m = Eigen::AngleAxisf(rad_x, Eigen::Vector3f::UnitX())
			                         * Eigen::AngleAxisf(rad_y, Eigen::Vector3f::UnitY())
			                         * Eigen::AngleAxisf(rad_z, Eigen::Vector3f::UnitZ());

		// Matrix for normal rotation
		auto n = m.transpose().inverse();

		for (auto&& p : points_) {
			// TODO: Make affine.
			p = Eigen::Translation<float, 3>(-pivot_.x(), -pivot_.y(), -pivot_.z()) * p;
			p = m*p;
			p = Eigen::Translation<float, 3>(pivot_.x(), pivot_.y(), pivot_.z()) * p;
		}
		for (auto&& normal_trio : *normals_) {
			normal_trio[0] = n * normal_trio[0];
			normal_trio[1] = n * normal_trio[1];
			normal_trio[2] = n * normal_trio[2];
			normal_trio[0].normalize();
			normal_trio[1].normalize();
			normal_trio[2].normalize();
		}
	}
	void Translate() {}
	void Scale() {}
	void CalculateNormals() {
		for (auto&& t : triangles_) {
			//Calculate 3 normals for the polygon as a perpendicular normalized vector
			Eigen::Vector3f n = points_[t[0]].cross(points_[t[1]]);
			n.normalize();
			normals_->push_back({n, n, n});
		}
	}


	std::size_t GetSize() {
		return triangles_.size();
	}
private:
	
	std::vector<Eigen::Vector3f> points_;
	triangles_collection_t triangles_;
	
	Eigen::Vector3f pivot_;
	
	std::unique_ptr<normals_collection_t> normals_;
};

// TODO: support more than just double buffer?
class Screen {

public:
	Screen(std::size_t width, std::size_t height, HDC d):
	width_(width),
	height_(height),
	device(d),
	screen_buffer_(1){
		
		auto row_stride = width * channels_;

		// Init pixel buffers
		buffers_[0] = std::make_unique<std::vector<uint8_t>>();
		buffers_[1] = std::make_unique<std::vector<uint8_t>>();
		
		buffers_[0]->resize(row_stride * height);
		buffers_[1]->resize(row_stride * height);
		std::fill(buffers_[0]->begin(), buffers_[0]->end(), 0);
		std::fill(buffers_[1]->begin(), buffers_[1]->end(), 0);

		// Init depth buffer
		depth_buffer_.resize(width * height);
		std::fill(depth_buffer_.begin(), depth_buffer_.end(), 0);

		// Setting BMP header values
		SetBMPHeaderValues();
	}

	void StorePixel(std::size_t x, std::size_t y, std::size_t z, uint8_t b = 255, uint8_t g = 255, uint8_t r = 255) {
		auto curr_buf = (screen_buffer_ + static_cast<unsigned short>(1)) % static_cast<unsigned short>(buffers_.size());
		if (depth_buffer_[y * width_ + x] < z) {
			// We change the pixel value only if its closer to the viewer
			(*buffers_[curr_buf])[channels_ * (y * width_ + x) + 0] = b;
			(*buffers_[curr_buf])[channels_ * (y * width_ + x) + 1] = g;
			(*buffers_[curr_buf])[channels_ * (y * width_ + x) + 2] = r;
			depth_buffer_[y * width_ + x] = z;
			//DEBUG
			//SetPixel(device, x, 512 - y, COLORREF(RGB(255, 255, 255)));
		}
		if (depth_buffer_[y * width_ + x] == z) {
			(*buffers_[curr_buf])[channels_ * (y * width_ + x) + 0] = max(b, (*buffers_[curr_buf])[channels_ * (y * width_ + x) + 0]);
			(*buffers_[curr_buf])[channels_ * (y * width_ + x) + 1] = max(g, (*buffers_[curr_buf])[channels_ * (y * width_ + x) + 0]);
			(*buffers_[curr_buf])[channels_ * (y * width_ + x) + 2] = max(r, (*buffers_[curr_buf])[channels_ * (y * width_ + x) + 0]);
		}
	}
	
	uint8_t* GetScreenData() {
		return buffers_[screen_buffer_]->data();
	}

	void Swap() {
		std::fill(buffers_[screen_buffer_]->begin(), buffers_[screen_buffer_]->end(), 0);
		std::fill(depth_buffer_.begin(), depth_buffer_.end(), 0);
		screen_buffer_ = (screen_buffer_ + static_cast<unsigned short>(1)) % static_cast<unsigned short>(buffers_.size());
	}

	void DrawScreen() {
		SetDIBitsToDevice(device,
			0, 0, Width(), Height(),
			0, 0, 0, Height(),
			buffers_[screen_buffer_]->data(),
			&bmi_,
			DIB_RGB_COLORS);
	}

	std::size_t Width() { return width_; }
	std::size_t Height() { return height_; }

private:
	HDC device;
	void SetBMPHeaderValues() {
		ZeroMemory(&bmi_, sizeof(bmi_));
		BITMAPINFOHEADER& h = bmi_.bmiHeader;
		h.biSize = sizeof(BITMAPINFOHEADER);
		h.biWidth = Width();
		h.biHeight = Height();
		h.biPlanes = 1;
		h.biBitCount = 24;
		h.biCompression = BI_RGB;
		h.biSizeImage = Width() * Height();
	}

	unsigned short screen_buffer_;
	std::array<std::unique_ptr<std::vector<uint8_t>>,2> buffers_;
	std::vector<uint8_t> depth_buffer_;

	std::size_t width_;
	std::size_t height_;
	
	const unsigned short channels_ = 3; //Supporting RGB channels only

	BITMAPINFO bmi_;
};

Mesh CreateCube() {
	std::vector<Eigen::Vector3f> pts{
		Eigen::Vector3f{ -1, 1 , 6 }, Eigen::Vector3f{ -1, -1, 6 }, Eigen::Vector3f{ 1,  1, 6 } , Eigen::Vector3f { 1, -1, 6},
		Eigen::Vector3f{ -1, 1 , 8 }, Eigen::Vector3f{ -1, -1, 8 }, Eigen::Vector3f{ 1,  1, 8 } , Eigen::Vector3f { 1, -1, 8}
	};
	std::vector<std::array<unsigned int, 3>> indices = {
		//front
		{0,1,2},
		{1,2,3},
		//back
		{4,5,6},
		{5,6,7},
		//Left
		{0,4,5},
		{1,5,0},
		//bottom
		{1,3,5},
		{5,7,3},
		//right
		{3,2,6},
		{6,7,3},
		//top
		{0,4,6},
		{0,2,6}
	};
	std::unique_ptr<Mesh::normals_collection_t> normals = std::make_unique<Mesh::normals_collection_t>(
		Mesh::normals_collection_t{
		//front
		std::array<Eigen::Vector3f, 3>{Eigen::Vector3f{0,0,1},Eigen::Vector3f{0,0,1},Eigen::Vector3f{0,0,1}},
		std::array<Eigen::Vector3f, 3>{Eigen::Vector3f{0,0,1},Eigen::Vector3f{0,0,1},Eigen::Vector3f{0,0,1}},
		//back
		std::array<Eigen::Vector3f, 3>{Eigen::Vector3f{0,0,-1},Eigen::Vector3f{0,0,-1},Eigen::Vector3f{0,0,-1}},
		std::array<Eigen::Vector3f, 3>{Eigen::Vector3f{0,0,-1},Eigen::Vector3f{0,0,-1},Eigen::Vector3f{0,0,-1}},
		//Left
		std::array<Eigen::Vector3f, 3>{Eigen::Vector3f{1,0,0},Eigen::Vector3f{1,0,0},Eigen::Vector3f{1,0,0}},
		std::array<Eigen::Vector3f, 3>{Eigen::Vector3f{1,0,0},Eigen::Vector3f{1,0,0},Eigen::Vector3f{1,0,0}},
		//bottom
		std::array<Eigen::Vector3f, 3>{Eigen::Vector3f{0,1,0},Eigen::Vector3f{0,1,0},Eigen::Vector3f{0,1,0}},
		std::array<Eigen::Vector3f, 3>{Eigen::Vector3f{0,1,0},Eigen::Vector3f{0,1,0},Eigen::Vector3f{0,1,0}},
		//right
		std::array<Eigen::Vector3f, 3>{Eigen::Vector3f{-1,0,0},Eigen::Vector3f{-1,0,0},Eigen::Vector3f{-1,0,0}},
		std::array<Eigen::Vector3f, 3>{Eigen::Vector3f{-1,0,0},Eigen::Vector3f{-1,0,0},Eigen::Vector3f{-1,0,0}},
		//top
		std::array<Eigen::Vector3f, 3>{Eigen::Vector3f{0,-1,0},Eigen::Vector3f{0,-1,0},Eigen::Vector3f{0,-1,0}},
		std::array<Eigen::Vector3f, 3>{Eigen::Vector3f{0,-1,0},Eigen::Vector3f{0,-1,0},Eigen::Vector3f{0,-1,0}},
		});
	return Mesh{ pts, indices, Eigen::Vector3f{0, 0, 7}, std::move(normals) };
}


Mesh CreateTriangle() {
	std::vector<Eigen::Vector3f> pts{
		Eigen::Vector3f{ -1, 0 , 6 }, Eigen::Vector3f{ 1, 0, 6 }, Eigen::Vector3f{ 0,  1, 6 }
	};
	std::unique_ptr<Mesh::normals_collection_t> normals = std::make_unique<Mesh::normals_collection_t>(
	Mesh::normals_collection_t{
		//front
		std::array<Eigen::Vector3f, 3>{Eigen::Vector3f{0,0,1},Eigen::Vector3f{0,0,1},Eigen::Vector3f{0,0,1}},
	});
	std::vector<std::array<unsigned int, 3>> indices = { {0,1,2} };
	return Mesh{ pts, indices, Eigen::Vector3f{0.33f, 0.33f, 6}, std::move(normals) };
}


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

#endif