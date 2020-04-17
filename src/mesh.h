#ifndef MESH_H
#define MESH_H

#include <vector>
#include <array>
#include <memory>

// Math helper
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common.h"

class Mesh {
public:
	typedef std::vector<std::array<Eigen::Vector3f, 3>> normals_collection_t;
	typedef std::vector<std::array<unsigned int, 3>> triangles_collection_t;
	typedef std::array<std::array<Eigen::Vector3f, 3>, 2>  triangle_copy_t;

	Mesh(std::vector<Eigen::Vector3f>& points,
		triangles_collection_t triangle_indices,
		Eigen::Vector3f pivot, std::unique_ptr<normals_collection_t> normals = nullptr)
		: pivot_(pivot) {
		// Deep copy
		std::copy(points.begin(), points.end(), std::back_inserter(points_));
		std::copy(triangle_indices.begin(), triangle_indices.end(), std::back_inserter(triangles_));

		if (normals != nullptr) {
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
		float rad_x = (angle_x)*M_PI / 180;
		float rad_y = (angle_y)*M_PI / 180;
		float rad_z = (angle_z)*M_PI / 180;

		Eigen::Matrix3f m;
		m = Eigen::AngleAxisf(rad_x, Eigen::Vector3f::UnitX())
			* Eigen::AngleAxisf(rad_y, Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(rad_z, Eigen::Vector3f::UnitZ());

		// Matrix for normal rotation
		auto n = m.transpose().inverse();

		for (auto&& p : points_) {
			// TODO: Make affine.
			p = Eigen::Translation<float, 3>(-pivot_.x(), -pivot_.y(), -pivot_.z()) * p;
			p = m * p;
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
			normals_->push_back({ n, n, n });
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

static Mesh CreateCube() {
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

static Mesh CreateTriangle() {
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


#endif // !MESH_H