#ifndef RASTERIZER_H
#define RASTERIZER_H



#include <functional>
#include <array>
#include <algorithm>
#include <utility>

//#include "common.h"
#include "directional_light.h"
#include "digital_differential_analyzer.h"
// Math helper
#include <Eigen/Core>
#include "projection.h"

template <class ProjectionPolicy>
struct TriangleScanline {
	void operator()(Mesh::triangle_copy_t triangle_data,
		DirectionalLight sun,
		/*const std::function<void(int, int, int, int)>& DrawScanline*/
		const std::function<void(int, int, int, int, int, int)>& WritePixel) const {
		
		// Projecting the vertices to screen space
		auto& t = triangle_data[0];
		(*projector_)(t[0]);
		(*projector_)(t[1]);
		(*projector_)(t[2]);

		// Calulate the vertices diffuse color based on lighting
		auto& normals = triangle_data[1];
		std::array<Eigen::Vector3f, 3> v_colors;
		for (int vertex_index = 0; vertex_index < 3; vertex_index++) {
			for (int rgb = 0; rgb < 3; rgb++) {
				float dp = normals[vertex_index].dot(sun.direction);
				float c = sun.diffuse_coef[rgb] * sun.intensity * dp;
				v_colors[vertex_index][rgb] = std::clamp(c * 255, 0.f, 255.f);
			}
		}

		// Making sure that the edge t[0]--t[2] is the largest edge in the triangle
		std::sort(t.begin(), t.end(), [](const Eigen::Vector3f a,const Eigen::Vector3f b) {
			return std::tie(a.y(), a.x()) < std::tie(b.y(), b.x());
		});
		
		if ((t[1] - t[0]).squaredNorm() > (t[2] - t[1]).squaredNorm()) {
			if ((t[1]-t[0]).squaredNorm() > (t[2]-t[0]).squaredNorm()) {
				std::swap(t[1], t[2]);
			}
		}
		else {
			if ((t[1] - t[2]).squaredNorm() > (t[2] - t[0]).squaredNorm()) {
				std::swap(t[0], t[1]);
			}
		}
		
		// In order to simplify scanline we divide the triangle into 2 90deg angle triangles. to do so we calculate
		// The intersection of perpendicualr vector from t[1] to the edge t[0]--t[2].
		auto C_A = (t[0] - t[2]);
		auto A_B = t[0] - t[1];
		float a = C_A.dot(A_B) / C_A.dot(C_A);

		// Scanlining the 2 resulted triangles
		ScanlineEdge(t, v_colors, { std::array<unsigned int, 2>{2,1}, std::array<unsigned int, 2>{2,0} }, WritePixel, a);
		ScanlineEdge(t, v_colors, { std::array<unsigned int, 2>{0,1}, std::array<unsigned int, 2>{0,2} }, WritePixel, a);

	}


	void ScanlineEdge(const std::array<Eigen::Vector3f,3>& vertices,
		const std::array<Eigen::Vector3f, 3>& colors,
		const std::array<std::array<unsigned int, 2>, 2>& edges,
		const std::function<void(int, int, int, int, int, int)>& WritePixel, float a) const {
		// DDA Scanline algorithm

		std::array<DDA ,2> edges_interpolation {
			DDA{ vertices[edges[0][0]], vertices[edges[0][1]] },
			DDA{ vertices[edges[1][0]], vertices[edges[1][1]] }
		};


		/*std::array<DDA, 2> color_interpolation = {
			DDA{colors[edges[0][0]], vertices[edges[0][1]]},
			DDA{colors[edges[1][0]], vertices[edges[1][1]]},
		};*/

		float max_step = edges_interpolation[0].GetMaxStep();
		for (float i = 0; i <= max_step; i+=1) {
			// Draw edge pixels
			std::array<Eigen::Vector3f, 2> inter_edge_res{
				edges_interpolation[0].GetValueByStep(i),
				edges_interpolation[1].GetValueByStep(i)
			};


			std::array<Eigen::Vector3f, 2> inter_colors_res{
				Interpolate(i / max_step, colors[edges[0][0]], colors[edges[0][1]]),
				Interpolate(i / max_step, colors[edges[1][0]], colors[edges[1][1]])
			};
			for (int curr_edg = 0; curr_edg < 2; curr_edg++) {
				auto edge_res = inter_edge_res[curr_edg];
				auto color_res = inter_colors_res[curr_edg];
				WritePixel(edge_res.x(), edge_res.y(), edge_res.z(), color_res[0], color_res[1], color_res[2]);
			}

			// Draw inner triangle pixesl
			DDA triangle_filling = {inter_edge_res[0], inter_edge_res[1]};
			float max_step_j = triangle_filling.GetMaxStep();
			for (float j = 0; j < max_step_j; j+=0.25) {
				auto edge_res = triangle_filling.GetValueByStep(j);
				auto triangle_filling_color = Interpolate(j / max_step_j, inter_colors_res[0], inter_colors_res[1]);
				WritePixel(edge_res.x(), edge_res.y(), edge_res.z(), triangle_filling_color[0], triangle_filling_color[0], triangle_filling_color[0]);
			}
		}
	}

	TriangleScanline(std::size_t screen_width, std::size_t screen_height, std::unique_ptr<ProjectionPolicy> projector) :
	screen_width_(screen_width),
	screen_height_(screen_height),
	projector_(std::move(projector))
	{}

private:

	typedef  std::pair<const Eigen::Vector3f*, const Eigen::Vector3f*> point_pair_t;
	
	enum  EdgeIndex {
		P0_P1 = 0,
		P1_P2 = 1,
		P0_P2 = 2
	};


	auto Interpolate(float a, float x0, float x1) const {
		return x0 + a * (x1 - x0);
	}


	auto Interpolate(float a, const Eigen::Vector3f& x0, const Eigen::Vector3f& x1) const {
		return x0 + a * (x1 - x0);
	}

	auto GetEdgePointX(float y_value, point_pair_t& edge, float slope) const {
		// TODO: Currently assuming valid y value..
		auto x0 = edge.first->x();
		auto x1 = edge. second->x();

		auto y0 = edge.first->y();
		auto y1 = edge.second->y();

		return x0 + slope * (y_value - y0);
	}


	std::size_t screen_width_;
	std::size_t screen_height_;
	// Policies
	std::unique_ptr<ProjectionPolicy> projector_;
};


//struct WireFrame {
//	WireFrame(Screen& scr) : scr_(scr) {}
//	void DrawWireframe(int y, int max_y, int x0, int x1) const {
//
//		if (y == (max_y)) {
//			for (int i = x0; i < x1; i++) {
//				scr_.StorePixel(i, y);
//			}
//		}
//		else {
//			scr_.StorePixel(x0, y);
//			scr_.StorePixel(x1, y);
//		}
//	}
//private:
//	Screen& scr_;
//};

//struct Solid {
//	Solid(Screen& scr) : scr_(scr) {}
//	void Draw(int y, int max_y, int x0, int x1) const {
//		for (int i = x0; i < x1; i++) {
//				scr_.StorePixel(i, y);
//			}
//	}
//private:
//	Screen& scr_;
//};




#endif /*RASTERIZER_H*/