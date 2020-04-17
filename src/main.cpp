#include <Windows.h>
#include <memory>
#include <chrono>
#include <thread>
#include <iostream>

#include "screen.h"
#include "mesh.h"
#include "rasterizer.h"



using namespace std::placeholders;
int main() {
	
	HDC consoleDC = GetDC(GetConsoleWindow());
	std::unique_ptr<Screen> screen_ = std::make_unique<Screen>(512,512, consoleDC);

	//WireFrame drawer{*screen_};
	//Solid solid_drawer{ *screen_ };

	TriangleScanline<PerspectiveProjection> s(screen_->Width(), screen_->Height(), 
		std::make_unique<PerspectiveProjection>(screen_->Width(),
			screen_->Height(), 255, 1.0f, 60.0f, 60.0f));
	//Mesh m = CreateTriangle();
	Mesh m = CreateCube();
	//m.RotateLocal(15, 0, 0);

	m.RotateLocal(90, 90, 0);

	std::chrono::time_point<std::chrono::system_clock> start;
	std::chrono::time_point<std::chrono::system_clock> stop;
	std::chrono::microseconds duration;

	DirectionalLight sun;
	sun.diffuse_coef = {1,1,1};
	sun.intensity = 1.f;
	sun.direction = { 0, 0, 1 };
	sun.direction.normalize();

	float fps_sum = 0;
	int frame_count = 0;
	while (true) {
		start = std::chrono::system_clock::now();
		m.RotateLocal(1, 3, 0);
		//sun.Rotate(0, 0, 1);
		for (int i = 0; i < 12; i++) {
			s(m[i], sun, std::bind(&Screen::StorePixel, std::ref(*screen_), _1, _2, _3, _4, _5, _6));
		}
		screen_->DrawScreen();
		
		screen_->Swap();
		stop = std::chrono::system_clock::now();
		duration = (std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
		std::this_thread::sleep_for(std::chrono::microseconds(16667) - duration);
		fps_sum += 1000000.f / duration.count();
		frame_count++;
		std::cout << fps_sum/frame_count << " fps\r";
	}

	return 0;
}