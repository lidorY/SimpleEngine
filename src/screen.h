#ifndef SCREEN_H
#define SCREEN_H

#include <Windows.h>
#include <cstddef>
#include <memory>
#include <vector>
#include <array>

// TODO: support more than just double buffer?
class Screen {

public:
	Screen(std::size_t width, std::size_t height, HDC d) :
		width_(width),
		height_(height),
		device(d),
		screen_buffer_(1) {

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
	std::array<std::unique_ptr<std::vector<uint8_t>>, 2> buffers_;
	std::vector<uint8_t> depth_buffer_;

	std::size_t width_;
	std::size_t height_;

	const unsigned short channels_ = 3; //Supporting RGB channels only

	BITMAPINFO bmi_;
};


#endif // !SCREEN_H

