#pragma once
#include <vector>
#include <string>
#include "cg_math.h"

struct Image {
	int width;
	int height;
	std::vector<Vec3> pixels; // RGB in [0,1]
	Image(): width(0), height(0) {}
	Image(int w, int h): width(w), height(h), pixels(static_cast<size_t>(w*h)) {}
	void set(int x, int y, const Vec3& c){
		if(x<0||x>=width||y<0||y>=height) return;
		pixels[static_cast<size_t>(y*width + x)] = c;
	}
	Vec3 get(int x, int y) const {
		if(x<0||x>=width||y<0||y>=height) return {0,0,0};
		return pixels[static_cast<size_t>(y*width + x)];
	}
	bool write_ppm(const std::string& path) const;
	void fill(const Vec3& c);
	void overlay_rect(int x0,int y0,int w,int h,const Vec3& color,double alpha);
	void draw_text_8x8(int x,int y,const std::string& text,const Vec3& color);
};


