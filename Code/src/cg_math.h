#pragma once
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstdint>

constexpr double PI = 3.14159265358979323846264338327950288;

struct Vec3 {
	double x, y, z;
	constexpr Vec3() : x(0.0), y(0.0), z(0.0) {}
	constexpr Vec3(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}
	double operator[](int i) const { return i==0?x:(i==1?y:z); }
	double& operator[](int i) { return i==0?x:(i==1?y:z); }
	Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
	Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
	Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
	Vec3 operator/(double s) const { return {x/s, y/s, z/s}; }
	Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
	Vec3& operator-=(const Vec3& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
	Vec3& operator*=(double s) { x*=s; y*=s; z*=s; return *this; }
	Vec3& operator/=(double s) { x/=s; y/=s; z/=s; return *this; }
	Vec3 operator-() const { return {-x,-y,-z}; }
};
inline Vec3 operator*(double s, const Vec3& v){ return v*s; }
inline Vec3 operator*(const Vec3& a, const Vec3& b){ return {a.x*b.x, a.y*b.y, a.z*b.z}; }
inline double dot(const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b){
	return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
inline double length(const Vec3& v){ return std::sqrt(dot(v,v)); }
inline Vec3 normalize(const Vec3& v){
	double len = length(v);
	return len>0.0 ? v/len : v;
}
inline Vec3 clamp01(const Vec3& v){
	auto c=[](double x){ return std::min(1.0, std::max(0.0,x)); };
	return {c(v.x), c(v.y), c(v.z)};
}

struct Vec2 {
	double x, y;
	constexpr Vec2(): x(0.0), y(0.0) {}
	constexpr Vec2(double xx, double yy): x(xx), y(yy) {}
	Vec2 operator+(const Vec2& o) const { return {x+o.x,y+o.y}; }
	Vec2 operator-(const Vec2& o) const { return {x-o.x,y-o.y}; }
	Vec2 operator*(double s) const { return {x*s,y*s}; }
};

inline uint8_t to_u8(double c){
	double x = std::min(1.0, std::max(0.0, c));
	return static_cast<uint8_t>(std::round(x*255.0));
}

constexpr double kInfinity = std::numeric_limits<double>::infinity();


