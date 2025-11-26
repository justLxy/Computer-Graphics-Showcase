#pragma once
#include <vector>
#include <memory>
#include "cg_math.h"
#include "ray.h"
#include "obj_loader.h"

struct AABB {
	Vec3 min, max;
	AABB(): min({ kInfinity, kInfinity, kInfinity }), max({-kInfinity,-kInfinity,-kInfinity}) {}
	void expand(const Vec3& p){
		min.x = std::min(min.x, p.x); min.y = std::min(min.y, p.y); min.z = std::min(min.z, p.z);
		max.x = std::max(max.x, p.x); max.y = std::max(max.y, p.y); max.z = std::max(max.z, p.z);
	}
	void expand(const AABB& b){ expand(b.min); expand(b.max); }
};

inline bool ray_aabb(const Ray& r, const AABB& box, double& tmin_out, double& tmax_out){
	double tmin = r.tmin, tmax = r.tmax;
	for(int a=0;a<3;++a){
		double invD = 1.0 / (a==0?r.dir.x:(a==1?r.dir.y:r.dir.z));
		double t0 = ((a==0?box.min.x:(a==1?box.min.y:box.min.z)) - (a==0?r.origin.x:(a==1?r.origin.y:r.origin.z))) * invD;
		double t1 = ((a==0?box.max.x:(a==1?box.max.y:box.max.z)) - (a==0?r.origin.x:(a==1?r.origin.y:r.origin.z))) * invD;
		if(invD < 0.0) std::swap(t0,t1);
		tmin = t0>tmin ? t0 : tmin;
		tmax = t1<tmax ? t1 : tmax;
		if(tmax <= tmin) return false;
	}
	tmin_out = tmin; tmax_out = tmax; return true;
}

struct TriRef {
	int index;
	AABB box;
};

struct BVHNode {
	AABB box;
	int left = -1;
	int right = -1;
	int start = 0;
	int count = 0; // leaf if count>0
};

struct BVH {
	std::vector<BVHNode> nodes;
	std::vector<int> indices; // permutation into triangles

	void build(const std::vector<Triangle>& tris);
	bool intersect(const std::vector<Triangle>& tris, const Ray& ray, int& hitIdx, double& tHit, Vec3& nHit, Vec2& uvHit, double& b0,double& b1,double& b2) const;
};


