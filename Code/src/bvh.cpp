#include "bvh.h"
#include <algorithm>
#include <stack>

static AABB tri_aabb(const Triangle& t){
	AABB b; b.expand(t.p0); b.expand(t.p1); b.expand(t.p2); return b;
}

void BVH::build(const std::vector<Triangle>& tris){
	indices.resize(tris.size());
	for(size_t i=0;i<tris.size();++i) indices[i] = static_cast<int>(i);
	nodes.clear(); nodes.reserve(tris.size()*2);

	struct BuildRange { int node; int start; int end; AABB box; };
	auto make_node = [&](const AABB& box){ BVHNode n; n.box=box; nodes.push_back(n); return (int)nodes.size()-1; };

	// Precompute centers
	std::vector<Vec3> centers(tris.size());
	for(size_t i=0;i<tris.size();++i){
		const Triangle& t = tris[i];
		centers[i] = (t.p0 + t.p1 + t.p2) / 3.0;
	}

	// initial box
	AABB rootBox;
	for(const auto& t: tris) rootBox.expand(tri_aabb(t));
	int root = make_node(rootBox);
	std::stack<BuildRange> st;
	st.push({root, 0, (int)tris.size(), rootBox});

	while(!st.empty()){
		BuildRange br = st.top(); st.pop();
		int count = br.end - br.start;
		if(count <= 8){
			nodes[br.node].start = br.start;
			nodes[br.node].count = count;
			continue;
		}
		// choose split axis by extent
		Vec3 ext = br.box.max - br.box.min;
		int axis = 0;
		if(ext.y > ext.x && ext.y >= ext.z) axis = 1;
		else if(ext.z > ext.x && ext.z >= ext.y) axis = 2;
		// partition by median of centers on axis
		int mid = br.start + count/2;
		std::nth_element(indices.begin()+br.start, indices.begin()+mid, indices.begin()+br.end,
			[&](int a, int b){
				const Vec3& ca = centers[(size_t)a];
				const Vec3& cb = centers[(size_t)b];
				return (axis==0?ca.x:(axis==1?ca.y:ca.z)) < (axis==0?cb.x:(axis==1?cb.y:cb.z));
			});
		// compute child boxes
		AABB leftBox, rightBox;
		for(int i=br.start;i<mid;++i) leftBox.expand(tri_aabb(tris[(size_t)indices[(size_t)i]]));
		for(int i=mid;i<br.end;++i)  rightBox.expand(tri_aabb(tris[(size_t)indices[(size_t)i]]));
		int left = make_node(leftBox);
		int right = make_node(rightBox);
		nodes[br.node].left = left;
		nodes[br.node].right = right;
		nodes[br.node].count = 0;
		nodes[br.node].start = 0;
		st.push({right, mid, br.end, rightBox});
		st.push({left, br.start, mid, leftBox});
	}
}

static bool ray_tri(const Ray& r, const Triangle& t, double& tHit, double& b0, double& b1, double& b2){
	// Moller-Trumbore
	const Vec3 v0v1 = t.p1 - t.p0;
	const Vec3 v0v2 = t.p2 - t.p0;
	const Vec3 pvec = cross(r.dir, v0v2);
	const double det = dot(v0v1, pvec);
	if(std::fabs(det) < 1e-9) return false;
	const double invDet = 1.0 / det;
	const Vec3 tvec = r.origin - t.p0;
	const double u = dot(tvec, pvec) * invDet;
	if(u < 0.0 || u > 1.0) return false;
	const Vec3 qvec = cross(tvec, v0v1);
	const double v = dot(r.dir, qvec) * invDet;
	if(v < 0.0 || u + v > 1.0) return false;
	const double tt = dot(v0v2, qvec) * invDet;
	if(tt < r.tmin || tt > r.tmax) return false;
	tHit = tt;
	b0 = 1.0 - u - v; b1 = u; b2 = v;
	return true;
}

bool BVH::intersect(const std::vector<Triangle>& tris, const Ray& ray, int& hitIdx, double& tHit, Vec3& nHit, Vec2& uvHit, double& b0,double& b1,double& b2) const{
	hitIdx = -1;
	tHit = ray.tmax;
	std::stack<int> st;
	if(nodes.empty()) return false;
	st.push(0);
	bool hit = false;
	while(!st.empty()){
		int ni = st.top(); st.pop();
		double t0,t1;
		if(!ray_aabb(ray, nodes[(size_t)ni].box, t0, t1) || t0 > tHit) continue;
		const BVHNode& n = nodes[(size_t)ni];
		if(n.count > 0){
			for(int i=0;i<n.count;++i){
				int ti = indices[(size_t)(n.start + i)];
				double th, bb0,bb1,bb2;
				if(ray_tri(ray, tris[(size_t)ti], th, bb0,bb1,bb2)){
					if(th < tHit){
						tHit = th;
						hitIdx = ti;
						b0=bb0; b1=bb1; b2=bb2;
						const Triangle& t = tris[(size_t)ti];
						Vec3 nh = normalize(b0*t.n0 + b1*t.n1 + b2*t.n2);
						nHit = nh;
						uvHit = t.uv0*b0 + t.uv1*b1 + t.uv2*b2;
						hit = true;
					}
				}
			}
		}else{
			if(n.left>=0) st.push(n.left);
			if(n.right>=0) st.push(n.right);
		}
	}
	return hit;
}


