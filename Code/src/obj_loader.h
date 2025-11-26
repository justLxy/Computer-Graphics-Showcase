#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include "cg_math.h"

struct Texture {
	int width = 0;
	int height = 0;
	std::vector<unsigned char> rgba; // 4 channels
	std::string path; // source filepath (for name-based material matching)
	bool valid() const { return width>0 && height>0 && !rgba.empty(); }
	// Sample nearest
	Vec3 sample(const Vec2& uv) const;
};

struct Triangle {
	Vec3 p0, p1, p2;
	Vec3 n0, n1, n2; // optional, can be zero
	Vec2 uv0, uv1, uv2; // optional
	int material_id = -1;
};

struct Material {
	Vec3 kd = {0.8,0.8,0.8};
	int tex_id = -1; // index into textures
};

struct Mesh {
	std::vector<Triangle> triangles;
	std::vector<Material> materials;
	std::vector<Texture> textures;
};

// Load a minimal subset of Wavefront OBJ with MTL and map_Kd
// Supports v/vt/vn, f (triangles/quads), usemtl, mtllib
bool load_obj_with_mtl(const std::string& obj_path, Mesh& out);


