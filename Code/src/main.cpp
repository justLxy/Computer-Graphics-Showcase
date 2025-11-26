#include <cstdio>
#include <string>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>
#include <vector>
#include <cstdlib>
#include <dirent.h>
#include <unordered_map>
#include <algorithm>
#include "cg_math.h"
#include "image.h"
#include "camera.h"
#include "obj_loader.h"
#include "bvh.h"
#include "scene.h"

static void ensure_dir(const char* path){
#ifdef _WIN32
  _mkdir(path);
#else
  mkdir(path, 0755);
#endif
}

static Vec3 sky_day_night(const Vec3& d, double night_factor){
	// Blend between daytime gradient and a deep blue‑purple night gradient with stars
	double t = 0.5*(d.y+1.0);
	Vec3 day = (1.0-t)*Vec3{0.9,0.85,0.8} + t*Vec3{0.6,0.75,0.95};
	// Night gradient (by elevation)
	double v = std::min(1.0, std::max(0.0, (d.y*0.5 + 0.5)));
	Vec3 top = {0.08, 0.10, 0.22};
	Vec3 mid = {0.12, 0.14, 0.30};
	Vec3 bot = {0.20, 0.18, 0.32};
	Vec3 nightGrad = (v<0.5) ? (top*(1.0 - v*2.0) + mid*(v*2.0)) : (mid*(1.0 - (v-0.5)*2.0) + bot*((v-0.5)*2.0));
	// Stars via hashed spherical coords
	auto hash13 = [](double x, double y, double z)->double{
		double s = std::sin(x*12.9898 + y*78.233 + z*37.719);
		double f = s*43758.5453;
		return f - std::floor(f);
	};
	double h = hash13(d.x, d.y, d.z);
	double star = (h > 0.995) ? ((h - 0.995) * 200.0) : 0.0; // sparse
	Vec3 stars = Vec3{0.8,0.85,1.0} * star;
	Vec3 night = clamp01(nightGrad + stars);
	return day*(1.0 - night_factor) + night*night_factor;
}

static bool file_exists(const char* p){
	FILE* f = std::fopen(p,"rb");
	if(!f) return false;
	std::fclose(f);
	return true;
}

static int get_thread_count(){
	const char* env = std::getenv("SHOW_THREADS");
	if(env){
		int n = std::atoi(env);
		if(n>0) return n;
	}
	unsigned int n = std::thread::hardware_concurrency();
	return n>0 ? static_cast<int>(n) : 1;
}

static std::string find_first_obj_in_dir(const char* dir){
	DIR* d = opendir(dir);
	if(!d) return {};
	std::string result;
	for(struct dirent* de = readdir(d); de; de = readdir(d)){
		const char* name = de->d_name;
		size_t len = std::strlen(name);
		if(len>4){
			const char* ext = name + (len - 4);
			if((ext[0]=='.') && ((ext[1]=='o'||ext[1]=='O')) && ((ext[2]=='b'||ext[2]=='B')) && ((ext[3]=='j'||ext[3]=='J'))){
				result = std::string(dir) + "/" + name;
				break;
			}
		}
	}
	closedir(d);
	return result;
}

static double env_double(const char* key, double defv){
	const char* s = std::getenv(key);
	if(!s) return defv;
	char* end=nullptr;
	double v = std::strtod(s,&end);
	if(end==s) return defv;
	return v;
}

static std::string env_string(const char* key, const std::string& defv){
	const char* s = std::getenv(key);
	return s ? std::string(s) : defv;
}

static std::string lower_str(std::string s){
	for(size_t i=0;i<s.size();++i){
		unsigned char c = (unsigned char)s[i];
		if(c>='A' && c<='Z') s[i] = (char)(c + ('a'-'A'));
	}
	return s;
}

static bool contains_ci(const std::string& hay, const std::string& needle){
	if(needle.empty()) return true;
	std::string h = lower_str(hay);
	std::string n = lower_str(needle);
	return h.find(n) != std::string::npos;
}

static Vec3 rotate_dir_rodrigues(const Vec3& v, const Vec3& axis_unit, double theta){
	const double c = std::cos(theta);
	const double s = std::sin(theta);
	// v_rot = v*c + (axis x v)*s + axis*(axis·v)*(1-c)
	Vec3 axv = cross(axis_unit, v);
	double adotv = dot(axis_unit, v);
	return v*c + axv*s + axis_unit*(adotv*(1.0 - c));
}

static Vec3 rotate_point_around_axis(const Vec3& p, const Vec3& pivot, const Vec3& axis_unit, double theta){
	Vec3 v = p - pivot;
	Vec3 vr = rotate_dir_rodrigues(v, axis_unit, theta);
	return pivot + vr;
}

// Simple Jacobi eigen decomposition for 3x3 symmetric matrix.
// Returns smallest-eigenvalue eigenvector.
static Vec3 smallest_eigenvector_sym3(const double A_in[3][3]){
	// Copy to mutable
	double A[3][3] = {
		{A_in[0][0], A_in[0][1], A_in[0][2]},
		{A_in[1][0], A_in[1][1], A_in[1][2]},
		{A_in[2][0], A_in[2][1], A_in[2][2]}
	};
	// Eigenvectors matrix (columns)
	double R[3][3] = {
		{1.0,0.0,0.0},
		{0.0,1.0,0.0},
		{0.0,0.0,1.0}
	};
	auto apply_rot = [&](int p, int q, double c, double s){
		// Rotate A (symmetric)
		for(int k=0;k<3;++k){
			if(k!=p && k!=q){
				double Apk = A[p][k];
				double Aqk = A[q][k];
				A[p][k] = A[k][p] = c*Apk - s*Aqk;
				A[q][k] = A[k][q] = s*Apk + c*Aqk;
			}
		}
		double App = A[p][p];
		double Aqq = A[q][q];
		double Apq = A[p][q];
		A[p][p] = c*c*App - 2.0*c*s*Apq + s*s*Aqq;
		A[q][q] = s*s*App + 2.0*c*s*Apq + c*c*Aqq;
		A[p][q] = A[q][p] = 0.0;
		// Rotate R
		for(int k=0;k<3;++k){
			double Rkp = R[k][p];
			double Rkq = R[k][q];
			R[k][p] = c*Rkp - s*Rkq;
			R[k][q] = s*Rkp + c*Rkq;
		}
	};
	for(int it=0; it<32; ++it){
		// Find largest off-diagonal
		int p=0,q=1;
		double maxv = std::fabs(A[0][1]);
		if(std::fabs(A[0][2]) > maxv){ maxv=std::fabs(A[0][2]); p=0; q=2; }
		if(std::fabs(A[1][2]) > maxv){ maxv=std::fabs(A[1][2]); p=1; q=2; }
		if(maxv < 1e-12) break;
		double tau = (A[q][q] - A[p][p]) / (2.0*A[p][q]);
		double t = (tau>=0.0 ? 1.0 : -1.0) / (std::fabs(tau) + std::sqrt(1.0 + tau*tau));
		double c = 1.0 / std::sqrt(1.0 + t*t);
		double s = t * c;
		apply_rot(p,q,c,s);
	}
	// Find index of smallest eigenvalue on diagonal of A
	double d0 = A[0][0], d1 = A[1][1], d2 = A[2][2];
	int idx = 0;
	if(d1 < d0) idx = 1, d0 = d1;
	if(d2 < d0) idx = 2;
	// Column idx of R
	Vec3 v = { R[0][idx], R[1][idx], R[2][idx] };
	return normalize(v);
}

int main(int argc, char** argv){

	// Prefer command-line override: ./showcase <path/to/model.obj>
	std::string obj_path;
	if(argc >= 2 && file_exists(argv[1])){
		obj_path = argv[1];
	}else{
		// Auto-discover: look for any .obj in common source folders
		const char* obj_dirs[] = {
			"../source",
			"Showcase/Code/source",
			"source",
			"../../Showcase/Code/source",
			"../../source",
			"."
		};
		for(const char* d : obj_dirs){
			std::string p = find_first_obj_in_dir(d);
			if(!p.empty()){ obj_path = p; break; }
		}
		if(obj_path.empty()){
			std::cerr<<"Failed to locate an OBJ file. Place your model in one of:\n";
			for(const char* d : obj_dirs) std::cerr<<"  "<<d<<"\n";
			std::cerr<<"Or run: ./showcase <path/to/model.obj>\n";
			return 1;
		}
	}

	// Load mesh
	Mesh mesh;
	if(!load_obj_with_mtl(obj_path, mesh)){
		std::cerr<<"Failed to load OBJ at "<<obj_path<<"\n";
		return 1;
	}
	std::cout<<"Loaded triangles: "<<mesh.triangles.size()<<", materials: "<<mesh.materials.size()<<", textures: "<<mesh.textures.size()<<"\n";

	Scene scene;
	scene.set_mesh(std::move(mesh));
	// Keep an immutable copy of the original mesh for per-frame deformations
	Mesh base_mesh = scene.mesh;
	// Lighting defaults (overridable via env)
	{
		const double amb = env_double("AMBIENT", 0.28); // moderate ambient
		scene.ambient = {amb, amb, amb};
		// Hemispheric skylight to brighten ground
		scene.sky_fill = env_double("SKY_FILL", 0.22);
		const double st_r = env_double("SKY_TINT_R", 0.6);
		const double st_g = env_double("SKY_TINT_G", 0.75);
		const double st_b = env_double("SKY_TINT_B", 0.95);
		scene.sky_tint = {st_r, st_g, st_b};
		// Emissive sky brightness controls
		scene.emissive_gain = env_double("EMISSIVE_GAIN", 1.8);
		scene.emissive_exposure_mul = env_double("EMISSIVE_EXPOSURE_MUL", 1.6);
		scene.emissive_linear = env_double("EMISSIVE_LINEAR", 0.0) > 0.5;
	}
	// Detect background board material(s): choose top-N largest triangles' material IDs
	{
		const int topN = static_cast<int>(env_double("EMISSIVE_TOP_TRI", 2.0));
		const double boost = env_double("EMISSIVE_BOOST", 0.9);
		const std::string em_match = env_string("EMISSIVE_TEX_MATCH", "sky");
		std::vector<std::pair<double,int>> triAreas; triAreas.reserve(scene.mesh.triangles.size());
		for(const auto& tri : scene.mesh.triangles){
			const Vec3 e1 = tri.p1 - tri.p0;
			const Vec3 e2 = tri.p2 - tri.p0;
			const double area = 0.5 * length(cross(e1,e2));
			triAreas.emplace_back(area, tri.material_id);
		}
		std::sort(triAreas.begin(), triAreas.end(), [](const auto& a, const auto& b){ return a.first > b.first; });
		// Preferred: filter by texture name match (e.g., "sky") to avoid selecting terrain
		std::vector<int> matched_ids;
		if(!em_match.empty()){
			for(size_t mi=0; mi<scene.mesh.materials.size(); ++mi){
				const Material& m = scene.mesh.materials[mi];
				if(m.tex_id>=0 && m.tex_id < (int)scene.mesh.textures.size()){
					const std::string& p = scene.mesh.textures[(size_t)m.tex_id].path;
					if(contains_ci(p, em_match)){
						matched_ids.push_back((int)mi);
					}
				}
			}
		}
		if(!matched_ids.empty()){
			scene.emissive_mat_ids = std::move(matched_ids);
		}else{
			// Fallback: use largest-triangle materials
			std::vector<int> ids;
			for(int i=0;i<(int)triAreas.size() && (int)ids.size()<std::max(1, topN); ++i){
				int mid = triAreas[i].second;
				if(mid>=0 && std::find(ids.begin(), ids.end(), mid)==ids.end()) ids.push_back(mid);
			}
			scene.emissive_mat_ids = std::move(ids);
		}
		scene.emissive_boost = boost;
	}
	// No window lights - rely on moonlight to illuminate the scene at night
	// Shading tunables
	scene.exposure = env_double("EXPOSURE", 1.35);
	scene.specular_strength = env_double("SPECULAR_STRENGTH", 0.05);  // Lower specular for matte wood
	scene.shadow_ambient = env_double("SHADOW_AMBIENT", 0.15);

	// Render settings
	const int W = 1920;
	const int H = 1080;
	const int total_frames = 480; // 8 seconds at 60fps
	const int title_frames = 150; // 2.5 seconds
	// Write frames to repository root Showcase/Piece/frames relative to build dir
	const char* out_dir = "../../Piece/frames";
	ensure_dir("../../Piece");
	ensure_dir(out_dir);

	// Compute scene bounds to center camera automatically
	Vec3 bbmin = { kInfinity, kInfinity, kInfinity };
	Vec3 bbmax = { -kInfinity, -kInfinity, -kInfinity };
	for(const auto& tri : scene.mesh.triangles){
		bbmin.x = std::min({bbmin.x, tri.p0.x, tri.p1.x, tri.p2.x});
		bbmin.y = std::min({bbmin.y, tri.p0.y, tri.p1.y, tri.p2.y});
		bbmin.z = std::min({bbmin.z, tri.p0.z, tri.p1.z, tri.p2.z});
		bbmax.x = std::max({bbmax.x, tri.p0.x, tri.p1.x, tri.p2.x});
		bbmax.y = std::max({bbmax.y, tri.p0.y, tri.p1.y, tri.p2.y});
		bbmax.z = std::max({bbmax.z, tri.p0.z, tri.p1.z, tri.p2.z});
	}
	const Vec3 center = (bbmin + bbmax) / 2.0;
	const double diag = length(bbmax - bbmin);
	// Tunable camera scales (can override with env: CAM_RADIUS_SCALE, CAM_HEIGHT_SCALE)
	const double scene_height = (bbmax.y - bbmin.y);
	// Aim a bit above ground rather than bbox center so the camera doesn't float in the sky
	Vec3 look_at = center;
	look_at.y = bbmin.y + std::max( scene_height*0.3, diag*0.05 );
	// Fixed heading (no rotation) around Y; dolly-in toward the scene
	// Default camera parameters (tuned for optimal view)
	const double angle_deg = env_double("CAM_ANGLE_DEG", 0.0);
	const double alpha = angle_deg * PI / 180.0;
	// Auto front direction using area-weighted average normal (front of largest surface e.g., sky plane)
	Vec3 areaN = {0,0,0};
	for(const auto& tri : scene.mesh.triangles){
		const Vec3 e1 = tri.p1 - tri.p0;
		const Vec3 e2 = tri.p2 - tri.p0;
		const Vec3 an = cross(e1,e2); // area-weighted normal (2*area)
		areaN += an;
	}
	Vec3 frontN = normalize(areaN);
	Vec3 frontXZ = {frontN.x, 0.0, frontN.z};
	if(length(frontXZ) < 1e-8){ frontXZ = {0.0,0.0,1.0}; }
	// Rotate around Y by alpha to get a slightly off-axis cinematic angle
	const auto rotY = [&](const Vec3& v)->Vec3{
		return { v.x*std::cos(alpha) + v.z*std::sin(alpha), 0.0, -v.x*std::sin(alpha) + v.z*std::cos(alpha) };
	};
	const Vec3 heading = normalize(rotY(normalize(frontXZ)));
	const double start_dist = env_double("CAM_START_DIST", 1973.68);
	const double end_dist   = env_double("CAM_END_DIST",   1682.45);
	const double eye_height = env_double("CAM_HEIGHT_ABS", -16.0);
	// Optional pan/strafe to nudge framing left/right
	const Vec3 left_vec = normalize(Vec3(-heading.z, 0.0, heading.x));
	const double cam_pan   = env_double("CAM_PAN", 0.0);    // +: pan/look to left
	const double cam_strafe= env_double("CAM_STRAFE", 0.0); // +: move camera left
	look_at = look_at + left_vec * cam_pan;
	// World-unit fine control along camera local axes
	const double pos_left     = env_double("CAM_POS_LEFT", 0.0);
	const double pos_forward  = env_double("CAM_POS_FORWARD", 350.0);
	const double look_left    = env_double("CAM_LOOK_LEFT", 0.0);
	const double look_forward = env_double("CAM_LOOK_FORWARD", 50.0);

	// Lock sun direction to initial camera view (frame 0) with yaw/pitch offsets
	const bool sun_lock_at_start = env_double("SUN_LOCK_AT_START", 1.0) > 0.5;
	Vec3 fixed_sun_dir = {0.0,-1.0,0.0};
	if(sun_lock_at_start){
		const Vec3 forward_vec0 = normalize(Vec3(heading.x,0.0,heading.z));
		Vec3 camPos0 = { center.x + heading.x*start_dist, eye_height, center.z + heading.z*start_dist };
		camPos0 = camPos0 + left_vec*cam_strafe + left_vec*pos_left + forward_vec0*pos_forward;
		Vec3 target0 = look_at + left_vec*look_left + forward_vec0*look_forward;
		const Vec3 camF0 = normalize(target0 - camPos0);
		Vec3 camR0 = cross(camF0, Vec3{0,1,0}); if(length(camR0) < 1e-8) camR0 = {1,0,0}; camR0 = normalize(camR0);
		const Vec3 camU0 = normalize(cross(camR0, camF0));
		const double yawDeg   = env_double("SUN_OFFSET_YAW_DEG", 0.0);
		const double pitchDeg = env_double("SUN_OFFSET_PITCH_DEG", -8.0);
		const double yaw = yawDeg * PI / 180.0;
		const double pitch = pitchDeg * PI / 180.0;
		const double cp = std::cos(pitch), sp = std::sin(pitch);
		const double cy = std::cos(yaw),   sy = std::sin(yaw);
		Vec3 dirLocal0 = camF0 * (cp*cy) + camR0 * (cp*sy) + camU0 * (sp);
		fixed_sun_dir = normalize(dirLocal0);
	}

	// Windmill rotation setup (selection by texture name substring and/or bounding box)
	const bool wind_enable = env_double("WIND_ENABLE", 1.0) > 0.5;
	const std::string wind_tex_match = env_string("WIND_TEX_MATCH", "wing");
	const Vec3 wind_box_min = {
		env_double("WIND_BOX_MIN_X", bbmin.x),
		env_double("WIND_BOX_MIN_Y", bbmin.y),
		env_double("WIND_BOX_MIN_Z", bbmin.z)
	};
	const Vec3 wind_box_max = {
		env_double("WIND_BOX_MAX_X", bbmax.x),
		env_double("WIND_BOX_MAX_Y", bbmax.y),
		env_double("WIND_BOX_MAX_Z", bbmax.z)
	};
	const auto in_wind_box = [&](const Vec3& p)->bool{
		return (p.x>=wind_box_min.x && p.x<=wind_box_max.x) &&
		       (p.y>=wind_box_min.y && p.y<=wind_box_max.y) &&
		       (p.z>=wind_box_min.z && p.z<=wind_box_max.z);
	};
	struct WindGroup { std::vector<int> tri_indices; Vec3 pivot; Vec3 axis; int tex_id=-1; };
	std::vector<WindGroup> wind_groups;
	if(wind_enable){
		// Group triangles by texture id so each fan set rotates about its own pivot
		std::unordered_map<int,int> tex_to_group;
		for(size_t i=0;i<base_mesh.triangles.size();++i){
			const Triangle& t = base_mesh.triangles[i];
			const Vec3 ctri = (t.p0 + t.p1 + t.p2) / 3.0;
			if(!in_wind_box(ctri)) continue;
			int tex_id = -1;
			if(t.material_id>=0 && t.material_id < (int)base_mesh.materials.size()){
				const Material& m = base_mesh.materials[(size_t)t.material_id];
				if(m.tex_id>=0 && m.tex_id < (int)base_mesh.textures.size()){
					const std::string& path = base_mesh.textures[(size_t)m.tex_id].path;
					if(!wind_tex_match.empty() && !contains_ci(path, wind_tex_match)) continue;
					tex_id = m.tex_id;
				}else{
					// no texture; if matching string required, skip
					if(!wind_tex_match.empty()) continue;
				}
			}else{
				if(!wind_tex_match.empty()) continue;
			}
			int gidx;
			auto it = tex_to_group.find(tex_id);
			if(it==tex_to_group.end()){
				gidx = (int)wind_groups.size();
				tex_to_group[tex_id] = gidx;
				WindGroup g; g.tex_id = tex_id;
				wind_groups.push_back(g);
			}else{
				gidx = it->second;
			}
			wind_groups[(size_t)gidx].tri_indices.push_back((int)i);
		}
		// Compute each group's pivot as centroid of its triangles' vertices
		for(auto& g : wind_groups){
			if(g.tri_indices.empty()) continue;
			Vec3 acc = {0,0,0}; size_t cnt = 0;
			std::vector<Vec3> pts; pts.reserve(g.tri_indices.size()*3);
			for(int ti : g.tri_indices){
				const Triangle& t = base_mesh.triangles[(size_t)ti];
				acc += t.p0; acc += t.p1; acc += t.p2; cnt += 3;
				pts.push_back(t.p0); pts.push_back(t.p1); pts.push_back(t.p2);
			}
			g.pivot = cnt>0 ? acc / (double)cnt : center;
			// Compute best-fit axis (smallest-variance eigenvector of covariance)
			if(!pts.empty()){
				double C[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
				for(const Vec3& p : pts){
					Vec3 d = p - g.pivot;
					C[0][0] += d.x*d.x; C[0][1] += d.x*d.y; C[0][2] += d.x*d.z;
					C[1][1] += d.y*d.y; C[1][2] += d.y*d.z;
					C[2][2] += d.z*d.z;
				}
				// Symmetric fill
				C[1][0] = C[0][1]; C[2][0] = C[0][2]; C[2][1] = C[1][2];
				Vec3 axis_auto = smallest_eigenvector_sym3(C);
				if(length(axis_auto) < 1e-8) axis_auto = {0,0,1};
				g.axis = axis_auto;
			}else{
				g.axis = {0,0,1};
			}
		}
	}
	// Axis and speed
	Vec3 wind_axis = normalize(Vec3{
		env_double("WIND_AXIS_X", 0.0),
		env_double("WIND_AXIS_Y", 0.0),
		env_double("WIND_AXIS_Z", 1.0)
	});
	if(length(wind_axis) < 1e-8) wind_axis = {0,0,1};
	const double wind_rpm = env_double("WIND_RPM", 12.0); // nice slow spin
	const double theta0   = env_double("WIND_THETA0_DEG", 0.0) * PI / 180.0;
	const double fps      = env_double("FPS", 60.0);
	const bool axis_auto  = env_double("WIND_AXIS_AUTO", 1.0) > 0.5;

	// Cloud drift controls for emissive background board
	const double cloud_spd_u = env_double("CLOUD_SPD_U", 0.03);
	const double cloud_spd_v = env_double("CLOUD_SPD_V", 0.0);
	const double cloud_wobble_amp = env_double("CLOUD_WOBBLE_AMP_V", 0.02);
	const double cloud_wobble_freq = env_double("CLOUD_WOBBLE_FREQ", 0.2);

	// Accumulated effective spin time (integrates speed scaling so blades don't "rewind")
	double accumulated_spin_time = 0.0;

	for(int f=0; f<total_frames; ++f){
		Image img(W,H);
		img.fill({0,0,0});

		double t = (double)f / std::max(1.0, (double)(total_frames - 1));
		double time_sec = (double)f / std::max(1.0, fps);
		// animate emissive UV offset (cloud drift)
		scene.emissive_uv_offset = {
			cloud_spd_u * time_sec,
			cloud_spd_v * time_sec + cloud_wobble_amp * std::sin(2.0*PI*cloud_wobble_freq*time_sec)
		};
		// Day -> Night blend factor across [0.5 .. 0.9]
		auto smoothstep01 = [](double a, double b, double x)->double{
			double tt = (x - a) / std::max(1e-9, (b - a));
			tt = std::min(1.0, std::max(0.0, tt));
			return tt*tt*(3.0 - 2.0*tt);
		};
		const double night_factor = smoothstep01(0.5, 0.9, t);
		scene.night_factor = night_factor;
		// Spin weight: 1 at day, 0 at night; integrated to avoid snapping/rewind
		const double dt = 1.0 / std::max(1.0, fps);
		double spin_weight = 1.0 - night_factor;
		if(spin_weight < 0.0) spin_weight = 0.0;
		if(spin_weight > 1.0) spin_weight = 1.0;
		accumulated_spin_time += dt * spin_weight;
		// Rebuild animated geometry for this frame (rotate windmill blades)
		if(wind_enable && !wind_groups.empty()){
			scene.mesh.triangles = base_mesh.triangles;
			// Use accumulated_spin_time so rotation smoothly slows and fully stops at night
			const double theta = theta0 + (2.0*PI * (wind_rpm/60.0)) * accumulated_spin_time;
			for(const auto& g : wind_groups){
				const Vec3 axis_use = axis_auto ? g.axis : wind_axis;
				for(int ti : g.tri_indices){
					Triangle& tr = scene.mesh.triangles[(size_t)ti];
					tr.p0 = rotate_point_around_axis(tr.p0, g.pivot, axis_use, theta);
					tr.p1 = rotate_point_around_axis(tr.p1, g.pivot, axis_use, theta);
					tr.p2 = rotate_point_around_axis(tr.p2, g.pivot, axis_use, theta);
					// Rotate normals as directions
					tr.n0 = normalize(rotate_dir_rodrigues(tr.n0, axis_use, theta));
					tr.n1 = normalize(rotate_dir_rodrigues(tr.n1, axis_use, theta));
					tr.n2 = normalize(rotate_dir_rodrigues(tr.n2, axis_use, theta));
				}
			}
			// Rebuild BVH for deformed mesh
			scene.bvh.build(scene.mesh.triangles);
		}else if(f==0){
			// Ensure BVH is consistent with (base) mesh for static case
			scene.mesh.triangles = base_mesh.triangles;
			scene.bvh.build(scene.mesh.triangles);
		}
		// Camera dolly forward (no rotation)
		const double dist = start_dist + (end_dist - start_dist) * t;
		Vec3 camPos = { center.x + heading.x*dist, eye_height, center.z + heading.z*dist };
		camPos = camPos + left_vec * cam_strafe;
		Vec3 target = look_at;
		Camera cam;
		// Apply strong, world-unit offsets along local axes
		const Vec3 forward_vec = normalize(Vec3(heading.x,0.0,heading.z));
		camPos = camPos + left_vec*cam_strafe + left_vec*pos_left + forward_vec*pos_forward;
		target = target + left_vec*look_left + forward_vec*look_forward;
		cam.eye = camPos;
		cam.target = target;
		cam.up = {0,1,0};
		cam.fov_y_deg = env_double("CAM_FOV", 35.0);
		cam.aspect = (double)W/(double)H;

		// Sun lighting. By default it follows the camera so the scene is front‑lit
		// like a key light behind the viewer. Override with:
		//  SUN_FOLLOW_CAMERA=0  to use fixed azimuth/elevation below
		//  SUN_OFFSET_YAW_DEG / SUN_OFFSET_PITCH_DEG to nudge relative to camera
		//  SUN_INTENSITY, SUN_AZIMUTH_DEG, SUN_ELEV_DEG for fixed mode
		const double sunIntensityBase = env_double("SUN_INTENSITY", 4.0);
		const bool followCam = env_double("SUN_FOLLOW_CAMERA", 1.0) > 0.5;
		// Determine base azimuth for the sun
		double baseAz = 0.0; // radians
		if(sun_lock_at_start){
			baseAz = std::atan2(fixed_sun_dir.z, fixed_sun_dir.x);
		}else if(followCam){
			// Use current camera orientation plus offsets to choose azimuth (yaw), but drive elevation below
			const Vec3 camF = normalize(target - camPos);
			Vec3 camR = cross(camF, Vec3{0,1,0}); if(length(camR) < 1e-8) camR = {1,0,0}; camR = normalize(camR);
			const Vec3 camU = normalize(cross(camR, camF));
			const double yawDeg   = env_double("SUN_OFFSET_YAW_DEG", 0.0);
			const double pitchDeg = env_double("SUN_OFFSET_PITCH_DEG", -8.0);
			const double yaw = yawDeg * PI / 180.0;
			const double pitch = pitchDeg * PI / 180.0;
			const double cp = std::cos(pitch), sp = std::sin(pitch);
			const double cy = std::cos(yaw),   sy = std::sin(yaw);
			Vec3 dirLocal = camF * (cp*cy) + camR * (cp*sy) + camU * (sp);
			baseAz = std::atan2(dirLocal.z, dirLocal.x);
		}else{
			baseAz = env_double("SUN_AZIMUTH_DEG", -30.0) * PI / 180.0;
		}
		// Smoothly lower the sun and raise the moon across night_factor
		const double sunElDayDeg    = env_double("SUN_ELEV_DEG", 32.0);
		const double sunSetDeg      = env_double("SUN_SET_ELEV_DEG", -6.0);     // below horizon at night
		const double moonElDayDeg   = env_double("MOON_ELEV_DAY_DEG", -10.0);   // below horizon at day
		const double moonElNightDeg = env_double("MOON_ELEV_NIGHT_DEG", 25.0);  // above horizon at night
		const double sunElDeg  = sunElDayDeg*(1.0 - night_factor) + sunSetDeg*night_factor;
		const double moonElDeg = moonElDayDeg*(1.0 - night_factor) + moonElNightDeg*night_factor;
		const double sunEl  = sunElDeg * PI / 180.0;
		const double moonEl = moonElDeg * PI / 180.0;
		// Place sun slightly to the left, moon to the right (relative to camera/base azimuth)
		const double sepDeg = env_double("DUAL_LIGHT_SEP_DEG", 16.0); // angular separation
		const double sepRad = sepDeg * PI / 180.0;
		const double sunAz  = baseAz - sepRad; // left
		const double moonAz = baseAz + sepRad; // right
		// y is negative because shade() uses L = -dir
		Vec3 sunDir  = normalize(Vec3(std::cos(sunAz)*std::cos(sunEl),  -std::sin(sunEl),  std::sin(sunAz)*std::cos(sunEl)));
		Vec3 moonDir = normalize(Vec3(std::cos(moonAz)*std::cos(moonEl),-std::sin(moonEl), std::sin(moonAz)*std::cos(moonEl)));
		scene.sun.dir = sunDir;
		// Warm the color as it sets
		Vec3 sunDayCol = {1.0, 0.98, 0.96};
		Vec3 sunDuskCol = {1.0, 0.75, 0.45};
		scene.sun.color = sunDayCol*(1.0 - night_factor) + sunDuskCol*night_factor;
		scene.sun.intensity = sunIntensityBase * (1.0 - night_factor);
		// Moonlight (bright and cool, main light source at night)
		scene.enable_moon = (night_factor > 0.0);
		scene.moon.color = {0.65, 0.75, 0.95};  // cool blue-white
		// Moon rises as night comes in; opposite azimuth to the sun
		scene.moon.dir = moonDir;
		scene.moon.intensity = 2.8 * night_factor;  // moderate moon brightness
		// Environment fills and exposure crossfade
		scene.ambient = {
			0.28*(1.0 - night_factor) + 0.08*night_factor,
			0.28*(1.0 - night_factor) + 0.09*night_factor,
			0.28*(1.0 - night_factor) + 0.12*night_factor
		};
		scene.sky_fill = 0.22*(1.0 - night_factor) + 0.25*night_factor;  // moderate sky fill at night
		scene.sky_tint = {
			0.6*(1.0 - night_factor) + 0.40*night_factor,
			0.75*(1.0 - night_factor) + 0.50*night_factor,
			0.95*(1.0 - night_factor) + 0.70*night_factor
		};
		scene.exposure = 1.35*(1.0 - night_factor) + 1.55*night_factor;  // moderate exposure for night

		// Simple 1 spp
		{
			const int T = std::max(1, get_thread_count());
			const int rows_per = (H + T - 1) / T;
			std::vector<std::thread> workers;
			workers.reserve(T);
			for(int tIdx=0; tIdx<T; ++tIdx){
				const int y0 = tIdx * rows_per;
				const int y1 = std::min(H, y0 + rows_per);
				if(y0 >= y1) continue;
				workers.emplace_back([&, y0, y1](){
					for(int y=y0; y<y1; ++y){
						for(int x=0; x<W; ++x){
							Ray r = cam.generate_ray(x,y,W,H);
							int hitIdx; double tHit; Vec3 nHit; Vec2 uvHit; double b0,b1,b2;
							if(scene.bvh.intersect(scene.mesh.triangles, r, hitIdx, tHit, nHit, uvHit, b0,b1,b2)){
								Vec3 color = scene.shade(r, hitIdx, nHit, uvHit, b0,b1,b2);
								img.set(x,y,color);
							}else{
								img.set(x,y, sky_day_night(r.dir, night_factor));
							}
						}
					}
				});
			}
			for(auto& th : workers) th.join();
		}

		// Title frames overlay (auto-sized box; uppercase font supported)
		if(f < title_frames){
			const std::string t1 = "THE WHISPERING MILLS";
			const std::string t2 = "XUANYI LYU";
			const std::string t3 = "CSC317 FALL 2025";
			const int charW = 8;
			const int charH = 8;
			const int padX = 16;
			const int padY = 12;
			const int gapY = 8;
			const int w1 = charW * static_cast<int>(t1.size());
			const int w2 = charW * static_cast<int>(t2.size());
			const int w3 = charW * static_cast<int>(t3.size());
			const int boxW = padX*2 + std::max({w1,w2,w3});
			const int boxH = padY*2 + charH*3 + gapY*2;
			const int x0 = 40;
			const int y0 = 40;
			img.overlay_rect(x0, y0, boxW, boxH, {0,0,0}, 0.55);
			int tx = x0 + padX;
			int ty = y0 + padY;
			img.draw_text_8x8(tx, ty, t1, {1,1,1});
			ty += charH + gapY;
			img.draw_text_8x8(tx, ty, t2, {0.9,0.9,0.9});
			ty += charH + gapY;
			img.draw_text_8x8(tx, ty, t3, {0.9,0.9,0.9});
		}

		char name[256];
		std::snprintf(name, sizeof(name), "%s/frame_%04d.ppm", out_dir, f);
		img.write_ppm(name);
		std::cout<<"Wrote "<<name<<"\n";
	}

	std::cout<<"Rendering complete. To make a video (requires ffmpeg):\n";
	std::cout<<"ffmpeg -y -framerate 60 -i ../../Piece/frames/frame_%04d.ppm -pix_fmt yuv420p ../../Piece/piece.mp4\n";
	return 0;
}


