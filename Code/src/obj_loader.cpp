#include "obj_loader.h"
#include <fstream>
#include <sstream>
#include <cctype>
#include <map>
#include <iostream>
#include <algorithm>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

static bool file_exists(const char* p){
  if(!p) return false;
  FILE* f = std::fopen(p,"rb");
  if(!f) return false;
  std::fclose(f);
  return true;
}
static bool file_exists(const std::string& p){
  return file_exists(p.c_str());
}

static std::string dir_of(const std::string& path){
	size_t p = path.find_last_of("/\\");
	if(p==std::string::npos) return std::string(".");
	return path.substr(0,p);
}

static std::string trim(const std::string& s){
	size_t b = s.find_first_not_of(" \t\r\n");
	if(b==std::string::npos) return "";
	size_t e = s.find_last_not_of(" \t\r\n");
	return s.substr(b, e-b+1);
}

Vec3 Texture::sample(const Vec2& uv) const{
	if(!valid()) return {1,1,1};
	double u = uv.x - std::floor(uv.x);
	double v = uv.y - std::floor(uv.y);
	int ix = static_cast<int>(std::floor(u * (width-1)));
	int iy = static_cast<int>(std::floor((1.0 - v) * (height-1)));
	size_t idx = static_cast<size_t>((iy*width + ix) * 4);
	double r = rgba[idx+0]/255.0;
	double g = rgba[idx+1]/255.0;
	double b = rgba[idx+2]/255.0;
	return {r,g,b};
}

static int load_texture(const std::string& path, std::vector<Texture>& textures){
	int w=0,h=0,n=0;
	unsigned char* data = stbi_load(path.c_str(), &w, &h, &n, 4);
	if(!data){
		std::cerr<<"Warning: failed to load texture "<<path<<std::endl;
		return -1;
	}
	Texture t;
	t.width = w; t.height = h;
	t.rgba.assign(data, data + static_cast<size_t>(w*h*4));
	t.path = path;
	stbi_image_free(data);
	int id = static_cast<int>(textures.size());
	textures.emplace_back(std::move(t));
	return id;
}

static std::string strip_quotes(const std::string& s){
	if(s.size()>=2 && ((s.front()=='"' && s.back()=='"') || (s.front()=='\'' && s.back()=='\''))){
		return s.substr(1, s.size()-2);
	}
	return s;
}

static bool is_absolute_path(const std::string& p){
#if defined(_WIN32) || defined(_WIN64)
	return p.size()>1 && p[1]==':';
#else
	return !p.empty() && p[0]=='/';
#endif
}

static bool parse_mtl(const std::string& mtl_path, std::map<std::string,Material>& mats, std::vector<Texture>& textures){
	std::ifstream in(mtl_path);
	if(!in) return false;
	std::string base = dir_of(mtl_path);
	std::string line;
	Material cur;
	std::string cur_name;
	while(std::getline(in,line)){
		line = trim(line);
		if(line.empty() || line[0]=='#') continue;
		std::istringstream ss(line);
		std::string tok; ss>>tok;
		if(tok=="newmtl"){
			if(!cur_name.empty()) mats[cur_name]=cur;
			cur = Material();
			ss>>cur_name;
		}else if(tok=="Kd"){
			ss>>cur.kd.x>>cur.kd.y>>cur.kd.z;
		}else if(tok=="map_Kd"){
			// Remainder may contain options and/or quoted paths with spaces.
			std::string rest; std::getline(ss,rest);
			rest = trim(rest);
			// Tokenize by spaces but keep last token that looks like an image file.
			std::istringstream ts(rest);
			std::vector<std::string> toks;
			std::string tkn;
			while(ts>>tkn) toks.push_back(tkn);
			std::string candidate;
			for(int i=(int)toks.size()-1; i>=0; --i){
				std::string c = strip_quotes(toks[(size_t)i]);
				std::string lc = c;
				std::transform(lc.begin(), lc.end(), lc.begin(), [](unsigned char ch){ return (char)std::tolower(ch); });
				if(lc.size()>=4 &&
					(lc.rfind(".png") == lc.size()-4 ||
					 lc.rfind(".jpg") == lc.size()-4 ||
					 lc.rfind(".jpeg")== lc.size()-5 ||
					 lc.rfind(".tga") == lc.size()-4 ||
					 lc.rfind(".bmp") == lc.size()-4 ||
					 lc.rfind(".webp")== lc.size()-5))
				{
					candidate = c;
					break;
				}
			}
			if(!candidate.empty()){
				// If path contains directories with spaces, we likely captured just the rightmost filename.
				// If candidate is absolute, use as-is; else join with base.
				std::string full;
				if(is_absolute_path(candidate)){
					full = candidate;
				}else{
					// Try direct relative first
					full = base + "/" + candidate;
					// If file not found, also try basename-only (MTLs sometimes embed full original path).
					if(!file_exists(full.c_str())){
						size_t pos = candidate.find_last_of("/\\");
						std::string bn = (pos==std::string::npos)?candidate:candidate.substr(pos+1);
						full = base + "/" + bn;
					}
				}
				cur.tex_id = load_texture(full, textures);
			}
		}
	}
	if(!cur_name.empty()) mats[cur_name]=cur;
	return true;
}

bool load_obj_with_mtl(const std::string& obj_path, Mesh& out){
	out = Mesh();
	std::ifstream in(obj_path);
	if(!in) return false;
	const std::string base = dir_of(obj_path);
	std::vector<Vec3> V;
	std::vector<Vec3> N;
	std::vector<Vec2> UV;
	std::map<std::string,Material> name_to_mat;
	int current_mat_id = -1;

	std::string line;
	while(std::getline(in,line)){
		line = trim(line);
		if(line.empty() || line[0]=='#') continue;
		std::istringstream ss(line);
		std::string tok; ss>>tok;
		if(tok=="v"){
			Vec3 p; ss>>p.x>>p.y>>p.z; V.push_back(p);
		}else if(tok=="vn"){
			Vec3 n; ss>>n.x>>n.y>>n.z; N.push_back(n);
		}else if(tok=="vt"){
			Vec2 t; ss>>t.x>>t.y; UV.push_back(t);
		}else if(tok=="f"){
			// supports v/vt/vn or v//vn or v/vt
			std::vector<std::string> fstr;
			std::string vstr;
			while(ss>>vstr) fstr.push_back(vstr);
			if(fstr.size()<3) continue;
			auto parse_index = [&](const std::string& s, int& vi, int& ti, int& ni){
				vi=ti=ni=-1;
				int a=-1,b=-1,c=-1;
				size_t p0 = s.find('/');
				if(p0==std::string::npos){ a = std::stoi(s); }
				else{
					size_t p1 = s.find('/', p0+1);
					a = std::stoi(s.substr(0,p0));
					if(p1==std::string::npos){
						std::string tstr = s.substr(p0+1);
						if(!tstr.empty()) b = std::stoi(tstr);
					}else{
						std::string t1 = s.substr(p0+1, p1-p0-1);
						std::string t2 = s.substr(p1+1);
						if(!t1.empty()) b = std::stoi(t1);
						if(!t2.empty()) c = std::stoi(t2);
					}
				}
				auto fix=[&](int idx,int size){ return idx>0?idx-1:(idx<0?size+idx:idx); };
				if(a!=0) vi = fix(a,(int)V.size());
				if(b!=0) ti = fix(b,(int)UV.size());
				if(c!=0) ni = fix(c,(int)N.size());
			};
			auto emit_tri = [&](int ia,int ib,int ic){
				auto getv=[&](int idx){ return (idx>=0 && idx<(int)V.size())? V[idx] : Vec3(); };
				auto getn=[&](int idx){ return (idx>=0 && idx<(int)N.size())? N[idx] : Vec3(); };
				auto getu=[&](int idx){ return (idx>=0 && idx<(int)UV.size())? UV[idx] : Vec2(); };
				Triangle t;
				t.p0 = getv(ia); t.p1 = getv(ib); t.p2 = getv(ic);
				t.n0 = Vec3(); t.n1 = Vec3(); t.n2 = Vec3();
				t.uv0 = Vec2(); t.uv1 = Vec2(); t.uv2 = Vec2();
				t.material_id = current_mat_id;
				// try to parse corresponding vt/vn if given in original tokens
				// Note: This function assumes we were called after parsing the same tokens
				return t;
			};
			// We need to parse each corner properly, so redo parse per corner:
			struct Idx { int v=-1,t=-1,n=-1; };
			std::vector<Idx> idx;
			for(const auto& s : fstr){
				int vi,ti,ni; parse_index(s,vi,ti,ni);
				idx.push_back({vi,ti,ni});
			}
			auto add_triangle_from = [&](const Idx& a,const Idx& b,const Idx& c){
				Triangle t;
				t.p0 = V[a.v]; t.p1 = V[b.v]; t.p2 = V[c.v];
				if(a.n>=0 && b.n>=0 && c.n>=0){ t.n0=N[a.n]; t.n1=N[b.n]; t.n2=N[c.n]; }
				else{
					// fallback flat normal
					Vec3 n = normalize(cross(t.p1 - t.p0, t.p2 - t.p0));
					t.n0=t.n1=t.n2=n;
				}
				if(a.t>=0 && b.t>=0 && c.t>=0){ t.uv0=UV[a.t]; t.uv1=UV[b.t]; t.uv2=UV[c.t]; }
				t.material_id = current_mat_id;
				out.triangles.emplace_back(t);
			};
			// triangulate polygon fan
			for(size_t k=1;k+1<idx.size();++k){
				add_triangle_from(idx[0], idx[k], idx[k+1]);
			}
		}else if(tok=="usemtl"){
			std::string name; ss>>name;
			if(name_to_mat.find(name)==name_to_mat.end()){
				// create default if missing
				name_to_mat[name] = Material();
			}
			// ensure materials array contains this material, remember id
			Material m = name_to_mat[name];
			current_mat_id = (int)out.materials.size();
			out.materials.push_back(m);
		}else if(tok=="mtllib"){
			// Support spaces/quotes in mtllib path
			std::string rest; std::getline(ss, rest);
			rest = trim(strip_quotes(rest));
			if(rest.empty()){
				// fallback to single token if rest was empty
				ss.clear(); ss.str(line);
				std::string dummy; ss>>dummy; ss>>rest;
			}
			if(!rest.empty()){
				std::string full;
				if(is_absolute_path(rest)) full = rest;
				else full = base + "/" + rest;
				parse_mtl(full, name_to_mat, out.textures);
			}
			// Push loaded materials in deterministic order
			// We will map names on demand in `usemtl`
		}
	}
	// If some triangles referenced a material name loaded from MTL before usemtl,
	// they would have default id -1; keep them as default gray
	return !out.triangles.empty();
}


