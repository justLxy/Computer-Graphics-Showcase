#include "scene.h"
#include <algorithm>

bool Scene::trace_shadow(const Vec3& p, const Vec3& lightDir) const{
	Ray sr;
	sr.origin = p + 1e-4 * lightDir;
	sr.dir = lightDir;
	for(int step=0; step<64; ++step){
		int hit; double t; Vec3 n; Vec2 uv; double b0,b1,b2;
		if(!bvh.intersect(mesh.triangles, sr, hit, t, n, uv, b0,b1,b2)) return false;
		// Ignore emissive/background materials when casting shadows so the sky plate
		// does not receive or cast shadows.
		const Triangle& tri = mesh.triangles[(size_t)hit];
		if(!emissive_mat_ids.empty() &&
		   std::find(emissive_mat_ids.begin(), emissive_mat_ids.end(), tri.material_id) != emissive_mat_ids.end()){
			// Advance past this triangle and continue searching
			sr.origin = sr.origin + sr.dir * (t + 1e-4);
			continue;
		}
		return true;
	}
	// Safety: if too many steps, assume occluded
	return true;
}

Vec3 Scene::shade(const Ray& ray, int triIdx, const Vec3& n, const Vec2& uv, double b0,double b1,double b2) const{
	const Triangle& t = mesh.triangles[(size_t)triIdx];
	Vec3 kd = {0.7,0.7,0.7};
	int tex_id = -1;
	if(t.material_id>=0 && t.material_id < (int)mesh.materials.size()){
		const Material& m = mesh.materials[(size_t)t.material_id];
		kd = m.kd;
		if(m.tex_id>=0 && m.tex_id < (int)mesh.textures.size()){
			tex_id = m.tex_id;
			kd = mesh.textures[(size_t)tex_id].sample(uv);
		}
	}
	const bool isEmissive = !emissive_mat_ids.empty() &&
		(std::find(emissive_mat_ids.begin(), emissive_mat_ids.end(), t.material_id) != emissive_mat_ids.end());
	// Lighting
	Vec3 V = normalize(-ray.dir);
	Vec3 N = normalize(n);
	Vec3 col = ambient * kd;
	// If this is a background/emissive plate (e.g., sky board), render it unlit and
	// ignore occluders to prevent scene silhouettes from appearing on the sky.
	if(isEmissive){
		// Identify if this emissive is the sky board (texture path contains "sky")
		bool isSkyBoard = false;
		if(t.material_id>=0 && t.material_id < (int)mesh.materials.size()){
			const Material& m = mesh.materials[(size_t)t.material_id];
			if(m.tex_id>=0 && m.tex_id < (int)mesh.textures.size()){
				const std::string& pth = mesh.textures[(size_t)m.tex_id].path;
				// crude case-insensitive contains
				for(char c : pth){
					if(c>='A' && c<='Z'){
						// lower copy on the fly
					}
				}
				std::string low = pth;
				for(size_t i=0;i<low.size();++i){ unsigned char c=(unsigned char)low[i]; if(c>='A'&&c<='Z') low[i]=(char)(c+('a'-'A')); }
				isSkyBoard = (low.find("sky") != std::string::npos);
			}
		}
		Vec3 e;
		if(isSkyBoard){
			// Procedural day->night blend for sky board to realize blue‑purple night gradient with stars.
			// uv.y assumed vertical; build gradient top (space) to bottom (horizon).
			double v = std::min(1.0, std::max(0.0, uv.y));
			// Day texture (scrolling) for daytime portion
			Vec3 dayCol = kd;
			if(tex_id>=0){
				Vec2 uv2 = uv + emissive_uv_offset;
				dayCol = mesh.textures[(size_t)tex_id].sample(uv2);
			}
			// Night gradient + stars
			Vec3 top = {0.08, 0.10, 0.22};   // deep indigo
			Vec3 mid = {0.12, 0.14, 0.30};   // cobalt
			Vec3 bot = {0.20, 0.18, 0.32};   // bluish purple near horizon
			Vec3 nightGrad = (v<0.5) ? (top*(1.0 - v*2.0) + mid*(v*2.0)) : (mid*(1.0 - (v-0.5)*2.0) + bot*((v-0.5)*2.0));
			// Star field via hashed uv (use ORIGINAL uv, not offset) with smooth kernel (no flicker)
			auto hash11 = [](double x)->double{
				double s = std::sin(x*12.9898 + 78.233);
				double f = s*43758.5453;
				return f - std::floor(f);
			};
			auto hash12 = [&](double x, double y)->double{
				return hash11(x*127.1 + y*311.7);
			};
			auto hash22 = [&](double x, double y)->Vec2{
				double h1 = hash12(x, y);
				double h2 = hash12(x+19.19, y+73.73);
				return Vec2{h1, h2};
			};
			// Wrap uv to [0,1) to match texture tiling
			Vec2 uvw = { uv.x - std::floor(uv.x), uv.y - std::floor(uv.y) };
			const double cellScale = 180.0; // number of star cells per UV tile
			double gx = std::floor(uvw.x * cellScale);
			double gy = std::floor(uvw.y * cellScale);
			double fx = uvw.x * cellScale - gx;
			double fy = uvw.y * cellScale - gy;
			// One random per cell decides if a star exists
			double exist = hash12(gx, gy);
			double star = 0.0;
			if(exist > 0.995){ // density (lower threshold -> more stars)
				// Random center inside the cell
				Vec2 c = hash22(gx+0.5, gy+0.5);
				// Gaussian radial kernel for anti-aliased star (static w.r.t. UV)
				double dx = fx - c.x;
				double dy = fy - c.y;
				double r2 = dx*dx + dy*dy;
				const double radius = 0.16; // in cell units
				const double sigma2 = (radius*radius)*0.5;
				double core = std::exp(-r2 / std::max(1e-9, sigma2));
				// Slight bloom by adding a softer halo
				const double haloRadius = radius*2.2;
				const double haloSigma2 = (haloRadius*haloRadius)*0.5;
				double halo = std::exp(-r2 / std::max(1e-9, haloSigma2)) * 0.25;
				star = std::min(1.0, core + halo);
			}
			Vec3 starsCol = Vec3{0.8,0.85,1.0} * star;
			Vec3 nightCol = clamp01(nightGrad + starsCol);
			// Blend by night_factor
			Vec3 blended = dayCol * (1.0 - night_factor) + nightCol * night_factor;
			const double base = 0.7;
			e = blended * (base + std::max(0.0, emissive_boost)) * std::max(0.0, emissive_gain);
		}else{
			// Non-sky emissive (if any): use texture with scroll
			if(tex_id>=0){
				Vec2 uv2 = uv + emissive_uv_offset;
				kd = mesh.textures[(size_t)tex_id].sample(uv2);
			}
			const double base = 0.7; // self-illumination base
			e = kd * (base + std::max(0.0, emissive_boost)) * std::max(0.0, emissive_gain);
		}
		if(emissive_linear){
			return clamp01(e);
		}else{
			// Filmic tone mapping (Hable/Uncharted2) with white point 11.2
			auto uncharted2 = [](double x)->double{
				const double A=0.15, B=0.50, C=0.10, D=0.20, E=0.02, F=0.30;
				return ((x*(A*x + C*B) + D*E) / (x*(A*x + B) + D*F)) - E/F;
			};
			const double white = 11.2;
			const double whiteScale = 1.0 / uncharted2(white);
			Vec3 x = e * std::max(0.0, exposure * emissive_exposure_mul);
			Vec3 mapped = {
				uncharted2(x.x) * whiteScale,
				uncharted2(x.y) * whiteScale,
				uncharted2(x.z) * whiteScale
			};
			return clamp01(mapped);
		}
	}
	// Shadow test
	// Compute actual hitPoint
	const Vec3 hp = t.p0*b0 + t.p1*b1 + t.p2*b2;
	auto add_dir_light = [&](const DirectionalLight& dlight){
		if(dlight.intensity <= 0.0) return;
		Vec3 Ld = normalize(-dlight.dir);
		Vec3 lcol = dlight.color * dlight.intensity;
		const bool inShadow = trace_shadow(hp, Ld);
		if(!inShadow){
			double ndotl = std::max(0.0, dot(N, Ld));
			Vec3 diffuse = kd * ndotl;
			Vec3 H = normalize(Ld + V);
			double ndoth = std::max(0.0, dot(N, H));
			double spec = std::pow(ndoth, 32.0);
			Vec3 specular = Vec3{1.0,1.0,1.0} * spec * specular_strength;
			col += (diffuse + specular) * lcol;
		}else{
			col += kd * shadow_ambient;
		}
	};
	// Sun (day, warming as it sets)
	// If sun and moon align (same direction), merge into a single key light to avoid double highlights
	if(enable_moon){
		Vec3 sunLd = normalize(-sun.dir);
		Vec3 moonLd = normalize(-moon.dir);
		double align = dot(sunLd, moonLd);
		if(align > 0.999){
			DirectionalLight key = sun;
			// Fold both lights into one by summing their luma contributions
			Vec3 lcol = sun.color * sun.intensity + moon.color * moon.intensity;
			key.color = lcol;
			key.intensity = 1.0; // lcol already encodes total intensity
			add_dir_light(key);
		}else{
			add_dir_light(sun);
			add_dir_light(moon);
		}
	}else{
		add_dir_light(sun);
	}
	// Hemispheric skylight fill (brightens upward-facing surfaces)
	if(sky_fill > 0.0){
		double hemi = std::max(0.0, N.y); // 0 for downward, 1 for up
		col += (kd * sky_tint) * (sky_fill * hemi);
	}
	// Emissive window glow if UV falls inside any configured window rectangle
	if(night_factor > 0.0 && tex_id >= 0 && !window_masks.empty()){
		// wrap uv like texture sampling
		double uu = uv.x - std::floor(uv.x);
		double vv = uv.y - std::floor(uv.y);
		const double pad = 0.01; // generous padding to catch edges
		// only glow if the base texel is dark (window interior)
		double lum = 0.2126*kd.x + 0.7152*kd.y + 0.0722*kd.z;
		if(lum < 0.18){
		for(const auto& wm : window_masks){
			if(wm.tex_id != tex_id) continue;
			if(uu >= (wm.uv_min.x - pad) && uu <= (wm.uv_max.x + pad) &&
			   vv >= (wm.uv_min.y - pad) && vv <= (wm.uv_max.y + pad)){
				double w = night_factor;
				col += wm.color * (wm.strength * w);
			}
		}
		}
	}
	// Warm window point lights at night (turn on progressively)
	if(!window_lights.empty() && night_factor > 0.0){
		for(const auto& pl : window_lights){
			// activation weight
			double w = 0.0;
			if(night_factor >= pl.turn_on){
				// smooth ramp in a short interval
				double t0 = pl.turn_on;
				double t1 = std::min(1.0, pl.turn_on + 0.15);
				if(night_factor >= t1) w = 1.0;
				else{
					double a = (night_factor - t0) / std::max(1e-6, (t1 - t0));
					// smoothstep
					w = a*a*(3.0 - 2.0*a);
				}
			}
			if(w <= 0.0) continue;
			Vec3 toL = pl.pos - hp;
			double dist = length(toL);
			if(dist <= 1e-6) continue;
			Vec3 Lp = toL / dist;
			// simple line-of-sight shadow
			bool occ = trace_shadow_to(hp, Lp, dist - 1e-3);
			if(occ) continue;
			// quadratic attenuation with soft radius
			double r = std::max(1e-3, pl.radius);
			double atten = 1.0 / (1.0 + (dist*dist)/(r*r));
			double ndotl = std::max(0.0, dot(N, Lp));
			Vec3 diff = kd * ndotl;
			Vec3 H = normalize(Lp + V);
			double ndoth = std::max(0.0, dot(N, H));
			double spec = std::pow(ndoth, 64.0);
			Vec3 specular = Vec3{1.0,1.0,1.0} * spec * (specular_strength*1.2);
			col += (diff + specular) * (pl.color * (pl.intensity * w * atten));
			// Make surfaces extremely close to the light appear emissive (window panes)
			if(dist < 2.0){
				col += pl.color * (3.0 * w);
			}
		}
	}
	// Emissive boost for designated materials (e.g., background sky board)
	if(!emissive_mat_ids.empty()){
		if(std::find(emissive_mat_ids.begin(), emissive_mat_ids.end(), t.material_id) != emissive_mat_ids.end()){
			col += kd * std::max(0.0, emissive_boost);
		}
	}
	// Filmic tone mapping (Hable/Uncharted2) with white point 11.2
	auto uncharted2 = [](double x)->double{
		const double A=0.15, B=0.50, C=0.10, D=0.20, E=0.02, F=0.30;
		return ((x*(A*x + C*B) + D*E) / (x*(A*x + B) + D*F)) - E/F;
	};
	const double white = 11.2;
	const double whiteScale = 1.0 / uncharted2(white);
	Vec3 x = col * std::max(0.0, exposure);
	Vec3 mapped = {
		uncharted2(x.x) * whiteScale,
		uncharted2(x.y) * whiteScale,
		uncharted2(x.z) * whiteScale
	};
	return clamp01(mapped);
}

bool Scene::trace_shadow_to(const Vec3& p, const Vec3& toLightDir, double maxDist) const{
	Ray sr;
	sr.origin = p + 1e-4 * toLightDir;
	sr.dir = toLightDir;
	sr.tmin = 0.0;
	sr.tmax = maxDist;
	for(int step=0; step<64; ++step){
		int hit; double t; Vec3 n; Vec2 uv; double b0,b1,b2;
		if(!bvh.intersect(mesh.triangles, sr, hit, t, n, uv, b0,b1,b2)) return false;
		if(t > maxDist) return false;
		const Triangle& tri = mesh.triangles[(size_t)hit];
		if(!emissive_mat_ids.empty() &&
		   std::find(emissive_mat_ids.begin(), emissive_mat_ids.end(), tri.material_id) != emissive_mat_ids.end()){
			// Skip emissive (e.g., sky board)
			sr.origin = sr.origin + sr.dir * (t + 1e-4);
			continue;
		}
		return true;
	}
	return true;
}


