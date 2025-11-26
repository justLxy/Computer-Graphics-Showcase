#include <cstdio>
#include <string>
#include <iostream>
#include <cstdlib>
#include <dirent.h>
#include <thread>
#include <vector>
#include <cmath>
#include <cstring>

// Silence macOS OpenGL deprecation warnings
#if defined(__APPLE__)
#define GL_SILENCE_DEPRECATION 1
#endif

// GLFW + OpenGL
#include <GLFW/glfw3.h>

#include "cg_math.h"
#include "image.h"
#include "camera.h"
#include "obj_loader.h"
#include "bvh.h"
#include "scene.h"

// Global state for camera control
struct CameraParams {
    double angle_deg = 20.0;
    double start_dist = 15.0;
    double end_dist = 10.0;
    double height_abs = 4.0;
    double pan = 0.0;
    double strafe = 0.0;
    double pos_left = 0.0;
    double pos_forward = 0.0;
    double look_left = 0.0;
    double look_forward = 0.0;
    double fov = 55.0;
    double time_t = 0.0; // 0..1 for animation preview
} g_cam;

bool g_need_render = true;
int g_width = 960;
int g_height = 540;
Image* g_image = nullptr;
Scene* g_scene = nullptr;
Vec3 g_center, g_bbmin, g_bbmax, g_heading, g_left_vec, g_look_at;
double g_scene_height = 0.0;
GLuint g_texture = 0;

static Vec3 sky(const Vec3& d){
    double t = 0.5*(d.y+1.0);
    return (1.0-t)*Vec3{0.9,0.85,0.8} + t*Vec3{0.6,0.75,0.95};
}

static bool file_exists(const char* p){
    FILE* f = std::fopen(p,"rb");
    if(!f) return false;
    std::fclose(f);
    return true;
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

void render_frame(){
    if(!g_image || !g_scene) return;
    
    g_image->fill({0,0,0});
    
    // Build camera from current params
    const double alpha = g_cam.angle_deg * PI / 180.0;
    const auto rotY = [&](const Vec3& v)->Vec3{
        return { v.x*std::cos(alpha) + v.z*std::sin(alpha), 0.0, -v.x*std::sin(alpha) + v.z*std::cos(alpha) };
    };
    Vec3 frontXZ = {g_heading.x, 0.0, g_heading.z};
    if(length(frontXZ) < 1e-8){ frontXZ = {0.0,0.0,1.0}; }
    const Vec3 heading = normalize(rotY(normalize(frontXZ)));
    const Vec3 left_vec = normalize(Vec3(-heading.z, 0.0, heading.x));
    
    const double dist = g_cam.start_dist + (g_cam.end_dist - g_cam.start_dist) * g_cam.time_t;
    Vec3 camPos = { g_center.x + heading.x*dist, g_cam.height_abs, g_center.z + heading.z*dist };
    Vec3 target = g_look_at + left_vec * g_cam.pan;
    
    const Vec3 forward_vec = normalize(Vec3(heading.x,0.0,heading.z));
    camPos = camPos + left_vec*g_cam.strafe + left_vec*g_cam.pos_left + forward_vec*g_cam.pos_forward;
    target = target + left_vec*g_cam.look_left + forward_vec*g_cam.look_forward;
    
    Camera cam;
    cam.eye = camPos;
    cam.target = target;
    cam.up = {0,1,0};
    cam.fov_y_deg = g_cam.fov;
    cam.aspect = (double)g_width/(double)g_height;
    
	// Sun: follow camera by default so scene is front-lit, like a key light.
	{
		const double sunIntensity = env_double("SUN_INTENSITY", 4.0);
		const bool followCam = true; // always follow camera in preview for clear key-lighting
		Vec3 sunDir;
		if(followCam){
			const Vec3 camF = normalize(target - camPos);
			Vec3 camR = cross(camF, Vec3{0,1,0});
			if(length(camR) < 1e-8) camR = {1,0,0};
			camR = normalize(camR);
			const Vec3 camU = normalize(cross(camR, camF));
			const double yaw   = env_double("SUN_OFFSET_YAW_DEG", 0.0) * PI / 180.0;
			const double pitch = env_double("SUN_OFFSET_PITCH_DEG", -8.0) * PI / 180.0;
			const double cp = std::cos(pitch), sp = std::sin(pitch);
			const double cy = std::cos(yaw),   sy = std::sin(yaw);
			Vec3 dirLocal = camF * (cp*cy) + camR * (cp*sy) + camU * (sp);
			// shade() uses L = -sun.dir. To front‑light surfaces (N ≈ -camF),
			// set sun.dir ≈ camF so L ≈ -camF and ndotl > 0.
			sunDir = normalize(dirLocal);
		}else{
			const double sunAzDeg = env_double("SUN_AZIMUTH_DEG", -30.0);
			const double sunElDeg = env_double("SUN_ELEV_DEG", 32.0);
			const double sunAz = sunAzDeg * PI / 180.0;
			const double sunEl = sunElDeg * PI / 180.0;
			sunDir = normalize(Vec3(std::cos(sunAz)*std::cos(sunEl), -std::sin(sunEl), std::sin(sunAz)*std::cos(sunEl)));
		}
		g_scene->sun.dir = sunDir;
		g_scene->sun.intensity = sunIntensity;
	}
	
    // Render (single-threaded for preview)
    int hit_count = 0;
    int total_pixels = g_width * g_height;
    double sum_brightness = 0.0;
    for(int y=0; y<g_height; ++y){
        for(int x=0; x<g_width; ++x){
            Ray r = cam.generate_ray(x,y,g_width,g_height);
            int hitIdx; double tHit; Vec3 nHit; Vec2 uvHit; double b0,b1,b2;
            if(g_scene->bvh.intersect(g_scene->mesh.triangles, r, hitIdx, tHit, nHit, uvHit, b0,b1,b2)){
                Vec3 color = g_scene->shade(r, hitIdx, nHit, uvHit, b0,b1,b2);
                g_image->set(x,y,color);
                sum_brightness += (color.x + color.y + color.z) / 3.0;
                hit_count++;
            }else{
                Vec3 skycolor = sky(r.dir);
                g_image->set(x,y, skycolor);
                sum_brightness += (skycolor.x + skycolor.y + skycolor.z) / 3.0;
            }
        }
    }
    
    double avg_brightness = sum_brightness / total_pixels;
    std::cout << "Hits: " << hit_count << " / " << total_pixels << " pixels (" 
              << (100.0 * hit_count / total_pixels) << "%)\n";
    std::cout << "Average brightness: " << avg_brightness << " (0=black, 1=white)\n";
    
    // Print current camera params to console
    std::cout << "\n=== Current Camera Parameters ===\n";
    std::cout << "Camera Position: (" << camPos.x << ", " << camPos.y << ", " << camPos.z << ")\n";
    std::cout << "Look At: (" << target.x << ", " << target.y << ", " << target.z << ")\n";
    std::cout << "CAM_ANGLE_DEG=" << g_cam.angle_deg << "\n";
    std::cout << "CAM_START_DIST=" << g_cam.start_dist << "\n";
    std::cout << "CAM_END_DIST=" << g_cam.end_dist << "\n";
    std::cout << "CAM_HEIGHT_ABS=" << g_cam.height_abs << "\n";
    std::cout << "CAM_PAN=" << g_cam.pan << "\n";
    std::cout << "CAM_STRAFE=" << g_cam.strafe << "\n";
    std::cout << "CAM_POS_LEFT=" << g_cam.pos_left << "\n";
    std::cout << "CAM_POS_FORWARD=" << g_cam.pos_forward << "\n";
    std::cout << "CAM_LOOK_LEFT=" << g_cam.look_left << "\n";
    std::cout << "CAM_LOOK_FORWARD=" << g_cam.look_forward << "\n";
    std::cout << "CAM_FOV=" << g_cam.fov << "\n";
    std::cout << "Time (0-1): " << g_cam.time_t << "\n";
    std::cout << "=================================\n";
    std::cout << "To use these params in final render:\n";
    std::cout << "CAM_ANGLE_DEG=" << g_cam.angle_deg 
              << " CAM_START_DIST=" << g_cam.start_dist
              << " CAM_END_DIST=" << g_cam.end_dist
              << " CAM_HEIGHT_ABS=" << g_cam.height_abs
              << " CAM_PAN=" << g_cam.pan
              << " CAM_STRAFE=" << g_cam.strafe
              << " CAM_POS_LEFT=" << g_cam.pos_left
              << " CAM_POS_FORWARD=" << g_cam.pos_forward
              << " CAM_LOOK_LEFT=" << g_cam.look_left
              << " CAM_LOOK_FORWARD=" << g_cam.look_forward
              << " CAM_FOV=" << g_cam.fov
              << " SHOW_THREADS=11 ./showcase\n";
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods){
    (void)scancode;
    if(action != GLFW_PRESS && action != GLFW_REPEAT) return;
    
    // Much larger steps for easier control
    const double dist_step = (mods & GLFW_MOD_SHIFT) ? 5.0 : 50.0;      // Distance: 50 units (or 5 with shift)
    const double height_step = (mods & GLFW_MOD_SHIFT) ? 2.0 : 20.0;    // Height: 20 units
    const double pos_step = (mods & GLFW_MOD_SHIFT) ? 5.0 : 50.0;       // Position: 50 units
    const double look_step = (mods & GLFW_MOD_SHIFT) ? 5.0 : 50.0;      // Look offset: 50 units
    const double angle_step = (mods & GLFW_MOD_SHIFT) ? 1.0 : 10.0;     // Angle: 10 degrees
    const double fov_step = (mods & GLFW_MOD_SHIFT) ? 1.0 : 5.0;        // FOV: 5 degrees
    
    switch(key){
        // Camera distance
        case GLFW_KEY_W: g_cam.start_dist -= dist_step; g_cam.end_dist -= dist_step; g_need_render = true; break;
        case GLFW_KEY_S: g_cam.start_dist += dist_step; g_cam.end_dist += dist_step; g_need_render = true; break;
        
        // Camera height
        case GLFW_KEY_Q: g_cam.height_abs += height_step; g_need_render = true; break;
        case GLFW_KEY_E: g_cam.height_abs -= height_step; g_need_render = true; break;
        
        // Pan (look left/right)
        case GLFW_KEY_A: g_cam.look_left += look_step; g_need_render = true; break;
        case GLFW_KEY_D: g_cam.look_left -= look_step; g_need_render = true; break;
        
        // Forward/back look
        case GLFW_KEY_R: g_cam.look_forward += look_step; g_need_render = true; break;
        case GLFW_KEY_F: g_cam.look_forward -= look_step; g_need_render = true; break;
        
        // Position left/right
        case GLFW_KEY_Z: g_cam.pos_left += pos_step; g_need_render = true; break;
        case GLFW_KEY_C: g_cam.pos_left -= pos_step; g_need_render = true; break;
        
        // Position forward/back
        case GLFW_KEY_T: g_cam.pos_forward += pos_step; g_need_render = true; break;
        case GLFW_KEY_G: g_cam.pos_forward -= pos_step; g_need_render = true; break;
        
        // Angle
        case GLFW_KEY_LEFT: g_cam.angle_deg -= angle_step; g_need_render = true; break;
        case GLFW_KEY_RIGHT: g_cam.angle_deg += angle_step; g_need_render = true; break;
        
        // FOV
        case GLFW_KEY_UP: g_cam.fov += fov_step; g_need_render = true; break;
        case GLFW_KEY_DOWN: g_cam.fov -= fov_step; g_need_render = true; break;
        
        // Time slider
        case GLFW_KEY_COMMA: g_cam.time_t = std::max(0.0, g_cam.time_t - 0.05); g_need_render = true; break;
        case GLFW_KEY_PERIOD: g_cam.time_t = std::min(1.0, g_cam.time_t + 0.05); g_need_render = true; break;
        
        // Reset
        case GLFW_KEY_SPACE: 
            g_cam = CameraParams(); 
            g_cam.start_dist = length(g_bbmax - g_bbmin) * 0.4;
            g_cam.end_dist = g_cam.start_dist * 0.7;
            g_cam.height_abs = g_bbmin.y + g_scene_height * 0.22;
            g_need_render = true; 
            break;
        
        case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GLFW_TRUE); break;
    }
}

int main(int argc, char** argv){
    // Load OBJ
    std::string obj_path;
    if(argc >= 2 && file_exists(argv[1])){
        obj_path = argv[1];
    }else{
        const char* obj_dirs[] = {
            "../source", "Showcase/Code/source", "source",
            "../../Showcase/Code/source", "../../source", "."
        };
        for(const char* d : obj_dirs){
            std::string p = find_first_obj_in_dir(d);
            if(!p.empty()){ obj_path = p; break; }
        }
        if(obj_path.empty()){
            std::cerr<<"Failed to locate an OBJ file.\n";
            return 1;
        }
    }
    
    Mesh mesh;
    if(!load_obj_with_mtl(obj_path, mesh)){
        std::cerr<<"Failed to load OBJ at "<<obj_path<<"\n";
        return 1;
    }
    std::cout<<"Loaded triangles: "<<mesh.triangles.size()<<", materials: "<<mesh.materials.size()<<", textures: "<<mesh.textures.size()<<"\n";
    
    Scene scene;
    scene.set_mesh(std::move(mesh));
    g_scene = &scene;
    
	// Match final renderer defaults (ambient/exposure/specular/shadow)
	{
		const double amb = env_double("AMBIENT", 0.28);
		scene.ambient = {amb, amb, amb};
		// Hemispheric skylight and tint
		scene.sky_fill = env_double("SKY_FILL", 0.22);
		const double st_r = env_double("SKY_TINT_R", 0.6);
		const double st_g = env_double("SKY_TINT_G", 0.75);
		const double st_b = env_double("SKY_TINT_B", 0.95);
		scene.sky_tint = {st_r, st_g, st_b};
		// Emissive sky brightness controls
		scene.emissive_gain = env_double("EMISSIVE_GAIN", 1.8);
		scene.emissive_exposure_mul = env_double("EMISSIVE_EXPOSURE_MUL", 1.6);
		scene.emissive_linear = env_double("EMISSIVE_LINEAR", 0.0) > 0.5;
		scene.exposure = env_double("EXPOSURE", 1.35);
		scene.specular_strength = env_double("SPECULAR_STRENGTH", 0.05);
		scene.shadow_ambient = env_double("SHADOW_AMBIENT", 0.15);
	}
	// Detect background/emissive board materials (largest triangles)
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
	
    // Compute bounds
    g_bbmin = { kInfinity, kInfinity, kInfinity };
    g_bbmax = { -kInfinity, -kInfinity, -kInfinity };
    for(const auto& tri : scene.mesh.triangles){
        g_bbmin.x = std::min({g_bbmin.x, tri.p0.x, tri.p1.x, tri.p2.x});
        g_bbmin.y = std::min({g_bbmin.y, tri.p0.y, tri.p1.y, tri.p2.y});
        g_bbmin.z = std::min({g_bbmin.z, tri.p0.z, tri.p1.z, tri.p2.z});
        g_bbmax.x = std::max({g_bbmax.x, tri.p0.x, tri.p1.x, tri.p2.x});
        g_bbmax.y = std::max({g_bbmax.y, tri.p0.y, tri.p1.y, tri.p2.y});
        g_bbmax.z = std::max({g_bbmax.z, tri.p0.z, tri.p1.z, tri.p2.z});
    }
    g_center = (g_bbmin + g_bbmax) / 2.0;
    const double diag = length(g_bbmax - g_bbmin);
    g_scene_height = (g_bbmax.y - g_bbmin.y);
    g_look_at = g_center;
    g_look_at.y = g_bbmin.y + std::max( g_scene_height*0.3, diag*0.05 );
    
    // Auto front direction
    Vec3 areaN = {0,0,0};
    for(const auto& tri : scene.mesh.triangles){
        const Vec3 e1 = tri.p1 - tri.p0;
        const Vec3 e2 = tri.p2 - tri.p0;
        const Vec3 an = cross(e1,e2);
        areaN += an;
    }
    Vec3 frontN = normalize(areaN);
    Vec3 frontXZ = {frontN.x, 0.0, frontN.z};
    if(length(frontXZ) < 1e-8){ frontXZ = {0.0,0.0,1.0}; }
    g_heading = normalize(frontXZ);
    g_left_vec = normalize(Vec3(-g_heading.z, 0.0, g_heading.x));
    
    // Initialize camera params - start much closer for preview
    g_cam.start_dist = diag * 0.15;  // Much closer initial view
    g_cam.end_dist = diag * 0.10;
    g_cam.height_abs = g_bbmin.y + std::max(g_scene_height*0.35, diag*0.06);
    g_cam.angle_deg = 0.0;  // Start with front view
    
    std::cout << "Scene bounds: (" << g_bbmin.x << ", " << g_bbmin.y << ", " << g_bbmin.z << ") to ("
              << g_bbmax.x << ", " << g_bbmax.y << ", " << g_bbmax.z << ")\n";
    std::cout << "Scene center: (" << g_center.x << ", " << g_center.y << ", " << g_center.z << ")\n";
    std::cout << "Scene diagonal: " << diag << "\n";
    
    // Initialize GLFW
    if(!glfwInit()){
        std::cerr<<"Failed to initialize GLFW\n";
        return 1;
    }
    
    // Request a legacy 2.1 context so fixed-function (glDrawPixels) works on macOS
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
#if defined(__APPLE__)
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_ANY_PROFILE);
#endif
    GLFWwindow* window = glfwCreateWindow(g_width, g_height, "Camera Preview - Use WASD/QE/Arrow Keys", nullptr, nullptr);
    if(!window){
        std::cerr<<"Failed to create GLFW window\n";
        glfwTerminate();
        return 1;
    }
    
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    
    // Create texture for display
    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &g_texture);
    glBindTexture(GL_TEXTURE_2D, g_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    glDisable(GL_DEPTH_TEST);
    glClearColor(0.f, 0.f, 0.f, 1.f);
    glfwSetKeyCallback(window, key_callback);
    
    g_image = new Image(g_width, g_height);
    
    std::cout << "\n=== Interactive Camera Preview ===\n";
    std::cout << "Controls:\n";
    std::cout << "  W/S: Move closer/farther\n";
    std::cout << "  Q/E: Move up/down\n";
    std::cout << "  A/D: Look left/right\n";
    std::cout << "  R/F: Look forward/back\n";
    std::cout << "  Z/C: Move camera left/right\n";
    std::cout << "  T/G: Move camera forward/back\n";
    std::cout << "  Arrow Left/Right: Rotate angle\n";
    std::cout << "  Arrow Up/Down: Change FOV\n";
    std::cout << "  ,/. (comma/period): Scrub animation time\n";
    std::cout << "  SPACE: Reset to defaults\n";
    std::cout << "  SHIFT + key: Fine adjustment (0.1x)\n";
    std::cout << "  ESC: Quit\n";
    std::cout << "==================================\n\n";
    
    // Render initial frame
    std::cout << "Rendering initial preview...\n";
    render_frame();
    
    // Upload initial frame to OpenGL texture
    std::vector<unsigned char> pixels(g_width * g_height * 3);
    for(int y=0; y<g_height; ++y){
        for(int x=0; x<g_width; ++x){
            Vec3 c = g_image->get(x, y);
            int idx = (y * g_width + x) * 3;
            pixels[idx+0] = to_u8(c.x);
            pixels[idx+1] = to_u8(c.y);
            pixels[idx+2] = to_u8(c.z);
        }
    }
    glBindTexture(GL_TEXTURE_2D, g_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_width, g_height, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    
    // Draw textured quad
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 1, 0, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glBegin(GL_QUADS);
    glTexCoord2f(0, 1); glVertex2f(0, 0);
    glTexCoord2f(1, 1); glVertex2f(1, 0);
    glTexCoord2f(1, 0); glVertex2f(1, 1);
    glTexCoord2f(0, 0); glVertex2f(0, 1);
    glEnd();
    
    glfwSwapBuffers(window);
    g_need_render = false;
    
    std::cout << "Ready! Use keyboard controls to adjust camera.\n\n";
    
    // Main loop
    while(!glfwWindowShouldClose(window)){
        if(g_need_render){
            std::cout << "Rendering preview...\n";
            render_frame();
            g_need_render = false;
            
            // Upload to OpenGL texture
            pixels.resize(g_width * g_height * 3);
            for(int y=0; y<g_height; ++y){
                for(int x=0; x<g_width; ++x){
                    Vec3 c = g_image->get(x, y);
                    int idx = (y * g_width + x) * 3;
                    pixels[idx+0] = to_u8(c.x);
                    pixels[idx+1] = to_u8(c.y);
                    pixels[idx+2] = to_u8(c.z);
                }
            }
            glBindTexture(GL_TEXTURE_2D, g_texture);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_width, g_height, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
            
            // Draw textured quad
            glClear(GL_COLOR_BUFFER_BIT);
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(0, 1, 0, 1, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            
            glBegin(GL_QUADS);
            glTexCoord2f(0, 1); glVertex2f(0, 0);
            glTexCoord2f(1, 1); glVertex2f(1, 0);
            glTexCoord2f(1, 0); glVertex2f(1, 1);
            glTexCoord2f(0, 0); glVertex2f(0, 1);
            glEnd();
            
            glfwSwapBuffers(window);
        }
        
        glfwPollEvents();
    }
    
    delete g_image;
    glfwDestroyWindow(window);
    glfwTerminate();
    
    return 0;
}

