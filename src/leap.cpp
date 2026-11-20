#include "leap.hpp"

#include <dirent.h>
#include <sys/stat.h>
#include <cstdlib>

#include <cmath>
#include <cstdio>
#include <cstring>

namespace {
std::string read_line(const std::string& path) {
    std::string s;
    if (FILE* f = std::fopen(path.c_str(), "r")) {
        char buf[256];
        if (std::fgets(buf, sizeof buf, f)) s = buf;
        std::fclose(f);
    }
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ')) s.pop_back();
    return s;
}

bool is_leap_id(const std::string& vid, const std::string& pid) {
    if (vid == "f182") return pid == "0003" || pid == "0300" || pid == "0007";   // Leap Motion Controller (and the 2015 re-spin)
    if (vid == "2936") return true;                                              // Ultraleap / Leap Motion Controller 2 / 3Di
    return false;
}

void add_box(std::vector<MapVertex>& out, glm::vec3 c, glm::vec3 half, glm::vec3 col) {
    static const int faces[6][4][3] = {   // each face: 4 corners as sign triples, outward normal in n[]
        {{1,-1,-1},{1,1,-1},{1,1,1},{1,-1,1}}, {{-1,-1,1},{-1,1,1},{-1,1,-1},{-1,-1,-1}},
        {{-1,1,-1},{-1,1,1},{1,1,1},{1,1,-1}}, {{-1,-1,1},{-1,-1,-1},{1,-1,-1},{1,-1,1}},
        {{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}}, {{1,-1,-1},{-1,-1,-1},{-1,1,-1},{1,1,-1}}};
    static const float normals[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    for (int f = 0; f < 6; ++f) {
        MapVertex v[4];
        for (int k = 0; k < 4; ++k) {
            const glm::vec3 p = c + glm::vec3(float(faces[f][k][0]) * half.x, float(faces[f][k][1]) * half.y, float(faces[f][k][2]) * half.z);
            v[k] = {{p.x, p.y, p.z}, {normals[f][0], normals[f][1], normals[f][2]}, {col.x, col.y, col.z}, {0, 0}};
        }
        const int idx[6] = {0, 1, 2, 0, 2, 3};
        for (int i : idx) out.push_back(v[i]);
    }
}
}  // namespace

LeapInfo detect_leap() {
    LeapInfo info;
    DIR* d = opendir("/sys/bus/usb/devices");
    if (!d) return info;
    while (dirent* e = readdir(d)) {
        if (e->d_name[0] == '.') continue;
        const std::string base = std::string("/sys/bus/usb/devices/") + e->d_name + "/";
        const std::string vid = read_line(base + "idVendor"), pid = read_line(base + "idProduct");
        if (vid.empty() || !is_leap_id(vid, pid)) continue;
        info.present = true;
        info.usb_id = vid + ":" + pid;
        info.name = read_line(base + "product");
        if (info.name.empty()) info.name = "Leap Motion Controller";
        break;
    }
    closedir(d);
    if (info.present) {   // the controller enumerates as a UVC camera: find its /dev/video node by name
        for (int i = 0; i < 16 && info.video.empty(); ++i) {
            const std::string n = read_line("/sys/class/video4linux/video" + std::to_string(i) + "/name");
            if (n.find("Leap") != std::string::npos || n.find("leap") != std::string::npos) info.video = "/dev/video" + std::to_string(i);
        }
    }
    return info;
}

std::vector<MapVertex> build_leap_mesh(bool lit) {
    // Real size: 80 x 30 mm footprint, 11.3 mm high. Silver aluminium body, black IR-transparent window on top with two
    // camera lenses and three IR LEDs in a row between them, and a green status LED on the front edge.
    std::vector<MapVertex> v;
    const glm::vec3 alu(0.62f, 0.64f, 0.68f), glass(0.03f, 0.03f, 0.04f), lens(0.10f, 0.12f, 0.20f);
    const glm::vec3 led = lit ? glm::vec3(1.0f, 0.25f, 0.2f) : glm::vec3(0.25f, 0.08f, 0.08f);
    const glm::vec3 status = lit ? glm::vec3(0.2f, 1.0f, 0.35f) : glm::vec3(0.08f, 0.2f, 0.1f);
    add_box(v, {0, 0.0045f, 0}, {0.040f, 0.0045f, 0.015f}, alu);          // body: y in [0, 9 mm]
    add_box(v, {0, 0.0098f, 0}, {0.038f, 0.0018f, 0.0135f}, glass);        // window slab: top at 11.6 mm
    add_box(v, {-0.0255f, 0.0120f, 0}, {0.0045f, 0.0005f, 0.0045f}, lens);   // left camera lens
    add_box(v, {0.0255f, 0.0120f, 0}, {0.0045f, 0.0005f, 0.0045f}, lens);    // right camera lens
    for (int i = -1; i <= 1; ++i) add_box(v, {0.0105f * float(i), 0.0120f, 0}, {0.0022f, 0.0005f, 0.0022f}, led);
    add_box(v, {0, 0.0045f, 0.0151f}, {0.0030f, 0.0008f, 0.0004f}, status);  // status LED on the front edge
    return v;
}

namespace {
void push_tri(std::vector<MapVertex>& out, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 col) {
    for (const glm::vec3& p : {a, b, c}) out.push_back({{p.x, p.y, p.z}, {0.0f, 1.0f, 0.0f}, {col.x, col.y, col.z}, {0, 0}});   // normal up: flat, evenly lit colour
}
glm::vec3 any_perp(glm::vec3 n) {
    glm::vec3 a = std::fabs(n.x) < 0.9f ? glm::vec3(1, 0, 0) : glm::vec3(0, 0, 1);
    return glm::normalize(glm::cross(n, a));
}
void ribbon(std::vector<MapVertex>& out, glm::vec3 p0, glm::vec3 p1, float half, glm::vec3 n, glm::vec3 col) {
    glm::vec3 d = p1 - p0;
    if (glm::length(d) < 1e-5f) return;
    glm::vec3 side = glm::cross(glm::normalize(d), n);
    side = glm::length(side) < 1e-4f ? any_perp(glm::normalize(d)) : glm::normalize(side);
    push_tri(out, p0 - side * half, p0 + side * half, p1 + side * half, col);
    push_tri(out, p0 - side * half, p1 + side * half, p1 - side * half, col);
}
void diamond(std::vector<MapVertex>& out, glm::vec3 c, float r, glm::vec3 n, glm::vec3 col) {
    const glm::vec3 a = any_perp(n), b = glm::normalize(glm::cross(n, a));
    push_tri(out, c + a * r, c + b * r, c - a * r, col);
    push_tri(out, c + a * r, c - a * r, c - b * r, col);
}
}  // namespace

void append_hand_geometry(const TrackedHand& h, std::vector<MapVertex>& out) {
    const glm::vec3 n = glm::length(h.normal) > 0.1f ? glm::normalize(h.normal) : glm::vec3(0, -1, 0);
    const glm::vec3 palm_col(0.85f, 0.10f, 0.08f), bone_col(1.0f, 0.5f, 0.1f), knuckle_col(1.0f, 0.9f, 0.15f), joint_col(1.0f, 0.65f, 0.25f), tip_col(1.0f, 0.95f, 0.8f);
    // Palm plate: the outline wrist - little - ring - middle - index - thumb base - wrist, filled as a fan around the palm centre.
    glm::vec3 across = h.joint[1][1] - h.joint[4][1];   // little -> index knuckle: the width direction of the hand
    across = glm::length(across) > 1e-4f ? glm::normalize(across) : glm::vec3(1, 0, 0);
    const glm::vec3 outline[7] = {h.wrist - across * 0.028f, h.joint[4][1], h.joint[3][1], h.joint[2][1], h.joint[1][1], h.joint[0][1], h.wrist + across * 0.028f};
    for (int i = 0; i < 7; ++i) push_tri(out, h.palm, outline[i], outline[(i + 1) % 7], palm_col);
    // Bones as flat ribbons, then the joints on top.
    for (int f = 0; f < 5; ++f) {
        const float half = f == 0 ? 0.0055f : 0.0045f;
        for (int k = f == 0 ? 0 : 1; k < 4; ++k) ribbon(out, h.joint[f][k], h.joint[f][k + 1], half, n, bone_col);
    }
    for (int f = 0; f < 5; ++f) {
        diamond(out, h.joint[f][1], 0.011f, n, knuckle_col);
        diamond(out, h.joint[f][2], 0.0075f, n, joint_col);
        diamond(out, h.joint[f][3], 0.0065f, n, joint_col);
        diamond(out, h.joint[f][4], 0.0075f, n, tip_col);
    }
}

#ifdef HAVE_LEAPC
#include <LeapC.h>

bool LeapTracker::start() {
    LEAP_CONNECTION c = nullptr;
    if (LeapCreateConnection(nullptr, &c) != eLeapRS_Success) return false;
    if (LeapOpenConnection(c) != eLeapRS_Success) { LeapDestroyConnection(c); return false; }
    conn_ = c;
    return true;
}

void LeapTracker::poll() {
    if (!conn_) return;
    LEAP_CONNECTION_MESSAGE msg;
    for (int guard = 0; guard < 64; ++guard) {   // drain everything that is waiting, keep the newest tracking frame
        if (LeapPollConnection(static_cast<LEAP_CONNECTION>(conn_), 0, &msg) != eLeapRS_Success) break;
        switch (msg.type) {
            case eLeapEventType_Connection: connected_ = true; break;
            case eLeapEventType_ConnectionLost: connected_ = false; device_ = false; hands_.clear(); break;
            case eLeapEventType_Device: device_ = true; break;
            case eLeapEventType_DeviceLost: case eLeapEventType_DeviceFailure: device_ = false; hands_.clear(); break;
            case eLeapEventType_Tracking: {
                const LEAP_TRACKING_EVENT* t = msg.tracking_event;
                hands_.clear();
                auto mm = [](const LEAP_VECTOR& v) { return glm::vec3(v.x, v.y, v.z) * 0.001f; };   // millimetres -> metres; +X right, +Y up, +Z towards the user
                for (uint32_t i = 0; i < t->nHands; ++i) {
                    TrackedHand h;
                    h.palm = mm(t->pHands[i].palm.position);
                    const LEAP_HAND& lh = t->pHands[i];
                    h.wrist = mm(lh.arm.next_joint);
                    h.normal = glm::vec3(lh.palm.normal.x, lh.palm.normal.y, lh.palm.normal.z);
                    for (int f = 0; f < 5; ++f) {
                        h.joint[f][0] = mm(lh.digits[f].bones[0].prev_joint);
                        h.joint[f][1] = mm(lh.digits[f].bones[1].prev_joint);
                        h.joint[f][2] = mm(lh.digits[f].bones[2].prev_joint);
                        h.joint[f][3] = mm(lh.digits[f].bones[3].prev_joint);
                        h.joint[f][4] = mm(lh.digits[f].bones[3].next_joint);
                    }
                    hands_.push_back(h);
                }
                break;
            }
            default: break;
        }
    }
}

LeapTracker::~LeapTracker() {
    if (conn_) { LeapCloseConnection(static_cast<LEAP_CONNECTION>(conn_)); LeapDestroyConnection(static_cast<LEAP_CONNECTION>(conn_)); }
}
#else
bool LeapTracker::start() { return false; }
void LeapTracker::poll() {}
LeapTracker::~LeapTracker() {}
#endif

static std::string leap_offset_path(bool make_dir) {
    const char* home = std::getenv("HOME");
    const std::string dir = std::string(home ? home : ".") + "/.config/dk2vr";
    if (make_dir) { mkdir((std::string(home ? home : ".") + "/.config").c_str(), 0755); mkdir(dir.c_str(), 0755); }
    return dir + "/leap_offset";
}

bool load_leap_offset(glm::vec3& out) {
    FILE* f = std::fopen(leap_offset_path(false).c_str(), "r");
    if (!f) return false;
    float x, y, z;
    const bool ok = std::fscanf(f, "%f %f %f", &x, &y, &z) == 3;
    std::fclose(f);
    if (ok) out = glm::vec3(x, y, z);
    return ok;
}

void save_leap_offset(const glm::vec3& v) {
    if (FILE* f = std::fopen(leap_offset_path(true).c_str(), "w")) { std::fprintf(f, "%.4f %.4f %.4f\n", v.x, v.y, v.z); std::fclose(f); }
}
