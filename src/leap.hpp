// Leap Motion Controller support for the explore scene: USB detection and a small 3D model of the device.
//
// There is no vendor SDK/daemon on this machine, so the device is recognised from the USB bus (sysfs) and shown at its
// place in the room; hand tracking itself is not available. Model axes: +X right, +Y up (the sensor face), +Z towards the user.
#pragma once
#include <algorithm>
#include <string>
#include <glm/glm.hpp>
#include <vector>

#include "mesh_types.hpp"

struct LeapInfo {
    bool present = false;
    std::string name;      // product string, e.g. "Leap Motion Controller"
    std::string usb_id;    // "f182:0003"
    std::string video;     // first /dev/videoN of the device, if any
};

// Scans /sys/bus/usb/devices for a Leap Motion / Ultraleap controller. Cheap (a few file reads); call every second or two.
LeapInfo detect_leap();

// Triangles of the device in metres, centred on the middle of its base. `lit` = true colours the status LEDs bright.
std::vector<MapVertex> build_leap_mesh(bool lit);

// Where the controller sits relative to the PTC (tracking frame: +X right, +Y up, +Z back), saved in ~/.config/dk2vr/leap_offset.
bool load_leap_offset(glm::vec3& out);
void save_leap_offset(const glm::vec3& v);

// --- Hand ---
// One tracked hand in the controller's frame (metres). joint[f][k] for the five digits f (thumb, index, middle, ring, little):
// k = 0 base of the metacarpal (at the wrist), 1 knuckle (metacarpophalangeal), 2 and 3 the finger joints, 4 the fingertip.
struct TrackedHand {
    glm::vec3 palm{0.0f}, wrist{0.0f}, normal{0.0f, -1.0f, 0.0f};   // normal: out of the palm
    glm::vec3 joint[5][5];
};

struct HandState {
    bool present = false;
    glm::vec3 palm{0.0f, 0.12f, 0.0f};
};

class SimHand {
public:
    // dir: x = right(+)/left(-), y = up(+)/down(-), z = towards you(+)/away(-), each -1..1.
    void update(float dt, const glm::vec3& dir) {
        palm_ += dir * (0.30f * dt);
        palm_.x = std::min(0.25f, std::max(-0.25f, palm_.x));
        palm_.y = std::min(0.45f, std::max(0.05f, palm_.y));   // the controller sees about 2.5 .. 60 cm above it
        palm_.z = std::min(0.25f, std::max(-0.25f, palm_.z));
    }
    void toggle() { present_ = !present_; }
    HandState state() const { return {present_, palm_}; }
    // A fixed relaxed open hand (palm down, fingers away from you) around the palm, to try the display without the tracking service.
    TrackedHand hand() const {
        TrackedHand h;
        h.palm = palm_;
        h.wrist = palm_ + glm::vec3(0.0f, 0.0f, 0.09f);
        h.normal = glm::vec3(0.0f, -1.0f, 0.0f);
        static const float fx[5] = {-0.06f, -0.03f, -0.005f, 0.02f, 0.043f};      // finger x offsets
        static const float len[5] = {0.7f, 1.0f, 1.08f, 1.0f, 0.8f};
        for (int f = 1; f < 5; ++f) {
            glm::vec3 p = palm_ + glm::vec3(fx[f], 0.0f, -0.04f);
            h.joint[f][0] = h.wrist + glm::vec3(fx[f] * 0.5f, 0.0f, 0.0f);
            h.joint[f][1] = p;
            const float seg[3] = {0.040f, 0.025f, 0.020f};
            for (int k = 0; k < 3; ++k) { p += glm::vec3(0.0f, -0.004f * float(k), -seg[k] * len[f]); h.joint[f][2 + k] = p; }
        }
        h.joint[0][0] = h.wrist + glm::vec3(-0.02f, 0.0f, 0.0f);
        h.joint[0][1] = palm_ + glm::vec3(-0.04f, 0.0f, 0.025f);
        h.joint[0][2] = palm_ + glm::vec3(-0.065f, 0.0f, 0.0f);
        h.joint[0][3] = palm_ + glm::vec3(-0.085f, -0.003f, -0.02f);
        h.joint[0][4] = palm_ + glm::vec3(-0.098f, -0.006f, -0.036f);
        return h;
    }
private:
    bool present_ = true;
    glm::vec3 palm_{0.0f, 0.12f, 0.0f};
};

// Appends a flat drawing of a hand, in painter's order (draw it with the depth test off): a red palm plate (wrist to knuckles), orange ribbons for the
// finger bones, yellow diamonds on the knuckles, smaller orange ones on the finger joints and pale ones on the fingertips.
void append_hand_geometry(const TrackedHand& hand, std::vector<MapVertex>& out);

// Real hand tracking through the Ultraleap tracking service (LeapC). Optional: compiled in only if libLeapC was found at build time.
// Call poll() once per frame (non-blocking); palms() are in the controller's frame, metres (same axes as HandState).
class LeapTracker {
public:
    ~LeapTracker();
    bool start();                       // connect to the service; false if LeapC is not available or the connection failed
    void poll();
    bool connected() const { return connected_; }   // service reachable
    bool device() const { return device_; }         // service reports a controller
    const std::vector<TrackedHand>& hands() const { return hands_; }
private:
    void* conn_ = nullptr;
    bool connected_ = false, device_ = false;
    std::vector<TrackedHand> hands_;
};
