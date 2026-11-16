// Debug: compare IMU orientation with camera-derived orientation. Prints Euler-ish angles for both, plus the
// implied camera->world extrinsics (which should stay constant if the two agree and axes are consistent).
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <glm/gtc/quaternion.hpp>

#include "../src/dk2_hid.hpp"
#include "../src/imu_fusion.hpp"
#include "../src/ptc_camera.hpp"
#include "../src/ptc_pose.hpp"
#include "../src/ptc_tracker.hpp"

static void ypr(const glm::quat& q, float& yaw, float& pitch, float& roll) {  // degrees; forward -Z, up +Y
    const glm::vec3 f = q * glm::vec3(0, 0, -1), u = q * glm::vec3(0, 1, 0);
    yaw = glm::degrees(std::atan2(-f.x, -f.z));
    pitch = glm::degrees(std::asin(glm::clamp(f.y, -1.f, 1.f)));
    const glm::vec3 right = glm::normalize(glm::cross(f, glm::vec3(0, 1, 0)));
    roll = glm::degrees(std::atan2(glm::dot(u, right), u.y));
}

int main(int argc, char** argv) {
    double seconds = argc > 1 ? std::atof(argv[1]) : 10;
    LedModel model;
    if (!model.load("data/dk2_leds.txt")) return 1;
    Dk2Hid hmd;
    if (!hmd.open() || !hmd.set_leds(true)) return 1;
    PtcCamera cam;
    if (!cam.open()) return 1;
    PtcIntrinsics k = cam.read_intrinsics();
    if (!cam.start(true)) return 1;
    PtcBlobTracker tracker(model);
    PtcPoseSolver solver(model, k);
    PtcPose pose;
    ImuFusion fusion;
    std::vector<ImuSample> imu;
    PtcFrame fr;
    const glm::dmat3 kView(glm::dvec3(-1, 0, 0), glm::dvec3(0, -1, 0), glm::dvec3(0, 0, 1));
    const glm::dmat3 kMc(glm::dvec3(-1, 0, 0), glm::dvec3(0, 1, 0), glm::dvec3(0, 0, 1));   // camera-side mirror (see head_tracker.cpp)
    const glm::dmat3 kMl(glm::dvec3(1, 0, 0), glm::dvec3(0, -1, 0), glm::dvec3(0, 0, 1));   // LED-layout mirror
    double lt = -1;
    auto t0 = std::chrono::steady_clock::now(), last = t0;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds) {
        imu.clear();
        hmd.poll(imu, 0);
        for (auto& s : imu) { fusion.update(s.gyro, s.accel, lt < 0 ? 0.001f : float(s.time - lt)); lt = s.time; }
        if (!cam.grab(fr, 100)) continue;
        pose = solver.solve(tracker.process(fr), pose);
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last).count() < 0.5 || !pose.valid || !fusion.initialised()) continue;
        last = now;
        glm::quat qi = fusion.orientation();
        glm::quat qh = glm::quat_cast(glm::mat3(kView * (kMc * pose.R * kMl)));  // headset -> camera-view frame
        glm::quat c = qi * glm::conjugate(qh);
        float y1, p1, r1, y2, p2, r2, y3, p3, r3;
        ypr(qi, y1, p1, r1);
        ypr(qh, y2, p2, r2);
        ypr(c, y3, p3, r3);
        std::printf("IMU ypr %+7.1f %+6.1f %+6.1f | CAM(view) ypr %+7.1f %+6.1f %+6.1f | C=imu*cam^-1 ypr %+7.1f %+6.1f %+6.1f | gyro|w|=%.3f\n",
                    y1, p1, r1, y2, p2, r2, y3, p3, r3, fusion.angular_speed());
    }
    return 0;
}
