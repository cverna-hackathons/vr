// Live test of PTC pose tracking: prints the headset position in camera coordinates (metres) and the fit quality.
// Run: build/ptc_pose [seconds]      (keep ./build/vr closed)
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "../src/dk2_hid.hpp"
#include "../src/ptc_camera.hpp"
#include "../src/ptc_pose.hpp"
#include "../src/ptc_tracker.hpp"

int main(int argc, char** argv) {
    double seconds = argc > 1 ? std::atof(argv[1]) : 15;
    LedModel model;
    if (!model.load("data/dk2_leds.txt")) { std::fprintf(stderr, "Cannot load data/dk2_leds.txt (run from the project directory)\n"); return 1; }

    Dk2Hid hmd;
    if (!hmd.open() || !hmd.set_leds(true)) { std::fprintf(stderr, "DK2 not available (is ./build/vr running?)\n"); return 1; }
    PtcCamera cam;
    if (!cam.open()) return 1;
    PtcIntrinsics k = cam.read_intrinsics();
    if (!k.valid) { std::fprintf(stderr, "Could not read camera calibration\n"); return 1; }
    if (!cam.start(true)) return 1;

    PtcBlobTracker tracker(model);
    PtcPoseSolver solver(model, k);
    PtcPose pose;
    std::vector<ImuSample> imu;
    PtcFrame fr;
    auto t0 = std::chrono::steady_clock::now(), last = t0;
    int frames = 0, valid = 0, misses = 0;
    double rms_sum = 0;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds && misses < 20) {
        imu.clear();
        hmd.poll(imu, 0);
        if (!cam.grab(fr, 100)) { ++misses; continue; }
        const auto& blobs = tracker.process(fr);
        pose = solver.solve(blobs, pose);
        ++frames;
        if (pose.valid) { ++valid; rms_sum += pose.rms_px; }
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last).count() >= 0.5) {
            last = now;
            if (pose.valid)
                std::printf("pos (cam, m): x=%+.3f y=%+.3f z=%+.3f  | %2d LEDs, rms %.2f px\n", pose.t.x, pose.t.y, pose.t.z, pose.inliers, pose.rms_px);
            else
                std::printf("no pose (blobs %zu)\n", blobs.size());
        }
    }
    std::printf("Done: %d frames, pose valid in %d (%.0f%%), mean rms %.2f px\n", frames, valid,
                frames ? 100.0 * valid / frames : 0, valid ? rms_sum / valid : 0);
    return 0;
}
