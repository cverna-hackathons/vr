// Measures whether the camera-derived position axes agree with the headset's accelerometer.
// Move the headset (worn or in hand) back and forth in front of the camera: forward/back, left/right, up/down,
// roughly facing the camera, for the given time. Prints, for each camera axis, how it correlates with each
// accelerometer axis (both expressed in the IMU world frame). A correct setup gives +1 on the diagonal.
// Run: build/axis_check [seconds]   (keep ./build/vr closed)
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

#include "../src/head_tracker.hpp"

int main(int argc, char** argv) {
    const double seconds = argc > 1 ? std::atof(argv[1]) : 20;
    HeadTracker tracker;
    if (!tracker.start("data/dk2_leds.txt", true) || !tracker.camera_active()) { std::fprintf(stderr, "tracker start failed\n"); return 1; }

    struct Rec { double t; glm::vec3 p; glm::vec3 a; bool ok; };
    std::vector<Rec> rec;
    auto t0 = std::chrono::steady_clock::now();
    std::printf("Move the headset back and forth: forward/back, left/right, up/down. Recording %.0f s...\n", seconds);
    glm::vec3 acc_sum(0); int acc_n = 0;
    while (true) {
        const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (t > seconds) break;
        tracker.update();
        glm::vec3 a;
        if (tracker.take_world_accel(a)) { acc_sum += a; ++acc_n; }
        rec.push_back({t, tracker.imu_world_position(), acc_n ? acc_sum / float(acc_n) : glm::vec3(0), tracker.camera_tracking()});
        acc_sum = glm::vec3(0); acc_n = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    tracker.stop();

    // Camera acceleration by second difference over a 5-frame stride (smooths noise), valid frames only.
    const int S = 3;
    std::vector<glm::vec3> ac, ai;
    std::vector<int> idx;
    for (size_t i = S; i + S < rec.size(); ++i) {
        bool ok = true;
        for (int k = -S; k <= S; ++k) ok = ok && rec[i + k].ok;
        if (!ok) continue;
        const double dt = (rec[i + S].t - rec[i - S].t) / 2.0;
        ac.push_back((rec[i + S].p - 2.0f * rec[i].p + rec[i - S].p) / float(dt * dt));
        glm::vec3 m(0);
        for (int k = -S; k <= S; ++k) m += rec[i + k].a;
        ai.push_back(m / float(2 * S + 1));
        idx.push_back(int(i));
    }
    std::printf("%zu usable samples of %zu\n", ac.size(), rec.size());
    if (ac.size() < 50) { std::printf("Too little tracked motion; keep the headset in view of the camera.\n"); return 2; }

    // Best time lag between camera and IMU (camera is delayed): shift IMU forward by `lag` samples.
    int best_lag = 0; double best_score = -1e9;
    for (int lag = 0; lag <= 8; ++lag) {
        double score = 0;
        for (int ax = 0; ax < 3; ++ax) {
            double sab = 0, saa = 0, sbb = 0;
            for (size_t i = 0; i + lag < ac.size(); ++i) { double x = ac[i][ax], y = ai[i + lag][ax]; sab += x * y; saa += x * x; sbb += y * y; }
            score += std::fabs(sab / std::sqrt(saa * sbb + 1e-12));
        }
        if (score > best_score) { best_score = score; best_lag = lag; }
    }
    std::printf("best lag: %d frames\n", best_lag);
    std::printf("correlation (rows: camera axis X Y Z in IMU world; columns: accelerometer X Y Z)\n");
    for (int a = 0; a < 3; ++a) {
        std::printf("  cam %c:", "XYZ"[a]);
        for (int b = 0; b < 3; ++b) {
            double sab = 0, saa = 0, sbb = 0;
            for (size_t i = 0; i + best_lag < ac.size(); ++i) { double x = ac[i][a], y = ai[i + best_lag][b]; sab += x * y; saa += x * x; sbb += y * y; }
            std::printf(" %+6.2f", sab / std::sqrt(saa * sbb + 1e-12));
        }
        std::printf("\n");
    }
    return 0;
}
