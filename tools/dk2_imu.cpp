// Test for the DK2 HID driver: reads the IMU for a few seconds, prints rate/gravity/gyro statistics and the
// LED blink phase sequence. Keep the headset still for a meaningful accel/gyro check.
// Run: build/dk2_imu [seconds] [--const]   (--const = LEDs constantly on instead of blinking)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "../src/dk2_hid.hpp"

int main(int argc, char** argv) {
    double seconds = argc > 1 ? std::atof(argv[1]) : 3.0;
    bool blink = true;
    for (int i = 1; i < argc; ++i) if (!std::strcmp(argv[i], "--const")) blink = false;

    Dk2Hid hmd;
    if (!hmd.open()) return 1;
    std::printf("Opened DK2, report rate %d Hz\n", hmd.sample_rate_hz());
    if (!hmd.set_leds(blink)) std::fprintf(stderr, "Warning: LED enable failed\n");

    std::vector<ImuSample> all, batch;
    double t0 = -1;
    std::vector<int> phases;
    while (true) {
        batch.clear();
        int n = hmd.poll(batch, 50);
        if (n < 0) { std::fprintf(stderr, "Device error\n"); return 1; }
        for (const ImuSample& s : batch) {
            if (t0 < 0) t0 = s.time;
            all.push_back(s);
            if (phases.empty() || phases.back() != s.led_phase) phases.push_back(s.led_phase);
        }
        if (!all.empty() && all.back().time - t0 >= seconds) break;
    }

    glm::dvec3 am(0), gm(0);
    for (auto& s : all) { am += glm::dvec3(s.accel); gm += glm::dvec3(s.gyro); }
    am /= double(all.size()); gm /= double(all.size());
    glm::dvec3 gv(0);
    for (auto& s : all) { glm::dvec3 d = glm::dvec3(s.gyro) - gm; gv += d * d; }
    gv = glm::sqrt(gv / double(all.size()));

    double span = all.back().time - all.front().time;
    std::printf("%zu samples over %.2f s = %.0f Hz\n", all.size(), span, (all.size() - 1) / span);
    std::printf("mean accel (m/s^2): %+.3f %+.3f %+.3f   |a| = %.3f\n", am.x, am.y, am.z, glm::length(am));
    std::printf("mean gyro (rad/s):  %+.4f %+.4f %+.4f   noise sd: %.4f %.4f %.4f\n", gm.x, gm.y, gm.z, gv.x, gv.y, gv.z);
    std::printf("LED phase transitions seen: %zu; first 25:", phases.size());
    for (size_t i = 0; i < phases.size() && i < 25; ++i) std::printf(" %d", phases[i]);
    std::printf("\nexposure count %u, exposure timestamp %u us\n", hmd.last_exposure_count(), hmd.last_exposure_timestamp_us());
    return 0;
}
