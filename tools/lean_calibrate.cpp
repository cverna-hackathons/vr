// Guided calibration of the tracked position directions, using the headset's accelerometer as ground truth.
//
//   build/lean_calibrate            (close every other VR program first: only one can use the headset)
//
// Hold the headset in front of your face as if wearing it, lenses towards you, facing the camera (or wear it and lean blind).
// A first phase checks that the camera's rotation agrees with the IMU's while you turn the headset; then three phases of 8 seconds each: (1) lean/move TOWARD and AWAY from the camera, (2) LEFT and RIGHT, (3) UP and DOWN. Move back and forth
// about once per second with clear 10-20 cm strokes, without turning the headset. For each axis the tool decides whether the tracked
// position already moves the same way as you really did, and saves the result for all programs (~/.config/dk2vr/position_signs).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

#include "../src/head_tracker.hpp"
#include "../src/lean_calibration.hpp"

using Clock = std::chrono::steady_clock;

static double now_s(Clock::time_point t0) { return std::chrono::duration<double>(Clock::now() - t0).count(); }

// Spread (median angle, degrees) of C_i = q_imu_i * conj(q_cam_i) over a recording, for a candidate mirror mapping applied to the raw camera pose.
// If the mapping is right, C is the same fixed rotation (the camera's mounting) for every pose.
static const glm::mat3 kCamToViewM(glm::vec3(-1, 0, 0), glm::vec3(0, -1, 0), glm::vec3(0, 0, 1));
static double mapping_spread(const std::vector<glm::quat>& q_imu, const std::vector<glm::quat>& q_hc, const glm::mat3& Mc, const glm::mat3& Ml) {
    std::vector<glm::quat> cs;
    for (size_t i = 0; i < q_imu.size(); ++i) {
        const glm::mat3 R_raw = glm::inverse(kCamToViewM) * glm::mat3_cast(q_hc[i]);      // undo the fixed view flip: the solver's pose
        const glm::quat q2 = glm::quat_cast(kCamToViewM * (Mc * R_raw * Ml));
        cs.push_back(q_imu[i] * glm::conjugate(q2));
    }
    glm::vec4 sum(0);
    for (const glm::quat& q : cs) { glm::vec4 v(q.x, q.y, q.z, q.w); if (glm::dot(v, sum) < 0) v = -v; sum += v; }
    const glm::vec4 n = glm::normalize(sum);
    std::vector<double> dev;
    for (const glm::quat& q : cs) dev.push_back(glm::degrees(2.0 * std::acos(std::min(1.0, std::fabs(double(glm::dot(glm::vec4(q.x, q.y, q.z, q.w), n)))))));
    std::sort(dev.begin(), dev.end());
    return dev[dev.size() / 2];
}

struct Phase { const char* title; const char* what; int axis; const char* axis_name; };

int main(int argc, char** argv) {
    bool dry = false;
    for (int i = 1; i < argc; ++i) if (!std::strcmp(argv[i], "--dry-run")) dry = true;   // analyse and print, do not save

    HeadTracker tracker;
    if (!tracker.start("data/dk2_leds.txt", true) || !tracker.camera_active()) {
        std::fprintf(stderr, "Could not start the headset and camera. Close other VR programs (vr, theater, display, explore) and try again.\n");
        return 1;
    }
    tracker.set_calibration_mode(true);

    std::printf("\nPosition calibration\n====================\n");
    std::printf("Hold the headset in front of your face, lenses towards you, facing the camera, at the height you normally use it.\n");
    std::printf("Waiting for the camera to see the headset");
    std::fflush(stdout);
    {
        const auto t0 = Clock::now();
        double last = 0;
        while (!tracker.camera_tracking() && now_s(t0) < 30) {
            tracker.update();
            if (now_s(t0) - last >= 2.0) {
                last = now_s(t0);
                const auto c = tracker.camera_status();
                std::printf("\n  camera poses %llu, LEDs in the last pose %d (at least 8 needed), alignment %d/25%s", (unsigned long long)c.fixes, c.last_leds, c.align_progress,
                            c.last_leds < 8 ? "  -> the camera does not see enough LEDs: face the front of the headset to the camera" : (!c.still ? "  -> hold the headset still for a moment" : ""));
                std::fflush(stdout);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
        if (!tracker.camera_tracking()) { std::printf("\nThe camera does not see the headset well enough. Check that the front of the headset (with the LEDs) faces the camera.\n"); return 2; }
    }
    std::printf(" tracking.\n");
    // Settle the orientation (so "forward" = the way you hold it now) and take the starting position as the origin.
    tracker.recentre();
    for (int i = 0; i < 90; ++i) { tracker.update(); std::this_thread::sleep_for(std::chrono::milliseconds(16)); }

    // Phase 0: does the camera's idea of the headset's rotation agree with the IMU's? (If the pose geometry were wrong, they would disagree
    // as soon as you turn.)
    {
        std::printf("\nPhase 0: TURN the headset slowly: look left and right, up and down, about 30 degrees each way, for 8 seconds.\n");
        for (int c = 3; c >= 1; --c) { std::printf("  starting in %d...\r", c); std::fflush(stdout); const auto t0 = Clock::now(); while (now_s(t0) < 1.0) { tracker.update(); std::this_thread::sleep_for(std::chrono::milliseconds(16)); } }
        std::printf("  GO\a\n");
        std::vector<float> err;
        std::vector<glm::quat> rec_imu, rec_hc;
        float yaw_lo = 1e9f, yaw_hi = -1e9f;
        uint64_t last_fix = tracker.camera_status().fixes, rec_last = last_fix;
        const auto t0 = Clock::now();
        while (now_s(t0) < 8.0) {
            tracker.update();
            const auto c = tracker.camera_status();
            if (c.fixes != last_fix && c.aligned) { err.push_back(c.last_orientation_error_deg); last_fix = c.fixes; }
            if (c.fixes != rec_last && c.last_leds >= 8) { rec_imu.push_back(c.last_q_imu); rec_hc.push_back(c.last_q_hc); rec_last = c.fixes; }
            const glm::vec3 f = tracker.orientation_raw() * glm::vec3(0, 0, -1);
            const float yaw = std::atan2(-f.x, -f.z) * 57.29578f;
            yaw_lo = std::min(yaw_lo, yaw);
            yaw_hi = std::max(yaw_hi, yaw);
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
        std::printf("  done.\a\n");
        if (err.size() < 30 || yaw_hi - yaw_lo < 20.0f) {
            std::printf("  RESULT rotation check: not enough turning or tracking (turned %.0f degrees, %zu poses). Skipped.\n", yaw_hi - yaw_lo, err.size());
        } else {
            std::sort(err.begin(), err.end());
            const float med = err[err.size() / 2], p90 = err[size_t(err.size() * 0.9)];
            std::printf("  turned %.0f degrees; camera-vs-IMU rotation disagreement: median %.1f, 90%% of poses below %.1f degrees\n", yaw_hi - yaw_lo, med, p90);
            if (p90 < 8.0f) std::printf("  RESULT rotation check: GOOD - the camera and the IMU agree about rotation.\n");
            else std::printf("  RESULT rotation check: POOR - they disagree by up to %.0f degrees while turning. The pose geometry is still wrong; please tell me this number.\n", p90);
            // Which camera/model mirror mapping makes the camera and the IMU agree best? (identity is what the program uses now)
            if (rec_imu.size() >= 30) {
                const glm::mat3 maps[4] = {glm::mat3(1.0f), glm::mat3(glm::vec3(-1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1)),
                                           glm::mat3(glm::vec3(1, 0, 0), glm::vec3(0, -1, 0), glm::vec3(0, 0, 1)), glm::mat3(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, -1))};
                const char* names[4] = {"none", "mirror X", "mirror Y", "mirror Z"};
                struct Row { double spread; int c, l; };
                std::vector<Row> rows;
                for (int c = 0; c < 4; ++c)
                    for (int l = 0; l < 4; ++l)
                        if (glm::determinant(maps[c]) * glm::determinant(maps[l]) > 0) rows.push_back({mapping_spread(rec_imu, rec_hc, maps[c], maps[l]), c, l});
                std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) { return a.spread < b.spread; });
                double identity = 0;
                for (const Row& r : rows) if (r.c == 0 && r.l == 0) identity = r.spread;
                std::printf("  mapping check (median spread of the camera-to-IMU rotation over the %zu poses; small = consistent):\n", rec_imu.size());
                for (size_t i = 0; i < rows.size() && i < 4; ++i)
                    std::printf("    camera side %-8s, model side %-8s: %6.1f degrees%s\n", names[rows[i].c], names[rows[i].l], rows[i].spread, (rows[i].c == 0 && rows[i].l == 0) ? "   <- used now" : "");
                if (rows[0].spread < identity - 5.0 && !(rows[0].c == 0 && rows[0].l == 0))
                    std::printf("  NOTE: another mapping fits much better (%.1f vs %.1f degrees). Please send me these lines.\n", rows[0].spread, identity);
            }
        }
    }

    const Phase phases[3] = {
        {"1/3", "FORWARD and BACK: toward the camera and away from it", 2, "Z (forward/back)"},
        {"2/3", "LEFT and RIGHT: sideways, keeping the headset facing the camera", 0, "X (left/right)"},
        {"3/3", "UP and DOWN: like standing up and sitting down, or lifting and lowering it", 1, "Y (up/down)"},
    };
    glm::vec3 signs = tracker.position_signs();
    bool trusted_all = true;

    for (const Phase& ph : phases) {
        std::printf("\nPhase %s: %s.\n", ph.title, ph.what);
        for (int c = 3; c >= 1; --c) { std::printf("  starting in %d...\r", c); std::fflush(stdout); const auto t0 = Clock::now(); while (now_s(t0) < 1.0) { tracker.update(); std::this_thread::sleep_for(std::chrono::milliseconds(16)); } }
        std::printf("  GO - move back and forth about once a second for 8 seconds.\a\n");
        std::vector<double> t;
        std::vector<glm::vec3> pos, acc;
        size_t frames = 0, tracked = 0;
        const auto t0 = Clock::now();
        double last_report = 0;
        float lo = 1e9f, hi = -1e9f;
        glm::vec3 a_sum(0);
        int a_n = 0;
        while (now_s(t0) < 8.0) {
            tracker.update();
            glm::vec3 a;
            if (tracker.take_user_accel(a)) { a_sum += a; ++a_n; }
            ++frames;
            if (tracker.camera_tracking()) {
                ++tracked;
                const glm::vec3 p = tracker.position_raw();
                t.push_back(now_s(t0));
                pos.push_back(p);
                acc.push_back(a_n ? a_sum / float(a_n) : glm::vec3(0));
                lo = std::min(lo, p[ph.axis]);
                hi = std::max(hi, p[ph.axis]);
            }
            a_sum = glm::vec3(0);
            a_n = 0;
            if (now_s(t0) - last_report >= 2.0) {
                last_report = now_s(t0);
                std::printf("  ... %s range so far: %.0f cm%s\n", ph.axis_name, hi > lo ? (hi - lo) * 100.0f : 0.0f, hi - lo < 0.15f ? "  (move more)" : "");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
        std::printf("  done.\a\n");

        std::vector<float> p_axis, a_axis;
        for (size_t i = 0; i < t.size(); ++i) { p_axis.push_back(pos[i][ph.axis]); a_axis.push_back(acc[i][ph.axis]); }
        const LeanAxisResult r = analyse_lean_axis(t, p_axis, a_axis);

        // Did you move mostly along the intended axis? (otherwise the other axes would contaminate the reading)
        double var[3] = {0, 0, 0};
        if (pos.size() > 10) {
            glm::vec3 mean(0);
            for (const auto& p : pos) mean += p;
            mean /= float(pos.size());
            for (const auto& p : pos) for (int k = 0; k < 3; ++k) var[k] += double(p[k] - mean[k]) * double(p[k] - mean[k]);
        }
        const double vmax = std::max(var[0], std::max(var[1], var[2]));
        const bool on_axis = vmax > 0 && var[ph.axis] > 0.45 * vmax;

        std::printf("  tracked %zu of %zu frames; lean amplitude %.0f cm; best camera delay %.0f ms; correlation %+.2f\n", tracked, frames,
                    r.amplitude_m * 100.0f, r.lag_ms, r.correlation);
        if (!on_axis && vmax > 0) std::printf("  note: most of the movement was along a different axis than requested.\n");
        if (r.trusted && on_axis && tracked * 10 > frames * 6) {
            signs[ph.axis] = r.sign;
            std::printf("  RESULT %s: the tracked position %s -> sign %+.0f\n", ph.axis_name, r.sign > 0 ? "already moves the same way as you did" : "moves OPPOSITE to you", r.sign);
        } else {
            trusted_all = false;
            std::printf("  RESULT %s: not decided (%s). Keeping the current sign %+.0f.\n", ph.axis_name,
                        tracked * 10 <= frames * 6 ? "the camera lost the headset too often" : (r.amplitude_m <= 0.04f ? "too little movement" : (!on_axis ? "wrong direction" : "no clear relation")),
                        signs[ph.axis]);
        }
    }

    std::printf("\nResult: position signs X %+.0f  Y %+.0f  Z %+.0f%s\n", signs.x, signs.y, signs.z, trusted_all ? "" : "  (some axes undecided; run again with bigger, steadier movements)");
    if (dry) std::printf("Dry run: nothing saved.\n");
    else {
        tracker.set_position_signs(signs);
        std::printf("Saved. All programs now use these signs (vr, theater, display, explore).\n");
    }
    tracker.stop();
    return trusted_all ? 0 : 3;
}
