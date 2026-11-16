// Live test: find, follow and identify the DK2's LEDs in the PTC image. Prints a summary once per second.
// Run: build/ptc_ids [seconds] [data/dk2_leds.txt]
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <vector>

#include "../src/dk2_hid.hpp"
#include "../src/ptc_camera.hpp"
#include "../src/ptc_tracker.hpp"

int main(int argc, char** argv) {
    double seconds = argc > 1 ? std::atof(argv[1]) : 10;
    const char* ledfile = argc > 2 ? argv[2] : "data/dk2_leds.txt";

    LedModel model;
    if (!model.load(ledfile)) { std::fprintf(stderr, "Cannot load %s\n", ledfile); return 1; }
    std::printf("Loaded %zu LEDs\n", model.leds.size());

    Dk2Hid hmd;
    if (!hmd.open() || !hmd.set_leds(true)) { std::fprintf(stderr, "DK2 not available (is ./build/vr running?)\n"); return 1; }
    PtcCamera cam;
    if (!cam.open() || !cam.start(true)) return 1;

    PtcBlobTracker tracker(model);
    std::vector<ImuSample> imu;
    PtcFrame fr;
    auto t0 = std::chrono::steady_clock::now(), last_print = t0;
    int frames = 0, id_frames = 0, blob_frames = 0, misses = 0;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds && misses < 20) {
        imu.clear();
        hmd.poll(imu, 0);
        if (!cam.grab(fr, 100)) { ++misses; continue; }
        const auto& blobs = tracker.process(fr);
        ++frames;
        int ids = 0;
        for (const Blob& b : blobs) ids += b.led >= 0;
        id_frames += ids; blob_frames += int(blobs.size());
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_print).count() >= 1.0) {
            last_print = now;
            std::printf("t=%4.1fs frames=%d  blobs=%2zu identified=%2d |", std::chrono::duration<double>(now - t0).count(), frames, blobs.size(), ids);
            std::map<int, const Blob*> by;
            for (const Blob& b : blobs) if (b.led >= 0) by[b.led] = &b;
            for (auto& kv : by) std::printf(" %d@(%.0f,%.0f)", kv.first, kv.second->x, kv.second->y);
            std::printf("\n");
        }
    }
    std::printf("Done: %d frames, avg %.1f blobs/frame, %.1f identified/frame\n", frames,
                frames ? double(blob_frames) / frames : 0, frames ? double(id_frames) / frames : 0);
    return 0;
}
