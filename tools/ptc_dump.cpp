// Record N synced PTC frames to a file for offline analysis. The headset is kept awake by our own HID driver,
// which also turns on its IR LEDs (blinking their ID patterns) and the sync output.
//
// Build: see CMakeLists.txt (target ptc_dump).  Run: build/ptc_dump out.bin [frames]
// File format: repeated records of  uint32 sequence, uint64 timestamp_us, uint32 w, uint32 h, w*h bytes.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

#include "../src/ptc_camera.hpp"
#include "../src/dk2_hid.hpp"

int main(int argc, char** argv) {
    const char* path = argc > 1 ? argv[1] : "ptc.bin";
    int frames = argc > 2 ? std::atoi(argv[2]) : 60;

    Dk2Hid hmd;
    if (!hmd.open() || !hmd.set_leds(true)) { std::fprintf(stderr, "DK2 headset not available (is ./build/vr still running?)\n"); return 1; }

    PtcCamera cam;
    if (!cam.open() || !cam.start(true)) return 1;

    FILE* f = std::fopen(path, "wb");
    if (!f) { std::perror(path); return 1; }
    int got = 0, misses = 0;
    PtcFrame fr;
    std::vector<ImuSample> imu;
    while (got < frames && misses < 20) {
        imu.clear();
        hmd.poll(imu, 0);  // drains IMU reports and sends keepalives so the headset stays awake
        if (!cam.grab(fr, 100)) { ++misses; continue; }
        uint32_t seq = fr.sequence, w = fr.width, h = fr.height;
        std::fwrite(&seq, 4, 1, f);
        std::fwrite(&fr.timestamp_us, 8, 1, f);
        std::fwrite(&w, 4, 1, f);
        std::fwrite(&h, 4, 1, f);
        std::fwrite(fr.pix.data(), 1, fr.pix.size(), f);
        ++got;
    }
    std::fclose(f);
    std::printf("Recorded %d frames to %s (%d timeouts)\n", got, path, misses);
    cam.stop();
    return got > 0 ? 0 : 2;
}
