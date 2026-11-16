// Read the Rift DK2's IR LED layout (3D positions + blink patterns) from the headset over HID
// feature reports 0x0f (positions) and 0x10 (patterns), and write it to a text file.
//
// Build: g++ -O2 -Wall -o build/ledlayout tools/ledlayout.cpp $(pkg-config --cflags --libs hidapi-libusb)
// (The DK2 headset has no /dev/hidraw node, so access goes through hidapi's libusb backend, like OpenHMD.)
// Run:   build/ledlayout [data/dk2_leds.txt]
// Output lines:  index  x y z (metres, headset frame)  nx ny nz (LED normal)  pattern (10 bits)
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <hidapi/hidapi.h>
#include <vector>

static int32_t le32(const uint8_t* p) { return int32_t(p[0] | p[1] << 8 | p[2] << 16 | uint32_t(p[3]) << 24); }
static int16_t le16(const uint8_t* p) { return int16_t(p[0] | p[1] << 8); }

struct Led { float pos[3]{}, nrm[3]{}; uint32_t pattern = 0; bool is_led = false; };

int main(int argc, char** argv) {
    const char* out = argc > 1 ? argv[1] : "data/dk2_leds.txt";
    if (hid_init() != 0) { std::fprintf(stderr, "hid_init failed\n"); return 1; }
    hid_device* dev = hid_open(0x2833, 0x0021, nullptr);
    if (!dev) { std::fprintf(stderr, "Cannot open DK2 headset (2833:0021): %ls\n", hid_error(nullptr)); return 1; }
    std::printf("Opened DK2 headset\n");

    std::vector<Led> leds;
    float imu[3] = {0, 0, 0};
    int num = -1;

    // --- Positions (report 0x0f, 30 bytes). Each GET_FEATURE returns the next entry. ---
    for (int i = 0; num < 0 || i < num; ++i) {
        uint8_t b[30] = {0x0f};
        int r = hid_get_feature_report(dev, b, sizeof b);
        if (r < 0) { std::fprintf(stderr, "get feature 0x0f failed: %ls\n", hid_error(dev)); return 1; }
        int index = le16(b + 24);
        num = le16(b + 26);
        int type = le16(b + 28);
        if (num <= 0 || num > 100 || index >= num) { std::fprintf(stderr, "Bad position entry: index=%d num=%d\n", index, num); return 1; }
        if (leds.empty()) leds.resize(num);
        float p[3] = {1e-6f * le32(b + 4), 1e-6f * le32(b + 8), 1e-6f * le32(b + 12)};  // micrometres -> metres
        if (type == 0) {
            Led& L = leds[index];
            std::memcpy(L.pos, p, sizeof p);
            float n[3] = {float(le16(b + 16)), float(le16(b + 18)), float(le16(b + 20))};
            float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            for (int k = 0; k < 3; ++k) L.nrm[k] = len > 0 ? n[k] / len : 0;
            L.is_led = true;
        } else {
            std::memcpy(imu, p, sizeof p);
        }
    }
    std::printf("Positions: %d entries\n", num);

    // --- Blink patterns (report 0x10, 12 bytes) ---
    int pnum = -1;
    for (int i = 0; pnum < 0 || i < pnum; ++i) {
        uint8_t b[12] = {0x10};
        int r = hid_get_feature_report(dev, b, sizeof b);
        if (r < 0) { std::fprintf(stderr, "get feature 0x10 failed: %ls\n", hid_error(dev)); return 1; }
        uint32_t raw = uint32_t(le32(b + 4));
        int index = le16(b + 8);
        pnum = le16(b + 10);
        if (b[3] != 10 || index >= int(leds.size()) || pnum > int(leds.size())) {
            std::fprintf(stderr, "Bad pattern entry: len=%d index=%d num=%d\n", b[3], index, pnum);
            return 1;
        }
        if ((raw & ~0xaaaaau) != 0x55555u) { std::fprintf(stderr, "Unexpected pattern 0x%x\n", raw); return 1; }
        // 10 two-bit values, 1 = dark, 3 = bright  ->  10 single bits (LSB = first)
        uint32_t bits = 0;
        for (int k = 0; k < 10; ++k) if (((raw >> (2 * k)) & 3) == 3) bits |= 1u << k;
        leds[index].pattern = bits;
    }
    std::printf("Patterns: %d entries\n", pnum);

    hid_close(dev);
    hid_exit();

    FILE* f = std::fopen(out, "w");
    if (!f) { std::perror(out); return 1; }
    std::fprintf(f, "# Rift DK2 IR LEDs: index x y z (m) nx ny nz pattern(10 bits, first bit = LSB)\n");
    std::fprintf(f, "# imu_position %.6f %.6f %.6f\n", imu[0], imu[1], imu[2]);
    for (size_t i = 0; i < leds.size(); ++i) {
        if (!leds[i].is_led) continue;
        const Led& L = leds[i];
        char bits[11];
        for (int k = 0; k < 10; ++k) bits[k] = (L.pattern >> k & 1) ? '1' : '0';
        bits[10] = 0;
        std::fprintf(f, "%zu %.6f %.6f %.6f %.4f %.4f %.4f %s\n", i, L.pos[0], L.pos[1], L.pos[2], L.nrm[0], L.nrm[1], L.nrm[2], bits);
    }
    std::fclose(f);
    size_t nled = 0; for (const Led& L : leds) nled += L.is_led; std::printf("Wrote %s (%zu LEDs)\n", out, nled);
    return 0;
}
