#include "dk2_hid.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <hidapi/hidapi.h>

namespace {
constexpr uint16_t kVid = 0x2833, kPid = 0x0021;

constexpr uint8_t kConfigReport = 0x02;       // 7 bytes
constexpr uint8_t kCalibrationReport = 0x03;  // 69 bytes
constexpr uint8_t kTrackingReport = 0x0c;     // 13 bytes
constexpr uint8_t kKeepaliveReport = 0x11;    // 6 bytes
constexpr uint8_t kSensorMessage = 0x0b;      // 64 bytes (input)

constexpr uint8_t kTrackEnable = 0x01, kTrackAutoIncrement = 0x02, kTrackUseCarrier = 0x04;
constexpr int kKeepaliveTimeoutMs = 10000;

uint64_t now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

uint16_t le16(const uint8_t* p) { return uint16_t(p[0] | p[1] << 8); }
uint32_t le32(const uint8_t* p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }
uint64_t be64(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v = v << 8 | p[i];
    return v;
}

// Three signed 21-bit values packed into a big-endian 64-bit word.
glm::vec3 unpack3x21(const uint8_t* p, float scale) {
    uint64_t v = be64(p);
    return glm::vec3(scale * float(int64_t(v) >> 43), scale * float(int64_t(v << 21) >> 43),
                     scale * float(int64_t(v << 42) >> 43));
}
}  // namespace

bool Dk2Hid::open() {
    if (dev_) return true;
    if (hid_init() != 0) return false;
    dev_ = hid_open(kVid, kPid, nullptr);
    if (!dev_) {
        std::fprintf(stderr, "DK2: cannot open headset (2833:0021). Is another program (e.g. the VR app) using it?\n");
        return false;
    }

    // Sensor config: report rate
    uint8_t cfg[7] = {kConfigReport};
    if (hid_get_feature_report(dev_, cfg, sizeof cfg) > 0) {
        int sample_rate = le16(cfg + 5);
        report_rate_ = sample_rate / (cfg[4] + 1);
        if (report_rate_ <= 0) report_rate_ = 1000;
    }
    load_calibration();
    send_keepalive();
    return true;
}

void Dk2Hid::close() {
    if (!dev_) return;
    // Turn the LEDs off before releasing the headset.
    uint8_t t[13] = {kTrackingReport};
    hid_send_feature_report(dev_, t, sizeof t);
    hid_close(dev_);
    dev_ = nullptr;
    hid_exit();
}

bool Dk2Hid::load_calibration() {
    uint8_t b[69] = {kCalibrationReport};
    if (hid_get_feature_report(dev_, b, sizeof b) < 0) return false;
    accel_offset_ = unpack3x21(b + 3, 1e-4f);
    gyro_offset_ = unpack3x21(b + 11, 1e-4f);
    const float scale = 1.0f / float((1 << 20) - 1);
    for (int i = 0; i < 3; ++i) {
        glm::vec3 a = unpack3x21(b + 19 + 8 * i, scale);
        glm::vec3 g = unpack3x21(b + 43 + 8 * i, scale);
        a[i] += 1.0f;
        g[i] += 1.0f;
        // Rows of the matrix as given; glm is column-major, so store transposed to get row i = a.
        for (int j = 0; j < 3; ++j) { accel_matrix_[j][i] = a[j]; gyro_matrix_[j][i] = g[j]; }
    }
    return true;
}

bool Dk2Hid::send_keepalive() {
    uint8_t k[6] = {kKeepaliveReport, 0, 0, 0x0b, uint8_t(kKeepaliveTimeoutMs & 255), uint8_t(kKeepaliveTimeoutMs >> 8)};
    last_keepalive_ms_ = now_ms();
    return hid_send_feature_report(dev_, k, sizeof k) >= 0;
}

bool Dk2Hid::set_leds(bool blink) {
    leds_blink_ = blink;
    leds_on_ = true;
    uint8_t t[13] = {kTrackingReport};
    // t[1..2] echo, t[3] pattern, t[4] flags, t[5] reserved, t[6..7] exposure_us, t[8..9] period_us,
    // t[10..11] vsync offset, t[12] duty cycle. DK2 values: exposure 350 us, period 16666 us (60 Hz).
    if (blink) {
        t[3] = 0;
        t[4] = kTrackEnable | kTrackUseCarrier | kTrackAutoIncrement;
    } else {
        t[3] = 0xff;
        t[4] = kTrackEnable | kTrackUseCarrier;
    }
    const uint16_t exposure = 350, period = 16666;
    t[6] = exposure & 255; t[7] = exposure >> 8;
    t[8] = period & 255;   t[9] = period >> 8;
    t[10] = 0; t[11] = 0;
    t[12] = 0x7f;
    return hid_send_feature_report(dev_, t, sizeof t) >= 0;
}

int Dk2Hid::poll(std::vector<ImuSample>& out, int timeout_ms) {
    if (!dev_) return -1;
    if (now_ms() - last_keepalive_ms_ > 3000) send_keepalive();

    int added = 0;
    uint8_t buf[64];
    int wait = timeout_ms;
    for (;;) {
        int n = hid_read_timeout(dev_, buf, sizeof buf, wait);
        if (n < 0) return -1;
        if (n == 0) break;
        wait = 0;  // drain whatever else is pending without blocking
        if (buf[0] != kSensorMessage || n < 64) continue;

        const int num_samples = buf[3] > 1 ? 2 : 1;
        const uint32_t ts = le32(buf + 8);
        if (have_ts_) time_base_ += double(uint32_t(ts - last_ts_)) * 1e-6;  // unwrap the 32-bit microsecond clock
        else time_base_ = double(ts) * 1e-6;
        have_ts_ = true;
        last_ts_ = ts;

        led_phase_ = buf[57];
        exposure_count_ = le16(buf + 58);
        exposure_timestamp_ = le32(buf + 60);

        const double interval = 1.0 / report_rate_;
        for (int i = 0; i < num_samples; ++i) {
            const uint8_t* s = buf + 12 + 16 * i;
            ImuSample m;
            // Samples in one message are `interval` apart; the message timestamp belongs to the last one.
            m.time = time_base_ - interval * (num_samples - 1 - i);
            m.accel = accel_matrix_ * (unpack3x21(s, 1e-4f) - accel_offset_);
            m.gyro = gyro_matrix_ * (unpack3x21(s + 8, 1e-4f) - gyro_offset_);
            m.led_phase = led_phase_;
            out.push_back(m);
            ++added;
        }
    }
    return added;
}
