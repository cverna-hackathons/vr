// Direct driver for the Oculus Rift DK2 headset over USB HID (via hidapi's libusb backend):
// IMU samples (accelerometer + gyroscope, factory calibrated), keepalive, and control of the IR tracking LEDs.
// Protocol per the open-source ouvrt/OpenHMD reverse engineering. Replaces OpenHMD for tracking, because the
// LEDs' blink mode (needed to identify them in the camera image) can only be set by the process that owns the HID.
#pragma once
#include <cstdint>
#include <vector>
#include <glm/glm.hpp>

struct hid_device_;
typedef struct hid_device_ hid_device;

struct ImuSample {
    double time = 0;          // seconds, from the headset's microsecond clock (unwrapped)
    glm::vec3 accel{0};       // m/s^2, calibrated, headset sensor frame
    glm::vec3 gyro{0};        // rad/s, calibrated, headset sensor frame
    uint8_t led_phase = 0;    // LED blink pattern phase (0..9) at the time of this message
};

class Dk2Hid {
public:
    ~Dk2Hid() { close(); }
    bool open();
    void close();
    bool is_open() const { return dev_ != nullptr; }

    // Enable the IR LEDs. blink = true: each LED blinks its own 10-bit pattern (auto-increment phase);
    // false: all LEDs constantly on.
    bool set_leds(bool blink);

    // Reads all pending input reports (waiting up to timeout_ms for the first), appending decoded IMU samples.
    // Also sends the keepalive when due. Returns the number of IMU samples appended, or -1 on a device error.
    int poll(std::vector<ImuSample>& out, int timeout_ms);

    int sample_rate_hz() const { return report_rate_; }
    uint8_t last_led_phase() const { return led_phase_; }
    uint16_t last_exposure_count() const { return exposure_count_; }
    uint32_t last_exposure_timestamp_us() const { return exposure_timestamp_; }

private:
    bool send_keepalive();
    bool load_calibration();

    hid_device* dev_ = nullptr;
    int report_rate_ = 1000;
    uint64_t last_keepalive_ms_ = 0;
    bool leds_blink_ = true, leds_on_ = false;

    // Factory calibration: calibrated = matrix * (raw - offset)
    glm::vec3 accel_offset_{0}, gyro_offset_{0};
    glm::mat3 accel_matrix_{1.0f}, gyro_matrix_{1.0f};

    uint32_t last_ts_ = 0;
    bool have_ts_ = false;
    double time_base_ = 0;
    uint8_t led_phase_ = 0;
    uint16_t exposure_count_ = 0;
    uint32_t exposure_timestamp_ = 0;
};
