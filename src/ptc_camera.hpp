// DK2 positional tracking camera (PTC): V4L2 capture plus MT9V034 sensor setup over the ESP570 bridge's
// UVC extension unit. In "sync" mode the sensor exposes briefly, triggered by the headset, so only the headset's
// IR LEDs show up. The headset must be kept awake by someone else (e.g. OpenHMD with the device open).
#pragma once
#include <cstdint>
#include <string>
#include <vector>

struct PtcFrame {
    int width = 0, height = 0;
    uint32_t sequence = 0;        // driver frame counter (gaps = dropped frames)
    uint64_t timestamp_us = 0;    // capture timestamp
    std::vector<uint8_t> pix;     // 8-bit greyscale, width*height
};

// Lens calibration stored in the camera's EEPROM (OpenCV model: pinhole + radial k1,k2,k3 + tangential p1,p2).
struct PtcIntrinsics {
    double fx = 0, fy = 0, cx = 0, cy = 0, k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
    bool valid = false;
};

class PtcCamera {
public:
    ~PtcCamera() { stop(); }
    // Finds the "Camera DK2" capture node (or uses `device` if non-empty). Prints errors to stderr.
    bool open(const std::string& device = "");
    // Starts streaming, then switches the sensor to headset-triggered exposure if `sync`.
    bool start(bool sync);
    // Waits up to timeout_ms for the next frame. Returns false on timeout/error.
    bool grab(PtcFrame& out, int timeout_ms = 500);
    void stop();
    // Reads the lens calibration from the camera's EEPROM (works before or after start()).
    PtcIntrinsics read_intrinsics();

private:
    int fd_ = -1;
    bool streaming_ = false, synced_ = false;
    int width_ = 0, height_ = 0;
    struct Buf { void* p; size_t n; };
    std::vector<Buf> bufs_;
};
