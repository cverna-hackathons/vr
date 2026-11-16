// Screen capture of one rectangle of the X11 root window (XShm), with the mouse pointer composited in.
#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <chrono>
#include <thread>
#include <vector>

struct XRect { int x = 0, y = 0, w = 0, h = 0; };

// Geometry of a connected RandR output; name empty = first "eDP*" (built-in) output.
bool find_output_rect(const std::string& name, XRect& out, std::string& found_name);

class ScreenGrabber {
public:
    ~ScreenGrabber() { stop(); }
    // cursor_scale: integer magnification of the mouse pointer drawn into the frames (it is tiny in the headset).
    bool start(const XRect& rect, int fps, int cursor_scale = 1);
    void stop();
    // Swaps the newest frame (BGRA, rect.w * rect.h * 4 bytes) into `out` if one arrived since the last call.
    bool take(std::vector<uint8_t>& out);
    double measured_fps() const { return fps_; }
    // Average milliseconds per frame spent waiting for the X server's screen copy, and copying/drawing the cursor (diagnostics).
    double grab_ms() const { return grab_ms_; }
    double copy_ms() const { return copy_ms_; }

private:
    void run(int index, int count);   // `count` threads capture in turn, overlapping, so one slow screen copy doesn't cap the frame rate
    XRect rect_;
    int fps_target_ = 30, cursor_scale_ = 1;
    std::vector<std::thread> threads_;
    std::atomic<long> frames_{0};
    std::chrono::steady_clock::time_point latest_{};
    std::atomic<bool> quit_{false};
    std::atomic<double> fps_{0}, grab_ms_{0}, copy_ms_{0};
    std::mutex mu_;
    std::vector<uint8_t> ready_;
    bool fresh_ = false;
};
