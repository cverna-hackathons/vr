// Looping video decoder (FFmpeg) running on its own thread; delivers RGB frames paced by the file's timestamps.
#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class VideoPlayer {
public:
    ~VideoPlayer() { stop(); }
    // Opens the file and starts decoding. Frames are scaled to at most max_width pixels wide.
    bool start(const std::string& path, int max_width = 1024);
    void stop();
    // Copies the newest frame into `rgb` (width()*height()*3 bytes) if one arrived since the last call.
    bool take_frame(std::vector<uint8_t>& rgb);
    int width() const { return w_; }
    int height() const { return h_; }

private:
    void run();
    std::string path_;
    int w_ = 0, h_ = 0, src_w_ = 0, src_h_ = 0;
    std::thread thread_;
    std::atomic<bool> quit_{false};
    std::mutex mu_;
    std::vector<uint8_t> frame_;
    bool fresh_ = false;
};
