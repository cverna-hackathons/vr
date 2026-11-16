// Movie player (FFmpeg + SDL audio): separate demux / audio / video threads, video paced by the audio clock.
// Supports pause, relative seeking and volume. Frames are delivered as RGB for upload to a texture.
#pragma once
#include <SDL.h>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct AVFormatContext;
struct AVPacket;

class MoviePlayer {
public:
    ~MoviePlayer() { close(); }
    bool open(const std::string& path, int max_width = 1280);
    void close();

    bool take_frame(std::vector<uint8_t>& rgb);  // newest frame since the last call, if any
    int width() const { return w_; }
    int height() const { return h_; }
    double duration() const { return duration_; }  // seconds (0 if unknown)
    double position();                             // current playback time, seconds

    void toggle_pause();
    bool paused() const { return paused_; }
    void seek_relative(double seconds);
    void set_volume(float v) { volume_ = v < 0 ? 0 : (v > 1.5f ? 1.5f : v); }
    float volume() const { return volume_; }
    bool has_audio() const { return has_audio_; }
    // Call once per frame: if the audio output stopped (another application took it, the device changed) reopen it, so the
    // sound -- and with it the picture, which follows the audio clock -- carries on.
    void service();

private:
    struct Item { AVPacket* pkt; int serial; };  // pkt == nullptr marks end of file
    struct Queue {
        std::deque<Item> q;
        size_t bytes = 0;
        std::mutex m;
        std::condition_variable cv;
    };

    void demux_thread();
    void audio_thread();
    void video_thread();
    bool pop(Queue& q, Item& out);
    void push(Queue& q, Item it);
    void clear(Queue& q);
    static void audio_callback(void* user, Uint8* stream, int len);
    void fill_audio(Uint8* stream, int len);
    bool clock_valid();

    std::string path_;
    AVFormatContext* fmt_ = nullptr;
    int vidx_ = -1, aidx_ = -1;
    double vtb_ = 0, atb_ = 0, duration_ = 0;
    int w_ = 0, h_ = 0;
    bool has_audio_ = false;

    std::thread t_demux_, t_audio_, t_video_;
    std::atomic<bool> quit_{false};
    std::atomic<int> serial_{0};
    std::atomic<double> seek_req_{-1.0};
    std::atomic<double> target_pts_{0.0};  // after a seek, drop frames before this time
    Queue vq_, aq_;

    // Audio output + clock (guarded by amu_).
    SDL_AudioDeviceID dev_ = 0;
    bool open_device();
    std::atomic<uint64_t> callbacks_{0};
    uint64_t seen_callbacks_ = 0;
    uint32_t last_progress_ms_ = 0, last_attempt_ms_ = 0;
    std::atomic<unsigned> device_id_{0};
    std::atomic<bool> removed_{false};   // SDL says our device was disconnected (it keeps calling the callback and discards the sound)
    void drop_device();
    static int SDLCALL on_event(void* user, SDL_Event* e);
    std::mutex amu_;
    std::vector<int16_t> ring_;
    size_t rpos_ = 0;
    bool clock_ok_ = false;
    double base_pts_ = 0;
    uint64_t played_ = 0;  // sample frames actually played since base_pts_
    std::atomic<float> volume_{1.0f};
    std::atomic<bool> paused_{false};

    std::mutex fmu_;
    std::vector<uint8_t> frame_;
    bool fresh_ = false;
};
