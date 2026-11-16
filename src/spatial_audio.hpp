// Positional audio: procedurally synthesised sounds placed in 3D around the listener, rendered to stereo headphones with a simple
// head model: interaural time difference (the far ear hears it up to 0.66 ms later), interaural level difference and head-shadow
// low-pass (the far ear is duller and quieter), a pinna-style dulling for sources behind, distance attenuation, and muffling when
// a wall is between you and the source. No sound files are needed: loops are generated at start-up.
#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

enum SoundKind { kSoundWater = 0, kSoundLeaves, kSoundBird, kSoundHum, kSoundFire, kSoundKinds };
enum StepKind { kStepStone = 0, kStepGrass, kStepSand, kStepWood, kStepKinds };

struct SoundSource {
    int id = 0;                 // stable identity (e.g. the map cell), so a source keeps its phase and fades smoothly
    int kind = kSoundWater;
    glm::vec3 pos{0.0f};        // world position, metres
    float occlusion = 0.0f;     // 0 open .. 1 completely behind walls
};

class SpatialAudio {
public:
    ~SpatialAudio() { close(); }
    // device=false only synthesises (for tests); render() can then be called directly.
    bool open(bool device = true);
    void close();

    // Listener: ear-centre position and head -> world rotation (-Z forward, +X right, +Y up in head space).
    void set_listener(const glm::vec3& pos, const glm::quat& head_to_world);
    // The sources to play now (the caller picks the nearest few). Sources that disappear fade out.
    void set_sources(const std::vector<SoundSource>& sources);
    void play_step(int step_kind, const glm::vec3& pos, float gain = 1.0f);

    // Call once per frame. If the audio device stops asking for samples (another application grabbed or switched the output,
    // a Bluetooth profile changed, the stream was killed), reopen the default device. Returns true if it just reopened.
    bool service();

    void set_volume(float v) { volume_ = v < 0 ? 0 : (v > 1.5f ? 1.5f : v); }
    float volume() const { return volume_; }
    void toggle_mute() { muted_ = !muted_; }
    bool muted() const { return muted_; }

    // Mixes `frames` stereo frames (interleaved float L,R) into `out` (overwrites). This is the audio callback body.
    void render(float* out, int frames);

    static int sound_kind(const std::string& name);   // "water" -> kSoundWater, ... ; -1 for none/unknown
    static int step_kind(const std::string& name);
    static constexpr int kRate = 48000;
    static constexpr int kMaxVoices = 20;
    static constexpr int kStepVariants = 6;   // every footstep kind has several different recordings, picked at random

private:
    struct Voice {
        bool active = false, one_shot = false, ending = false;
        int id = 0, kind = 0;
        const std::vector<float>* buf = nullptr;
        size_t pos = 0;
        float fpos = 0, rate = 1;                        // one-shots: fractional position and playback speed (pitch variation)
        float gl = 0, gr = 0, tgl = 0, tgr = 0;          // ear gains (current / target)
        float dl = 0, dr = 0, tdl = 0, tdr = 0;          // ear delays in samples
        float al = 1, ar = 1, tal = 1, tar = 1;          // low-pass coefficients
        float lpl = 0, lpr = 0;                          // low-pass states
        float ring[256] = {0};
        unsigned wr = 0;
    };
    struct Params { float gl, gr, dl, dr, al, ar; };
    Params spatialise(const glm::vec3& pos, float base_gain, float occlusion) const;
    Voice* allocate();

    void synthesise();
    std::vector<float> loops_[kSoundKinds], steps_[kStepKinds][kStepVariants], wind_l_, wind_r_;
    uint32_t rng_ = 2463534242u;
    float rand01() { rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5; return float(rng_ & 0xffffff) / float(0x1000000); }
    float loop_gain_[kSoundKinds];

    std::mutex mu_;
    Voice voices_[kMaxVoices];
    glm::vec3 listener_pos_{0.0f};
    glm::quat listener_q_{1, 0, 0, 0};
    size_t wind_pos_ = 0;
    float wind_gain_ = 0.0f;
    float volume_ = 0.8f;
    bool muted_ = false;
    unsigned device_ = 0;
    bool open_device();
    std::atomic<uint64_t> callbacks_{0};     // audio callbacks so far (the watchdog watches this)
    std::atomic<unsigned> device_id_{0};     // opened device id, visible to the SDL event watch
    std::atomic<bool> removed_{false};       // SDL reported our device as disconnected (SDL keeps calling the callback, discarding the sound)
    void drop_device();
    uint64_t seen_callbacks_ = 0;
    uint32_t last_progress_ms_ = 0, last_attempt_ms_ = 0;
    bool device_wanted_ = false, lost_reported_ = false;
};
