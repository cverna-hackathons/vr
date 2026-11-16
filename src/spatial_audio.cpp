#include "spatial_audio.hpp"

#include <SDL.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {
constexpr float kPi = 3.14159265358979f;
constexpr int R = SpatialAudio::kRate;

struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 1) {}
    float next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return float(s & 0xffffff) / float(0x1000000); }  // [0,1)
    float noise() { return next() * 2.0f - 1.0f; }
};

std::vector<float> white(Rng& r, size_t n) {
    std::vector<float> v(n);
    for (float& x : v) x = r.noise();
    return v;
}

// Low-pass in place: `poles` cascaded one-pole stages (6 dB/octave each; 2 poles = 12 dB/octave, much duller than a single pole).
void lowpass(std::vector<float>& v, float fc, int poles = 2) {
    const float a = 1.0f - std::exp(-2.0f * kPi * fc / float(R));
    for (int p = 0; p < poles; ++p) {
        float y = 0;
        for (float& x : v) { y += a * (x - y); x = y; }
    }
}

std::vector<float> lowpassed(const std::vector<float>& v, float fc, int poles = 2) {
    std::vector<float> o = v;
    lowpass(o, fc, poles);
    return o;
}

float rms(const std::vector<float>& v) {
    double a = 0;
    for (float x : v) a += double(x) * x;
    return float(std::sqrt(a / double(v.size())));
}

void normalise_rms(std::vector<float>& v, float target) {
    const float r = rms(v);
    if (r > 1e-9f) for (float& x : v) x *= target / r;
}

// Make a loop seamless: `x` has n + xf samples; the last xf samples are cross-faded into the first xf.
std::vector<float> seamless(const std::vector<float>& x, size_t n) {
    const size_t xf = x.size() - n;
    std::vector<float> o(x.begin(), x.begin() + long(n));
    for (size_t i = 0; i < xf; ++i) {
        const float w = float(i) / float(xf);
        o[i] = x[i] * w + x[n + i] * (1.0f - w);
    }
    return o;
}

void add_chirp(std::vector<float>& v, size_t start, float f0, float f1, float dur_s, float amp) {
    const size_t n = size_t(dur_s * R);
    double ph = 0;
    for (size_t i = 0; i < n && start + i < v.size(); ++i) {
        const float t = float(i) / float(n);
        const float f = f0 + (f1 - f0) * t;
        ph += 2.0 * kPi * f / R;
        const float env = std::sin(kPi * t);
        v[start + i] += amp * env * env * float(std::sin(ph));
    }
}
}  // namespace

int SpatialAudio::sound_kind(const std::string& n) {
    if (n == "water") return kSoundWater;
    if (n == "leaves") return kSoundLeaves;
    if (n == "bird") return kSoundBird;
    if (n == "hum") return kSoundHum;
    if (n == "fire") return kSoundFire;
    return -1;
}

int SpatialAudio::step_kind(const std::string& n) {
    if (n == "stone") return kStepStone;
    if (n == "grass") return kStepGrass;
    if (n == "sand") return kStepSand;
    if (n == "wood") return kStepWood;
    return -1;
}

// ---- synthesis -------------------------------------------------------------------------------------------

void SpatialAudio::synthesise() {
    Rng rng(12345);
    const size_t xf = size_t(R / 5);
    auto band = [](const std::vector<float>& w, float lo, float hi) {  // crude band-pass: low-pass(hi) - low-pass(lo)
        std::vector<float> a = lowpassed(w, hi), b = lowpassed(w, lo);
        for (size_t i = 0; i < a.size(); ++i) a[i] -= b[i];
        return a;
    };

    {   // River: only low, slow, flowing undertones -- no bubbles, no sparkle. Two broad layers that swell at different slow rates.
        const size_t n = size_t(12 * R);
        std::vector<float> w1 = white(rng, n + xf), w2 = white(rng, n + xf);
        std::vector<float> body = band(w1, 90.0f, 520.0f), upper = band(w2, 300.0f, 900.0f);
        std::vector<float> m1 = lowpassed(white(rng, n + xf), 0.5f), m2 = lowpassed(white(rng, n + xf), 0.35f);
        std::vector<float> x(n + xf);
        for (size_t i = 0; i < x.size(); ++i) {
            const float e1 = 0.8f + 3.0f * m1[i], e2 = 0.8f + 3.0f * m2[i];
            x[i] = body[i] * std::max(0.3f, e1) + 0.55f * upper[i] * std::max(0.2f, e2);
        }
        lowpass(x, 600.0f, 3);   // nothing above the low mids: just flowing undertones
        loops_[kSoundWater] = seamless(x, n);
        normalise_rms(loops_[kSoundWater], 0.10f);
    }
    {   // Leaves: a gentle, dull rustle.
        const size_t n = size_t(10 * R);
        std::vector<float> w = white(rng, n + xf);
        std::vector<float> a = band(w, 250.0f, 1700.0f), slow = lowpassed(white(rng, n + xf), 0.4f);
        std::vector<float> x(n + xf);
        for (size_t i = 0; i < x.size(); ++i) x[i] = a[i] * (0.3f + 5.0f * std::fabs(slow[i]));
        loops_[kSoundLeaves] = seamless(x, n);
        normalise_rms(loops_[kSoundLeaves], 0.08f);
    }
    {   // Birds: a few soft phrases, silence in between.
        const size_t n = size_t(9 * R);
        std::vector<float> x(n + xf, 0.0f);
        for (int phrase = 0; phrase < 3; ++phrase) {
            size_t at = size_t((0.5f + 3.0f * float(phrase) + rng.next() * 1.2f) * R);
            const int chirps = 3 + int(rng.next() * 3);
            const float base = 2200 + 1400 * rng.next();
            for (int c = 0; c < chirps; ++c) {
                const float dur = 0.07f + 0.07f * rng.next();
                const bool up = rng.next() > 0.4f;
                add_chirp(x, at, base * (up ? 0.85f : 1.15f), base * (up ? 1.25f : 0.92f), dur, 0.5f);
                at += size_t((dur + 0.06f + 0.07f * rng.next()) * R);
            }
        }
        lowpass(x, 5200.0f);
        loops_[kSoundBird] = seamless(x, n);
        normalise_rms(loops_[kSoundBird], 0.07f);
    }
    {   // Gem hum: a soft chord with a slow shimmer (integer cycles in 4 s, so it loops seamlessly).
        const size_t n = size_t(4 * R);
        std::vector<float> x(n);
        for (size_t i = 0; i < n; ++i) {
            const float t = float(i) / R;
            const float trem = 0.8f + 0.2f * std::sin(2 * kPi * 2.0f * t);
            x[i] = trem * (std::sin(2 * kPi * 440.0f * t) + 0.5f * std::sin(2 * kPi * 660.0f * t) + 0.25f * std::sin(2 * kPi * 880.0f * t + 1.0f));
        }
        loops_[kSoundHum] = x;
        normalise_rms(loops_[kSoundHum], 0.09f);
    }
    {   // Fire: low rumble and soft crackles.
        const size_t n = size_t(4 * R);
        std::vector<float> w = white(rng, n + xf);
        std::vector<float> x = lowpassed(w, 380.0f);
        for (float& v : x) v *= 1.5f;
        for (int k = 0; k < 70; ++k) {
            const size_t at = size_t(rng.next() * float(n));
            const float amp = 0.2f + 0.5f * rng.next();
            for (size_t i = 0; i < size_t(0.014f * R) && at + i < x.size(); ++i) x[at + i] += amp * rng.noise() * std::exp(-float(i) / (0.004f * R));
        }
        lowpass(x, 3500.0f);
        loops_[kSoundFire] = seamless(x, n);
        normalise_rms(loops_[kSoundFire], 0.16f);
    }
    {   // Ambient air bed: very low, slow, barely there (stereo, decorrelated).
        const size_t n = size_t(14 * R);
        for (int ch = 0; ch < 2; ++ch) {
            std::vector<float> w = white(rng, n + xf);
            std::vector<float> a = lowpassed(w, 170.0f), slow = lowpassed(white(rng, n + xf), 0.12f);
            std::vector<float> x(n + xf);
            for (size_t i = 0; i < x.size(); ++i) x[i] = a[i] * (0.6f + 4.0f * std::fabs(slow[i]));
            (ch == 0 ? wind_l_ : wind_r_) = seamless(x, n);
            normalise_rms(ch == 0 ? wind_l_ : wind_r_, 0.12f);
        }
    }
    loop_gain_[kSoundWater] = 0.30f;   // the nearest dozen river cells add up, so each is quiet
    loop_gain_[kSoundLeaves] = 0.55f;
    loop_gain_[kSoundBird] = 0.8f;
    loop_gain_[kSoundHum] = 0.6f;
    loop_gain_[kSoundFire] = 0.8f;

    // ---- Footsteps: every kind has several different variants (different noise, grain patterns and timing) ----
    auto envelope = [](float t, float attack, float tau) { return (1.0f - std::exp(-t / attack)) * std::exp(-t / tau); };
    for (int v = 0; v < kStepVariants; ++v) {
        const size_t len = size_t(0.34f * R);
        {   // Stone: a soft clap (heel) with a smaller one (toe) and a hint of gravel ground under the foot. No tonal "drum" part.
            std::vector<float> w = white(rng, len), body = lowpassed(white(rng, len), 300.0f);
            std::vector<float> clap = band(w, 800.0f, 3800.0f);
            std::vector<float> x(len, 0.0f);
            const float toe_at = 0.085f + 0.03f * rng.next();
            for (size_t i = 0; i < len; ++i) {
                const float t = float(i) / R;
                x[i] = clap[i] * envelope(t, 0.0015f, 0.011f) * 0.9f + body[i] * envelope(t, 0.004f, 0.025f) * 1.4f;
                if (t > toe_at) x[i] += clap[i] * envelope(t - toe_at, 0.0015f, 0.008f) * 0.35f;
            }
            const int grains = 40 + int(rng.next() * 25);
            for (int g = 0; g < grains; ++g) {
                const float gt = 0.01f + rng.next() * rng.next() * 0.2f;      // denser early, thinning out
                const size_t at = size_t(gt * R), glen = size_t((0.0008f + 0.002f * rng.next()) * R);
                const float amp = (0.05f + 0.22f * rng.next()) * std::exp(-gt / 0.08f);
                for (size_t i = 0; i < glen && at + i < len; ++i) {
                    const float ge = std::sin(kPi * float(i) / float(glen));
                    x[at + i] += amp * ge * rng.noise();
                }
            }
            lowpass(x, 6500.0f);
            steps_[kStepStone][v] = x;
        }
        {   // Grass: a very soft, slow brush -- no thud at all (a thud reads as kicking). Slow onset, long gentle decay, dull.
            std::vector<float> w = white(rng, len);
            std::vector<float> swish = band(w, 110.0f, 700.0f), air = band(white(rng, len), 700.0f, 1500.0f);
            std::vector<float> x(len);
            const float lean = 0.85f + 0.3f * rng.next();
            for (size_t i = 0; i < len; ++i) {
                const float t = float(i) / R;
                x[i] = swish[i] * envelope(t, 0.045f * lean, 0.13f) + air[i] * envelope(t, 0.06f, 0.09f) * 0.12f;
            }
            lowpass(x, 1100.0f);
            steps_[kStepGrass][v] = x;
        }
        {   // Sand: a soft scuff.
            std::vector<float> w = white(rng, len);
            std::vector<float> scuff = band(w, 300.0f, 1900.0f), thud = lowpassed(white(rng, len), 160.0f);
            std::vector<float> x(len);
            for (size_t i = 0; i < len; ++i) {
                const float t = float(i) / R;
                x[i] = scuff[i] * envelope(t, 0.025f, 0.1f) + thud[i] * envelope(t, 0.01f, 0.04f) * 0.7f;
            }
            const int grains = 25 + int(rng.next() * 15);
            for (int g = 0; g < grains; ++g) {
                const float gt = 0.02f + rng.next() * 0.2f;
                const size_t at = size_t(gt * R), glen = size_t(0.0015f * R);
                const float amp = (0.04f + 0.1f * rng.next()) * std::exp(-gt / 0.12f);
                for (size_t i = 0; i < glen && at + i < len; ++i) x[at + i] += amp * std::sin(kPi * float(i) / float(glen)) * rng.noise();
            }
            lowpass(x, 3000.0f);
            steps_[kStepSand][v] = x;
        }
        {   // Wood: a muted knock.
            std::vector<float> w = white(rng, len);
            std::vector<float> knock = band(w, 200.0f, 1500.0f), thump = lowpassed(white(rng, len), 220.0f);
            std::vector<float> x(len, 0.0f);
            const float second = 0.075f + 0.03f * rng.next();
            for (size_t i = 0; i < len; ++i) {
                const float t = float(i) / R;
                x[i] = knock[i] * envelope(t, 0.002f, 0.028f) + thump[i] * envelope(t, 0.004f, 0.03f) * 1.1f;
                if (t > second) x[i] += (knock[i] * 0.6f + thump[i] * 0.5f) * envelope(t - second, 0.002f, 0.02f);
            }
            lowpass(x, 2500.0f);
            steps_[kStepWood][v] = x;
        }
    }
    const float step_peak[kStepKinds] = {0.55f, 0.20f, 0.30f, 0.50f};   // grass and sand are much softer than stone and wood
    for (int k = 0; k < kStepKinds; ++k)
        for (int v = 0; v < kStepVariants; ++v) {
            float peak = 1e-9f;
            for (float x : steps_[k][v]) peak = std::max(peak, std::fabs(x));
            std::vector<float>& buf = steps_[k][v];
            for (float& x : buf) x *= step_peak[k] / peak;
            const size_t fade = size_t(0.06f * R);   // fade the tail to silence so no recording ends with a click
            for (size_t i = 0; i < fade && i < buf.size(); ++i) buf[buf.size() - 1 - i] *= 0.5f - 0.5f * std::cos(kPi * float(i) / float(fade));
        }
}

// ---- device ------------------------------------------------------------------------------------------------

namespace {
// SDL does not stop the callback when a device disappears (another application switched the output, a Bluetooth profile changed,
// the audio server dropped our stream): it keeps calling it and throws the sound away. The only signal is this event.
int SDLCALL on_event(void* user, SDL_Event* e) {
    if (e->type == SDL_AUDIODEVICEREMOVED && !e->adevice.iscapture) {
        auto* removed = static_cast<std::pair<std::atomic<unsigned>*, std::atomic<bool>*>*>(user);
        if (e->adevice.which == removed->first->load()) removed->second->store(true);
    }
    return 0;
}
std::pair<std::atomic<unsigned>*, std::atomic<bool>*> g_watch_data{nullptr, nullptr};
}  // namespace

void SpatialAudio::drop_device() {
    if (!device_) return;
    SDL_DelEventWatch(on_event, &g_watch_data);
    SDL_CloseAudioDevice(device_);
    device_ = 0;
    device_id_ = 0;
}

bool SpatialAudio::open_device() {
    SDL_AudioSpec want{}, have{};
    want.freq = kRate;
    want.format = AUDIO_F32SYS;
    want.channels = 2;
    want.samples = 1024;
    want.userdata = this;
    want.callback = [](void* u, Uint8* stream, int len) {
        auto* self = static_cast<SpatialAudio*>(u);
        self->callbacks_.fetch_add(1, std::memory_order_relaxed);
        self->render(reinterpret_cast<float*>(stream), len / int(sizeof(float) * 2));
    };
    device_ = SDL_OpenAudioDevice(nullptr, 0, &want, &have, 0);
    if (!device_) return false;
    device_id_ = device_;
    removed_ = false;
    g_watch_data = {&device_id_, &removed_};
    SDL_AddEventWatch(on_event, &g_watch_data);
    SDL_PauseAudioDevice(device_, 0);
    seen_callbacks_ = callbacks_.load();
    last_progress_ms_ = SDL_GetTicks();
    return true;
}

bool SpatialAudio::open(bool device) {
    synthesise();
    if (!device) return true;
    device_wanted_ = true;
    if (!open_device()) { std::fprintf(stderr, "Audio: no device (%s); will keep trying.\n", SDL_GetError()); lost_reported_ = true; return false; }
    return true;
}

bool SpatialAudio::service() {
    if (!device_wanted_) return false;
    const uint32_t now = SDL_GetTicks();
    if (device_) {
        const uint64_t c = callbacks_.load(std::memory_order_relaxed);
        const bool disconnected = removed_.exchange(false);
        if (c != seen_callbacks_) { seen_callbacks_ = c; last_progress_ms_ = now; }
        // Two failure modes: SDL reports the device removed (callbacks keep coming, sound is discarded), or the callbacks stop.
        if (!disconnected && now - last_progress_ms_ < 1200) return false;
        std::fprintf(stderr, "Audio: the output %s (another application took it, or the device changed). Reconnecting...\n",
                     disconnected ? "was disconnected" : "stopped");
        drop_device();
        last_attempt_ms_ = 0;
        lost_reported_ = false;
    }
    if (now - last_attempt_ms_ < 2000) return false;       // retry every 2 s until a device is available
    last_attempt_ms_ = now;
    if (open_device()) { std::printf("Audio: output restored.\n"); lost_reported_ = false; return true; }
    if (!lost_reported_) { std::fprintf(stderr, "Audio: no output device available yet (%s); retrying.\n", SDL_GetError()); lost_reported_ = true; }
    return false;
}

void SpatialAudio::close() {
    device_wanted_ = false;
    drop_device();
}

// ---- spatialisation ---------------------------------------------------------------------------------------------

SpatialAudio::Params SpatialAudio::spatialise(const glm::vec3& pos, float base_gain, float occlusion) const {
    const glm::vec3 v = glm::conjugate(listener_q_) * (pos - listener_pos_);  // source in head space: +X right, -Z forward
    const float dist = std::max(0.3f, glm::length(v));
    const float lat = std::max(-1.0f, std::min(1.0f, v.x / dist));              // +1 = fully to the right
    const float front = -v.z / dist;                                            // +1 ahead, -1 behind
    float gain = base_gain / (1.0f + dist / 1.5f);
    gain *= std::max(0.0f, std::min(1.0f, (18.0f - dist) / 4.0f));              // out of range beyond 18 m
    gain *= 1.0f - 0.7f * occlusion;

    const float itd = (0.0875f / 343.0f) * (std::asin(std::fabs(lat)) + std::fabs(lat)) * float(kRate);  // samples, Woodworth model
    const float a = std::fabs(lat);
    const float far_gain = 1.0f - 0.4f * a;
    const float near_fc = 16000.0f, far_fc = 16000.0f * std::pow(3000.0f / 16000.0f, a);
    const float behind = std::max(0.0f, -front);
    const float dull = (1.0f - 0.5f * behind) * (1.0f - 0.85f * occlusion);
    auto alpha = [&](float fc) { return 1.0f - std::exp(-2.0f * kPi * std::max(200.0f, fc * dull) / float(kRate)); };

    Params p;
    const bool right = lat >= 0;
    p.gr = right ? gain : gain * far_gain;
    p.gl = right ? gain * far_gain : gain;
    p.dr = right ? 0.0f : itd;
    p.dl = right ? itd : 0.0f;
    p.ar = alpha(right ? near_fc : far_fc);
    p.al = alpha(right ? far_fc : near_fc);
    return p;
}

SpatialAudio::Voice* SpatialAudio::allocate() {
    for (Voice& v : voices_) if (!v.active) return &v;
    return nullptr;
}

void SpatialAudio::set_listener(const glm::vec3& pos, const glm::quat& q) {
    std::lock_guard<std::mutex> lock(mu_);
    listener_pos_ = pos;
    listener_q_ = q;
}

void SpatialAudio::set_sources(const std::vector<SoundSource>& sources) {
    std::lock_guard<std::mutex> lock(mu_);
    for (Voice& v : voices_) if (v.active && !v.one_shot) v.ending = true;
    for (const SoundSource& s : sources) {
        if (s.kind < 0 || s.kind >= kSoundKinds) continue;
        Voice* v = nullptr;
        for (Voice& c : voices_) if (c.active && !c.one_shot && c.id == s.id) { v = &c; break; }
        if (!v) {
            v = allocate();
            if (!v) continue;
            *v = Voice();
            v->active = true;
            v->id = s.id;
            v->kind = s.kind;
            v->buf = &loops_[s.kind];
            v->pos = size_t((uint32_t(s.id) * 2654435761u) % uint32_t(v->buf->size()));  // decorrelate neighbours
        }
        v->ending = false;
        const Params p = spatialise(s.pos, loop_gain_[s.kind], s.occlusion);
        v->tgl = p.gl; v->tgr = p.gr; v->tdl = p.dl; v->tdr = p.dr; v->tal = p.al; v->tar = p.ar;
        if (v->gl == 0 && v->gr == 0) { v->dl = p.dl; v->dr = p.dr; v->al = p.al; v->ar = p.ar; }  // new voice: no sweep from zero
    }
    for (Voice& v : voices_) if (v.active && v.ending) { v.tgl = v.tgr = 0; }
}

void SpatialAudio::play_step(int kind, const glm::vec3& pos, float gain) {
    if (kind < 0 || kind >= kStepKinds) return;
    std::lock_guard<std::mutex> lock(mu_);
    Voice* v = allocate();
    if (!v) return;
    *v = Voice();
    v->active = true;
    v->one_shot = true;
    v->kind = kind;
    v->buf = &steps_[kind][int(rand01() * float(kStepVariants)) % kStepVariants];
    v->rate = 0.92f + 0.16f * rand01();                    // slightly different pitch every time
    const Params p = spatialise(pos, 1.4f * gain * (0.75f + 0.25f * rand01()), 0.0f);
    v->gl = v->tgl = p.gl; v->gr = v->tgr = p.gr; v->dl = v->tdl = p.dl; v->dr = v->tdr = p.dr; v->al = v->tal = p.al; v->ar = v->tar = p.ar;
}

// ---- mixing --------------------------------------------------------------------------------------------------------

void SpatialAudio::render(float* out, int frames) {
    std::fill(out, out + size_t(frames) * 2, 0.0f);
    std::lock_guard<std::mutex> lock(mu_);
    const float inv = 1.0f / float(std::max(1, frames));

    // Ambient wind bed.
    if (!wind_l_.empty()) {
        wind_gain_ += (0.022f - wind_gain_) * 0.002f;   // slow fade in; the bed is meant to be felt, not heard
        for (int i = 0; i < frames; ++i) {
            out[2 * i] += wind_gain_ * wind_l_[wind_pos_];
            out[2 * i + 1] += wind_gain_ * wind_r_[wind_pos_];
            if (++wind_pos_ >= wind_l_.size()) wind_pos_ = 0;
        }
    }

    for (Voice& v : voices_) {
        if (!v.active) continue;
        const std::vector<float>& buf = *v.buf;
        const float dgl = (v.tgl - v.gl) * inv, dgr = (v.tgr - v.gr) * inv, ddl = (v.tdl - v.dl) * inv, ddr = (v.tdr - v.dr) * inv,
                    dal = (v.tal - v.al) * inv, dar = (v.tar - v.ar) * inv;
        for (int i = 0; i < frames; ++i) {
            float x = 0.0f;
            if (v.one_shot) {
                const size_t i0 = size_t(v.fpos);
                if (i0 + 1 >= buf.size()) { v.active = false; break; }
                const float f = v.fpos - float(i0);
                x = buf[i0] * (1.0f - f) + buf[i0 + 1] * f;
                v.fpos += v.rate;
            } else {
                x = buf[v.pos];
                if (++v.pos >= buf.size()) v.pos = 0;
            }
            v.ring[v.wr & 255u] = x;
            auto tap = [&](float delay) {  // fractional delay by linear interpolation
                const unsigned d0 = unsigned(delay);
                const float f = delay - float(d0);
                return v.ring[(v.wr - d0) & 255u] * (1.0f - f) + v.ring[(v.wr - d0 - 1) & 255u] * f;
            };
            const float xl = tap(v.dl), xr = tap(v.dr);
            ++v.wr;
            v.lpl += v.al * (xl - v.lpl);
            v.lpr += v.ar * (xr - v.lpr);
            out[2 * i] += v.gl * v.lpl;
            out[2 * i + 1] += v.gr * v.lpr;
            v.gl += dgl; v.gr += dgr; v.dl += ddl; v.dr += ddr; v.al += dal; v.ar += dar;
        }
        v.gl = v.tgl; v.gr = v.tgr; v.dl = v.tdl; v.dr = v.tdr; v.al = v.tal; v.ar = v.tar;
        if (v.active && v.ending && v.gl < 1e-4f && v.gr < 1e-4f) v.active = false;
    }

    const float master = muted_ ? 0.0f : volume_;
    for (int i = 0; i < frames * 2; ++i) out[i] = std::tanh(out[i] * master);  // soft limiter
}
