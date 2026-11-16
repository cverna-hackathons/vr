// Renders the explore sounds to WAV files (stereo, 48 kHz, 16 bit) so they can be auditioned with any player, no headset needed.
//   sound_demo OUTPUT_DIR
// Files: steps_stone/grass/sand/wood.wav (eight steps each), river.wav, forest.wav (leaves and birds), gems.wav, ambient.wav.
#include "spatial_audio.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

static void write_wav(const std::string& path, const std::vector<float>& lr) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) { std::perror(path.c_str()); return; }
    const uint32_t n = uint32_t(lr.size()) * 2, rate = SpatialAudio::kRate;
    auto u32 = [&](uint32_t v) { std::fwrite(&v, 4, 1, f); };
    auto u16 = [&](uint16_t v) { std::fwrite(&v, 2, 1, f); };
    std::fwrite("RIFF", 1, 4, f); u32(36 + n); std::fwrite("WAVEfmt ", 1, 8, f); u32(16); u16(1); u16(2); u32(rate); u32(rate * 4); u16(4); u16(16);
    std::fwrite("data", 1, 4, f); u32(n);
    for (float x : lr) { const int16_t s = int16_t(std::max(-1.0f, std::min(1.0f, x)) * 32767.0f); std::fwrite(&s, 2, 1, f); }
    std::fclose(f);
}

static std::vector<float> render(SpatialAudio& a, float seconds, const std::vector<std::pair<float, std::pair<int, glm::vec3>>>& steps) {
    const int frames = int(seconds * SpatialAudio::kRate);
    std::vector<float> out(size_t(frames) * 2), blk(1024 * 2);
    size_t next = 0;
    std::vector<std::pair<float, std::pair<int, glm::vec3>>> ev = steps;
    int done = 0;
    while (done < frames) {
        while (next < ev.size() && ev[next].first * SpatialAudio::kRate <= done) { a.play_step(ev[next].second.first, ev[next].second.second); ++next; }
        a.render(blk.data(), 1024);
        for (int i = 0; i < 1024 * 2 && done * 2 + i < int(out.size()); ++i) out[size_t(done) * 2 + size_t(i)] = blk[size_t(i)];
        done += 1024;
    }
    return out;
}

static void report(const char* name, const std::vector<float>& x) {
    double e = 0; float peak = 0;
    for (float v : x) { e += double(v) * v; peak = std::max(peak, std::fabs(v)); }
    const double rms = std::sqrt(e / double(x.size()));
    std::printf("  %-18s rms %6.1f dBFS   peak %6.1f dBFS\n", name, 20 * std::log10(std::max(rms, 1e-9)), 20 * std::log10(std::max(double(peak), 1e-9)));
}

int main(int argc, char** argv) {
    const std::string dir = argc > 1 ? argv[1] : ".";
    SpatialAudio a;
    a.open(false);
    a.set_volume(0.8f);
    a.set_listener(glm::vec3(0.0f), glm::quat(1, 0, 0, 0));
    const char* names[kStepKinds] = {"stone", "grass", "sand", "wood"};
    std::printf("Footsteps (eight steps, 0.55 s apart, alternating feet):\n");
    for (int k = 0; k < kStepKinds; ++k) {
        std::vector<std::pair<float, std::pair<int, glm::vec3>>> ev;
        for (int i = 0; i < 8; ++i) ev.push_back({0.2f + 0.55f * float(i) * (0.92f + 0.16f * float((i * 37) % 7) / 7.0f), {k, glm::vec3(i % 2 ? 0.12f : -0.12f, -1.7f, 0.0f)}});
        const auto out = render(a, 5.2f, ev);
        write_wav(dir + "/steps_" + names[k] + ".wav", out);
        report((std::string("steps_") + names[k]).c_str(), out);
    }
    std::printf("Ambience:\n");
    {
        std::vector<SoundSource> src;   // a river to the right: six nearby cells, like the nearest ones in a real map
        for (int i = 0; i < 6; ++i) src.push_back({i, kSoundWater, glm::vec3(2.5f + 0.4f * float(i), -0.1f, -1.0f - 0.5f * float(i)), 0.0f});
        a.set_sources(src);
        auto out = render(a, 10.0f, {});
        write_wav(dir + "/river.wav", out); report("river.wav", out);
        src.clear();
        for (int i = 0; i < 6; ++i) src.push_back({100 + i, i % 3 == 0 ? kSoundBird : kSoundLeaves, glm::vec3(-3.0f + 2.0f * float(i), 2.2f, -4.0f - float(i)), 0.0f});
        a.set_sources(src);
        out = render(a, 10.0f, {});
        write_wav(dir + "/forest.wav", out); report("forest.wav", out);
        a.set_sources({{200, kSoundHum, glm::vec3(1.5f, 0.0f, -2.0f), 0.0f}});
        out = render(a, 6.0f, {});
        write_wav(dir + "/gems.wav", out); report("gems.wav", out);
        a.set_sources({});
        out = render(a, 3.0f, {});                                  // let the sources fade out first
        out = render(a, 10.0f, {});
        write_wav(dir + "/ambient.wav", out); report("ambient.wav (air bed)", out);
    }
    return 0;
}
