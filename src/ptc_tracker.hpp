// Finds the DK2's IR LED dots in PTC frames, follows them from frame to frame, and identifies each by its
// blink pattern (10 frames, bright/dim, any starting phase) against the layout read from the headset.
#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "ptc_camera.hpp"

struct LedModel {
    struct Led { glm::vec3 pos{0}, normal{0}; uint16_t pattern = 0; };  // pattern bit k = bright in phase k
    std::vector<Led> leds;
    bool load(const std::string& path);  // file written by tools/ledlayout
};

struct Blob {
    float x = 0, y = 0;     // intensity-weighted centroid, pixels
    float sum = 0;          // summed intensity above the detection threshold
    int area = 0;
    int track = -1;         // persistent track id
    int led = -1;           // identified LED index, or -1
};

class PtcBlobTracker {
public:
    explicit PtcBlobTracker(const LedModel& model) : model_(model) {}
    // Process one frame; returns blobs found (with led ids where known).
    const std::vector<Blob>& process(const PtcFrame& f);
    int frames_processed() const { return frames_; }

private:
    struct Track {
        int id; float x, y, vx = 0, vy = 0;
        std::vector<float> hist;  // brightness (sum) per frame, newest last; cleared on frame gaps
        int led = -1;
        int missed = 0;
    };
    int identify(const std::vector<float>& hist) const;

    const LedModel& model_;
    std::vector<Track> tracks_;
    std::vector<Blob> blobs_;
    std::vector<uint8_t> visited_;
    uint32_t last_seq_ = 0;
    bool have_seq_ = false;
    int next_track_ = 0, frames_ = 0;
};
