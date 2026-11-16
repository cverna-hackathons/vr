#include "ptc_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

namespace {
constexpr int kThreshold = 40;          // pixel value above which a pixel belongs to a dot
constexpr int kMinArea = 12, kMaxArea = 900;
constexpr float kMaxMove = 12.0f;       // max pixels a dot may move between frames
constexpr int kHistory = 10;            // frames of brightness needed to read a pattern
constexpr int kMaxMissed = 2;
constexpr float kMinContrast = 1.15f;   // max/min brightness ratio needed to call a bit pattern
}  // namespace

bool LedModel::load(const std::string& path) {
    std::ifstream in(path);
    if (!in) return false;
    leds.clear();
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        size_t idx; Led L; std::string bits;
        if (!(ss >> idx >> L.pos.x >> L.pos.y >> L.pos.z >> L.normal.x >> L.normal.y >> L.normal.z >> bits) || bits.size() != 10) continue;
        for (int k = 0; k < 10; ++k) if (bits[k] == '1') L.pattern |= 1u << k;
        if (idx >= leds.size()) leds.resize(idx + 1);
        leds[idx] = L;
    }
    return !leds.empty();
}

// Rotation-invariant match: observed 10-bit sequence (oldest = bit 0) against each LED's pattern.
int PtcBlobTracker::identify(const std::vector<float>& hist) const {
    if (int(hist.size()) < kHistory) return -1;
    const float* h = hist.data() + hist.size() - kHistory;
    float lo = h[0], hi = h[0];
    for (int i = 1; i < kHistory; ++i) { lo = std::min(lo, h[i]); hi = std::max(hi, h[i]); }
    if (lo <= 0 || hi / lo < kMinContrast) return -1;
    const float mid = 0.5f * (lo + hi);
    unsigned obs = 0;
    for (int i = 0; i < kHistory; ++i) if (h[i] > mid) obs |= 1u << i;

    int best = -1, best_d = 99, second_d = 99;
    for (size_t l = 0; l < model_.leds.size(); ++l) {
        const unsigned p = model_.leds[l].pattern;
        int d = 99;
        for (int r = 0; r < 10; ++r) {
            const unsigned rot = ((p >> r) | (p << (10 - r))) & 0x3ff;
            d = std::min(d, __builtin_popcount(rot ^ obs));
        }
        if (d < best_d) { second_d = best_d; best_d = d; best = int(l); }
        else if (d < second_d) second_d = d;
    }
    // Accept an exact match, or a single-bit error that is clearly better than any other candidate.
    if (best_d == 0 && second_d >= 1) return best;
    if (best_d == 1 && second_d >= 3) return best;
    return -1;
}

const std::vector<Blob>& PtcBlobTracker::process(const PtcFrame& f) {
    ++frames_;
    const bool gap = have_seq_ && f.sequence != last_seq_ + 1;
    last_seq_ = f.sequence;
    have_seq_ = true;
    if (gap) for (Track& t : tracks_) t.hist.clear();  // dropped frame: the phase sequence is broken

    // --- 1. Detect dots: connected components of pixels > threshold (8-connected). ---
    blobs_.clear();
    const int W = f.width, H = f.height;
    visited_.assign(size_t(W) * H, 0);
    std::vector<int> stack;
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const size_t i = size_t(y) * W + x;
            if (f.pix[i] <= kThreshold || visited_[i]) continue;
            double sx = 0, sy = 0, sw = 0;
            int area = 0, minx = x, maxx = x, miny = y, maxy = y;
            stack.assign(1, int(i));
            visited_[i] = 1;
            while (!stack.empty()) {
                const int c = stack.back(); stack.pop_back();
                const int cx = c % W, cy = c / W;
                const double v = f.pix[c];
                sx += cx * v; sy += cy * v; sw += v; ++area;
                minx = std::min(minx, cx); maxx = std::max(maxx, cx);
                miny = std::min(miny, cy); maxy = std::max(maxy, cy);
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int nx = cx + dx, ny = cy + dy;
                        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                        const size_t n = size_t(ny) * W + nx;
                        if (f.pix[n] > kThreshold && !visited_[n]) { visited_[n] = 1; stack.push_back(int(n)); }
                    }
            }
            const int bw = maxx - minx + 1, bh = maxy - miny + 1;
            if (area < kMinArea || area > kMaxArea) continue;
            if (2 * bw <= bh || bw >= 2 * bh) continue;  // reject streaks (not round)
            Blob b;
            b.x = float(sx / sw); b.y = float(sy / sw); b.sum = float(sw); b.area = area;
            blobs_.push_back(b);
        }
    }

    // --- 2. Associate with existing tracks (nearest within kMaxMove of the predicted position). ---
    std::vector<int> taken(blobs_.size(), 0);
    for (Track& t : tracks_) {
        const float px = t.x + t.vx, py = t.y + t.vy;
        int best = -1; float bd = kMaxMove;
        for (size_t i = 0; i < blobs_.size(); ++i) {
            if (taken[i]) continue;
            const float d = std::hypot(blobs_[i].x - px, blobs_[i].y - py);
            if (d < bd) { bd = d; best = int(i); }
        }
        if (best < 0) { ++t.missed; continue; }
        taken[best] = 1;
        Blob& b = blobs_[best];
        t.vx = b.x - t.x; t.vy = b.y - t.y;
        t.x = b.x; t.y = b.y; t.missed = 0;
        t.hist.push_back(b.sum);
        if (t.hist.size() > 24) t.hist.erase(t.hist.begin(), t.hist.begin() + (t.hist.size() - 24));
        b.track = t.id;
    }
    // A track that disappears for a frame loses phase continuity.
    for (Track& t : tracks_) if (t.missed > 0) t.hist.clear();
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [](const Track& t) { return t.missed > kMaxMissed; }), tracks_.end());
    for (size_t i = 0; i < blobs_.size(); ++i) {
        if (taken[i]) continue;
        Track t;
        t.id = next_track_++; t.x = blobs_[i].x; t.y = blobs_[i].y;
        t.hist.push_back(blobs_[i].sum);
        blobs_[i].track = t.id;
        tracks_.push_back(std::move(t));
    }

    // --- 3. Identify tracks whose brightness history is long enough. ---
    for (Track& t : tracks_) {
        if (t.led < 0 || t.hist.size() >= size_t(kHistory)) {
            const int id = identify(t.hist);
            if (id >= 0) t.led = id;
            else if (t.hist.size() >= size_t(kHistory)) t.led = -1;  // re-check every frame; drop stale IDs
        }
    }
    for (Blob& b : blobs_)
        for (const Track& t : tracks_) if (t.id == b.track) { b.led = t.led; break; }
    return blobs_;
}
