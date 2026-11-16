// Decides, from a deliberate back-and-forth lean, whether a tracked position axis points the same way as the real motion.
//
// The headset's accelerometer is the ground truth. For a back-and-forth motion x(t) = A sin(wt) the acceleration is -w^2 x(t),
// so a position signal with the correct sign is NEGATIVELY correlated with the acceleration, and a reversed one POSITIVELY.
// The camera's position is delayed by its latency, so the best lag (0..250 ms) is searched. Both signals are band-passed first
// (0.3 - 3 Hz), which removes offsets, drift and noise.
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

struct LeanAxisResult {
    float sign = 0;         // +1: the tracked axis already matches reality, -1: it is reversed, 0: undecided
    float correlation = 0;  // at the best lag (negative = correct sign, as explained above)
    float amplitude_m = 0;  // typical lean amplitude (half the 5-95 percentile range), metres
    float lag_ms = 0;
    bool trusted = false;   // enough motion and a clear correlation
};

namespace lean_detail {
// Resample (t, v) to a uniform rate by linear interpolation.
inline std::vector<float> resample(const std::vector<double>& t, const std::vector<float>& v, double rate, double t0, int n) {
    std::vector<float> out(static_cast<size_t>(n), 0.0f);
    size_t j = 0;
    for (int i = 0; i < n; ++i) {
        const double ti = t0 + i / rate;
        while (j + 1 < t.size() && t[j + 1] < ti) ++j;
        if (j + 1 >= t.size()) { out[static_cast<size_t>(i)] = v.back(); continue; }
        const double w = std::min(1.0, std::max(0.0, (ti - t[j]) / std::max(1e-9, t[j + 1] - t[j])));
        out[static_cast<size_t>(i)] = float(v[j] * (1.0 - w) + v[j + 1] * w);
    }
    return out;
}
inline std::vector<float> moving_average(const std::vector<float>& x, int w) {   // centred, edge-clamped
    std::vector<float> o(x.size());
    std::vector<double> c(x.size() + 1, 0.0);
    for (size_t i = 0; i < x.size(); ++i) c[i + 1] = c[i] + x[i];
    for (size_t i = 0; i < x.size(); ++i) {
        const long a = std::max(0L, long(i) - w / 2), b = std::min(long(x.size()), long(i) + w / 2 + 1);
        o[i] = float((c[size_t(b)] - c[size_t(a)]) / double(b - a));
    }
    return o;
}
inline std::vector<float> bandpass(const std::vector<float>& x, double rate) {
    const std::vector<float> slow = moving_average(x, int(rate * 2.0));   // removes everything below ~0.3 Hz
    std::vector<float> d(x.size());
    for (size_t i = 0; i < x.size(); ++i) d[i] = x[i] - slow[i];
    return moving_average(d, std::max(1, int(rate * 0.12)));              // removes noise above ~3 Hz
}
}  // namespace lean_detail

// t: sample times (s); position: tracked position along one axis (m); accel: real acceleration along the same axis (m/s^2).
inline LeanAxisResult analyse_lean_axis(const std::vector<double>& t, const std::vector<float>& position, const std::vector<float>& accel) {
    LeanAxisResult r;
    if (t.size() < 60 || t.back() - t.front() < 3.0) return r;
    const double rate = 100.0;
    const int n = int((t.back() - t.front()) * rate);
    const std::vector<float> p = lean_detail::bandpass(lean_detail::resample(t, position, rate, t.front(), n), rate);
    const std::vector<float> a = lean_detail::bandpass(lean_detail::resample(t, accel, rate, t.front(), n), rate);

    std::vector<float> sorted = p;
    std::sort(sorted.begin(), sorted.end());
    r.amplitude_m = 0.5f * (sorted[size_t(n * 0.95)] - sorted[size_t(n * 0.05)]);

    const int margin = int(rate * 0.5);   // ignore filter edges
    double best = 0;
    for (int lag = 0; lag <= int(0.25 * rate); ++lag) {
        double sab = 0, saa = 0, sbb = 0;
        for (int i = margin + lag; i < n - margin; ++i) {
            const double x = p[size_t(i)], y = a[size_t(i - lag)];   // the camera lags: compare p(t) with a(t - lag)
            sab += x * y; saa += x * x; sbb += y * y;
        }
        const double c = sab / std::sqrt(saa * sbb + 1e-18);
        if (std::fabs(c) > std::fabs(best)) { best = c; r.lag_ms = float(lag * 1000.0 / rate); }
    }
    r.correlation = float(best);
    r.sign = best < 0 ? 1.0f : -1.0f;
    r.trusted = r.amplitude_m > 0.04f && std::fabs(best) > 0.35;
    return r;
}
