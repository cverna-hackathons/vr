#include "ptc_pose.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <glm/gtc/matrix_transform.hpp>

namespace {
glm::dmat3 rodrigues(const glm::dvec3& w) {
    const double a = glm::length(w);
    if (a < 1e-12) return glm::dmat3(1.0);
    const glm::dvec3 k = w / a;
    const glm::dmat3 K(0, k.z, -k.y, -k.z, 0, k.x, k.y, -k.x, 0);  // column-major cross-product matrix
    return glm::dmat3(1.0) + std::sin(a) * K + (1 - std::cos(a)) * (K * K);
}
}  // namespace

glm::dvec2 PtcPoseSolver::undistort(double u, double v) const {
    const double xd = (u - k_.cx) / k_.fx, yd = (v - k_.cy) / k_.fy;
    double x = xd, y = yd;
    for (int i = 0; i < 12; ++i) {  // fixed-point inversion of the OpenCV distortion model
        const double r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
        const double radial = 1 + k_.k1 * r2 + k_.k2 * r4 + k_.k3 * r6;
        const double dx = 2 * k_.p1 * x * y + k_.p2 * (r2 + 2 * x * x);
        const double dy = k_.p1 * (r2 + 2 * y * y) + 2 * k_.p2 * x * y;
        x = (xd - dx) / radial;
        y = (yd - dy) / radial;
    }
    return {x, y};
}

glm::dvec2 PtcPoseSolver::project_px(const glm::dvec3& p) const {
    const double x = p.x / p.z, y = p.y / p.z;
    const double r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
    const double radial = 1 + k_.k1 * r2 + k_.k2 * r4 + k_.k3 * r6;
    const double xd = x * radial + 2 * k_.p1 * x * y + k_.p2 * (r2 + 2 * x * x);
    const double yd = y * radial + k_.p1 * (r2 + 2 * y * y) + 2 * k_.p2 * x * y;
    return {k_.fx * xd + k_.cx, k_.fy * yd + k_.cy};
}

// Levenberg-Marquardt on normalised-image reprojection error, 6 parameters (rotation increment, translation).
double PtcPoseSolver::refine(const std::vector<Obs>& obs, glm::dmat3& R, glm::dvec3& t) const {
    const int n = int(obs.size());
    auto residuals = [&](const glm::dmat3& Rm, const glm::dvec3& tv, std::vector<double>& r) {
        r.resize(2 * n);
        double sse = 0;
        for (int i = 0; i < n; ++i) {
            const glm::dvec3 p = Rm * obs[i].model + tv;
            if (p.z < 0.05) { r[2 * i] = r[2 * i + 1] = 1.0; sse += 2.0; continue; }  // behind the camera: big penalty
            r[2 * i] = p.x / p.z - obs[i].norm.x;
            r[2 * i + 1] = p.y / p.z - obs[i].norm.y;
            sse += r[2 * i] * r[2 * i] + r[2 * i + 1] * r[2 * i + 1];
        }
        return sse;
    };
    std::vector<double> r, r2;
    double sse = residuals(R, t, r);
    double lambda = 1e-3;
    for (int iter = 0; iter < 40; ++iter) {
        // Numerical Jacobian (2n x 6)
        std::vector<double> J(size_t(2) * n * 6);
        const double eps = 1e-6;
        for (int c = 0; c < 6; ++c) {
            glm::dmat3 Rp = R; glm::dvec3 tp = t;
            if (c < 3) { glm::dvec3 w(0); w[c] = eps; Rp = rodrigues(w) * R; } else tp[c - 3] += eps;
            residuals(Rp, tp, r2);
            for (int i = 0; i < 2 * n; ++i) J[size_t(i) * 6 + c] = (r2[i] - r[i]) / eps;
        }
        double A[6][6] = {}, g[6] = {};
        for (int i = 0; i < 2 * n; ++i)
            for (int a = 0; a < 6; ++a) {
                g[a] += J[size_t(i) * 6 + a] * r[i];
                for (int b = 0; b < 6; ++b) A[a][b] += J[size_t(i) * 6 + a] * J[size_t(i) * 6 + b];
            }
        bool improved = false;
        for (int attempt = 0; attempt < 8 && !improved; ++attempt) {
            double M[6][7];
            for (int a = 0; a < 6; ++a) {
                for (int b = 0; b < 6; ++b) M[a][b] = A[a][b] + (a == b ? lambda * (A[a][a] + 1e-9) : 0);
                M[a][6] = -g[a];
            }
            // Gaussian elimination with partial pivoting
            bool ok = true;
            for (int c = 0; c < 6 && ok; ++c) {
                int piv = c;
                for (int rr = c + 1; rr < 6; ++rr) if (std::fabs(M[rr][c]) > std::fabs(M[piv][c])) piv = rr;
                if (std::fabs(M[piv][c]) < 1e-18) { ok = false; break; }
                for (int k = 0; k < 7; ++k) std::swap(M[c][k], M[piv][k]);
                for (int rr = c + 1; rr < 6; ++rr) {
                    const double f = M[rr][c] / M[c][c];
                    for (int k = c; k < 7; ++k) M[rr][k] -= f * M[c][k];
                }
            }
            if (!ok) { lambda *= 10; continue; }
            double dx[6];
            for (int a = 5; a >= 0; --a) {
                double s = M[a][6];
                for (int b = a + 1; b < 6; ++b) s -= M[a][b] * dx[b];
                dx[a] = s / M[a][a];
            }
            const glm::dmat3 Rn = rodrigues(glm::dvec3(dx[0], dx[1], dx[2])) * R;
            const glm::dvec3 tn = t + glm::dvec3(dx[3], dx[4], dx[5]);
            std::vector<double> rn;
            const double sn = residuals(Rn, tn, rn);
            if (sn < sse) {
                const double step = std::fabs(sse - sn);
                R = Rn; t = tn; r = rn; sse = sn; lambda = std::max(lambda * 0.3, 1e-9); improved = true;
                if (step < 1e-14) return std::sqrt(sse / n);
            } else lambda *= 10;
        }
        if (!improved) break;
    }
    return std::sqrt(sse / n);
}

PtcPose PtcPoseSolver::solve(const std::vector<Blob>& blobs, const PtcPose& prev) const {
    PtcPose out;
    std::vector<Obs> obs;
    for (const Blob& b : blobs) {
        if (b.led < 0 || b.led >= int(model_.leds.size())) continue;
        Obs o;
        o.model = glm::dvec3(model_.leds[b.led].pos);
        o.norm = undistort(b.x, b.y);
        if (mirror_) o.norm.x = -o.norm.x;   // the camera picture is mirrored left-right (see set_mirror_image)
        o.px = {b.x, b.y};
        o.led = b.led;
        obs.push_back(o);
    }
    if (obs.size() < 5) return out;

    glm::dmat3 R(1.0); glm::dvec3 t(0);
    double best = 1e9;
    auto try_start = [&](glm::dmat3 R0, glm::dvec3 t0) {
        double rms = refine(obs, R0, t0);
        // Plausibility: in front of the camera, and the LEDs we see should mostly face the camera.
        int facing = 0;
        for (const Obs& o : obs) {
            const glm::dvec3 n = R0 * glm::dvec3(model_.leds[o.led].normal);
            const glm::dvec3 p = R0 * o.model + t0;
            if (glm::dot(n, -glm::normalize(p)) > -0.2) ++facing;
        }
        if (t0.z > 0.2 && t0.z < 5.0 && facing >= int(0.8 * obs.size()) && rms < best) { best = rms; R = R0; t = t0; }
    };
    if (prev.valid) {
        try_start(prev.R, prev.t);
    }
    if (best > 0.003) {  // no good tracking fit (or no previous pose): global search
        // Start from the 24 axis-aligned orientations; if none of them leads to a good fit, also from each of them tilted by +-30 degrees
        // about every axis, so that any orientation is within ~35 degrees of some start (the least-squares fit converges from there).
        // The search stops as soon as a start gives a clearly good fit.
        static const int perm[6][3] = {{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
        std::vector<glm::dmat3> bases;
        for (const auto& p : perm)
            for (int sgn = 0; sgn < 8; ++sgn) {
                glm::dmat3 M(0.0);
                for (int c = 0; c < 3; ++c) M[c][p[c]] = (sgn >> c & 1) ? -1.0 : 1.0;
                if (glm::determinant(M) > 0) bases.push_back(M);
            }
        const double good = 0.002;   // normalised image units (~1.4 px)
        for (int round = 0; round < 2 && best > good; ++round) {
            for (const glm::dmat3& M : bases) {
                for (int variant = 0; variant < (round == 0 ? 1 : 6) && best > good; ++variant) {
                    glm::dmat3 R0 = M;
                    if (round == 1) {
                        glm::dvec3 axis(0.0);
                        axis[variant / 2] = 1.0;
                        R0 = rodrigues(axis * ((variant % 2) ? -0.52 : 0.52)) * M;
                    }
                    for (double z : {0.6, 1.2}) try_start(R0, glm::dvec3(0, 0, z));
                }
                if (best <= good) break;
            }
        }
    }
    if (best > 1e8) return out;

    // Drop gross outliers (wrong IDs) and refit.
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<Obs> keep;
        for (const Obs& o : obs) {
            const glm::dvec3 p = R * o.model + t;
            if (p.z <= 0.05) continue;
            const glm::dvec2 px = project_px(mirror_ ? glm::dvec3(-p.x, p.y, p.z) : p);
            if (glm::length(px - o.px) < 4.0) keep.push_back(o);
        }
        if (keep.size() < 5) return out;
        if (keep.size() == obs.size()) break;
        obs = keep;
        best = refine(obs, R, t);
    }
    double sse = 0;
    for (const Obs& o : obs) {
        const glm::dvec3 pc = R * o.model + t;
        const glm::dvec2 d = project_px(mirror_ ? glm::dvec3(-pc.x, pc.y, pc.z) : pc) - o.px;
        sse += glm::dot(d, d);
    }
    out.R = R; out.t = t; out.rms_px = std::sqrt(sse / obs.size()); out.inliers = int(obs.size()); out.valid = out.rms_px < 3.0;
    return out;
}
