// Headset pose from identified LED dots: undistort the dots with the camera's lens calibration, then solve
// the perspective-n-point problem (Levenberg-Marquardt on reprojection error).
#pragma once
#include <vector>
#include <glm/glm.hpp>

#include "ptc_camera.hpp"
#include "ptc_tracker.hpp"

struct PtcPose {
    glm::dmat3 R{1.0};      // headset -> camera rotation
    glm::dvec3 t{0.0};      // headset origin in camera coordinates (metres); camera: +X right, +Y down, +Z forward
    double rms_px = 0;      // RMS reprojection error of the inlier LEDs
    int inliers = 0;        // LEDs used
    bool valid = false;
};

class PtcPoseSolver {
public:
    PtcPoseSolver(const LedModel& model, const PtcIntrinsics& k) : model_(model), k_(k) {}
    // Solve from the identified blobs. If `prev` is valid it is used as the starting point (fast tracking);
    // otherwise a global search over starting orientations is done (acquisition).
    PtcPose solve(const std::vector<Blob>& blobs, const PtcPose& prev) const;

    // The PTC delivers a left-right MIRRORED picture (relative to the LED model). The observations are therefore mirrored back before solving,
    // so the fit is a proper rigid pose. (Without it, only a flipped pose "fits" a flat pattern such as the front plate seen head-on, with the LED
    // normals pointing away from the camera -- no valid pose, or a mirror-image one.) Turn off only to reproduce the old behaviour.
    void set_mirror_image(bool on) { mirror_ = on; }
    bool mirror_image() const { return mirror_; }

    glm::dvec2 undistort(double u, double v) const;                 // pixel -> normalised camera coordinates
    glm::dvec2 project_px(const glm::dvec3& cam_point) const;       // camera point -> pixel (with distortion)

private:
    struct Obs { glm::dvec3 model; glm::dvec2 norm; glm::dvec2 px; int led; };
    double refine(const std::vector<Obs>& obs, glm::dmat3& R, glm::dvec3& t) const;  // returns rms (normalised)
    const LedModel& model_;
    PtcIntrinsics k_;
    bool mirror_ = true;
};
