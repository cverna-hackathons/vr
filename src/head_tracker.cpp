#include "head_tracker.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sys/stat.h>

namespace {
const glm::vec3 kUp(0, 1, 0);

// Heading of a headset->world rotation: angle (radians) of the forward vector (-Z) about +Y; positive = turned left.
float yaw_of(const glm::quat& q) {
    const glm::vec3 f = q * glm::vec3(0, 0, -1);
    return std::atan2(-f.x, -f.z);
}

float wrap_pi(float a) {
    while (a > glm::pi<float>()) a -= 2 * glm::pi<float>();
    while (a < -glm::pi<float>()) a += 2 * glm::pi<float>();
    return a;
}

// Camera frame (+X right in image, +Y down, +Z away from the camera) -> viewer-style frame
// (+X right, +Y up, +Z toward the camera), a 180 degree turn about Z (keeps handedness).
constexpr int kMinLeds = 8;   // fewest LEDs for a camera fix to be trusted

// The PnP solution is expressed in a frame that is mirrored relative to the headset's IMU frame: the camera image
// is mirrored in X, and the LED layout is mirrored in Y (the two mirrors cancel in the fit, which is why the fit is
// good, but they reverse the sense of rotations about one axis). Measured by comparing IMU and camera orientation
// over varied head rotations: with this correction the camera->IMU-world alignment stays constant (2 deg mean).
// Corrected pose: R = Mcam * R_pnp * Mled,  t = Mcam * t_pnp.
// Old workaround, only used with DK2_LEGACY_MIRROR=1: the solver used to return mirror-flipped poses (the camera picture is mirrored and was
// not corrected before solving), and these constants patched the result. With the picture mirrored back first (PtcPoseSolver) the poses are
// proper rigid poses, and both are the identity.
const glm::dmat3 kLegacyMirrorCam(glm::dvec3(-1, 0, 0), glm::dvec3(0, 1, 0), glm::dvec3(0, 0, 1));
const glm::dmat3 kLegacyMirrorLed(glm::dvec3(1, 0, 0), glm::dvec3(0, -1, 0), glm::dvec3(0, 0, 1));

const glm::dmat3 kCamToView(glm::dvec3(-1, 0, 0), glm::dvec3(0, -1, 0), glm::dvec3(0, 0, 1));
}  // namespace

namespace {
std::string signs_path(bool create_dir) {
    const char* xdg = std::getenv("XDG_CONFIG_HOME");
    const char* home = std::getenv("HOME");
    std::string base = xdg && *xdg ? std::string(xdg) : (home ? std::string(home) + "/.config" : std::string("."));
    const std::string dir = base + "/dk2vr";
    if (create_dir) { mkdir(base.c_str(), 0755); mkdir(dir.c_str(), 0755); }
    return dir + "/position_signs";
}
}  // namespace

void HeadTracker::load_position_signs() {
    std::ifstream f(signs_path(false));
    float x, y, z;
    if (f >> x >> y >> z) {
        pos_sign_ = glm::vec3(x < 0 ? -1.0f : 1.0f, y < 0 ? -1.0f : 1.0f, z < 0 ? -1.0f : 1.0f);
        std::printf("HeadTracker: position signs X %+.0f  Y %+.0f  Z %+.0f (from %s)\n", pos_sign_.x, pos_sign_.y, pos_sign_.z, signs_path(false).c_str());
    }
}

void HeadTracker::save_position_signs() const {
    std::ofstream f(signs_path(true));
    if (f) f << pos_sign_.x << " " << pos_sign_.y << " " << pos_sign_.z << "\n";
    else std::fprintf(stderr, "HeadTracker: could not save the position signs to %s\n", signs_path(false).c_str());
}

void HeadTracker::set_position_signs(const glm::vec3& s) {
    pos_sign_ = glm::vec3(s.x < 0 ? -1.0f : 1.0f, s.y < 0 ? -1.0f : 1.0f, s.z < 0 ? -1.0f : 1.0f);
    save_position_signs();
}

void HeadTracker::flip_position_axis(int axis) {
    if (axis < 0 || axis > 2) return;
    pos_sign_[axis] = -pos_sign_[axis];
    save_position_signs();
    std::printf("Position sign: X %+.0f  Y %+.0f  Z %+.0f  (saved for all programs)\n", pos_sign_.x, pos_sign_.y, pos_sign_.z);
}

bool HeadTracker::take_user_accel(glm::vec3& out) {
    glm::vec3 a;
    if (!take_world_accel(a)) return false;
    out = q_ref_ * a;
    return true;
}

bool HeadTracker::start(const std::string& led_file, bool use_camera) {
    legacy_mirror_ = std::getenv("DK2_LEGACY_MIRROR") != nullptr;
    // Default position signs: with proper rigid poses the tracked position already matches reality (+1); the old flipped poses needed -1.
    pos_sign_ = legacy_mirror_ ? glm::vec3(-1.0f, -1.0f, -1.0f) : glm::vec3(1.0f, 1.0f, 1.0f);
    load_position_signs();   // saved/calibrated signs override the defaults above
    mirror_cam_ = kLegacyMirrorCam;   // measured with lean_calibrate: camera-side X mirror + model-side Y mirror keeps camera and IMU rotations consistent (1.6 vs 12 deg)
    mirror_led_ = kLegacyMirrorLed;
    if (legacy_mirror_) std::printf("HeadTracker: DK2_LEGACY_MIRROR set, using the old pose handling.\n");
    if (!hid_.open()) return false;
    imu_.reserve(64);
    if (use_camera) {
        if (!model_.load(led_file)) {
            std::fprintf(stderr, "HeadTracker: cannot load %s; camera tracking disabled\n", led_file.c_str());
        } else if (!hid_.set_leds(true)) {
            std::fprintf(stderr, "HeadTracker: LED enable failed; camera tracking disabled\n");
        } else if (!camera_.open()) {
            std::fprintf(stderr, "HeadTracker: PTC not available; orientation-only tracking\n");
        } else {
            intrinsics_ = camera_.read_intrinsics();
            if (!intrinsics_.valid) {
                std::fprintf(stderr, "HeadTracker: no camera calibration; orientation-only tracking\n");
            } else if (!camera_.start(true)) {
                std::fprintf(stderr, "HeadTracker: PTC start failed; orientation-only tracking\n");
            } else {
                run_ = true;
                camera_ok_ = true;
                thread_ = std::thread(&HeadTracker::camera_thread, this);
            }
        }
    }
    // Let the filter settle on gravity before the first frame.
    for (int i = 0; i < 20; ++i) update();
    return true;
}

void HeadTracker::stop() {
    run_ = false;
    if (thread_.joinable()) thread_.join();
    camera_.stop();
    hid_.close();
    camera_ok_ = false;
}

void HeadTracker::camera_thread() {
    PtcBlobTracker tracker(model_);
    PtcPoseSolver solver(model_, intrinsics_);
    solver.set_mirror_image(!legacy_mirror_);
    PtcPose pose;
    PtcFrame frame;
    uint64_t counter = 0;
    while (run_) {
        if (!camera_.grab(frame, 100)) continue;
        const auto& blobs = tracker.process(frame);
        pose = solver.solve(blobs, pose);
        CamFix fix;
        fix.valid = pose.valid;
        fix.R = pose.R; fix.t = pose.t; fix.rms = pose.rms_px; fix.leds = pose.inliers;
        fix.id = ++counter;
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = fix;
    }
}

bool HeadTracker::take_world_accel(glm::vec3& out) {
    if (accel_n_ == 0) return false;
    out = accel_sum_ / float(accel_n_);
    accel_sum_ = glm::vec3(0);
    accel_n_ = 0;
    return true;
}

float HeadTracker::yaw_correction_deg() const { return glm::degrees(yaw_drift_); }

void HeadTracker::recentre() { want_recentre_ = true; want_pos_ref_ = true; }

void HeadTracker::update() {
    // --- IMU ---
    imu_.clear();
    hid_.poll(imu_, 0);
    for (const ImuSample& s : imu_) {
        const float dt = last_imu_time_ < 0 ? 0.001f : float(std::min(0.02, s.time - last_imu_time_));
        last_imu_time_ = s.time;
        clock_ = s.time;
        fusion_.update(s.gyro, s.accel, dt);
        if (fusion_.initialised()) { accel_sum_ += fusion_.orientation() * s.accel - glm::vec3(0, 9.81f, 0); ++accel_n_; }
    }
    if (!fusion_.initialised()) return;
    const glm::quat q_imu = fusion_.orientation();   // headset -> IMU world

    // --- Camera fix ---
    CamFix fix;
    { std::lock_guard<std::mutex> lock(mutex_); fix = latest_; }
    if (fix.id != seen_id_) {
        seen_id_ = fix.id;
        ++status_.fixes;
        status_.last_leds = fix.valid ? fix.leds : 0;
        status_.still = fusion_.angular_speed() < 0.6f;
        if (fix.valid) {
            // Headset -> view-style camera frame -> as a quaternion
            const glm::dmat3 R_fix = mirror_cam_ * fix.R * mirror_led_;
            const glm::dvec3 t_fix = mirror_cam_ * fix.t;
            const glm::quat q_hc = glm::quat_cast(glm::mat3(kCamToView * R_fix));   // headset -> camera-view frame
            const glm::quat c_obs = q_imu * glm::conjugate(q_hc);                    // camera-view frame -> IMU world
            status_.last_q_imu = q_imu;
            status_.last_q_hc = q_hc;
            const bool slow = fusion_.angular_speed() < 0.6f;

            // Sanity checks. A fix from few LEDs (e.g. when looking up, most LEDs face away from the camera) can be
            // a wrong, ambiguous solution; it must also agree with the gyro's orientation and not jump in position.
            bool accept = fix.leds >= kMinLeds;
            if (!accept) ++status_.rejected_leds;
            glm::vec3 p(0);
            if (accept && calibrated_) {
                const glm::quat e = glm::angleAxis(-yaw_drift_, kUp) * (c_obs * glm::conjugate(cam_to_world_));
                const float angle = 2.0f * std::acos(std::min(1.0f, std::fabs(e.w)));   // radians
                status_.last_orientation_error_deg = glm::degrees(angle);
                if (angle > (slow ? 0.20f : 0.45f)) { accept = false; ++status_.rejected_orientation; }   // ~11 / 26 degrees
                // Many LEDs, yet it keeps disagreeing: the stored camera-to-IMU alignment is wrong (e.g. it was taken while the headset
                // sat tilted back on the head). Throw it away and align again from the next steady poses.
                if (angle > 0.20f && fix.leds >= 12) ++orient_reject_run_; else if (angle <= 0.20f) orient_reject_run_ = 0;
                if (orient_reject_run_ >= 60) {
                    orient_reject_run_ = 0;
                    calibrated_ = false; calib_count_ = 0; calib_sum_ = glm::vec4(0); yaw_drift_ = 0;
                    have_pos_ = false; want_pos_ref_ = true; status_.aligned = false; status_.align_progress = 0;
                    std::printf("HeadTracker: camera and IMU disagree for a long time, aligning again.\n");
                    accept = false;
                }
                const glm::dvec3 t_view = kCamToView * t_fix;
                p = glm::angleAxis(-yaw_drift_, kUp) * (cam_to_world_ * glm::vec3(t_view));
                // Position jump relative to the smoothed track; after a long rejection streak accept (re-acquire).
                if (accept) status_.last_jump_m = have_pos_ ? glm::length(p - cam_pos_) : 0.0f;
                if (accept && have_pos_ && glm::length(p - cam_pos_) > 0.15f && rejected_run_ < 45) { accept = false; ++status_.rejected_jump; }
            }
            rejected_run_ = accept ? 0 : rejected_run_ + 1;
            if (accept) ++status_.accepted;

            if (accept) {
                if (!calibrated_) {
                    if (slow) {
                        glm::vec4 v(c_obs.x, c_obs.y, c_obs.z, c_obs.w);
                        if (calib_count_ > 0 && glm::dot(v, calib_sum_) < 0) v = -v;  // keep hemisphere consistent
                        calib_sum_ += v;
                        status_.align_progress = calib_count_ + 1;
                        if (++calib_count_ >= 25) {
                            const glm::vec4 n = glm::normalize(calib_sum_);
                            cam_to_world_ = glm::quat(n.w, n.x, n.y, n.z);
                            calibrated_ = true;
                            status_.aligned = true;
                            std::printf("HeadTracker: camera aligned with gravity/IMU world.\n");
                        }
                    }
                } else {
                    if (slow && fix.leds >= 10) {
                        // Yaw part of the disagreement between this camera fix and the calibrated extrinsics
                        const glm::quat e = c_obs * glm::conjugate(cam_to_world_);   // rotation in IMU world
                        yaw_drift_ += 0.05f * wrap_pi(yaw_of(e) - yaw_drift_);
                    }
                    cam_pos_ = have_pos_ && (clock_ - fix_time_) < 0.5 ? cam_pos_ + 0.6f * (p - cam_pos_) : p;
                    have_pos_ = true;
                    fix_time_ = clock_;
                }
                leds_ = fix.leds;
                rms_ = float(fix.rms);
            }
        }
    }
    tracking_ = camera_ok_ && have_pos_ && (clock_ - fix_time_) < 0.5;

    // --- Combine ---
    const glm::quat q_corr = glm::angleAxis(-yaw_drift_, kUp) * q_imu;   // yaw-corrected headset -> world
    if (want_recentre_) {
        ref_yaw_ = yaw_of(q_corr);
        want_recentre_ = false;
    }
    if (want_pos_ref_ && have_pos_) {
        ref_pos_ = cam_pos_;
        want_pos_ref_ = false;
    }
    const glm::quat q_ref = glm::angleAxis(-ref_yaw_, kUp);
    q_ref_ = q_ref;
    orientation_raw_ = q_ref * q_corr;
    // Predict: the view is shown ~predict_s_ from now. The gyro's body-frame angular velocity is unchanged by the re-centring
    // and yaw-correction rotations (they act on the world side), so it applies directly.
    orientation_ = predict_orientation(orientation_raw_, fusion_.angular_velocity(), predict_s_);

    // Position: follow the camera while it sees the headset; otherwise ease back to the centre (rather than freezing
    // at the last stale value). Re-acquisition eases in too, so there is no visible jump.
    if (have_pos_ && !want_pos_ref_) position_raw_ = q_ref * (cam_pos_ - ref_pos_);
    const glm::vec3 target = (have_pos_ && !want_pos_ref_ && tracking_) ? position_raw_ * pos_sign_ : glm::vec3(0.0f);
    const float pdt = last_pos_clock_ < 0 ? 0.0f : float(std::min(0.1, std::max(0.0, clock_ - last_pos_clock_)));
    last_pos_clock_ = clock_;
    const float tau = tracking_ ? 0.12f : 1.0f;
    position_ += (target - position_) * (1.0f - std::exp(-pdt / tau));
}
