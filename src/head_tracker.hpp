// Head tracking for the DK2: IMU orientation (fast, 1 kHz) corrected and extended by the PTC camera
// (yaw-drift correction and 3D position). Owns the headset HID connection and the camera thread.
#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "dk2_hid.hpp"
#include "imu_fusion.hpp"
#include "ptc_camera.hpp"
#include "ptc_pose.hpp"
#include "ptc_tracker.hpp"

class HeadTracker {
public:
    ~HeadTracker() { stop(); }
    // Opens the headset (and, if use_camera, the PTC). Camera failure is not fatal: orientation-only tracking.
    bool start(const std::string& led_file, bool use_camera);
    void stop();

    // Call once per rendered frame: drains the IMU and applies the latest camera pose.
    void update();
    // Make the current heading "straight ahead" and the current position the origin.
    void recentre();

    // Headset -> world (world: +Y up, -Z forward at recentre), predicted forward by prediction() seconds to compensate for
    // render + scan-out delay (so what you see matches where your head is when the light reaches your eye).
    glm::quat orientation() const { return orientation_; }
    glm::quat orientation_raw() const { return orientation_raw_; }   // latest measured orientation, no prediction
    void set_prediction(float seconds) { predict_s_ = seconds < 0 ? 0 : (seconds > 0.06f ? 0.06f : seconds); }
    float prediction() const { return predict_s_; }
    // Metres, relative to the recentre position. When the PTC loses the headset the position eases back to the centre
    // (instead of freezing); when it finds it again it eases to the tracked value.
    glm::vec3 position() const { return position_; }
    // The same, before the sign correction and without easing -- for calibration. Only meaningful while camera_tracking().
    glm::vec3 position_raw() const { return position_raw_; }
    // Per-axis sign applied to position() (user frame: +X right, +Y up, +Z back). It is the same for every program: it is read from
    // ~/.config/dk2vr/position_signs at start, and changing it (flip_position_axis, set_position_signs) saves it there.
    // Where the PTC itself is, in the same frame and with the same signs as position() (so a headset lean towards it shrinks the
    // distance). False until the position origin has been set by a first camera fix.
    bool camera_position(glm::vec3& out) const {
        if (!have_pos_ || want_pos_ref_) return false;
        out = (q_ref_ * (-ref_pos_)) * pos_sign_;
        return true;
    }
    glm::vec3 position_signs() const { return pos_sign_; }
    void set_position_signs(const glm::vec3& s);
    void flip_position_axis(int axis);
    // Linear acceleration (gravity removed) in the user frame, averaged over the IMU samples since the last call.
    bool take_user_accel(glm::vec3& out);
    // While true the tilt correction is weak, so accelerations from leaning are not mistaken for head tilt (see ImuFusion).
    void set_calibration_mode(bool on) { fusion_.set_gravity_gain(on ? 0.03f : 1.0f); }
    bool camera_active() const { return camera_ok_; }        // PTC thread running
    bool camera_tracking() const { return tracking_; }       // PTC pose valid recently
    int camera_leds() const { return leds_; }                // LEDs used in the last PTC fix
    float camera_rms_px() const { return rms_; }
    // Debug/calibration: camera position in the IMU world frame (before re-centring) and linear acceleration
    // (gravity removed) in the same frame, averaged over the IMU samples since the last call.
    glm::vec3 imu_world_position() const { return cam_pos_; }
    bool take_world_accel(glm::vec3& out);
    // Why isn't the camera tracking? (for diagnostics)
    struct CameraStatus {
        uint64_t fixes = 0;      // camera poses received so far
        int last_leds = 0;       // LEDs identified in the latest pose (a pose needs at least 8 to be trusted)
        bool aligned = false;    // camera aligned with the IMU world (needs ~25 trusted poses while the headset is held fairly still)
        int align_progress = 0;  // trusted still poses collected towards the alignment (of 25)
        bool still = false;      // the headset is turning slowly enough for the alignment to collect poses
        uint64_t accepted = 0, rejected_leds = 0, rejected_orientation = 0, rejected_jump = 0;   // why poses were dropped
        float last_orientation_error_deg = 0, last_jump_m = 0;
        glm::quat last_q_imu{1, 0, 0, 0}, last_q_hc{1, 0, 0, 0};   // IMU orientation and camera-pose orientation of the latest pose (diagnostics)
    };
    CameraStatus camera_status() const { return status_; }
    float yaw_correction_deg() const;                        // current camera yaw-drift estimate

private:
    void camera_thread();

    Dk2Hid hid_;
    ImuFusion fusion_;
    std::vector<ImuSample> imu_;
    double last_imu_time_ = -1;

    LedModel model_;
    PtcCamera camera_;
    PtcIntrinsics intrinsics_;
    std::thread thread_;
    std::atomic<bool> run_{false};
    bool camera_ok_ = false;

    struct CamFix { bool valid = false; glm::dmat3 R{1.0}; glm::dvec3 t{0.0}; double rms = 0; int leds = 0; uint64_t id = 0; };
    std::mutex mutex_;
    CamFix latest_;
    uint64_t seen_id_ = 0;

    // Camera -> IMU-world extrinsics (rotation), calibrated from the first still camera fixes; yaw drift estimate.
    glm::quat cam_to_world_{1, 0, 0, 0};
    int calib_count_ = 0;
    glm::vec4 calib_sum_{0};
    bool calibrated_ = false;
    float yaw_drift_ = 0;          // radians: IMU yaw has drifted by this much relative to the camera
    glm::vec3 cam_pos_{0};         // smoothed position in the corrected world frame
    bool have_pos_ = false;
    double fix_time_ = -1e9;
    CameraStatus status_;
    bool legacy_mirror_ = false;
    glm::dmat3 mirror_cam_{1.0}, mirror_led_{1.0};
    int orient_reject_run_ = 0;    // consecutive good-looking fixes (many LEDs) that disagree with the IMU: the alignment is probably wrong
    int rejected_run_ = 0;         // consecutive camera fixes rejected by the sanity checks
    double clock_ = 0;

    // Recentre reference
    float ref_yaw_ = 0;
    glm::vec3 ref_pos_{0};
    bool want_recentre_ = true;
    bool want_pos_ref_ = true;     // set the position origin at the next valid camera position

    glm::vec3 accel_sum_{0};
    int accel_n_ = 0;

    glm::quat orientation_{1, 0, 0, 0}, orientation_raw_{1, 0, 0, 0};
    glm::vec3 position_{0}, position_raw_{0};
    glm::vec3 pos_sign_{1.0f, 1.0f, 1.0f};   // default until a saved/calibrated value exists (see start())
    glm::quat q_ref_{1, 0, 0, 0};
    void load_position_signs();
    void save_position_signs() const;
    float predict_s_ = 0.020f;   // ~1 frame of rendering/vsync wait + half a scan-out at 60 Hz
    double last_pos_clock_ = -1;
    bool tracking_ = false;
    int leds_ = 0;
    float rms_ = 0;
};
