// Orientation from gyroscope + accelerometer (Mahony complementary filter). Tilt (pitch/roll) is kept
// drift-free by gravity; yaw is integrated from the gyro only and is corrected externally (by the PTC).
#pragma once
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Rotates `q` forward in time: q(t + dt) for a headset turning at body-frame angular velocity `omega` (rad/s).
// The prediction angle is capped (a glitchy gyro must never fling the view).
inline glm::quat predict_orientation(const glm::quat& q, const glm::vec3& omega, float dt, float max_angle = 0.4f) {
    const float speed = glm::length(omega);
    float ang = speed * dt;
    if (ang < 1e-6f || speed < 1e-6f) return q;
    ang = std::min(ang, max_angle);
    return glm::normalize(q * glm::angleAxis(ang, omega / speed));
}

class ImuFusion {
public:
    // q maps headset (sensor) coordinates to the gravity-aligned world (+Y up).
    glm::quat orientation() const { return q_; }
    bool initialised() const { return init_; }
    float angular_speed() const { return speed_; }
    // Bias-corrected, lightly smoothed angular velocity in the headset (sensor) frame, rad/s.
    glm::vec3 angular_velocity() const { return omega_; }
    // Strength of the gravity (tilt) correction. 1 is normal. A low value (calibration of linear acceleration) keeps leaning accelerations
    // from being absorbed as head tilt, at the price of slow tilt drift.
    void set_gravity_gain(float kp) { kp_ = kp; }

    void update(const glm::vec3& gyro_in, const glm::vec3& accel, float dt) {
        // Slowly learned gyro bias while the headset is still
        const float a_norm = glm::length(accel);
        const bool still = glm::length(gyro_in - bias_) < 0.06f && std::fabs(a_norm - 9.81f) < 0.4f;
        if (still && init_) bias_ += 0.0005f * (gyro_in - bias_);
        glm::vec3 gyro = gyro_in - bias_;
        speed_ = glm::length(gyro);
        omega_ += std::min(1.0f, dt / 0.004f) * (gyro - omega_);  // ~4 ms smoothing: removes sample noise, keeps real motion

        if (!init_) {
            if (a_norm < 1.0f) return;
            // Start with the sensor's "up" aligned to world +Y.
            const glm::vec3 up = accel / a_norm;
            const glm::vec3 axis = glm::cross(up, glm::vec3(0, 1, 0));
            const float d = glm::dot(up, glm::vec3(0, 1, 0));
            q_ = d < -0.9999f ? glm::angleAxis(3.14159265f, glm::vec3(1, 0, 0))
                              : glm::normalize(glm::quat(1.0f + d, axis.x, axis.y, axis.z));  // shortest rotation up -> +Y
            // Gyro bias starts at zero (the headset's factory calibration is already subtracted) and is refined while still.
            init_ = true;
            return;
        }

        // Gravity correction: compare the measured "up" with the filter's idea of "up" in the sensor frame.
        if (a_norm > 6.0f && a_norm < 14.0f) {
            const glm::vec3 a = accel / a_norm;
            const glm::vec3 v = glm::conjugate(q_) * glm::vec3(0, 1, 0);
            const glm::vec3 e = glm::cross(a, v);
            gyro += kp_ * e;
        }
        // Integrate: q <- q * exp(0.5 * w * dt)
        const float ang = glm::length(gyro) * dt;
        if (ang > 1e-9f) {
            const glm::vec3 axis = gyro / glm::length(gyro);
            q_ = glm::normalize(q_ * glm::angleAxis(ang, axis));
        }
    }

private:
    glm::quat q_{1, 0, 0, 0};
    glm::vec3 bias_{0}, omega_{0};
    float kp_ = 1.0f, speed_ = 0;
    bool init_ = false;
};
