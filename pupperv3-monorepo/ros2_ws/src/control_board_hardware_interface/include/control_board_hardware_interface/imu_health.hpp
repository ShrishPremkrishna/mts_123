#pragma once
#include <chrono>
#include <algorithm>
#include <array>
#include <initializer_list>
#include <cmath>
#include <utility>

namespace pupper_imu {
// Receipt freshness plus known batch-age lower bounds cannot prove true
// measurement freshness or calibration. The existing bound remains 100 ms.
constexpr auto max_receipt_age = std::chrono::milliseconds(100);
constexpr double stationary_gyro_limit_rad_s = 0.05;
constexpr double stationary_gyro_mean_limit_rad_s = 0.025;
constexpr auto stationary_window = std::chrono::seconds(3);
constexpr auto startup_timeout = std::chrono::seconds(15);

template <class Output>
const char *feedback_error(const Output &sample,
                          std::chrono::steady_clock::time_point now) {
  if (!sample.quaternion_valid || !sample.gyro_valid) return "missing complete IMU report";
  if (!sample.quat.coeffs().allFinite() || !sample.gyro.allFinite()) return "nonfinite IMU report";
  const double norm = sample.quat.norm();
  if (norm < 0.9 || norm > 1.1) return "invalid quaternion norm";
  const std::array<std::pair<std::chrono::steady_clock::time_point, std::chrono::microseconds>, 2> components{{
      {sample.quaternion_receipt_time, sample.quaternion_minimum_age_at_receipt},
      {sample.gyro_receipt_time, sample.gyro_minimum_age_at_receipt}}};
  for (const auto &component : components) {
    const auto receipt = component.first;
    const auto minimum_age = component.second;
    if (receipt == std::chrono::steady_clock::time_point{} || receipt > now ||
        now - receipt > max_receipt_age) return "stale IMU component receipt";
    if (minimum_age < std::chrono::microseconds::zero()) return "invalid IMU minimum age";
    if (now - receipt + minimum_age > max_receipt_age) return "IMU report known too old";
  }
  return nullptr;
}

// Used only before enabling motors on a supported robot. Gravity stability
// permits heading drift without treating it as body tilt; peak gyro and the
// distinct-report vector mean still bound motion and persistent rate bias.
// Passing is not calibration, heading accuracy, or future sensor-health proof.
// This screens sampled cached reports, not every intermediate hardware report.
struct StartupObservation {
  const char *reason = "eligible";
  const char *feedback = nullptr;
  bool observation_gap = false;
  double arc_rad = 0.0;
  double eligible_seconds = 0.0;
  double gyro_mean_norm = 0.0;
  std::size_t gyro_samples = 0;
};

class StartupWindow {
 public:
  template <class Output>
  bool observe(const Output &sample, std::chrono::steady_clock::time_point now,
               StartupObservation *diagnostic = nullptr) {
    if (diagnostic) *diagnostic = StartupObservation{};
    if (last_observation_ != std::chrono::steady_clock::time_point{} &&
        (now < last_observation_ || now-last_observation_ > max_receipt_age)) {
      clear_window();
      if (diagnostic) diagnostic->observation_gap = true;
    }
    last_observation_ = now;
    const char *health_error = feedback_error(sample, now);
    if (health_error || sample.gyro.norm() > stationary_gyro_limit_rad_s) {
      if (diagnostic) {
        diagnostic->feedback = health_error;
        diagnostic->reason = health_error ? "feedback" : "gyro_norm";
      }
      clear_window();
      return false;
    }
    const double norm = sample.quat.norm();
    const std::array<double, 4> q{sample.quat.x()/norm, sample.quat.y()/norm,
                                  sample.quat.z()/norm, sample.quat.w()/norm};
    // Gravity expressed in the sensor frame: inverse(q) * world-Z.
    // World-heading changes and q/-q leave this vector unchanged. A fixed
    // mounting rotation preserves the angle between these gravity vectors.
    const std::array<double, 3> gravity{
        2*(q[0]*q[2]-q[3]*q[1]), 2*(q[1]*q[2]+q[3]*q[0]),
        1-2*(q[0]*q[0]+q[1]*q[1])};
    if (!started_) restart(now, gravity);
    double dot = 0;
    for (unsigned i=0; i<3; ++i) dot += anchor_[i]*gravity[i];
    const double arc = std::acos(std::max(-1.0, std::min(1.0, dot)));
    if (diagnostic) diagnostic->arc_rad = arc;
    if (arc > 0.017453292519943295) {
      if (diagnostic) diagnostic->reason = "gravity";
      restart(now, gravity);
      return false;
    }
    // Equal cached receipt clocks count once. Do not include a receipt before
    // this candidate window, or let an older cached report re-enter the mean.
    if (sample.gyro_receipt_time >= start_ &&
        sample.gyro_receipt_time > last_gyro_receipt_) {
      last_gyro_receipt_ = sample.gyro_receipt_time;
      ++gyro_samples_;
      gyro_mean_[0] += (sample.gyro.x()-gyro_mean_[0])/gyro_samples_;
      gyro_mean_[1] += (sample.gyro.y()-gyro_mean_[1])/gyro_samples_;
      gyro_mean_[2] += (sample.gyro.z()-gyro_mean_[2])/gyro_samples_;
    }
    double mean_norm = 0;
    if (gyro_samples_) {
      for (double v : gyro_mean_) mean_norm += v*v;
      mean_norm = std::sqrt(mean_norm);
    }
    const auto elapsed = std::min(sample.quaternion_receipt_time,
                                  sample.gyro_receipt_time) - start_;
    if (diagnostic) {
      diagnostic->eligible_seconds = std::chrono::duration<double>(elapsed).count();
      diagnostic->gyro_mean_norm = mean_norm;
      diagnostic->gyro_samples = gyro_samples_;
    }
    if (elapsed < stationary_window) return false;
    if (!gyro_samples_ || mean_norm > stationary_gyro_mean_limit_rad_s) {
      if (diagnostic) diagnostic->reason = "gyro_mean";
      restart(now, gravity);
      return false;
    }
    return true;
  }
 private:
  std::chrono::steady_clock::time_point last_observation_{};
  void clear_window() {
    started_ = false; gyro_mean_ = {}; gyro_samples_ = 0;
    last_gyro_receipt_ = std::chrono::steady_clock::time_point{};
  }
  void restart(std::chrono::steady_clock::time_point now,
               const std::array<double, 3> &gravity) {
    clear_window(); start_ = now; anchor_ = gravity; started_ = true;
  }
  std::array<double, 3> anchor_{};
  std::array<double, 3> gyro_mean_{};
  std::size_t gyro_samples_ = 0;
  std::chrono::steady_clock::time_point last_gyro_receipt_{};
  bool started_ = false;
  std::chrono::steady_clock::time_point start_{};
};
}  // namespace pupper_imu
