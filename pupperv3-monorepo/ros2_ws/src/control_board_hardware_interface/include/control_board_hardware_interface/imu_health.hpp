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
constexpr double stationary_gyro_limit_rad_s = 0.005;
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

// Used only before enabling motors on a supported, stationary robot. It rejects
// the large at-rest bias observed in the failed session; passing cannot prove
// calibration, correct heading, or readiness on a moving/unsupported robot.
// This screens sampled cached reports, not every intermediate hardware report.
class StartupWindow {
 public:
  template <class Output>
  bool observe(const Output &sample, std::chrono::steady_clock::time_point now) {
    if (last_observation_ != std::chrono::steady_clock::time_point{} &&
        (now < last_observation_ || now-last_observation_ > max_receipt_age)) started_ = false;
    last_observation_ = now;
    if (feedback_error(sample, now) || sample.gyro.norm() > stationary_gyro_limit_rad_s) {
      started_ = false;
      return false;
    }
    const double norm = sample.quat.norm();
    const std::array<double, 4> q{sample.quat.x()/norm, sample.quat.y()/norm,
                                  sample.quat.z()/norm, sample.quat.w()/norm};
    if (!started_) { start_ = now; anchor_ = q; started_ = true; }
    double dot = 0;
    for (unsigned i=0; i<4; ++i) dot += anchor_[i]*q[i];
    // abs(dot) identifies q and -q as the same orientation. One-degree bound.
    const double arc = 2*std::acos(std::min(1.0, std::abs(dot)));
    if (arc > 0.017453292519943295) {
      start_ = now; anchor_ = q;
      return false;
    }
    // Only new component receipts advance the window; repeated polls of a
    // cached value cannot manufacture three seconds of distinct report data.
    return std::min(sample.quaternion_receipt_time, sample.gyro_receipt_time) - start_
             >= stationary_window;
  }
 private:
  std::chrono::steady_clock::time_point last_observation_{};
  std::array<double, 4> anchor_{};
  bool started_ = false;
  std::chrono::steady_clock::time_point start_{};
};
}  // namespace pupper_imu
