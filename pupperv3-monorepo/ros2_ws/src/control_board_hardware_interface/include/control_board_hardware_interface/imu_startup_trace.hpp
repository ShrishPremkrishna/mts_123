#pragma once
#include "control_board_hardware_interface/imu_health.hpp"
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>
#include <memory>
#include <string>
#include <vector>

namespace pupper_imu {
// Optional diagnostics only. Reserve before the startup deadline; no allocation
// or I/O while observing. Raw samples are the exact cached samples the guard sees.
class StartupTrace {
 public:
  static constexpr std::size_t capacity = 4096;
  static std::unique_ptr<StartupTrace> from_environment() noexcept {
    const char *path = std::getenv("PUPPER_IMU_STARTUP_TRACE");
    if (!path || !*path) return nullptr;
    try { return std::unique_ptr<StartupTrace>(new StartupTrace(path)); }
    catch (...) { return nullptr; }  // Diagnostics cannot block activation.
  }
  explicit StartupTrace(const char *path) : path_(path) { rows_.reserve(capacity); }
  template<class Output>
  void record(const Output &sample, std::chrono::steady_clock::time_point now,
              const StartupObservation &decision, bool ready) noexcept {
    if (rows_.size() == capacity) { ++dropped_; return; }
    Row row{};
    row.now = seconds(now.time_since_epoch());
    row.q_receipt = seconds(sample.quaternion_receipt_time.time_since_epoch());
    row.g_receipt = seconds(sample.gyro_receipt_time.time_since_epoch());
    row.q_valid = sample.quaternion_valid; row.g_valid = sample.gyro_valid;
    row.q = {{sample.quat.x(), sample.quat.y(), sample.quat.z(), sample.quat.w()}};
    row.g = {{sample.gyro.x(), sample.gyro.y(), sample.gyro.z()}};
    row.g_norm = sample.gyro.norm(); row.q_norm = sample.quat.norm();
    row.decision = decision; row.ready = ready;
    rows_.push_back(row);
  }
  // Called once after the startup outcome, before enable; never overwrites an
  // earlier trace. Failure is reported to the caller and changes no gate result.
  bool finish(bool passed) noexcept {
    int fd = ::open(path_.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (fd < 0) return false;
    FILE *file = ::fdopen(fd, "w");
    if (!file) { ::close(fd); return false; }
    bool ok = std::fprintf(file, "# startup_passed=%d,rows=%zu,dropped=%zu\n", passed, rows_.size(), dropped_) >= 0;
    ok = std::fprintf(file, "steady_s,quaternion_receipt_s,gyro_receipt_s,quaternion_age_s,gyro_age_s,q_valid,g_valid,qx,qy,qz,qw,gx,gy,gz,q_norm,gyro_norm,reason,feedback,observation_gap,gravity_arc_rad,eligible_seconds,gyro_mean_norm,gyro_samples,ready\n") >= 0 && ok;
    for (const auto &r : rows_) {
      ok = std::fprintf(file, "%.9f,%.9f,%.9f,%.9f,%.9f,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%s,%s,%d,%.9g,%.9g,%.9g,%zu,%d\n",
          r.now, r.q_receipt, r.g_receipt, r.now-r.q_receipt, r.now-r.g_receipt,
          r.q_valid, r.g_valid, r.q[0],r.q[1],r.q[2],r.q[3],r.g[0],r.g[1],r.g[2],r.q_norm,r.g_norm,
          r.decision.reason, r.decision.feedback ? r.decision.feedback : "", r.decision.observation_gap,
          r.decision.arc_rad,r.decision.eligible_seconds,r.decision.gyro_mean_norm,r.decision.gyro_samples,r.ready) >= 0 && ok;
    }
    return std::fclose(file) == 0 && ok;
  }
  std::size_t size() const { return rows_.size(); }
  std::size_t dropped() const { return dropped_; }
 private:
  template<class Duration> static double seconds(Duration d) { return std::chrono::duration<double>(d).count(); }
  struct Row {
    double now, q_receipt, g_receipt, q_norm, g_norm;
    std::array<double,4> q;
    std::array<double,3> g;
    bool q_valid,g_valid,ready;
    StartupObservation decision;
  };
  std::string path_;
  std::vector<Row> rows_;
  std::size_t dropped_ = 0;
};
}  // namespace pupper_imu
