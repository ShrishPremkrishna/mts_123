#pragma once
#include "control_board_hardware_interface/imu_health.hpp"
#include <atomic>
#include <cerrno>
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
  static std::unique_ptr<StartupTrace> from_environment(const std::atomic<bool> &home_fault) noexcept {
    const char *path = std::getenv("PUPPER_IMU_STARTUP_TRACE");
    try {
      if (path && *path) return std::make_unique<StartupTrace>(path);
      const char *health=std::getenv("PUPPER_FC_BOARD_HEALTH_PATH");
      if (!health || *health!='/') return nullptr;
      const std::string parent(health);
      return std::make_unique<StartupTrace>(parent.substr(0,parent.rfind('/')+1)+"native-startup-trace.csv",&home_fault);
    }
    catch (...) { return nullptr; }  // Diagnostics cannot block activation.
  }
  explicit StartupTrace(const std::string &path, const std::atomic<bool> *home_fault=nullptr)
      : path_(path), home_fault_(home_fault) { rows_.reserve(capacity); }
  ~StartupTrace() {
    // A mechanical fault after primary qualification also keeps the IMU record.
    // The terminal flag is published only after neutral, including on hardware faults.
    if (primary_passed_ && home_fault_ && home_fault_->load())
      write_trace(true,true);
  }
  template<class Output>
  void record(const Output &sample, std::chrono::steady_clock::time_point now,
              const StartupObservation &decision, bool ready, const char *source_reason, bool alternate_ready) noexcept {
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
    row.source_reason=source_reason; row.alternate_ready=alternate_ready;
    const auto &alt=sample.alternative;
    row.alt_q_receipt=seconds(alt.quaternion_receipt_time.time_since_epoch());
    row.alt_g_receipt=seconds(alt.gyro_receipt_time.time_since_epoch());
    row.batch_us={{sample.quaternion_minimum_age_at_receipt.count(),sample.gyro_minimum_age_at_receipt.count(),
                   alt.quaternion_minimum_age_at_receipt.count(),alt.gyro_minimum_age_at_receipt.count()}};
    row.alt_q_seq=alt.quaternion_report_sequence; row.alt_g_seq=alt.gyro_report_sequence;
    row.alt_q_accuracy=alt.quaternion_accuracy; row.alt_g_accuracy=alt.gyro_accuracy;
    row.alt_q_valid=alt.quaternion_valid; row.alt_g_valid=alt.gyro_valid;
    row.alt_gaps=alt.gap_count; row.resets=alt.reset_count;
    rows_.push_back(row);
  }
  // Called once after the startup outcome, before enable; never overwrites an
  // earlier trace. Failure is reported to the caller and changes no gate result.
  bool finish(bool passed) noexcept {
    primary_passed_=passed;
    if (passed && home_fault_) return true;
    return write_trace(passed,false);
  }
  std::size_t size() const { return rows_.size(); }
  std::size_t dropped() const { return dropped_; }
 private:
  bool write_trace(bool passed, bool later_fault) noexcept {
    int fd = ::open(path_.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (fd<0 && home_fault_ && errno==EEXIST) {
      // A second Home in the same owned run gets a separate non-overwriting file.
      try {
        const auto stamp=std::chrono::steady_clock::now().time_since_epoch().count();
        const auto next=path_.substr(0,path_.size()-4)+"-"+std::to_string(stamp)+".csv";
        fd=::open(next.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
      } catch (...) { return false; }
    }
    if (fd < 0) return false;
    FILE *file = ::fdopen(fd, "w");
    if (!file) { ::close(fd); return false; }
    bool ok = std::fprintf(file, "# startup_passed=%d,rows=%zu,dropped=%zu,later_home_fault=%d\n", passed, rows_.size(), dropped_,later_fault) >= 0;
    ok = std::fprintf(file, "steady_s,quaternion_receipt_s,gyro_receipt_s,quaternion_age_s,gyro_age_s,q_valid,g_valid,qx,qy,qz,qw,gx,gy,gz,q_norm,gyro_norm,reason,feedback,observation_gap,gravity_arc_rad,eligible_seconds,gyro_mean_norm,gyro_samples,ready,source_reason,alternate_ready,alt_quaternion_receipt_s,alt_gyro_receipt_s,primary_q_batch_us,primary_g_batch_us,alt_q_batch_us,alt_g_batch_us,alt_q_seq,alt_g_seq,alt_q_accuracy,alt_g_accuracy,alt_q_valid,alt_g_valid,alt_gaps,sensor_resets\n") >= 0 && ok;
    for (const auto &r : rows_) {
      ok = std::fprintf(file, "%.9f,%.9f,%.9f,%.9f,%.9f,%d,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%s,%s,%d,%.9g,%.9g,%.9g,%zu,%d",
          r.now, r.q_receipt, r.g_receipt, r.now-r.q_receipt, r.now-r.g_receipt,
          r.q_valid, r.g_valid, r.q[0],r.q[1],r.q[2],r.q[3],r.g[0],r.g[1],r.g[2],r.q_norm,r.g_norm,
          r.decision.reason, r.decision.feedback ? r.decision.feedback : "", r.decision.observation_gap,
          r.decision.arc_rad,r.decision.eligible_seconds,r.decision.gyro_mean_norm,r.decision.gyro_samples,r.ready) >= 0 && ok;
      ok = std::fprintf(file,",%s,%d,%.9f,%.9f,%lld,%lld,%lld,%lld,%d,%d,%d,%d,%d,%d,%llu,%llu\n",
          r.source_reason,r.alternate_ready,r.alt_q_receipt,r.alt_g_receipt,r.batch_us[0],r.batch_us[1],
          r.batch_us[2],r.batch_us[3],r.alt_q_seq,r.alt_g_seq,r.alt_q_accuracy,r.alt_g_accuracy,
          r.alt_q_valid,r.alt_g_valid,r.alt_gaps,r.resets)>=0 && ok;
    }
    return std::fclose(file) == 0 && ok;
  }
  template<class Duration> static double seconds(Duration d) { return std::chrono::duration<double>(d).count(); }
  struct Row {
    double now, q_receipt, g_receipt, q_norm, g_norm;
    std::array<double,4> q;
    std::array<double,3> g;
    bool q_valid,g_valid,ready;
    const char *source_reason;
    bool alternate_ready,alt_q_valid,alt_g_valid;
    int alt_q_seq,alt_g_seq,alt_q_accuracy,alt_g_accuracy;
    double alt_q_receipt,alt_g_receipt;
    std::array<long long,4> batch_us;
    unsigned long long alt_gaps,resets;
    StartupObservation decision;
  };
  std::string path_;
  const std::atomic<bool> *home_fault_;
  bool primary_passed_=false;
  std::vector<Row> rows_;
  std::size_t dropped_ = 0;
};
}  // namespace pupper_imu
