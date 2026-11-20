// Sensor-only reproduction of the FC Home observers. No ROS or SPI linkage.
// Records the native feature setup and both gates even if a gate passes early.
#include "control_board_hardware_interface/redundant_imu.hpp"
#include <atomic>
#include <fstream>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using Output = BNO055::Output;

int main(int argc, char **argv) try {
  if (argc != 3 || std::string(argv[1]) != "--supported-no-motors")
    throw std::runtime_error("usage: probe_dual_startup --supported-no-motors NEW.csv");
  // Hold the same stable inode as the hardware driver for the entire sensor
  // lifetime. The OS releases this lock even on diagnostic process failure.
  const int owner = ::open("/tmp/control_board_hardware.lock", O_RDONLY | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0666);
  if (owner < 0 || ::flock(owner, LOCK_EX | LOCK_NB) < 0)
    throw std::runtime_error("hardware owner active or lock unavailable; sensor untouched");
  const int capture = ::open(argv[2], O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (capture < 0) throw std::runtime_error("a new writable capture path is required");
  std::ofstream file("/proc/self/fd/" + std::to_string(capture)); // Linux Pi only.
  if (!file) throw std::runtime_error("could not create output");
  file << "t,primary_ready,backup_ready,primary_reason,primary_error,source_error,span,arc,mean,n,mode,reset,gaps";
  for (auto prefix : {"p", "a"}) {
    for (auto key : {"q_seq","g_seq","q_valid","g_valid","q_accuracy","g_accuracy","q_receipt","g_receipt",
                     "q_min_age","g_min_age","qx","qy","qz","qw","gx","gy","gz"})
      file << ',' << prefix << '_' << key;
  }
  file << '\n' << std::setprecision(12);
  // These are the same two calls used by IMUManager::begin on Home. They
  // reset the BNO and request 02/05/07/08 at 100 Hz. Never open SPI.
  BNO055 sensor(1, 10000);
  sensor.enable_redundant_reports(10000);
  std::mutex lock;
  Output latest{};
  std::atomic<bool> stopping{false};
  std::thread reader([&] {
    while (!stopping) {
      for (unsigned burst = 0; burst < 32 && !stopping; ++burst) {
        auto s = sensor.sample();
        std::lock_guard<std::mutex> copy(lock);
        const bool advanced = latest.quaternion_receipt_time != s.quaternion_receipt_time ||
            latest.gyro_receipt_time != s.gyro_receipt_time ||
            latest.alternative.quaternion_receipt_time != s.alternative.quaternion_receipt_time ||
            latest.alternative.gyro_receipt_time != s.alternative.gyro_receipt_time;
        latest = s;
        if (!advanced) break;
      }
      std::this_thread::sleep_for(1ms);
    }
  });
  pupper_imu::StartupWindow primary;
  pupper_imu::RedundantImu backup;
  const auto start = Clock::now();
  auto elapsed = [&](Clock::time_point t) { return std::chrono::duration<double>(t - start).count(); };
  unsigned n = 0, primary_ready = 0, backup_ready = 0, both = 0;
  while (true) {
    Output s{};
    { std::lock_guard<std::mutex> copy(lock); s = latest; }
    const auto now = Clock::now();
    pupper_imu::StartupObservation diag{};
    const bool ready = primary.observe(s, now, &diag);
    Output selected{};
    const char *err = backup.select(s, now, false, selected);
    file << elapsed(now) << ',' << ready << ',' << backup.qualified << ',' << diag.reason << ','
         << (diag.feedback ? diag.feedback : "") << ',' << (err ? err : "") << ',' << diag.eligible_seconds
         << ',' << diag.arc_rad << ',' << diag.gyro_mean_norm << ',' << diag.gyro_samples << ','
         << backup.mode << ',' << s.alternative.reset_count << ',' << s.alternative.gap_count;
    auto components = [&](const auto &v) {
      file << ',' << v.quaternion_report_sequence << ',' << v.gyro_report_sequence << ',' << v.quaternion_valid
           << ',' << v.gyro_valid << ',' << int(v.quaternion_accuracy) << ',' << int(v.gyro_accuracy)
           << ',' << elapsed(v.quaternion_receipt_time) << ',' << elapsed(v.gyro_receipt_time)
           << ',' << v.quaternion_minimum_age_at_receipt.count() << ',' << v.gyro_minimum_age_at_receipt.count()
           << ',' << v.quat.x() << ',' << v.quat.y() << ',' << v.quat.z() << ',' << v.quat.w()
           << ',' << v.gyro.x() << ',' << v.gyro.y() << ',' << v.gyro.z();
    };
    components(s); components(s.alternative); file << '\n';
    ++n; primary_ready += ready; backup_ready += backup.qualified; both += ready && !err && backup.qualified;
    if (now - start >= pupper_imu::startup_timeout) break;
    std::this_thread::sleep_for(5ms);
  }
  stopping = true;
  reader.join();
  file.close();
  if (!file) throw std::runtime_error("capture write failed");
  std::cout << "samples=" << n << " primary_ready=" << primary_ready << " backup_ready=" << backup_ready
            << " both_ready=" << both << " no motors opened\n";
  return 0;
} catch (const std::exception &e) {
  std::cerr << e.what() << '\n'; return 1;
}
