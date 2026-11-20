#include "control_board_hardware_interface/imu_manager.hpp"

IMUManager::~IMUManager() { stop(); }

// set rotation matrix based on roll, pitch, yaw
void IMUManager::set_imu_offset(double roll, double pitch, double yaw) {
  offset_quaternion.setRPY(roll, pitch, yaw);
  offset_rotation_matrix = tf2::Matrix3x3(offset_quaternion);
}

bool IMUManager::contains_nan(const Eigen::Quaternionf &q) {
  return std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) || std::isnan(q.w());
}

std::optional<std::pair<BNO055::Output, uint32_t>> IMUManager::sample_imu_multiple_attempts(
    int max_samples, int delay_ms) {
  /*
   * Sample the IMU up to max_samples times until a good sample is received.
   * Require independently observed, finite quaternion and gyro reports.
   * Consumers separately enforce host-receipt age; this is not calibration proof.
   */

  // Record start time
  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < max_samples; i++) {
    BNO055::Output output = imu_->sample();
    // Keep console I/O off the polling thread. The driver records a single
    // timestamped diagnostic for the sample that actually disables actuation.
    if (redundant_reports) return std::make_pair(output, 0u);
    if (output.quaternion_valid && output.gyro_valid && output.quat.coeffs().allFinite() &&
        output.gyro.allFinite() && !output.quat.isApprox(Eigen::Quaternionf(0, 0, 0, 0), 1e-6)) {
      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
      return std::make_pair(output, duration.count());
    }
    if (delay_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
  }
  return std::nullopt;
}

bool IMUManager::poll_imu() {
  auto maybe_output = sample_imu_multiple_attempts(/*max_samples=*/5, /*delay_ms=*/1);
  if (maybe_output) {
    // lock and copy imu data
    std::lock_guard<std::mutex> lock(imu_mutex_);
    const auto &output = maybe_output->first;
    const bool advanced = output.quaternion_receipt_time != imu_data.quaternion_receipt_time ||
                          output.gyro_receipt_time != imu_data.gyro_receipt_time ||
                          (redundant_reports && (output.alternative.quaternion_receipt_time != imu_data.alternative.quaternion_receipt_time ||
                           output.alternative.gyro_receipt_time != imu_data.alternative.gyro_receipt_time));
    imu_data = output;
    imu_sampling_time_us = maybe_output->second;
    return advanced;
  } else {
    // An observed invalid stream must not hide behind the last good cache.
    // Retain values and receipt clocks for diagnosis, but revoke usability.
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_data.quaternion_valid = false;
    imu_data.gyro_valid = false;
    return false;
  }
}

BNO055::Output IMUManager::get_imu_data() {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return imu_data;
}

void IMUManager::imu_read_thread_fn() {
  while (!stop_imu_read_thread_) {
    // Publish each pair under the mutex, release it, then service a queued
    // packet immediately. Empty reads back off and long bursts still yield.
    for (unsigned i = 0; i < 32 && !stop_imu_read_thread_ && poll_imu(); ++i) {}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void IMUManager::begin(uint32_t micros_between_reports) {
  imu_ = std::make_unique<BNO055>(IMU_I2C_DEVICE_NUMBER, micros_between_reports);

  if (redundant_reports) imu_->enable_redundant_reports(micros_between_reports);
  stop_imu_read_thread_ = false;
  imu_read_thread_ = std::make_unique<std::thread>(&IMUManager::imu_read_thread_fn, this);
}

void IMUManager::stop() {
  stop_imu_read_thread_ = true;
  if (imu_read_thread_ && imu_read_thread_->joinable()) {
    imu_read_thread_->join();
  }
  imu_.reset();  // Close the I2C descriptor before relinquishing board ownership.
}
