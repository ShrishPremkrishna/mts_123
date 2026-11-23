#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "board_health.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "imu_manager.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rt/rt_spi.h"

// Overload the << operator for vector and array
template <typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &container) {
  for (const auto &e : container) {
    os << e << " ";
  }
  return os;
}
template <typename T, std::size_t N>
std::ostream &operator<<(std::ostream &os, const std::array<T, N> &container) {
  for (const auto &e : container) {
    os << e << " ";
  }
  return os;
}

namespace control_board_hardware_interface {

// NaN and +/-Inf are both unusable state.
template <typename T>
bool contains_nonfinite(const T &container) {
  return std::any_of(container.begin(), container.end(),
                     [](double val) { return !std::isfinite(val); });
}

class ControlBoardHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ControlBoardHardwareInterface)

  virtual ~ControlBoardHardwareInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

 private:
  void deactivate_motors();
  // Prepares the packet from the command interfaces. Returns nullptr, or the
  // rejection reason with the packet unchanged: a NaN/+-Inf command, or a
  // final value (after limits and zero offset) that is not a finite float.
  const char *copy_actuator_commands(bool use_position_limits = false);
  void copy_actuator_states();
  bool do_homing();
  // Command admission before any transmission (homed write(), every Home
  // step): NaN/+-Inf in any joint command, or a packet value the float wire
  // format cannot hold, faults to neutral instead of being prepared.
  bool admit_commands(const char *phase, bool use_position_limits);

  // Joint feedback from the latest exchange. Returns nullptr only when both
  // boards replied validly and every configured joint's position and velocity
  // is finite; otherwise the reason. Nothing is copied into joint state.
  const char *joint_feedback_error() const;
  // Raw board-frame position/velocity for joint i in the latest exchange.
  bool raw_joint_feedback(std::size_t i, float &position, float &velocity) const;
  void fault_joint_feedback(const BNO055::Output &sample, const char *phase, const char *reason);
  // One INFO line per Home on its first valid read; report only.
  void log_home_residual();

  struct ExchangeCounters {
    int mask = -1;
    uint64_t valid = 0, rejected = 0, streak = 0;
    std::chrono::steady_clock::time_point last_valid{};
  };
  // One board exchange plus transport-receipt accounting for diagnostics.
  void exchange();
  ExchangeCounters exchange_counters() const;
  // Neutral first, then revoke the receipt, then format one bounded diagnostic.
  void disable_for_fault(const BNO055::Output &sample, const char *phase, const char *kind,
                         const char *reason, const char *prefix = "",
                         const std::function<void(std::ostream &)> &extra = nullptr);
  void terminal_health(const BNO055::Output &sample);
  void publish_read_health(const BNO055::Output &sample);
  std::string board_snapshot(std::chrono::steady_clock::time_point now,
                             const ExchangeCounters &at_fault) const;

  int lock_fd_ = -1;  // File descriptor for lock file; -1 unless this instance owns the lock

  // FC_BOARD_HEALTH_V1 strict receipt (profile 1). Healthy only after this
  // activation's completed mechanical Home, with a validated IMU sample and
  // valid replies from both boards in the same read().
  pupper_imu::BoardHealth board_health_;
  // True only between a completed Home and the next fault/deactivation. While
  // false, every exchange carries the neutral packet.
  std::atomic<bool> homed_session_{false};
  bool home_residual_pending_ = false;
  uint64_t spi_valid_exchanges_ = 0, spi_rejected_exchanges_ = 0, spi_rejected_streak_ = 0;
  uint64_t unhomed_rejected_reads_ = 0;
  std::chrono::steady_clock::time_point last_valid_exchange_{}, last_unhomed_reject_log_{};

  IMUManager imu_manager_;

  // TODO: Refactor the horrible c-style spi code
  spi_command_t *spi_command_ = nullptr;
  spi_data_t *spi_data_ = nullptr;

  // IMU state
  std::array<double, 4> hw_state_imu_orientation_;          // x, y, z, w
  std::array<double, 3> hw_state_imu_angular_velocity_;     // x, y, z
  std::array<double, 3> hw_state_imu_linear_acceleration_;  // x, y, z
  double imu_roll_, imu_pitch_, imu_yaw_;

  // Actuator CAN config
  std::vector<int> hw_actuator_can_channels_;
  std::vector<int> hw_actuator_can_ids_;

  // Actuator homing
  std::vector<int> hw_actuator_homing_stages_;
  std::vector<double> hw_actuator_homing_velocities_;
  std::vector<double> hw_actuator_homing_kps_;
  std::vector<double> hw_actuator_homing_kds_;
  std::vector<double> hw_actuator_homed_positions_;
  std::vector<double> hw_actuator_post_homing_positions_;
  std::vector<double> hw_actuator_zero_positions_;
  std::vector<double> hw_actuator_homing_torque_thresholds_;
  std::vector<bool> hw_actuator_is_homed_;

  // Actuator limits
  std::vector<double> hw_actuator_position_mins_;
  std::vector<double> hw_actuator_position_maxs_;
  std::vector<double> hw_actuator_velocity_maxs_;
  std::vector<double> hw_actuator_effort_maxs_;
  std::vector<double> hw_actuator_kp_maxs_;
  std::vector<double> hw_actuator_kd_maxs_;

  // Actuator states
  std::vector<double> hw_state_positions_;
  std::vector<double> hw_state_velocities_;
  std::vector<double> hw_state_efforts_;

  // Actuator commands
  std::vector<double> hw_command_positions_;
  std::vector<double> hw_command_velocities_;
  std::vector<double> hw_command_efforts_;
  std::vector<double> hw_command_kps_;
  std::vector<double> hw_command_kds_;

  int64_t imu_packet_timestamp_micros_;
  int64_t imu_measurement_timestamp_micros_;
  int64_t imu_time_since_measurement_micros_;
  double imu_time_since_measurement_seconds_;
};

}  // namespace control_board_hardware_interface
