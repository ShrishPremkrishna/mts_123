#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "imu_manager.hpp"
#include "imu_health.hpp"
#include "board_health.hpp"
#include "redundant_imu.hpp"
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

template <typename T>
bool contains_nan(const T &container) {
  return std::any_of(container.begin(), container.end(),
                     [](double val) { return std::isnan(val); });
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

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> &start, const std::vector<std::string> &stop) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string> &start, const std::vector<std::string> &stop) override;

 private:
  // Bench-only. Once homed on its stand, the driver sends disabled packets for
  // the rest of the process. No post-fault reading can re-enable a motor.
  bool supported_standby_ = false;
  bool standby_homed_ = false;
  bool standby_feedback_lost_ = false;
  pupper_imu::StartupWindow standby_recovery_;
  bool field_rehome_ = false, redundant_field_ = false;
  pupper_imu::RedundantImu redundant_;
  pupper_imu::BoardHealth board_health_;
  double fc_balance_mode_=0, fc_balance_incident_=0, fc_balance_generation_=0;
  void terminal_health(const BNO055::Output &sample);
  bool supported_damping_ = false;
  std::chrono::steady_clock::time_point last_joint_exchange_{}, last_valid_q_{}, last_valid_g_{};
  std::chrono::steady_clock::time_point damping_deadline_{}, recovery_since_{}, recovery_q_{}, recovery_g_{};
  unsigned recovery_pairs_ = 0;
  std::vector<float> bench_kds_;
  std::vector<double> prior_field_positions_;
  std::atomic<bool> field_backup_active_{false};
  // Only the sensor observation owner writes the proof; claim callbacks read its expiry.
  std::atomic<uint64_t> backup_claim_until_ns_{0};
  std::atomic<uint64_t> joint_claim_until_ns_{0};
  bool unclaimed_joint_fault_reported_ = false;
  std::atomic<uint64_t> field_claim_generation_{0}, prepared_field_generation_{0};
  void publish_backup_claim_proof(std::chrono::steady_clock::time_point observed_at);
  bool can_claim_on_field_feedback(uint64_t generation) const;
  struct JointIssue {
    const char *reason = nullptr;
    size_t axis = static_cast<size_t>(-1);
    double value = 0, low = 0, high = 0;
  };
  JointIssue joint_issue(std::chrono::steady_clock::time_point now, double step_dt = -1) const;
  void log_joint_issue(const JointIssue &issue, const char *phase) const;
  void publish_joint_claim_proof(std::chrono::steady_clock::time_point now, const JointIssue &issue);
  bool bench_joint_feedback_ok(std::chrono::steady_clock::time_point now) const;
  hardware_interface::return_type read_bench(const BNO055::Output &sample);
  void report_imu_fault(const BNO055::Output &sample, std::chrono::steady_clock::time_point now,
                        const char *phase, const char *error);
  std::vector<std::string> recovery_interfaces_;
  enum class FieldState { running, fault, configured, homing, awaiting_claim, zero_first };
  std::atomic<FieldState> field_state_{FieldState::running};
  std::atomic<bool> fresh_claim_prepared_{false};
  bool complete_joint_claim(const std::vector<std::string> &start) const;
  bool touches_joints(const std::vector<std::string> &start) const;
  void latch_field_fault();
  void clear_joint_commands();
  void invalidate_balance();
  void deactivate_motors();
  bool check_motor_feedback(const BNO055::Output &sample, const char *phase);
  void disable_for_imu_fault(const BNO055::Output &sample, std::chrono::steady_clock::time_point now,
                             const char *phase, const char *error);
  void copy_actuator_commands(bool use_position_limits = false);
  void copy_actuator_states();
  bool do_homing();
  bool hw_states_contains_nan();

  int lock_fd_ = -1;  // File descriptor for lock file

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
