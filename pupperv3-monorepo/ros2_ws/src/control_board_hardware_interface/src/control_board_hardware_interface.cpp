#include "control_board_hardware_interface/control_board_hardware_interface.hpp"
#include "control_board_hardware_interface/imu_health.hpp"
#include "control_board_hardware_interface/imu_startup_trace.hpp"

#include <fcntl.h>
#include <sched.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace control_board_hardware_interface {

const char *RED_ANSI = "\033[1;31m";
const char *YELLOW_ANSI = "\033[1;33m";
const char *RESET_ANSI = "\033[0m";

namespace {
int64_t mono_us(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
}
int spi_reply_mask(const spi_data_t *data) {
  return data ? data->spi_driver_status & SPI_BOTH_REPLIES_VALID : 0;
}
}  // namespace

void ControlBoardHardwareInterface::exchange() {
  spi_driver_run();
  if (spi_reply_mask(spi_data_) == SPI_BOTH_REPLIES_VALID) {
    ++spi_valid_exchanges_;
    spi_rejected_streak_ = 0;
    last_valid_exchange_ = std::chrono::steady_clock::now();
  } else {
    ++spi_rejected_exchanges_;
    ++spi_rejected_streak_;
  }
}

ControlBoardHardwareInterface::ExchangeCounters ControlBoardHardwareInterface::exchange_counters()
    const {
  return {spi_data_ ? spi_reply_mask(spi_data_) : -1, spi_valid_exchanges_,
          spi_rejected_exchanges_, spi_rejected_streak_, last_valid_exchange_};
}

std::string ControlBoardHardwareInterface::board_snapshot(
    std::chrono::steady_clock::time_point now, const ExchangeCounters &at_fault) const {
  std::ostringstream log;
  log << " board={fault_spi_mask=" << at_fault.mask
      << " neutral_spi_mask=" << (spi_data_ ? spi_reply_mask(spi_data_) : -1)
      << " expected_spi_mask=" << SPI_BOTH_REPLIES_VALID << " valid_exchanges=" << at_fault.valid
      << " rejected_exchanges=" << at_fault.rejected << " rejected_streak=" << at_fault.streak
      << " last_valid_exchange_age_us=";
  if (at_fault.last_valid == std::chrono::steady_clock::time_point{}) {
    log << "none";
  } else {
    log << std::chrono::duration_cast<std::chrono::microseconds>(now - at_fault.last_valid).count();
  }
  log << " unhomed_rejected_reads=" << unhomed_rejected_reads_
      << " generation=" << board_health_.homes.load()
      << " faults=" << board_health_.faults.load() << " profile=" << board_health_.profile << "}";
  return log.str();
}

void ControlBoardHardwareInterface::terminal_health(const BNO055::Output &sample) {
  // Revoke healthy status before any later publisher can observe the Home.
  // Only a new activation and Home can set homed_session_ again.
  homed_session_ = false;
  home_residual_pending_ = false;
  if (board_health_.terminal.exchange(true)) return;
  ++board_health_.faults;
  board_health_.publish(4, sample, sample, spi_reply_mask(spi_data_), false);
}

void ControlBoardHardwareInterface::disable_for_fault(
    const BNO055::Output &sample, const char *phase, const char *kind, const char *reason,
    const char *prefix, const std::function<void(std::ostream &)> &extra) {
  // Capture plain values, send neutral, revoke the receipt, and only then format.
  const auto now = std::chrono::steady_clock::now();
  const auto at_fault = exchange_counters();
  deactivate_motors();
  terminal_health(sample);
  using namespace std::chrono;
  std::ostringstream log;
  // "IMU startup check failed: IMU fault phase=startup reason=... sampled_mono_us=" is
  // parsed by persistent_session; fc_operator shows the text before sampled_mono_us.
  log << prefix << kind << " fault phase=" << phase << " reason=" << reason
      << " sampled_mono_us=" << mono_us(now);
  const auto component = [&](const char *name, int sequence, bool valid,
                             steady_clock::time_point receipt, microseconds minimum_age) {
    log << " " << name << "={seq=" << sequence << " valid=" << valid;
    if (receipt == steady_clock::time_point{}) {
      log << " receipt_mono_us=none age_us=none";
    } else {
      log << " receipt_mono_us=" << mono_us(receipt)
          << " age_us=" << duration_cast<microseconds>(now - receipt).count();
    }
    log << " batch_min_age_us=" << minimum_age.count() << "}";
  };
  log << " imu_enabled=" << imu_manager_.enabled;
  component("quaternion", sample.quaternion_report_sequence, sample.quaternion_valid,
            sample.quaternion_receipt_time, sample.quaternion_minimum_age_at_receipt);
  component("gyro", sample.gyro_report_sequence, sample.gyro_valid, sample.gyro_receipt_time,
            sample.gyro_minimum_age_at_receipt);
  log << board_snapshot(now, at_fault);
  if (extra) extra(log);
  RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"), "%s", log.str().c_str());
}

void ControlBoardHardwareInterface::publish_read_health(const BNO055::Output &sample) {
  const int mask = spi_reply_mask(spi_data_);
  // Before a completed Home, or without an IMU, the snapshot stays non-healthy.
  // A homed read reaches here only with accepted joint feedback (mask 3).
  const bool healthy = homed_session_ && imu_manager_.enabled && mask == SPI_BOTH_REPLIES_VALID;
  board_health_.publish(healthy ? 1 : 0, sample, sample, mask, false);
}

bool ControlBoardHardwareInterface::raw_joint_feedback(std::size_t i, float &position,
                                                       float &velocity) const {
  const uint can_channel = hw_actuator_can_channels_[i] - 1;
  // ID 1: abad, ID 2: hip, ID 3: knee (not corresponding to the actual joint names, just used
  // to make the Cheetah code send to the CAN IDs we want)
  switch (hw_actuator_can_ids_[i]) {
    case 1:
      position = spi_data_->q_abad[can_channel];
      velocity = spi_data_->qd_abad[can_channel];
      return true;
    case 2:
      position = spi_data_->q_hip[can_channel];
      velocity = spi_data_->qd_hip[can_channel];
      return true;
    case 3:
      position = spi_data_->q_knee[can_channel];
      velocity = spi_data_->qd_knee[can_channel];
      return true;
  }
  return false;
}

const char *ControlBoardHardwareInterface::joint_feedback_error() const {
  if (spi_reply_mask(spi_data_) != SPI_BOTH_REPLIES_VALID) return "missing or corrupt motor board reply";
  for (std::size_t i = 0; i < hw_state_positions_.size(); ++i) {
    float position = 0, velocity = 0;
    if (raw_joint_feedback(i, position, velocity) &&
        (!std::isfinite(position) || !std::isfinite(velocity)))
      return "nonfinite motor feedback";
  }
  return nullptr;
}

void ControlBoardHardwareInterface::fault_joint_feedback(const BNO055::Output &sample,
                                                         const char *phase, const char *reason) {
  // Capture the rejected values before the neutral exchange replaces them.
  std::ostringstream rejected;
  for (std::size_t i = 0; i < hw_state_positions_.size(); ++i) {
    float position = 0, velocity = 0;
    if (raw_joint_feedback(i, position, velocity) &&
        (!std::isfinite(position) || !std::isfinite(velocity)))
      rejected << " " << info_.joints[i].name << "={board_rad=" << position
               << " board_rad_s=" << velocity << "}";
  }
  const std::string nonfinite = rejected.str();
  disable_for_fault(sample, phase, "Board", reason, "", [&](std::ostream &log) {
    // Joint state keeps the last accepted values; report them for comparison.
    log << " kept_state={positions=[" << hw_state_positions_ << "] velocities=["
        << hw_state_velocities_ << "]}";
    if (!nonfinite.empty()) log << " nonfinite={" << nonfinite.substr(1) << "}";
  });
}

bool ControlBoardHardwareInterface::admit_commands(const char *phase, bool use_position_limits) {
  const char *reason = copy_actuator_commands(use_position_limits);
  if (!reason) return true;
  // The neutral packet sent by disable_for_fault is all zero and is not
  // admitted through here, so the fault path cannot recurse.
  disable_for_fault(imu_manager_.get_imu_data(), phase, "Command", reason, "", [&](std::ostream &log) {
    log << " commands={";
    for (std::size_t i = 0; i < hw_command_positions_.size(); ++i)
      log << (i ? " " : "") << info_.joints[i].name << "={position=" << hw_command_positions_[i]
          << " velocity=" << hw_command_velocities_[i] << " effort=" << hw_command_efforts_[i]
          << " kp=" << hw_command_kps_[i] << " kd=" << hw_command_kds_[i]
          << " zero_offset=" << hw_actuator_zero_positions_[i] << "}";
    log << "}";
  });
  return false;
}

void ControlBoardHardwareInterface::log_home_residual() {
  // Per joint, in the joint frame after this Home's zero offset (the frame of
  // the position state/command interfaces and URDF limits), in radians:
  // commanded = target in the packet this read exchanged, rest = configured
  // post_homing_position, measured = accepted encoder position. Report only:
  // no threshold, and no claim that the pose or the robot is ready.
  std::ostringstream log;
  log << std::setprecision(6) << "FC_HOME_RESIDUAL_V1 generation=" << board_health_.homes.load()
      << " frame=joint_after_home_offset units=rad"
      << " sampled_mono_us=" << mono_us(std::chrono::steady_clock::now())
      << " joints=" << hw_state_positions_.size();
  for (std::size_t i = 0; i < hw_state_positions_.size(); ++i) {
    const uint can_channel = hw_actuator_can_channels_[i] - 1;
    float packet_position = 0, packet_kp = 0;
    switch (hw_actuator_can_ids_[i]) {
      case 1: packet_position = spi_command_->q_des_abad[can_channel]; packet_kp = spi_command_->kp_abad[can_channel]; break;
      case 2: packet_position = spi_command_->q_des_hip[can_channel]; packet_kp = spi_command_->kp_hip[can_channel]; break;
      case 3: packet_position = spi_command_->q_des_knee[can_channel]; packet_kp = spi_command_->kp_knee[can_channel]; break;
    }
    const double commanded = packet_position - hw_actuator_zero_positions_[i];
    const double rest = hw_actuator_post_homing_positions_[i];
    const double measured = hw_state_positions_[i];
    log << " " << info_.joints[i].name << "={commanded=" << commanded << " rest=" << rest
        << " measured=" << measured << " measured_minus_commanded=" << measured - commanded
        << " measured_minus_rest=" << measured - rest << " commanded_kp=" << packet_kp << "}";
  }
  log << " assessment=none";
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "%s", log.str().c_str());
}

ControlBoardHardwareInterface::~ControlBoardHardwareInterface() {
  // An instance that never acquired the lock owns no boards, sensor or lock:
  // it must not send a packet or close/unlink another owner's resources.
  if (lock_fd_ == -1) return;
  // Hold ownership through the last device I/O: neutral packet and terminal
  // receipt, then stop the IMU thread and close I2C/SPI, then release the lock.
  // The lock inode is never unlinked, so a concurrent owner cannot be bypassed.
  on_deactivate(rclcpp_lifecycle::State());
  imu_manager_.stop();
  if (spi_command_ != nullptr) close_spi();
  close(lock_fd_);
  lock_fd_ = -1;
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  lock_fd_ = open("/tmp/control_board_hardware.lock", O_CREAT | O_CLOEXEC, 0666);
  if (lock_fd_ == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"), "Failed to open lock file");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (flock(lock_fd_, LOCK_EX | LOCK_NB) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "Another instance is already running");
    close(lock_fd_);
    lock_fd_ = -1;  // The destructor must not act for the other owner.
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Strict profile only. The receipt is optional; without a path nothing is published.
  board_health_.profile = 1;
  if (const char *path = std::getenv("PUPPER_FC_BOARD_HEALTH_PATH")) {
    if (!board_health_.open(path)) {
      RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                   "FC_BOARD_HEALTH_V1 could not initialize selected receipt");
      return hardware_interface::CallbackReturn::ERROR;
    }
    board_health_.publish(0, BNO055::Output{}, BNO055::Output{}, 0, false);
    RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "FC_BOARD_HEALTH_V1 strict receipt profile=1 path=%s", path);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "FC_BOARD_HEALTH_V1 receipt not requested (PUPPER_FC_BOARD_HEALTH_PATH unset)");
  }

  hw_state_positions_.resize(info_.joints.size(), 0.0);
  hw_state_velocities_.resize(info_.joints.size(), 0.0);
  hw_state_efforts_.resize(info_.joints.size(), 0.0);

  hw_command_positions_.resize(info_.joints.size(), 0.0);
  hw_command_velocities_.resize(info_.joints.size(), 0.0);
  hw_command_efforts_.resize(info_.joints.size(), 0.0);
  hw_command_kps_.resize(info_.joints.size(), 0.0);
  hw_command_kds_.resize(info_.joints.size(), 0.0);

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // Set params for each joint
    hw_actuator_can_channels_.push_back(std::stoi(joint.parameters.at("can_channel")));
    hw_actuator_can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));

    // Set limits for each joint
    hw_actuator_position_mins_.push_back(std::stod(joint.parameters.at("position_min")));
    hw_actuator_position_maxs_.push_back(std::stod(joint.parameters.at("position_max")));
    hw_actuator_velocity_maxs_.push_back(std::stod(joint.parameters.at("velocity_max")));
    hw_actuator_effort_maxs_.push_back(std::stod(joint.parameters.at("effort_max")));
    hw_actuator_kp_maxs_.push_back(std::stod(joint.parameters.at("kp_max")));
    hw_actuator_kd_maxs_.push_back(std::stod(joint.parameters.at("kd_max")));

    // Homing parameters
    hw_actuator_homing_stages_.push_back(std::stoi(joint.parameters.at("homing_stage")));
    hw_actuator_homing_velocities_.push_back(std::stod(joint.parameters.at("homing_velocity")));
    hw_actuator_homing_kps_.push_back(std::stod(joint.parameters.at("homing_kp")));
    hw_actuator_homing_kds_.push_back(std::stod(joint.parameters.at("homing_kd")));
    hw_actuator_homed_positions_.push_back(std::stod(joint.parameters.at("homed_position")));
    hw_actuator_post_homing_positions_.push_back(
        std::stod(joint.parameters.at("post_homing_position")));
    hw_actuator_zero_positions_.push_back(0.0);
    hw_actuator_homing_torque_thresholds_.push_back(
        std::stod(joint.parameters.at("homing_torque_threshold")));
    hw_actuator_is_homed_.push_back(false);
  }

  if (info_.hardware_parameters.count("use_imu") &&
      (info_.hardware_parameters.at("use_imu") == "false" ||
       info_.hardware_parameters.at("use_imu") == "False")) {
    RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"), "%sIMU not enabled%s",
                YELLOW_ANSI, RESET_ANSI);
    imu_manager_.enabled = false;
  } else {
    imu_manager_.enabled = true;

    // Set up IMU parameters
    double imu_roll = std::stod(info_.sensors[0].parameters.at("roll"));
    double imu_pitch = std::stod(info_.sensors[0].parameters.at("pitch"));
    double imu_yaw = std::stod(info_.sensors[0].parameters.at("yaw"));
    imu_manager_.set_imu_offset(imu_roll, imu_pitch, imu_yaw);

    // Set up the IMU
    // TODO: make micros_between_reports a parameter
    imu_manager_.begin(/*micros_between_reports=*/10000);
  }

  // Set up SPI
  init_spi();
  spi_command_ = get_spi_command();
  spi_data_ = get_spi_data();

  // Do not return ready while the background reader still contains default or
  // stale data. This sends no enable command; failed initialization disables.
  if (imu_manager_.enabled) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (true) {
      // Snapshot first, then read the clock, so a concurrently received sample
      // cannot appear to come from the future.
      const auto sample = imu_manager_.get_imu_data();
      const auto now = std::chrono::steady_clock::now();
      const char *error = pupper_imu::feedback_error(sample, now);
      if (!error) break;
      if (now >= deadline) {
        disable_for_fault(sample, "initialization", "IMU", error,
                          "No fresh quaternion and gyro reports during IMU initialization: ");
        return hardware_interface::CallbackReturn::ERROR;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ControlBoardHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Add joint state interfaces
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_state_efforts_[i]));
  }

  // Add IMU state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.x",
                                                                   &hw_state_imu_orientation_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.y",
                                                                   &hw_state_imu_orientation_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.z",
                                                                   &hw_state_imu_orientation_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.w",
                                                                   &hw_state_imu_orientation_[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.x", &hw_state_imu_angular_velocity_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.y", &hw_state_imu_angular_velocity_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.z", &hw_state_imu_angular_velocity_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.x", &hw_state_imu_linear_acceleration_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.y", &hw_state_imu_linear_acceleration_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.z", &hw_state_imu_linear_acceleration_[2]));

  // TODO: Add once int64 type is supported
  //   state_interfaces.emplace_back(
  //       hardware_interface::StateInterface("imu_sensor", "imu_packet_timestamp",
  //       &imu_packet_timestamp_micros_));
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //       "imu_sensor", "imu_measurement_timestamp", &imu_measurement_timestamp_micros_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "time_since_measurement_seconds", &imu_time_since_measurement_seconds_));

  return state_interfaces;
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_state_positions_.size(); i++) {
    hw_state_positions_[i] = 0.0;
    hw_state_velocities_[i] = 0.0;
    hw_state_efforts_[i] = 0.0;
    hw_command_positions_[i] = 0.0;
    hw_command_velocities_[i] = 0.0;
    hw_command_efforts_[i] = 0.0;
    hw_command_kps_[i] = 0.0;
    hw_command_kds_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
ControlBoardHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_command_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_command_efforts_[i]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "kp", &hw_command_kps_[i]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "kd", &hw_command_kds_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // A new activation requires a new mechanical Home. Revoke the previous Home
  // before publishing, and never reuse homed flags, offsets, efforts or a
  // prepared packet from an earlier activation of this instance.
  homed_session_ = false;
  home_residual_pending_ = false;
  board_health_.terminal = false;
  board_health_.publish(5, BNO055::Output{}, BNO055::Output{}, 0, false);
  std::fill(hw_actuator_is_homed_.begin(), hw_actuator_is_homed_.end(), false);
  std::fill(hw_actuator_zero_positions_.begin(), hw_actuator_zero_positions_.end(), 0.0);
  for (auto *values : {&hw_command_positions_, &hw_command_velocities_, &hw_command_efforts_,
                       &hw_command_kps_, &hw_command_kds_}) {
    std::fill(values->begin(), values->end(), 0.0);
  }
  *spi_command_ = spi_command_t{};

  if (imu_manager_.enabled) {
    pupper_imu::StartupWindow stationary;
    auto trace = pupper_imu::StartupTrace::from_environment();
    const auto finish_trace = [&](bool passed) {
      if (trace && !trace->finish(passed))
        RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                    "IMU startup diagnostic trace could not be written (gate outcome unchanged)");
    };
    const auto deadline = std::chrono::steady_clock::now() + pupper_imu::startup_timeout;
    RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "Keep body supported and stationary: checking fresh IMU reports before motor enable");
    // Bounded counts of control polls (not unique reports) for the failure line.
    unsigned polls = 0, feedback = 0, gyro_peak = 0, gravity = 0, gyro_mean = 0, gaps = 0;
    double max_stationary_s = 0, max_gyro = 0;
    while (true) {
      const auto sample = imu_manager_.get_imu_data();
      const auto observed_at = std::chrono::steady_clock::now();
      pupper_imu::StartupObservation observation;
      const bool ready = stationary.observe(sample, observed_at, &observation);
      if (trace) trace->record(sample, observed_at, observation, ready);
      ++polls;
      max_stationary_s = std::max(max_stationary_s, observation.eligible_seconds);
      max_gyro = std::max(max_gyro, static_cast<double>(sample.gyro.norm()));
      gaps += observation.observation_gap;
      const std::string_view why(observation.reason);
      feedback += why == "feedback";
      gyro_peak += why == "gyro_norm";
      gravity += why == "gravity";
      gyro_mean += why == "gyro_mean";
      if (ready) { finish_trace(true); break; }
      if (observed_at >= deadline) {
        const char *error = pupper_imu::feedback_error(sample, observed_at);
        disable_for_fault(sample, "startup", "IMU", error ? error : "stationary startup window timed out",
                          "IMU startup check failed: ", [&](std::ostream &log) {
          log << " startup={polls=" << polls << " max_stationary_s=" << max_stationary_s
              << " max_gyro_rad_s=" << max_gyro << " rejections={feedback:" << feedback
              << ",gyro_peak:" << gyro_peak << ",gravity:" << gravity << ",gyro_mean:" << gyro_mean
              << ",observation_gap:" << gaps << "} requires={window_s:"
              << std::chrono::duration<double>(pupper_imu::stationary_window).count()
              << ",gyro_peak_rad_s:" << pupper_imu::stationary_gyro_limit_rad_s
              << ",gyro_mean_rad_s:" << pupper_imu::stationary_gyro_mean_limit_rad_s
              << ",gravity_change_deg:1}}";
        });
        finish_trace(false);
        return hardware_interface::CallbackReturn::ERROR;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // The boards can return an empty first reply while waking. Probe with the
  // neutral packet (every enable bit clear) and never send an enable command or
  // start Home on a transport without complete valid replies from both boards.
  const auto board_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while (true) {
    *spi_command_ = spi_command_t{};
    exchange();
    if (spi_reply_mask(spi_data_) == SPI_BOTH_REPLIES_VALID) break;
    if (std::chrono::steady_clock::now() >= board_deadline) {
      const auto now = std::chrono::steady_clock::now();
      const auto at_fault = exchange_counters();
      deactivate_motors();
      terminal_health(imu_manager_.get_imu_data());
      RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                  "Board fault phase=probe reason=spi_reply_mask sampled_mono_us=%lld%s",
                  static_cast<long long>(mono_us(now)), board_snapshot(now, at_fault).c_str());
      // Exact line parsed by persistent_session.pre_motor_fault.
      RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                   "Home refused before motor enable: missing motor board reply (mask=%d)", at_fault.mask);
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // Enable actuators. Send the command multiple times to ensure it is received.
  for (int i = 0; i < 10; i++) {
    if (imu_manager_.enabled) {
      const auto sample = imu_manager_.get_imu_data();
      if (const char *error = pupper_imu::feedback_error(sample, std::chrono::steady_clock::now())) {
        disable_for_fault(sample, "enable", "IMU", error);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    spi_command_->flags[0] = 1;
    spi_command_->flags[1] = 1;
    spi_command_->flags[2] = 1;
    spi_command_->flags[3] = 1;
    exchange();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Homing. The last enable exchange is validated there before it is copied.
  if (!do_homing()) return hardware_interface::CallbackReturn::ERROR;

  // A completed mechanical Home is the only source of a new generation. Set it
  // before allowing read() to publish this session's first healthy snapshot.
  // A configuration without joints ran no mechanical search and gets none.
  if (hw_actuator_is_homed_.empty() ||
      !std::all_of(hw_actuator_is_homed_.begin(), hw_actuator_is_homed_.end(),
                   [](bool homed) { return homed; })) {
    RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "FC_BOARD_HEALTH_V1 no mechanical Home generation: %zu joints referenced",
                hw_actuator_is_homed_.size());
    RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  const auto home_us = pupper_imu::BoardHealth::ns(std::chrono::steady_clock::now()) / 1000;
  board_health_.homes = std::max(board_health_.homes.load() + 1, home_us);
  home_residual_pending_ = true;
  homed_session_ = true;
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
              "FC_BOARD_HEALTH_V1 Home generation=%llu faults=%llu imu_enabled=%d",
              static_cast<unsigned long long>(board_health_.homes.load()),
              static_cast<unsigned long long>(board_health_.faults.load()),
              static_cast<int>(imu_manager_.enabled));
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void ControlBoardHardwareInterface::deactivate_motors() {
  // Send a neutral disable packet; flags alone leave the previous targets,
  // gains and feed-forward torques in the command sent to both boards.
  if (spi_command_ == nullptr) return;  // Initialization may have failed before SPI setup.
  *spi_command_ = spi_command_t{};
  exchange();
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_error(
    [[maybe_unused]] const rclcpp_lifecycle::State &previous_state) {
  deactivate_motors();
  // The causing callback already logged its fault; add no later ERROR line.
  terminal_health(imu_manager_.get_imu_data());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  deactivate_motors();
  terminal_health(imu_manager_.get_imu_data());
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ControlBoardHardwareInterface::read(
    [[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period) {
  // Validate BEFORE transmitting previously prepared actuator commands. Each
  // sensor component has its own monotonic receipt; one cannot freshen another.
  const BNO055::Output imu_output = imu_manager_.get_imu_data();
  if (imu_manager_.enabled) {
    if (const char *error = pupper_imu::feedback_error(imu_output, std::chrono::steady_clock::now())) {
      disable_for_fault(imu_output, "read", "IMU", error);
      return hardware_interface::return_type::ERROR;
    }
  }
  // Without this activation's completed Home, send only the neutral packet,
  // whatever write() or an earlier session left in the command buffer.
  if (!homed_session_) *spi_command_ = spi_command_t{};
  // Write to and read from the actuators
  exchange();
  // Joint feedback is checked before it is copied: an invalid reply or a
  // nonfinite value never replaces the last accepted joint state. While homed,
  // the first such exchange (including the first read after Home) faults to
  // neutral and requires a new activation; while not homed it is only counted.
  if (const char *reason = joint_feedback_error()) {
    if (homed_session_) {
      fault_joint_feedback(imu_output, home_residual_pending_ ? "post_home" : "read", reason);
      return hardware_interface::return_type::ERROR;
    }
    ++unhomed_rejected_reads_;
    const auto now = std::chrono::steady_clock::now();
    if (now - last_unhomed_reject_log_ >= std::chrono::seconds(1)) {
      last_unhomed_reject_log_ = now;
      RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                  "Board feedback rejected while not homed (state kept) reason=%s%s", reason,
                  board_snapshot(now, exchange_counters()).c_str());
    }
  } else {
    copy_actuator_states();
    // Not homed, this exchange carried the neutral packet (no targets or
    // gains): no PD effort, whatever values remain in the command interfaces.
    if (!homed_session_) std::fill(hw_state_efforts_.begin(), hw_state_efforts_.end(), 0.0);
  }

  // Print joint position
  // RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Joint 0: %f",
  // spi_data_->q_abad[1]);

  tf2::Quaternion corrected_quat = tf2::Quaternion::getIdentity();
  tf2::Vector3 angular_velocity(0, 0, 0);
  tf2::Vector3 linear_acceleration(0, 0, 0);

  if (imu_manager_.enabled) {
    // TODO: put this functionality into the imu_manager
    // imu_output was validated before the actuator exchange above.
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();

    imu_packet_timestamp_micros_ = std::chrono::duration_cast<std::chrono::microseconds>(
                                       imu_output.packet_timestamp.time_since_epoch())
                                       .count();
    imu_measurement_timestamp_micros_ = std::chrono::duration_cast<std::chrono::microseconds>(
                                            imu_output.measurement_timestamp.time_since_epoch())
                                            .count();
    imu_time_since_measurement_micros_ = std::chrono::duration_cast<std::chrono::microseconds>(
                                             now - imu_output.measurement_timestamp)
                                             .count();
    imu_time_since_measurement_seconds_ = imu_time_since_measurement_micros_ / 1e6;

    // Print imu timestamps and age
    // RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
    //             "Packet: %ld us\tmesurement: %ldus\tage as of read(): %ldus = %fs",
    //             imu_packet_timestamp_micros_, imu_measurement_timestamp_micros_,
    //             imu_time_since_measurement_micros_, imu_time_since_measurement_seconds_);

    // Represent IMU orientation as quaternion
    tf2::Quaternion imu_quat(imu_output.quat.x(), imu_output.quat.y(), imu_output.quat.z(),
                             imu_output.quat.w());

    // Applying the offset to the IMU quaternion
    corrected_quat = imu_quat * imu_manager_.offset_quaternion.inverse();
    corrected_quat.normalize();

    // Rotating the angular velocity
    tf2::Vector3 imu_ang_vel(imu_output.gyro.x(), imu_output.gyro.y(), imu_output.gyro.z());
    angular_velocity = imu_manager_.offset_rotation_matrix * imu_ang_vel;

    // Rotating the linear acceleration
    tf2::Vector3 imu_acc(imu_output.acc.x(), imu_output.acc.y(), imu_output.acc.z());
    linear_acceleration = imu_manager_.offset_rotation_matrix * imu_acc;
  }

  const std::array<double, 4> orientation{corrected_quat.x(), corrected_quat.y(),
                                          corrected_quat.z(), corrected_quat.w()};
  const std::array<double, 3> gyro{angular_velocity.x(), angular_velocity.y(),
                                   angular_velocity.z()};
  const std::array<double, 3> acceleration{linear_acceleration.x(), linear_acceleration.y(),
                                           linear_acceleration.z()};

  // Any NaN or Inf (IMU, or efforts estimated from commands) faults before the
  // IMU values are published to the state interfaces.
  if (contains_nonfinite(orientation) || contains_nonfinite(gyro) ||
      contains_nonfinite(acceleration) || contains_nonfinite(hw_state_positions_) ||
      contains_nonfinite(hw_state_velocities_) || contains_nonfinite(hw_state_efforts_)) {
    disable_for_fault(imu_output, "read", "Board", "HW state array contained nonfinite value", "",
                      [&](std::ostream &log) {
      log << " state={orientation=[" << orientation << "] angular_velocity=[" << gyro
          << "] linear_acceleration=[" << acceleration << "] positions=[" << hw_state_positions_
          << "] velocities=[" << hw_state_velocities_ << "] efforts=[" << hw_state_efforts_ << "]}";
    });
    return hardware_interface::return_type::ERROR;
  }

  // Updating the state interfaces with corrected values
  std::copy(orientation.begin(), orientation.end(), hw_state_imu_orientation_.begin());
  std::copy(gyro.begin(), gyro.end(), hw_state_imu_angular_velocity_.begin());
  std::copy(acceleration.begin(), acceleration.end(), hw_state_imu_linear_acceleration_.begin());

  // Reaching here while homed means this read's joint feedback was accepted.
  if (homed_session_ && home_residual_pending_) {
    home_residual_pending_ = false;
    log_home_residual();
  }
  publish_read_health(imu_output);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ControlBoardHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // Not homed (inactive, faulted or deactivated): no prepared actuator payload.
  if (!homed_session_) {
    *spi_command_ = spi_command_t{};
    return hardware_interface::return_type::OK;
  }
  if (!admit_commands("write", true)) return hardware_interface::return_type::ERROR;
  return hardware_interface::return_type::OK;
}

bool ControlBoardHardwareInterface::do_homing() {
  // The IMU is checked before every transmission. Each exchange's joint
  // feedback is checked before it is copied into joint state: a missing or
  // corrupt reply, or a nonfinite value, cannot locate a mechanical stop, so
  // the Home stops with neutral rather than completing.
  const auto imu_ok = [this]() {
    if (!imu_manager_.enabled) return true;
    const auto sample = imu_manager_.get_imu_data();
    const char *error = pupper_imu::feedback_error(sample, std::chrono::steady_clock::now());
    if (!error) return true;
    disable_for_fault(sample, "homing", "IMU", error);
    return false;
  };
  const auto accept_feedback = [this]() {
    if (const char *reason = joint_feedback_error()) {
      fault_joint_feedback(imu_manager_.get_imu_data(), "homing", reason);
      return false;
    }
    copy_actuator_states();
    return true;
  };
  const auto step = [&]() {
    if (!admit_commands("homing", false)) return false;
    if (!imu_ok()) return false;
    exchange();
    return accept_feedback();
  };
  // The last enable exchange from on_activate.
  if (!accept_feedback() || !imu_ok()) return false;
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Homing actuators...");

  int dt_ms = 10;

  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  if (!step()) return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));

  // Set the initial commands
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    hw_command_positions_[i] = hw_state_positions_[i];
    hw_command_velocities_[i] = 0.0;
    hw_command_kps_[i] = 0.0;
    hw_command_kds_[i] = 0.0;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  if (!step()) return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));

  // Keep track of filtered torques
  std::vector<double> filtered_torques;
  filtered_torques.resize(hw_state_positions_.size(), 0.0);
  double alpha = 0.5;

  // Loop until all actuators are homed
  bool all_homed = false;
  bool all_returned = false;
  while (!all_homed) {
    // Set the homing stage to that of the lowest-stage unhomed actuator
    int homing_stage = 999;
    for (auto i = 0u; i < hw_actuator_homing_stages_.size(); i++) {
      if (!hw_actuator_is_homed_[i] && (hw_actuator_homing_stages_[i] < homing_stage)) {
        homing_stage = hw_actuator_homing_stages_[i];
      }
    }
    // Check if all actuators are homed
    all_homed = true;
    for (auto i = 0u; i < hw_state_positions_.size(); i++) {
      if (homing_stage < hw_actuator_homing_stages_[i]) {
        hw_command_positions_[i] = hw_state_positions_[i];
        hw_command_velocities_[i] = 0.0;
        hw_command_kps_[i] = 0.0;
        hw_command_kds_[i] = 0.0;
        all_homed = false;
        continue;
      } else {
        hw_command_kps_[i] = hw_actuator_homing_kps_[i];
        hw_command_kds_[i] = hw_actuator_homing_kds_[i];
      }
      // std::cout << "Commanded torque: " << filtered_torques[i] << std::endl;
      // std::cout << "Current position: " << hw_state_positions_[i] << std::endl;
      // std::cout << "Commanded position: " << hw_command_positions_[i] << std::endl;
      if (!hw_actuator_is_homed_[i]) {
        filtered_torques[i] =
            (1.0 - alpha) * filtered_torques[i] +
            alpha * ((hw_command_positions_[i] - hw_state_positions_[i]) * hw_command_kps_[i] +
                     (hw_command_velocities_[i] - hw_state_velocities_[i]) * hw_command_kds_[i]);
        if (std::abs(filtered_torques[i]) >= hw_actuator_homing_torque_thresholds_[i]) {
          hw_actuator_zero_positions_[i] = hw_state_positions_[i] - hw_actuator_homed_positions_[i];
          hw_command_positions_[i] = hw_actuator_homed_positions_[i];
          hw_command_velocities_[i] = 0.0;
          hw_actuator_is_homed_[i] = true;
          RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Homed actuator %d", i);
        } else {
          hw_command_positions_[i] += hw_actuator_homing_velocities_[i] * dt_ms * 0.001;
          hw_command_velocities_[i] = hw_actuator_homing_velocities_[i];
          all_homed = false;
        }
      }
    }
    if (!step()) return false;

    // Sleep for dt
    std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  }
  while (!all_returned) {
    all_returned = true;
    for (auto i = 0u; i < hw_state_positions_.size(); i++) {
      if (hw_command_positions_[i] < hw_actuator_post_homing_positions_[i] - 0.01) {
        hw_command_positions_[i] += std::abs(hw_actuator_homing_velocities_[i]) * dt_ms * 0.001;
        hw_command_velocities_[i] = std::abs(hw_actuator_homing_velocities_[i]);
        all_returned = false;
      } else if (hw_command_positions_[i] > hw_actuator_post_homing_positions_[i] + 0.01) {
        hw_command_positions_[i] -= std::abs(hw_actuator_homing_velocities_[i]) * dt_ms * 0.001;
        hw_command_velocities_[i] = -std::abs(hw_actuator_homing_velocities_[i]);
        all_returned = false;
      } else {
        hw_command_positions_[i] = hw_actuator_post_homing_positions_[i];
        hw_command_velocities_[i] = 0.0;
      }
    }
    if (!step()) return false;

    // Sleep for dt
    std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  }
  // Every homing exchange, including the final return exchange, was accepted.
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Finished homing!");
  return true;
}

const char *ControlBoardHardwareInterface::copy_actuator_commands(bool use_position_limits) {
  // Staged: the packet changes only if every joint is admitted. Each command
  // is read once; NaN and +/-Inf are invalid whatever a clamp would turn them
  // into, so they are rejected before any clamp. The final values must be
  // finite floats; that bound is the wire type's own range, not a tuned magnitude.
  const auto encodable = [](double value) {
    return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
  };
  spi_command_t next = *spi_command_;
  // Iterate through the joints
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    const std::array<double, 5> raw{hw_command_positions_[i], hw_command_velocities_[i],
                                    hw_command_efforts_[i], hw_command_kps_[i], hw_command_kds_[i]};
    if (contains_nonfinite(raw)) return "nonfinite actuator command";
    double cmd_pos = raw[0];
    double cmd_vel = std::clamp(raw[1], -hw_actuator_velocity_maxs_[i], hw_actuator_velocity_maxs_[i]);
    double cmd_eff = std::clamp(raw[2], -hw_actuator_effort_maxs_[i], hw_actuator_effort_maxs_[i]);
    double cmd_kp = std::clamp(raw[3], 0.0, hw_actuator_kp_maxs_[i]);
    double cmd_kd = std::clamp(raw[4], 0.0, hw_actuator_kd_maxs_[i]);

    if (use_position_limits && cmd_kp > 0.0) {
      if (cmd_pos < hw_actuator_position_mins_[i]) {
        cmd_pos = hw_actuator_position_mins_[i];
        cmd_vel = std::clamp(cmd_vel, 0.0, hw_actuator_velocity_maxs_[i]);
        cmd_eff = std::clamp(cmd_eff, 0.0, hw_actuator_effort_maxs_[i]);
      } else if (cmd_pos > hw_actuator_position_maxs_[i]) {
        cmd_pos = hw_actuator_position_maxs_[i];
        cmd_vel = std::clamp(cmd_vel, -hw_actuator_velocity_maxs_[i], 0.0);
        cmd_eff = std::clamp(cmd_eff, -hw_actuator_effort_maxs_[i], 0.0);
      }
      cmd_pos = std::clamp(cmd_pos, hw_actuator_position_mins_[i], hw_actuator_position_maxs_[i]);
    }

    cmd_pos += hw_actuator_zero_positions_[i];
    if (!encodable(cmd_pos) || !encodable(cmd_vel) || !encodable(cmd_eff) || !encodable(cmd_kp) ||
        !encodable(cmd_kd))
      return "nonfinite encoded actuator command";

    uint can_channel = hw_actuator_can_channels_[i] - 1;
    // ID 1: abad, ID 2: hip, ID 3: knee (not corresponding to the actual joint names, just used
    // to make the Cheetah code send to the CAN IDs we want)
    switch (hw_actuator_can_ids_[i]) {
      case 1:
        next.q_des_abad[can_channel] = cmd_pos;
        next.qd_des_abad[can_channel] = cmd_vel;
        next.kp_abad[can_channel] = cmd_kp;
        next.kd_abad[can_channel] = cmd_kd;
        next.tau_abad_ff[can_channel] = cmd_eff;
        break;
      case 2:
        next.q_des_hip[can_channel] = cmd_pos;
        next.qd_des_hip[can_channel] = cmd_vel;
        next.kp_hip[can_channel] = cmd_kp;
        next.kd_hip[can_channel] = cmd_kd;
        next.tau_hip_ff[can_channel] = cmd_eff;
        break;
      case 3:
        next.q_des_knee[can_channel] = cmd_pos;
        next.qd_des_knee[can_channel] = cmd_vel;
        next.kp_knee[can_channel] = cmd_kp;
        next.kd_knee[can_channel] = cmd_kd;
        next.tau_knee_ff[can_channel] = cmd_eff;
        break;
    }
  }
  *spi_command_ = next;
  return nullptr;
}

void ControlBoardHardwareInterface::copy_actuator_states() {
  // Iterate through the joints
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    float state_pos = hw_state_positions_[i];
    float state_vel = hw_state_velocities_[i];
    raw_joint_feedback(i, state_pos, state_vel);
    hw_state_positions_[i] = state_pos - hw_actuator_zero_positions_[i];
    hw_state_velocities_[i] = state_vel;

    // Estimate actuator efforts based on motor driver PD control
    hw_state_efforts_[i] =
        (hw_command_positions_[i] - hw_state_positions_[i]) * hw_command_kps_[i] +
        (hw_command_velocities_[i] - hw_state_velocities_[i]) * hw_command_kds_[i];
  }
}

}  // namespace control_board_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(control_board_hardware_interface::ControlBoardHardwareInterface,
                       hardware_interface::SystemInterface)
