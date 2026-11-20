#include "control_board_hardware_interface/control_board_hardware_interface.hpp"
#include "control_board_hardware_interface/imu_health.hpp"
#include "control_board_hardware_interface/imu_startup_trace.hpp"

#include <fcntl.h>
#include <sched.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string_view>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace control_board_hardware_interface {

const char *RED_ANSI = "\033[1;31m";
const char *YELLOW_ANSI = "\033[1;33m";
const char *RESET_ANSI = "\033[0m";

void ControlBoardHardwareInterface::publish_backup_claim_proof(std::chrono::steady_clock::time_point observed_at) {
  const auto now=pupper_imu::BoardHealth::ns(observed_at);
  const auto next=pupper_imu::BoardHealth::ns(redundant_.claim_deadline());
  // A newly healthy read cannot revive a prepared claim whose observed proof expired
  // between reads, even when no callback ran to explicitly clear that preparation.
  if (next>now && backup_claim_until_ns_.load()<=now) ++field_claim_generation_;
  backup_claim_until_ns_=next;
}

bool ControlBoardHardwareInterface::can_claim_on_field_feedback(uint64_t generation) const {
  const auto now = pupper_imu::BoardHealth::ns(std::chrono::steady_clock::now());
  return !redundant_field_ || (generation && generation==field_claim_generation_.load() &&
      !board_health_.terminal && !field_backup_active_ &&
      backup_claim_until_ns_.load() > now && joint_claim_until_ns_.load() > now);
}

bool ControlBoardHardwareInterface::check_motor_feedback(const BNO055::Output &sample, const char *phase) {
  // Sample first, then read the clock. Validate and diagnose the same snapshot.
  const auto now = std::chrono::steady_clock::now();
  const char *error = pupper_imu::feedback_error(sample, now);
  if (redundant_field_) {
    BNO055::Output primary;
    error = redundant_.select(sample, now, false, primary);
    publish_backup_claim_proof(now);
  }
  if (error) {
    disable_for_imu_fault(sample, now, phase, error);
    return false;
  }
  return true;
}

void ControlBoardHardwareInterface::terminal_health(const BNO055::Output &sample) {
  backup_claim_until_ns_=0;
  joint_claim_until_ns_=0;
  if (board_health_.terminal.exchange(true)) return;
  ++board_health_.faults;
  fc_balance_mode_=4;
  board_health_.publish(4,sample,sample,spi_data_?spi_data_->spi_driver_status&3:0,false);
}

void ControlBoardHardwareInterface::disable_for_imu_fault(const BNO055::Output &sample,
    std::chrono::steady_clock::time_point now, const char *phase, const char *error) {
  deactivate_motors();  // Send neutral before formatting or writing a diagnostic.
  latch_field_fault();
  invalidate_balance();
  terminal_health(sample);
  report_imu_fault(sample, now, phase, error);
}

void ControlBoardHardwareInterface::report_imu_fault(const BNO055::Output &sample,
    std::chrono::steady_clock::time_point now, const char *phase, const char *error) {
  using namespace std::chrono;
  std::ostringstream log;
  // Preserve the startup failure marker consumed by persistent_session.
  if (std::string_view(phase) == "startup") log << "IMU startup check failed: ";
  log << "IMU fault phase=" << phase << " reason=" << error << " sampled_mono_us="
      << duration_cast<microseconds>(now.time_since_epoch()).count();
  const auto component = [&](const char *name, int sequence, bool valid,
                             steady_clock::time_point receipt, microseconds minimum_age) {
    log << " " << name << "={seq=" << sequence << " valid=" << valid;
    if (receipt == steady_clock::time_point{}) {
      log << " receipt_mono_us=none age_us=none";
    } else {
      log << " receipt_mono_us=" << duration_cast<microseconds>(receipt.time_since_epoch()).count()
          << " age_us=" << duration_cast<microseconds>(now - receipt).count();
    }
    log << " batch_min_age_us=" << minimum_age.count() << "}";
  };
  component("quaternion", sample.quaternion_report_sequence, sample.quaternion_valid,
            sample.quaternion_receipt_time, sample.quaternion_minimum_age_at_receipt);
  component("gyro", sample.gyro_report_sequence, sample.gyro_valid,
            sample.gyro_receipt_time, sample.gyro_minimum_age_at_receipt);
  RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"), "%s", log.str().c_str());
}

ControlBoardHardwareInterface::~ControlBoardHardwareInterface() {
  if (lock_fd_ != -1) {
    // Keep one stable lock inode and hold ownership through the last device I/O.
    on_deactivate(rclcpp_lifecycle::State());
    imu_manager_.stop();
    if (spi_command_) close_spi();
    close(lock_fd_);
  }
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
    lock_fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto profile = info_.hardware_parameters.find("fc_imu_recovery");
  if (profile != info_.hardware_parameters.end()) {
    supported_standby_ = profile->second == "supported_standby";
    supported_damping_ = profile->second == "supported_damping";
    redundant_field_ = profile->second == "field_redundant_imu";
    imu_manager_.redundant_reports = redundant_field_;
    field_rehome_ = profile->second == "supported_rehome" || supported_damping_ || redundant_field_;
    if (redundant_field_) RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
        "FC_FIELD_REDUNDANT_IMU_V1 FC_JOINT_CLAIM_V1: qualified joint exchange and independent SH-2 reports");
    if (redundant_field_ && info_.joints.size()!=12) return hardware_interface::CallbackReturn::ERROR;
    if (supported_damping_) {
      if (info_.joints.size() != 12) return hardware_interface::CallbackReturn::ERROR;
      RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                  "FC_SUPPORTED_DAMPING_V1: permanent body support required; no gait or position control");
    }
    if (!supported_standby_ && !field_rehome_ && profile->second != "strict") {
      RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"), "Unknown fc_imu_recovery profile");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  board_health_.profile = redundant_field_?3u:(supported_damping_||supported_standby_?4u:(field_rehome_?2u:1u));
  if (const auto *path=std::getenv("PUPPER_FC_BOARD_HEALTH_PATH")) {
    if (!board_health_.open(path)) {
      RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"), "FC_BOARD_HEALTH_V1 could not initialize selected receipt");
      return hardware_interface::CallbackReturn::ERROR;
    }
    board_health_.publish(0,BNO055::Output{},BNO055::Output{},0,false);
  } else if (redundant_field_) {
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"), "Field redundant IMU requires selected health receipt path");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (field_rehome_ || supported_standby_) {
    RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "FC_IMU_RECOVERY_CONTRACT_V1_SUPPORTED_REHOME profile=%s", profile->second.c_str());
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
    if (field_rehome_) {
      for (const auto *kind : {"position", "velocity", "effort", "kp", "kd"})
        if (!supported_damping_ || std::string_view(kind) == "effort")
          recovery_interfaces_.push_back(joint.name + "/" + kind);
    }
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

  if (supported_damping_ || redundant_field_) {
    std::array<bool, 12> assigned{};
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      const int channel = hw_actuator_can_channels_[i], id = hw_actuator_can_ids_[i];
      if (channel < 1 || channel > 4 || id < 1 || id > 3 || assigned[(channel-1)*3 + id-1] ||
          !std::isfinite(hw_actuator_position_mins_[i]) || !std::isfinite(hw_actuator_position_maxs_[i]) ||
          hw_actuator_position_mins_[i] >= hw_actuator_position_maxs_[i]) return hardware_interface::CallbackReturn::ERROR;
      assigned[(channel-1)*3 + id-1] = true;
      for (double limit : {hw_actuator_velocity_maxs_[i], hw_actuator_effort_maxs_[i],
                           hw_actuator_kd_maxs_[i], hw_actuator_homing_kds_[i]})
        if (!std::isfinite(limit) || limit <= 0) return hardware_interface::CallbackReturn::ERROR;
      if (!supported_damping_) continue;
      // The published board ELF packs MIT CAN velocity ±65, Kd [0,5] and
      // feed-forward ±112.5 with 12 bits. Reserve a full zero-code torque step
      // and velocity step for quantization, even at the wire velocity limit.
      const double cap = std::min(0.50, hw_actuator_effort_maxs_[i]);
      const double max_kd = (cap - 225.0/4095) / (65.0 + 130.0/4095);
      const double gain = std::min({0.10, hw_actuator_kd_maxs_[i], hw_actuator_homing_kds_[i], max_kd});
      if (gain < 5.0/4095) return hardware_interface::CallbackReturn::ERROR;
      bench_kds_.push_back(static_cast<float>(gain));
    }
  }

  if (redundant_field_) prior_field_positions_.resize(info_.joints.size());

  if (info_.hardware_parameters.count("use_imu") &&
      (info_.hardware_parameters.at("use_imu") == "false" ||
       info_.hardware_parameters.at("use_imu") == "False")) {
    RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"), "%sIMU not enabled%s",
                YELLOW_ANSI, RESET_ANSI);
    if (supported_standby_ || field_rehome_) {
      RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                   "FC IMU recovery requires the IMU");
      return hardware_interface::CallbackReturn::ERROR;
    }
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
      const auto sample = imu_manager_.get_imu_data();
      const auto now = std::chrono::steady_clock::now();
      const char *error = pupper_imu::feedback_error(sample, now);
      if (!error) break;
      if (now >= deadline) {
        disable_for_imu_fault(sample, now, "initialization", error);
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

  if (redundant_field_) {
    state_interfaces.emplace_back("imu_sensor","fc_balance_mode",&fc_balance_mode_);
    state_interfaces.emplace_back("imu_sensor","fc_balance_incident",&fc_balance_incident_);
    state_interfaces.emplace_back("imu_sensor","fc_balance_generation",&fc_balance_generation_);
  }
  return state_interfaces;
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (field_rehome_) {
    fresh_claim_prepared_ = false;
    field_state_ = FieldState::configured;
    damping_deadline_ = {};
    last_joint_exchange_ = last_valid_q_ = last_valid_g_ = {};
  }
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

bool ControlBoardHardwareInterface::touches_joints(const std::vector<std::string> &start) const {
  for (const auto &joint : info_.joints) {
    for (const auto &name : start) {
      if (name.compare(0, joint.name.size(), joint.name) == 0 &&
          name.size() > joint.name.size() && name[joint.name.size()] == '/') return true;
    }
  }
  return false;
}

bool ControlBoardHardwareInterface::complete_joint_claim(const std::vector<std::string> &start) const {
  if (supported_damping_) {
    // The bench lease claims effort alone. Both FC policies claim five handles per joint.
    for (const auto &name : start)
      if (touches_joints({name}) && std::find(recovery_interfaces_.begin(), recovery_interfaces_.end(), name)
          == recovery_interfaces_.end()) return false;
  }
  for (const auto &name : recovery_interfaces_)
    if (std::find(start.begin(), start.end(), name) == start.end()) return false;
  return !recovery_interfaces_.empty();
}

hardware_interface::return_type ControlBoardHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string> &start, const std::vector<std::string> &stop) {
  if (redundant_field_ && (field_backup_active_ || board_health_.terminal) && touches_joints(start))
    return hardware_interface::return_type::ERROR;
  if ((supported_standby_ || field_rehome_) && touches_joints(start)) {
    if (supported_standby_) return hardware_interface::return_type::ERROR;
    if (supported_damping_ && (touches_joints(stop) || !complete_joint_claim(start) ||
        field_state_ != FieldState::awaiting_claim)) return hardware_interface::return_type::ERROR;
    const auto state = field_state_.load();
    if (state == FieldState::awaiting_claim) {
      fresh_claim_prepared_ = false;
      const auto generation=field_claim_generation_.load();
      if (!can_claim_on_field_feedback(generation)) {
        RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                    "Supported Home retained; a fresh leg claim requires qualified primary and alternate IMU agreement "
                    "and current, in-range joint feedback");
        return hardware_interface::return_type::ERROR;
      }
      prepared_field_generation_=generation;
      fresh_claim_prepared_ = complete_joint_claim(start);
      if (!fresh_claim_prepared_) return hardware_interface::return_type::ERROR;
    } else if (state != FieldState::running) return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::SystemInterface::prepare_command_mode_switch(start, stop);
}

hardware_interface::return_type ControlBoardHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string> &start, const std::vector<std::string> &stop) {
  auto state_at_stop = field_state_.load();
  if (field_rehome_ && (state_at_stop == FieldState::running || state_at_stop == FieldState::awaiting_claim ||
                        state_at_stop == FieldState::zero_first) && touches_joints(stop) && !touches_joints(start)) {
    deactivate_motors();
    clear_joint_commands();
    fresh_claim_prepared_ = false;
    joint_claim_until_ns_ = 0; // A warm Start must observe the newly disabled exchange.
    const bool stopping_in_loss = (supported_damping_ && damping_deadline_ != std::chrono::steady_clock::time_point{}) ||
                                  (redundant_field_ && field_backup_active_);
    const auto next = stopping_in_loss ? FieldState::fault : FieldState::awaiting_claim;
    field_state_.compare_exchange_strong(state_at_stop, next);
    if (next == FieldState::fault) {
      invalidate_balance();
      const auto sample=imu_manager_.get_imu_data();
      terminal_health(sample);
      report_imu_fault(sample, std::chrono::steady_clock::now(), "stop", "operator Stop during IMU loss");
    }
  }
  if (redundant_field_ && (field_backup_active_ || board_health_.terminal) && touches_joints(start))
    return hardware_interface::return_type::ERROR;
  if ((supported_standby_ || field_rehome_) && touches_joints(start)) {
    if (supported_standby_) return hardware_interface::return_type::ERROR;
    if (supported_damping_ && (touches_joints(stop) || !complete_joint_claim(start) ||
        field_state_ != FieldState::awaiting_claim)) return hardware_interface::return_type::ERROR;
    const auto state = field_state_.load();
    if (state == FieldState::awaiting_claim) {
      if (!fresh_claim_prepared_.exchange(false) || !complete_joint_claim(start) ||
          !can_claim_on_field_feedback(prepared_field_generation_))
        return hardware_interface::return_type::ERROR;
      // Redundant mode uses the read owner's atomic dual-source proof here.
      // Do not mutate its selector from the control-mode-switch callback.
      if (!redundant_field_ && !check_motor_feedback(imu_manager_.get_imu_data(), "rearm"))
        return hardware_interface::return_type::ERROR;
      auto expected = FieldState::awaiting_claim;
      if (!field_state_.compare_exchange_strong(expected, FieldState::zero_first))
        return hardware_interface::return_type::ERROR;
    } else if (state != FieldState::running) return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::SystemInterface::perform_command_mode_switch(start, stop);
}

bool contains_nan(const Eigen::Quaternionf &q) {
  return std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) || std::isnan(q.w());
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (field_rehome_) {
    auto expected = FieldState::configured;
    if (!field_state_.compare_exchange_strong(expected, FieldState::homing)) {
      deactivate_motors();
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (supported_standby_ && standby_homed_) {
    deactivate_motors();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  redundant_ = {};
  field_backup_active_=false;
  backup_claim_until_ns_=0;
  joint_claim_until_ns_=0;
  board_health_.terminal=false;
  fc_balance_mode_=5;
  board_health_.publish(5,BNO055::Output{},BNO055::Output{},0,false);
  // An activation requests a new supported home. Old offsets and a prepared
  // packet can survive on_error on the same ROS component even in strict mode.
  std::fill(hw_actuator_is_homed_.begin(), hw_actuator_is_homed_.end(), false);
  std::fill(hw_actuator_zero_positions_.begin(), hw_actuator_zero_positions_.end(), 0.0);
  clear_joint_commands();
  *spi_command_ = spi_command_t{};
  // Retain the bounded startup observations until the mechanical Home also finishes.
  auto trace=imu_manager_.enabled ? pupper_imu::StartupTrace::from_environment(board_health_.terminal) : nullptr;
  if (imu_manager_.enabled) {
    pupper_imu::StartupWindow stationary;
    const auto finish_trace = [&](bool passed) {
      if (trace && !trace->finish(passed))
        RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                    "IMU startup diagnostic trace could not be written (gate outcome unchanged)");
    };
    const auto deadline = std::chrono::steady_clock::now() + pupper_imu::startup_timeout;
    RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "Keep body supported and stationary: checking fresh IMU reports before motor enable");
    const auto startup_resets=imu_manager_.get_imu_data().alternative.reset_count;
    std::array<unsigned, pupper_imu::RedundantImu::gate_names.size()> source_polls{};
    unsigned polls=0, primary_ready_polls=0, backup_ready_polls=0;
    unsigned bad_feedback=0, peak_rejections=0, gravity_rejections=0, mean_rejections=0, sampling_gaps=0;
    double max_span=0, max_gyro=0;
    while (true) {
      if (field_rehome_ && field_state_ != FieldState::homing) return hardware_interface::CallbackReturn::ERROR;
      const auto sample = imu_manager_.get_imu_data();
      const auto observed_at = std::chrono::steady_clock::now();
      pupper_imu::StartupObservation observation;
      const bool ready = stationary.observe(sample, observed_at, &observation);
      ++polls;
      primary_ready_polls+=ready;
      max_span=std::max(max_span,observation.eligible_seconds);
      max_gyro=std::max(max_gyro,double(sample.gyro.norm()));
      sampling_gaps+=observation.observation_gap;
      const std::string_view why(observation.reason);
      bad_feedback+=why=="feedback"; peak_rejections+=why=="gyro_norm";
      gravity_rejections+=why=="gravity"; mean_rejections+=why=="gyro_mean";
      const char *primary_error=nullptr;
      bool sources_ready=true;
      bool reset_during_startup=false;
      const char *source_reason="not_configured";
      if (redundant_field_) {
        BNO055::Output observed;
        pupper_imu::RedundantImu::Gate gate;
        primary_error=redundant_.select(sample,observed_at,false,observed,&gate);
        ++source_polls[static_cast<size_t>(gate)];
        source_reason=pupper_imu::RedundantImu::gate_names[static_cast<size_t>(gate)];
        publish_backup_claim_proof(observed_at);
        sources_ready=!primary_error && redundant_.claim_deadline()>observed_at;
        backup_ready_polls+=sources_ready;
        reset_during_startup=sample.alternative.reset_count!=startup_resets;
      }
      if (trace) trace->record(sample, observed_at, observation, ready, source_reason, redundant_field_ && sources_ready);
      if (ready && !primary_error && !reset_during_startup) {
        if (!sources_ready)
          RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
                      "Primary stationary check passed for supported Home; first leg claim awaits alternate IMU agreement (%s)",
                      source_reason);
        finish_trace(true);
        break;
      }
      if (reset_during_startup || observed_at >= deadline) {
        const char *error=primary_error ? primary_error : pupper_imu::feedback_error(sample,observed_at);
        disable_for_imu_fault(sample, observed_at, "startup",
                              reset_during_startup ? "BNO reset during supported startup" :
                              error ? error : "stationary startup window timed out");
        // Only after neutral. Record both independent gates even when no optional
        // CSV was requested. Counts are control polls, not unique reports.
        std::ostringstream detail;
        detail << "IMU startup evidence before motor enable: polls=" << polls
               << " primary_ready=" << primary_ready_polls << " max_stationary_s=" << max_span
               << " max_gyro_rad_s=" << max_gyro << " primary_rejections={feedback:" << bad_feedback
               << ",gyro_peak:" << peak_rejections << ",gravity:" << gravity_rejections
               << ",gyro_mean:" << mean_rejections << ",observation_gap:" << sampling_gaps << "}";
        if (redundant_field_) {
          const auto &a=sample.alternative;
          detail << " backup_ready=" << backup_ready_polls << " qualification_polls={";
          for (size_t i=0;i<source_polls.size();++i)
            detail << (i?",":"") << pupper_imu::RedundantImu::gate_names[i] << ":" << source_polls[i];
          detail << "} alt_q={seq:" << a.quaternion_report_sequence << ",accuracy:" << int(a.quaternion_accuracy)
                 << ",valid:" << a.quaternion_valid << "} alt_g={seq:" << a.gyro_report_sequence
                 << ",accuracy:" << int(a.gyro_accuracy) << ",valid:" << a.gyro_valid
                 << "} alt_gaps=" << a.gap_count << " sensor_resets=" << a.reset_count-startup_resets;
        }
        RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"), "%s", detail.str().c_str());
        finish_trace(false);
        return hardware_interface::CallbackReturn::ERROR;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // The boards can return an empty first reply while waking. Probe with every
  // enable bit clear and never start the joint ramp on an unverified transport.
  const auto board_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while (true) {
    if (field_rehome_ && field_state_ != FieldState::homing) return hardware_interface::CallbackReturn::ERROR;
    *spi_command_ = {};
    spi_driver_run();
    if ((spi_data_->spi_driver_status & SPI_BOTH_REPLIES_VALID) == SPI_BOTH_REPLIES_VALID) break;
    if (std::chrono::steady_clock::now() >= board_deadline) {
      const int replies = spi_data_->spi_driver_status & SPI_BOTH_REPLIES_VALID;
      deactivate_motors();
      latch_field_fault();
      terminal_health(BNO055::Output{});
      RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                   "Home refused before motor enable: missing motor board reply (mask=%d)", replies);
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // Enable actuators. Send the command multiple times to ensure it is received.
  for (int i = 0; i < 10; i++) {
    if (field_rehome_ && field_state_ != FieldState::homing) return hardware_interface::CallbackReturn::ERROR;
    if (imu_manager_.enabled && !check_motor_feedback(imu_manager_.get_imu_data(), "enable")) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    spi_command_->flags[0] = 1;
    spi_command_->flags[1] = 1;
    spi_command_->flags[2] = 1;
    spi_command_->flags[3] = 1;
    spi_driver_run();
  }
  copy_actuator_states();
  const auto enable_settle_until=std::chrono::steady_clock::now()+std::chrono::milliseconds(100);
  while (std::chrono::steady_clock::now()<enable_settle_until) {
    std::this_thread::sleep_until(std::min(enable_settle_until,std::chrono::steady_clock::now()+std::chrono::milliseconds(5)));
    if (imu_manager_.enabled && !check_motor_feedback(imu_manager_.get_imu_data(),"enable settle"))
      return hardware_interface::CallbackReturn::ERROR;
  }

  // Homing
  if (!do_homing()) return hardware_interface::CallbackReturn::ERROR;

  if (field_rehome_) {
    deactivate_motors();
    clear_joint_commands();
    if (redundant_field_) {
      copy_actuator_states();
      last_joint_exchange_ = std::chrono::steady_clock::now();
    }
  }
  if (supported_standby_) {
    deactivate_motors();
    standby_homed_ = true;
    RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                "Supported standby: homed, motors disabled, all joint controller claims blocked");
  }

  // A completed mechanical Home is the only new source generation. Set all
  // fields before allowing read() to publish the new session's first snapshot.
  const auto home_us = pupper_imu::BoardHealth::ns(std::chrono::steady_clock::now()) / 1000;
  const auto home_id = std::max(board_health_.homes.load() + 1, home_us);
  board_health_.homes = home_id;
  board_health_.incidents = 0;
  fc_balance_incident_ = 0;
  fc_balance_generation_ = static_cast<double>(home_id);
  fc_balance_mode_ = 1;
  if (field_rehome_) {
    auto expected = FieldState::homing;
    if (!field_state_.compare_exchange_strong(expected, FieldState::awaiting_claim))
      return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void ControlBoardHardwareInterface::latch_field_fault() {
  if (!field_rehome_) return;
  field_state_ = FieldState::fault;
  fresh_claim_prepared_ = false;
}

void ControlBoardHardwareInterface::clear_joint_commands() {
  for (auto *values : {&hw_command_positions_, &hw_command_velocities_, &hw_command_efforts_,
                       &hw_command_kps_, &hw_command_kds_}) {
    std::fill(values->begin(), values->end(), 0.0);
  }
}

void ControlBoardHardwareInterface::invalidate_balance() {
  const auto unknown = std::numeric_limits<double>::quiet_NaN();
  hw_state_imu_orientation_.fill(unknown);
  hw_state_imu_angular_velocity_.fill(unknown);
  hw_state_imu_linear_acceleration_.fill(unknown);
  imu_time_since_measurement_seconds_ = unknown;
}

void ControlBoardHardwareInterface::deactivate_motors() {
  // Send a neutral disable packet; flags alone leave the previous targets,
  // gains and feed-forward torques in the command sent to both boards.
  if (spi_command_ == nullptr) return;  // Initialization may have failed before SPI setup.
  *spi_command_ = spi_command_t{};
  spi_driver_run();
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_error(
    [[maybe_unused]] const rclcpp_lifecycle::State &previous_state) {
  deactivate_motors();
  latch_field_fault();
  terminal_health(BNO055::Output{});
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  deactivate_motors();
  latch_field_fault();
  terminal_health(BNO055::Output{});
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool ControlBoardHardwareInterface::hw_states_contains_nan() {
  return contains_nan(hw_state_positions_) || contains_nan(hw_state_velocities_) ||
         contains_nan(hw_state_efforts_) || contains_nan(hw_state_imu_orientation_) ||
         contains_nan(hw_state_imu_angular_velocity_) ||
         contains_nan(hw_state_imu_linear_acceleration_);
}

ControlBoardHardwareInterface::JointIssue ControlBoardHardwareInterface::joint_issue(
    std::chrono::steady_clock::time_point now, double step_dt) const {
  const int mask = spi_data_->spi_driver_status & SPI_BOTH_REPLIES_VALID;
  constexpr size_t no_axis = static_cast<size_t>(-1);
  if (mask != SPI_BOTH_REPLIES_VALID)
    return {"spi_reply_mask", no_axis, static_cast<double>(mask), SPI_BOTH_REPLIES_VALID, SPI_BOTH_REPLIES_VALID};
  const double age = std::chrono::duration<double>(now - last_joint_exchange_).count();
  if (last_joint_exchange_ == std::chrono::steady_clock::time_point{} || age < 0 || age > 0.050)
    return {"exchange_age_s", no_axis, age, 0., 0.050};
  if (step_dt > 0.050) return {"exchange_interval_s", no_axis, step_dt, 0., 0.050};
  for (size_t i = 0; i < hw_state_positions_.size(); ++i) {
    const double q = hw_state_positions_[i], v = hw_state_velocities_[i];
    const double low = hw_actuator_position_mins_[i], high = hw_actuator_position_maxs_[i];
    const double vmax = hw_actuator_velocity_maxs_[i];
    if (!std::isfinite(q) || q < low || q > high)
      return {"position_rad", i, q, low, high};
    if (!std::isfinite(v) || std::abs(v) > vmax)
      return {"velocity_rad_s", i, v, -vmax, vmax};
    if (step_dt >= 0) {
      const double delta = q - prior_field_positions_[i], limit = .02 + vmax * step_dt;
      if (std::abs(delta) > limit) return {"displacement_rad", i, delta, -limit, limit};
    }
  }
  return {};
}

void ControlBoardHardwareInterface::log_joint_issue(const JointIssue &issue, const char *phase) const {
  const char *joint = issue.axis < info_.joints.size() ? info_.joints[issue.axis].name.c_str() : "board";
  RCLCPP_WARN(rclcpp::get_logger("ControlBoardHardwareInterface"),
              "FC joint feedback phase=%s predicate=%s joint=%s observed=%.9g min=%.9g max=%.9g",
              phase, issue.reason, joint, issue.value, issue.low, issue.high);
}

void ControlBoardHardwareInterface::publish_joint_claim_proof(std::chrono::steady_clock::time_point now,
                                                            const JointIssue &issue) {
  if (issue.reason) {
    joint_claim_until_ns_ = 0;
    fresh_claim_prepared_ = false;
    if (!unclaimed_joint_fault_reported_) log_joint_issue(issue, "unclaimed");
    unclaimed_joint_fault_reported_ = true;
    return;
  }
  unclaimed_joint_fault_reported_ = false;
  const auto stamp = pupper_imu::BoardHealth::ns(now);
  if (joint_claim_until_ns_.load() <= stamp) ++field_claim_generation_;
  joint_claim_until_ns_ = pupper_imu::BoardHealth::ns(now + std::chrono::milliseconds(50));
}

bool ControlBoardHardwareInterface::bench_joint_feedback_ok(std::chrono::steady_clock::time_point now) const {
  return !joint_issue(now).reason;
}

hardware_interface::return_type ControlBoardHardwareInterface::read_bench(const BNO055::Output &sample) {
  using namespace std::chrono;
  using Return = hardware_interface::return_type;
  const auto now = steady_clock::now();
  const char *error = pupper_imu::feedback_error(sample, now);
  const bool in_loss = damping_deadline_ != steady_clock::time_point{};
  const std::string_view cause = error ? error : "";
  const bool age_expired = cause == "stale gyro receipt" || cause == "stale quaternion receipt" ||
                           cause == "gyro report known too old" || cause == "quaternion report known too old";
  // Bridge an aging report that this driver actually saw fresh. A newly
  // delivered old batch, missing report or future receipt is never a gap waiver.
  const auto known_component = [&](steady_clock::time_point receipt, microseconds age, steady_clock::time_point &last) {
    if (receipt == steady_clock::time_point{} || receipt > now || age < microseconds::zero() ||
        age > pupper_imu::max_receipt_age || receipt < last) return false;
    if (now - receipt + age <= pupper_imu::max_receipt_age) last = receipt;
    return receipt == last;
  };
  const bool known_q = known_component(sample.quaternion_receipt_time, sample.quaternion_minimum_age_at_receipt, last_valid_q_);
  const bool known_g = known_component(sample.gyro_receipt_time, sample.gyro_minimum_age_at_receipt, last_valid_g_);
  const bool timed_loss = age_expired && known_q && known_g;
  const auto last_balance = timed_loss ? std::min(sample.quaternion_receipt_time - sample.quaternion_minimum_age_at_receipt,
      sample.gyro_receipt_time - sample.gyro_minimum_age_at_receipt) : now;
  if (!known_q || !known_g || (in_loss && now >= damping_deadline_) || (error && !timed_loss) ||
      (timed_loss && now - last_balance >= seconds(3)) || !bench_joint_feedback_ok(now)) {
    disable_for_imu_fault(sample, now, "read", error ? error : (in_loss ? "supported bench recovery deadline" :
                           "supported bench joint feedback invalid"));
    return Return::ERROR;
  }
  const bool entering = !in_loss && timed_loss;
  if (entering) damping_deadline_ = last_balance + seconds(3);
  if (timed_loss) {
    recovery_pairs_ = 0;
    recovery_q_ = sample.quaternion_receipt_time;
    recovery_g_ = sample.gyro_receipt_time;
  } else if (in_loss && sample.quaternion_receipt_time > recovery_q_ && sample.gyro_receipt_time > recovery_g_) {
    recovery_q_ = sample.quaternion_receipt_time;
    recovery_g_ = sample.gyro_receipt_time;
    if (recovery_pairs_++ == 0) recovery_since_ = now;
    // Only balance telemetry resumes. The exclusively leased motor mode never
    // changes and a field controller still cannot acquire these handles.
    if (recovery_pairs_ >= 3 && now - recovery_since_ >= milliseconds(20)) {
      damping_deadline_ = {};
      RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                  "Supported bench balance restored; actuator-local damping continues");
    }
  }
  // Never synthesize feed-forward torque from the Pi's cached CAN readings.
  // The motor drive applies its derivative gain to its own encoder velocity.
  *spi_command_ = {};
  for (auto &flag : spi_command_->flags) flag = 1;
  if (field_state_ == FieldState::running) {
    float *groups[]{spi_command_->kd_abad, spi_command_->kd_hip, spi_command_->kd_knee};
    for (size_t i = 0; i < bench_kds_.size(); ++i)
      groups[hw_actuator_can_ids_[i]-1][hw_actuator_can_channels_[i]-1] = bench_kds_[i];
  }
  last_joint_exchange_ = now;
  spi_driver_run();
  copy_actuator_states();
  if (!bench_joint_feedback_ok(steady_clock::now())) {
    disable_for_imu_fault(sample, steady_clock::now(), "read", "supported bench joint exchange failed");
    return Return::ERROR;
  }
  if (entering) report_imu_fault(sample, now, "supported_damping", error);
  auto first = FieldState::zero_first;
  field_state_.compare_exchange_strong(first, FieldState::running);
  if (damping_deadline_ != steady_clock::time_point{}) invalidate_balance();
  return Return::OK;
}

hardware_interface::return_type ControlBoardHardwareInterface::read(
    [[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period) {
  // Validate BEFORE transmitting previously prepared actuator commands. Each
  // sensor component has its own monotonic receipt; one cannot freshen another.
  if (field_rehome_ && field_state_ == FieldState::homing) return hardware_interface::return_type::OK;
  const BNO055::Output raw = imu_manager_.get_imu_data();
  BNO055::Output imu_output = raw;
  if (field_rehome_ && (field_state_ == FieldState::fault || field_state_ == FieldState::configured)) {
    deactivate_motors();
    clear_joint_commands();
    copy_actuator_states();
    invalidate_balance();
    return field_state_ == FieldState::fault ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
  }
  const bool bench_active = supported_damping_ &&
      (field_state_ == FieldState::running || field_state_ == FieldState::zero_first);
  if (bench_active) {
    const auto result = read_bench(imu_output);
    if (result != hardware_interface::return_type::OK || damping_deadline_ != std::chrono::steady_clock::time_point{})
      return result;
  } else if (supported_standby_ && standby_homed_) {
    const auto now = std::chrono::steady_clock::now();
    const char *error = pupper_imu::feedback_error(imu_output, now);
    if (error && !standby_feedback_lost_) {
      disable_for_imu_fault(imu_output, now, "supported_standby", error);
      standby_feedback_lost_ = true;
      standby_recovery_ = {};
    } else {
      deactivate_motors();
    }
    if (standby_feedback_lost_) {
      // Never republish the last valid orientation after loss. Reuse both
      // independent receipt clocks and the existing stationary qualification.
      standby_feedback_lost_ = !standby_recovery_.observe(imu_output, now);
      if (standby_feedback_lost_) {
        invalidate_balance();
      } else {
        RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"),
                    "Supported standby: balance telemetry recovered; motors remain disabled");
      }
    }
    clear_joint_commands();
    copy_actuator_states();
    if (contains_nan(hw_state_positions_) || contains_nan(hw_state_velocities_) ||
        contains_nan(hw_state_efforts_)) return hardware_interface::return_type::ERROR;
    if (standby_feedback_lost_) return hardware_interface::return_type::OK;
  } else {
    double field_dt=0;
    if (redundant_field_) {
      const auto now=std::chrono::steady_clock::now();
      const bool running=field_state_==FieldState::running;
      const auto previous=redundant_.mode;
      const char *error=redundant_.select(raw,now,running,imu_output);
      publish_backup_claim_proof(now);
      if (redundant_.claim_deadline()<=now) fresh_claim_prepared_=false;
      if (!error && running) {
        const auto issue = joint_issue(now);
        if (issue.reason) {
          disable_for_imu_fault(raw, now, "read", issue.reason);
          log_joint_issue(issue, "before_exchange");
          return hardware_interface::return_type::ERROR;
        }
      }
      if (error) {
        disable_for_imu_fault(raw, now, "read", error);
        return hardware_interface::return_type::ERROR;
      }
      if (previous!=2 && redundant_.mode==2) {
        fc_balance_incident_=static_cast<double>(++board_health_.incidents);
        clear_joint_commands();
        *spi_command_={};
        for (auto &flag : spi_command_->flags) flag=1;
      }
      field_backup_active_=redundant_.mode==2;
      fc_balance_mode_=redundant_.mode;
      field_dt=std::chrono::duration<double>(now-last_joint_exchange_).count();
      prior_field_positions_=hw_state_positions_;
      last_joint_exchange_=now;
    } else if (imu_manager_.enabled && !check_motor_feedback(imu_output, "read")) {
      return hardware_interface::return_type::ERROR;
    }
    const auto dispatch_state=field_state_.load();
    if (field_rehome_ && dispatch_state==FieldState::zero_first) {
      if (!can_claim_on_field_feedback(prepared_field_generation_)) {
        disable_for_imu_fault(raw,std::chrono::steady_clock::now(),"first leg claim",
                              "joint or dual IMU proof expired before first enabled packet");
        return hardware_interface::return_type::ERROR;
      }
      // Construct the first packet on the device-I/O owner. Prepare/perform
      // publish authority only and cannot race an awaiting-claim neutral write.
      clear_joint_commands();
      *spi_command_={};
      for (auto &flag : spi_command_->flags) flag=1;
    }
    if (supported_standby_ || (field_rehome_ && dispatch_state!=FieldState::running && dispatch_state!=FieldState::zero_first)) {
      if (supported_damping_) last_joint_exchange_ = std::chrono::steady_clock::now();
      deactivate_motors();
    } else {
      spi_driver_run();
      if (field_rehome_ && dispatch_state==FieldState::zero_first) {
        auto expected = FieldState::zero_first;
        field_state_.compare_exchange_strong(expected, FieldState::running);
      }
    }
    copy_actuator_states();
    if (redundant_field_) {
      const auto sampled = std::chrono::steady_clock::now();
      const auto issue = joint_issue(sampled, field_dt);
      if (dispatch_state == FieldState::awaiting_claim) {
        // An unpowered leg may settle outside the walking envelope. Keep the reference
        // and refuse a new claim until the operator restores a valid supported pose.
        publish_joint_claim_proof(sampled, issue);
      } else if ((dispatch_state == FieldState::running || dispatch_state == FieldState::zero_first) && issue.reason) {
        disable_for_imu_fault(raw, sampled, "read", issue.reason);
        log_joint_issue(issue, "after_exchange");
        return hardware_interface::return_type::ERROR;
      }
    }
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

  // Updating the state interfaces with corrected values
  hw_state_imu_orientation_[0] = corrected_quat.x();
  hw_state_imu_orientation_[1] = corrected_quat.y();
  hw_state_imu_orientation_[2] = corrected_quat.z();
  hw_state_imu_orientation_[3] = corrected_quat.w();

  hw_state_imu_angular_velocity_[0] = angular_velocity.x();
  hw_state_imu_angular_velocity_[1] = angular_velocity.y();
  hw_state_imu_angular_velocity_[2] = angular_velocity.z();

  hw_state_imu_linear_acceleration_[0] = linear_acceleration.x();
  hw_state_imu_linear_acceleration_[1] = linear_acceleration.y();
  hw_state_imu_linear_acceleration_[2] = linear_acceleration.z();

  // Check if any NaNs in hardware state arrays. Catches IMU issues.
  if (hw_states_contains_nan()) {
    deactivate_motors();
    latch_field_fault();
    terminal_health(raw);
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "HW state array contained NaN. Deactivating motors");
    std::stringstream ss;
    ss << hw_state_imu_orientation_ << '\n'
       << hw_state_imu_angular_velocity_ << '\n'
       << hw_state_imu_linear_acceleration_ << '\n'
       << hw_state_positions_ << '\n'
       << hw_state_velocities_ << '\n'
       << hw_state_efforts_ << '\n';
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"), "%s", ss.str().c_str());
    return hardware_interface::return_type::ERROR;
  }

  board_health_.publish(board_health_.homes ? (redundant_field_ ? redundant_.mode : 1) : 0,
                        raw, imu_output, spi_data_->spi_driver_status&3, redundant_field_ && redundant_.mode==2);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ControlBoardHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (supported_damping_) return hardware_interface::return_type::OK;
  if (field_rehome_ && (field_state_ == FieldState::zero_first || field_state_ == FieldState::homing))
    return hardware_interface::return_type::OK;
  if (supported_standby_ || (field_rehome_ && field_state_ != FieldState::running)) {
    *spi_command_ = spi_command_t{};
    return hardware_interface::return_type::OK;
  }
  copy_actuator_commands(true);
  return hardware_interface::return_type::OK;
}

bool ControlBoardHardwareInterface::do_homing() {
  using Clock = std::chrono::steady_clock;
  constexpr int dt_ms = 10;
  std::vector<double> return_tolerance(hw_state_positions_.size());
  bool returning = false;
  auto refuse = [&](const char *reason, bool report_axes = false) {
    deactivate_motors();
    latch_field_fault();
    // The last sampled coordinates and pre-disable targets are still available here.
    // Never postpone the neutral packet to format or write the failure evidence.
    std::string travel_summary;
    if (report_axes) {
      std::ostringstream summary;
      summary << "; phase=" << (returning ? "return" : "search");
      size_t pending_axes = 0;
      for (size_t i = 0; i < hw_state_positions_.size(); ++i) {
        const double rest = hw_actuator_post_homing_positions_[i];
        const double error = hw_state_positions_[i] - rest;
        const bool at_target = std::abs(hw_command_positions_[i] - rest) <= 0.01;
        const bool in_range = hw_state_positions_[i] >= hw_actuator_position_mins_[i] &&
                              hw_state_positions_[i] <= hw_actuator_position_maxs_[i];
        if (returning ? at_target && std::abs(error) <= return_tolerance[i] && (!redundant_field_ || in_range)
                      : hw_actuator_is_homed_[i]) continue;
        if (pending_axes++ == 0)
          summary << " first_pending_joint=" << info_.joints[i].name << " command_rad=" << hw_command_positions_[i]
                  << " measured_rad=" << hw_state_positions_[i] << " rest_rad=" << rest
                  << " measured_minus_rest_rad=" << error << " tolerance_rad=" << return_tolerance[i];
        RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                     "Home travel evidence phase=%s axis=%zu joint=%s stage=%d referenced=%d "
                     "command_rad=%.6f measured_rad=%.6f rest_rad=%.6f measured_minus_rest_rad=%.6f "
                     "tolerance_rad=%.6f target_reached=%d position_min_rad=%.6f position_max_rad=%.6f",
                     returning ? "return" : "search", i, info_.joints[i].name.c_str(),
                     static_cast<int>(hw_actuator_homing_stages_[i]), static_cast<int>(hw_actuator_is_homed_[i]),
                     hw_command_positions_[i], hw_state_positions_[i], rest, error, return_tolerance[i],
                     static_cast<int>(at_target), hw_actuator_position_mins_[i], hw_actuator_position_maxs_[i]);
      }
      summary << " pending_axes=" << pending_axes;
      travel_summary = summary.str();
    }
    std::fill(hw_actuator_is_homed_.begin(), hw_actuator_is_homed_.end(), false);
    std::fill(hw_actuator_zero_positions_.begin(), hw_actuator_zero_positions_.end(), 0.0);
    clear_joint_commands();
    terminal_health(BNO055::Output{});
    RCLCPP_ERROR(rclcpp::get_logger("ControlBoardHardwareInterface"),
                 "Mechanical Home stopped: %s%s; neutral sent, supported Home required", reason, travel_summary.c_str());
    return false;
  };
  // Axes in one stage travel together. Budget the full physical span plus the
  // position error needed to detect the stop, then the longest return stroke.
  std::map<int, double> stage_seconds;
  std::vector<double> remaining_search(hw_state_positions_.size());
  double return_seconds = 0;
  for (size_t i = 0; i < hw_state_positions_.size(); ++i) {
    const double speed = std::abs(hw_actuator_homing_velocities_[i]);
    const double kp = hw_actuator_homing_kps_[i], threshold = hw_actuator_homing_torque_thresholds_[i];
    const double low = hw_actuator_position_mins_[i], high = hw_actuator_position_maxs_[i];
    const double stop = hw_actuator_homed_positions_[i], rest = hw_actuator_post_homing_positions_[i];
    const auto kd = hw_actuator_homing_kds_[i];
    if (!std::isfinite(speed) || speed <= 0 || !std::isfinite(kp) || kp <= 0 || !std::isfinite(kd) || kd < 0 ||
        !std::isfinite(threshold) || threshold < 0 || !std::isfinite(low) ||
        !std::isfinite(high) || high <= low || !std::isfinite(stop) || !std::isfinite(rest))
      return refuse("unusable actuator homing parameters");
    const double span = std::max(high, stop) - std::min(low, stop);
    remaining_search[i] = 2.0 * (span + threshold / kp);
    // A lag larger than the configured stop-load deflection can still
    // represent a blocked motor. Preserve the original return settling band.
    return_tolerance[i] = std::max(0.01, threshold / kp);
    auto &duration = stage_seconds[hw_actuator_homing_stages_[i]];
    duration = std::max(duration, (span + threshold / kp) / speed);
    return_seconds = std::max(return_seconds, std::abs(rest - stop) / speed);
  }
  double travel_seconds = return_seconds;
  for (const auto &stage : stage_seconds) travel_seconds += stage.second;
  // Allow twice the ideal traversal and one second for exchange/filter settling.
  const double budget_seconds = 1.0 + 2.0 * travel_seconds;
  if (!std::isfinite(budget_seconds)) return refuse("unusable actuator travel budget");
  const auto deadline = Clock::now() + std::chrono::duration<double>(budget_seconds);
  auto feedback_ok = [&]() {
    if (field_rehome_ && field_state_ != FieldState::homing) return false;
    if (Clock::now() >= deadline) return refuse("joint stop or return exceeded its travel budget", true);
    if ((spi_data_->spi_driver_status & SPI_BOTH_REPLIES_VALID) != SPI_BOTH_REPLIES_VALID)
      return refuse("missing or corrupt reply from a motor board");
    for (size_t i = 0; i < hw_state_positions_.size(); ++i) {
      // Offsets are unknown before Home. Only the finiteness of encoder data
      // is meaningful here; calibrated position limits apply during walking.
      if (!std::isfinite(hw_state_positions_[i]) || !std::isfinite(hw_state_velocities_[i]))
        return refuse("nonfinite encoder feedback");
    }
    return !imu_manager_.enabled || check_motor_feedback(imu_manager_.get_imu_data(), "homing");
  };
  if (!feedback_ok()) return false;
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Homing actuators...");

  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  copy_actuator_commands();
  if (!feedback_ok()) return false;
  spi_driver_run();
  copy_actuator_states();
  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));

  // Set the initial commands
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    hw_command_positions_[i] = hw_state_positions_[i];
    hw_command_velocities_[i] = 0.0;
    hw_command_kps_[i] = 0.0;
    hw_command_kds_[i] = 0.0;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  copy_actuator_commands();
  if (!feedback_ok()) return false;
  spi_driver_run();
  copy_actuator_states();
  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));

  // Keep track of filtered torques
  std::vector<double> filtered_torques;
  filtered_torques.resize(hw_state_positions_.size(), 0.0);
  double alpha = 0.5;

  // Loop until all actuators are homed
  bool all_homed = false;
  bool all_returned = false;
  while (!all_homed) {
    if (!feedback_ok()) return false;
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
          const double step = hw_actuator_homing_velocities_[i] * dt_ms * 0.001;
          if (std::abs(step) > remaining_search[i]) return refuse("joint stop exceeded its search travel", true);
          remaining_search[i] -= std::abs(step);
          hw_command_positions_[i] += step;
          hw_command_velocities_[i] = hw_actuator_homing_velocities_[i];
          all_homed = false;
        }
      }
    }
    copy_actuator_commands();
    if (!feedback_ok()) return false;
    spi_driver_run();
    copy_actuator_states();

    // Sleep for dt
    std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  }
  returning = true;
  while (!all_returned) {
    if (!feedback_ok()) return false;
    all_returned = true;
    for (auto i = 0u; i < hw_state_positions_.size(); i++) {
      const auto rest = hw_actuator_post_homing_positions_[i];
      const auto delta = rest - hw_command_positions_[i];
      if (std::abs(delta) > 0.01) {
        const auto speed = std::abs(hw_actuator_homing_velocities_[i]);
        const auto step = speed * dt_ms * .001;
        hw_command_positions_[i] += std::clamp(delta, -step, step);
        hw_command_velocities_[i] = std::copysign(speed, delta);
        all_returned = false;
      } else {
        hw_command_positions_[i] = rest;
        hw_command_velocities_[i] = 0.0;
      }
    }
    copy_actuator_commands();
    if (!feedback_ok()) return false;
    spi_driver_run();
    copy_actuator_states();
    // Confirm the final exchange, including its encoder positions, before
    // declaring the commanded return complete.
    for (size_t i = 0; i < hw_state_positions_.size() && all_returned; ++i)
      all_returned = std::abs(hw_state_positions_[i] - hw_actuator_post_homing_positions_[i]) <= return_tolerance[i] &&
          (!redundant_field_ || (hw_state_positions_[i] >= hw_actuator_position_mins_[i] &&
                                hw_state_positions_[i] <= hw_actuator_position_maxs_[i]));

    // Sleep for dt
    std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  }
  if (!feedback_ok()) return false;
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Finished homing!");
  return true;
}

void ControlBoardHardwareInterface::copy_actuator_commands(bool use_position_limits) {
  // Iterate through the joints
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    double cmd_pos = hw_command_positions_[i];
    double cmd_vel = std::clamp(hw_command_velocities_[i], -hw_actuator_velocity_maxs_[i],
                                hw_actuator_velocity_maxs_[i]);
    double cmd_eff = std::clamp(hw_command_efforts_[i], -hw_actuator_effort_maxs_[i],
                                hw_actuator_effort_maxs_[i]);
    double cmd_kp = std::clamp(hw_command_kps_[i], 0.0, hw_actuator_kp_maxs_[i]);
    double cmd_kd = std::clamp(hw_command_kds_[i], 0.0, hw_actuator_kd_maxs_[i]);

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

    uint can_channel = hw_actuator_can_channels_[i] - 1;
    // ID 1: abad, ID 2: hip, ID 3: knee (not corresponding to the actual joint names, just used
    // to make the Cheetah code send to the CAN IDs we want)
    switch (hw_actuator_can_ids_[i]) {
      case 1:
        spi_command_->q_des_abad[can_channel] = cmd_pos;
        spi_command_->qd_des_abad[can_channel] = cmd_vel;
        spi_command_->kp_abad[can_channel] = cmd_kp;
        spi_command_->kd_abad[can_channel] = cmd_kd;
        spi_command_->tau_abad_ff[can_channel] = cmd_eff;
        break;
      case 2:
        spi_command_->q_des_hip[can_channel] = cmd_pos;
        spi_command_->qd_des_hip[can_channel] = cmd_vel;
        spi_command_->kp_hip[can_channel] = cmd_kp;
        spi_command_->kd_hip[can_channel] = cmd_kd;
        spi_command_->tau_hip_ff[can_channel] = cmd_eff;
        break;
      case 3:
        spi_command_->q_des_knee[can_channel] = cmd_pos;
        spi_command_->qd_des_knee[can_channel] = cmd_vel;
        spi_command_->kp_knee[can_channel] = cmd_kp;
        spi_command_->kd_knee[can_channel] = cmd_kd;
        spi_command_->tau_knee_ff[can_channel] = cmd_eff;
        break;
    }
  }
}

void ControlBoardHardwareInterface::copy_actuator_states() {
  // Iterate through the joints
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    float state_pos = hw_state_positions_[i];
    float state_vel = hw_state_velocities_[i];

    uint can_channel = hw_actuator_can_channels_[i] - 1;
    // ID 1: abad, ID 2: hip, ID 3: knee (not corresponding to the actual joint names, just used
    // to make the Cheetah code send to the CAN IDs we want)
    switch (hw_actuator_can_ids_[i]) {
      case 1:
        state_pos = spi_data_->q_abad[can_channel];
        state_vel = spi_data_->qd_abad[can_channel];
        break;
      case 2:
        state_pos = spi_data_->q_hip[can_channel];
        state_vel = spi_data_->qd_hip[can_channel];
        break;
      case 3:
        state_pos = spi_data_->q_knee[can_channel];
        state_vel = spi_data_->qd_knee[can_channel];
        break;
    }
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
