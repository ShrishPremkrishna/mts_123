// Real driver read/activation callbacks with fake SPI/BNO and isolated locks.
// No device implementation is linked and every open except the fake lock fails.
#include "control_board_hardware_interface/control_board_hardware_interface.hpp"
#include "control_board_hardware_interface/imu_health.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/component_parser.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <atomic>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <sstream>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <cstdio>
#include <rcutils/logging.h>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>
namespace {
spi_command_t command{}; spi_data_t data{};
std::vector<spi_command_t> sent;
std::string fault_message;
std::vector<std::string> travel_evidence;
std::string mechanical_refusal;
bool neutral_at_log = false;
void capture_fault(const rcutils_log_location_t *, int, const char *, rcutils_time_point_value_t,
                   const char *format, va_list *args) {
  char message[1024];
  std::vsnprintf(message, sizeof(message), format, *args);
  if (std::strncmp(message, "Mechanical Home stopped:", 24) == 0) mechanical_refusal = message;
  if (std::strncmp(message, "Home travel evidence ", 21) == 0) {
    const spi_command_t zero{};
    if (sent.empty() || std::memcmp(&sent.back(), &zero, sizeof(zero)) != 0) std::abort();
    travel_evidence.emplace_back(message);
  }
  if (std::strncmp(message, "IMU fault ", 10) == 0) {
    const spi_command_t zero{};
    neutral_at_log = !sent.empty() && std::memcmp(&sent.back(), &zero, sizeof(zero)) == 0;
    fault_message = message;
  }
}
std::atomic<int> mode{0};  // 0fresh,1stalegyro,2stalequat,3bias,4badnorm,5missing
// Written before the atomic mode transition, then constant for this outage.
using ReportPeriod = std::chrono::duration<int64_t, std::centi>;
bool clocked_reports = false;
std::atomic<int> aux_fault{0};
std::string receipt_path;
auto report_tick(std::chrono::steady_clock::time_point time) {
  return std::chrono::time_point_cast<ReportPeriod>(time);
}
std::chrono::steady_clock::time_point last_report;
constexpr int freeze_gyro = 8, freeze_quat = 9, freeze_both = 10;
std::chrono::steady_clock::time_point start_dropout(int component) {
  last_report = report_tick(std::chrono::steady_clock::now());
  mode = component;
  return last_report;
}
bool lose_during_enable=false;
bool simulate_empty_wakeup=false;
int spi_replies = SPI_BOTH_REPLIES_VALID;
bool follow_without_endstop = false;
bool follow_return = false;
struct VirtualJointStop { float position; bool positive; };
VirtualJointStop mechanical_stops[3][4]{};
bool lose_reply_during_homing = false;
constexpr int lock_fd=19322;
void require(bool ok,const char *why) { if(!ok) throw std::runtime_error(why); }
BNO055::Output output(std::chrono::steady_clock::time_point now) {
  BNO055::Output o;
  o.quat=Eigen::Quaternionf::Identity();o.gyro=Eigen::Vector3f::Zero();
  o.quaternion_valid=o.gyro_valid=true;
  o.quaternion_report_sequence=17; o.gyro_report_sequence=31;
  o.quaternion_receipt_time=o.gyro_receipt_time=now;
  if (clocked_reports) o.quaternion_receipt_time = o.gyro_receipt_time = report_tick(now);
  o.packet_timestamp=o.measurement_timestamp=std::chrono::high_resolution_clock::now();
  auto &a=o.alternative;
  a.quat=Eigen::Quaternionf(Eigen::AngleAxisf(.5f,Eigen::Vector3f::UnitZ()));
  a.gyro=o.gyro;
  a.quaternion_accuracy=a.gyro_accuracy=3;
  a.quaternion_valid=a.gyro_valid=true;
  a.quaternion_receipt_time=a.gyro_receipt_time=report_tick(now);
  if(aux_fault==1) a.quaternion_receipt_time=a.gyro_receipt_time=last_report;
  if(aux_fault==2) a.gyro.z()=2;
  if(aux_fault==4) a.reset_count=1;
  if(aux_fault==5) a.gap_count=1;
  if(aux_fault==6) a.quaternion_accuracy=1;
  if(aux_fault==7) o.quaternion_minimum_age_at_receipt=std::chrono::milliseconds(24);
  if(aux_fault==3) a.quaternion_minimum_age_at_receipt=std::chrono::seconds(2);
  a.quaternion_report_sequence=report_tick(a.quaternion_receipt_time).time_since_epoch().count()&255;
  a.gyro_report_sequence=report_tick(a.gyro_receipt_time).time_since_epoch().count()&255;
  switch(mode.load()) {
    case 1:o.gyro_receipt_time-=std::chrono::seconds(1);break;
    case 2:o.quaternion_receipt_time-=std::chrono::seconds(1);break;
    case 3:o.gyro.x()=.13;break;
    case 4:o.quat.w()=.5;break;
    case 5:o.quaternion_valid=false;break;
    case 6:o.gyro_minimum_age_at_receipt=std::chrono::seconds(78);break;
    case 7:o.quaternion_minimum_age_at_receipt=std::chrono::seconds(78);break;
    case freeze_gyro: o.gyro_receipt_time = last_report; break;
    case freeze_quat: o.quaternion_receipt_time = last_report; break;
    case freeze_both: o.quaternion_receipt_time = o.gyro_receipt_time = last_report; break;
  }
  if (clocked_reports) {
    o.gyro_report_sequence = report_tick(o.gyro_receipt_time).time_since_epoch().count() & 255;
    o.quaternion_report_sequence = report_tick(o.quaternion_receipt_time).time_since_epoch().count() & 255;
  }
  return o;
}
void neutral_only() {
  require(!sent.empty(),"no disable sent");const spi_command_t zero{};
  for(auto &p:sent)require(std::memcmp(&p,&zero,sizeof zero)==0,"nonneutral command escaped failed check");
}
void recovery_joints(hardware_interface::HardwareInfo &info) {
  for (int i = 0; i < 12; ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint" + std::to_string(i);
    joint.parameters = {{"can_channel", std::to_string(i/3+1)}, {"can_id", std::to_string(i%3+1)},
        {"position_min", "-2"}, {"position_max", "2"}, {"velocity_max", "3"},
        {"effort_max", "4"}, {"kp_max", "20"}, {"kd_max", "2"}, {"homing_stage", "1"},
        {"homing_velocity", "1"}, {"homing_kp", "1"}, {"homing_kd", "1"},
        {"homed_position", "0"}, {"post_homing_position", "0"}, {"homing_torque_threshold", "0"}};
    info.joints.push_back(joint);
  }
}

void change_encoder_coordinates(float offset) {
  for (int i = 0; i < 4; ++i) {
    data.q_abad[i] = offset + i;
    data.q_hip[i] = offset + i + 1;
    data.q_knee[i] = offset + i + 2;
  }
}

void require_all_homed(const std::vector<hardware_interface::StateInterface> &states) {
  unsigned positions = 0;
  for (const auto &state : states) {
    if (state.get_interface_name() == "position") {
      require(std::abs(state.get_value()) < .001, "old encoder coordinate silently reused");
      ++positions;
    }
  }
  require(positions == 12, "missing calibrated joint state");
}

void recovery_case(control_board_hardware_interface::ControlBoardHardwareInterface &driver, bool bench) {
  using Return = hardware_interface::return_type;
  using Callback = hardware_interface::CallbackReturn;
  using namespace std::chrono;
  const auto dt = rclcpp::Duration::from_seconds(.005);
  const auto read = [&] { return driver.read(rclcpp::Time(0), dt); };
  const auto write = [&] { return driver.write(rclcpp::Time(0), dt); };
  auto state = driver.export_state_interfaces();
  auto controls = driver.export_command_interfaces();
  std::vector<std::string> all;
  for (auto &handle : controls) all.push_back(handle.get_name());
  require(all.size() == 60, "expected twelve actuators and five interfaces per actuator");
  const auto sensor = std::find_if(state.begin(), state.end(), [](const auto &handle) {
    return handle.get_name() == "imu_sensor/orientation.w";
  });
  require(sensor != state.end(), "missing sensor handle");
  auto sample_mode = [&](int value) { mode = value; std::this_thread::sleep_for(milliseconds(40)); };
  require(driver.on_configure({}) == Callback::SUCCESS, "initial configure failed");
  change_encoder_coordinates(2);
  require(driver.on_activate({}) == Callback::SUCCESS, "initial supported homing failed");
  require(read() == Return::OK && std::abs(state[0].get_value()) < .001, "joint zero not established");
  require(driver.prepare_command_mode_switch(all, {}) == (bench ? Return::ERROR : Return::OK), "initial claim gate");
  if (!bench) {
    require(driver.perform_command_mode_switch(all, {}) == Return::OK && read() == Return::OK,
            "new cold owner could not start");
    require(driver.prepare_command_mode_switch(all, {}) == Return::OK, "pre-fault pending switch failed");
  }
  for (auto &handle : controls) handle.set_value(1.0);
  write();
  sent.clear();
  const auto duration = seconds(bench ? 1 : 2);
  const auto fault_at = start_dropout(bench ? freeze_gyro : freeze_quat);
  while (true) {
    sent.clear();
    const auto before_read = steady_clock::now();
    const auto result = read();
    if (!std::isfinite(sensor->get_value())) {
      require(result == (bench ? Return::OK : Return::ERROR), "wrong fault disposition");
      break;
    }
    require(result == Return::OK && before_read - fault_at <= pupper_imu::max_receipt_age,
            "accepted a report after its freshness limit");
    std::this_thread::sleep_for(milliseconds(5));
  }
  const auto trip_ms = duration_cast<milliseconds>(steady_clock::now() - fault_at).count();
  require(trip_ms >= 100, "watchdog faulted while the frozen component was still fresh");
  neutral_only();
  if (!bench) {
    require(driver.on_error({}) == Callback::SUCCESS, "error could not return to unconfigured");
    require(driver.on_activate({}) == Callback::ERROR, "unconfigured hardware armed itself");
  }
  while (steady_clock::now() - fault_at < duration) {
    // Try to restore an old command on both the ROS and wire side.
    for (auto &handle : controls) handle.set_value(1.0);
    command.kp_hip[0] = 3;
    command.flags[0] = 1;
    require(write() == Return::OK, "failed neutral write");
    require(read() == (bench ? Return::OK : Return::ERROR), "stale state changed during outage");
    std::this_thread::sleep_for(milliseconds(5));
  }
  neutral_only();
  require(std::isnan(sensor->get_value()), "stale orientation republished as valid");
  const auto restored_at = steady_clock::now();
  std::cout << "dropout_ms=" << duration_cast<milliseconds>(restored_at - fault_at).count()
            << " watchdog_observed_ms=" << trip_ms << " neutral_packets=" << sent.size() << '\n';
  sample_mode(0);
  require(read() == (bench ? Return::OK : Return::ERROR), "IMU alone changed recovery authority");
  neutral_only();
  if (bench) {
    const auto deadline = steady_clock::now() + seconds(4);
    while (std::isnan(sensor->get_value()) && steady_clock::now() < deadline) {
      require(read() == Return::OK, "bench session lost");
      std::this_thread::sleep_for(milliseconds(5));
    }
    require(std::isfinite(sensor->get_value()), "stationary telemetry failed to recover");
    require(driver.prepare_command_mode_switch(all, {}) == Return::ERROR, "bench walker allowed");
    require(driver.perform_command_mode_switch(all, {}) == Return::ERROR, "bench kick allowed");
    require(steady_clock::now() - restored_at >= pupper_imu::stationary_window, "bench telemetry resumed early");
    require_all_homed(state);
    std::cout << "same_driver=1 calibrated_joints=12 telemetry_resumed_ms="
              << duration_cast<milliseconds>(steady_clock::now() - restored_at).count() << '\n';
    require(driver.on_activate({}) == Callback::SUCCESS, "park reactivation failed");
    neutral_only();
    return;
  }
  change_encoder_coordinates(6); // simulate every encoder changing after a board reset
  require(driver.on_configure({}) == Callback::SUCCESS, "could not reuse driver in same process");
  require(read() == Return::OK, "inactive read failed");
  require(driver.prepare_command_mode_switch(all,{}) == Return::ERROR, "claim accepted before supported home");
  neutral_only();
  sent.clear();
  const auto started = steady_clock::now();
  require(driver.on_activate({}) == Callback::SUCCESS, "supported rehome failed");
  const auto elapsed = duration_cast<milliseconds>(steady_clock::now()-started).count();
  require(elapsed >= 3000, "supported rehome skipped stationary gate");
  require_all_homed(state);
  sent.clear();
  for (auto &handle : controls) handle.set_value(1.0);
  write();
  require(read() == Return::OK && std::isfinite(sensor->get_value()), "fresh telemetry not available");
  neutral_only();
  require(driver.perform_command_mode_switch(all, {})==Return::ERROR, "queued pre-fault switch armed driver");
  require(driver.prepare_command_mode_switch({all.front()}, {})==Return::ERROR, "partial ownership rearmed driver");
  require(driver.prepare_command_mode_switch(all, {})==Return::OK &&
          driver.perform_command_mode_switch(all, {})==Return::OK, "explicit post-home claim rejected");
  require(driver.perform_command_mode_switch({}, all) == Return::OK, "new Stop rejected during rearm");
  require(read() == Return::OK, "Stop lost fresh feedback");
  neutral_only();
  require(driver.perform_command_mode_switch(all, {}) == Return::ERROR, "canceled claim revived");
  require(driver.prepare_command_mode_switch(all, {}) == Return::OK &&
          driver.perform_command_mode_switch(all, {}) == Return::OK, "fresh claim after Stop failed");
  for (auto &handle : controls) require(handle.get_value() == 0, "old nonzero command survived");
  write();
  require(read() == Return::OK, "fresh zero activation failed");
  spi_command_t enable_only{};
  for (auto &flag : enable_only.flags) flag = 1;
  require(std::memcmp(&sent.back(), &enable_only, sizeof(enable_only)) == 0, "first enabled packet replayed a command");
  controls[3].set_value(.25); // fresh kp after rearm
  require(write() == Return::OK && read() == Return::OK && sent.back().kp_abad[0] == .25f,
          "fresh post-rearm command did not reach simulated wire");
  sent.clear();
  require(driver.perform_command_mode_switch({}, all) == Return::OK, "Stop of running owner failed");
  for (auto &handle : controls) handle.set_value(1.0);
  require(write() == Return::OK && read() == Return::OK, "post-Stop callback failed");
  neutral_only();
  require(driver.prepare_command_mode_switch({all.front()}, {}) == Return::ERROR,
          "partial claim revived old owner after Stop");
  std::cout << "same_driver=1 changed_encoder_zeros_remeasured=12 supported_rehome_ms=" << elapsed << '\n';
}

void resource_manager_case(hardware_interface::HardwareInfo &info, bool strict) {
  using Return = hardware_interface::return_type;
  using namespace std::chrono;
  using State = lifecycle_msgs::msg::State;
  int argc = 0;
  rclcpp::init(argc, nullptr);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  hardware_interface::ResourceManager manager(clock, rclcpp::get_logger("fc_memory_hardware"));
  info.rw_rate = 500;
  manager.import_component(std::make_unique<control_board_hardware_interface::ControlBoardHardwareInterface>(), info);
  auto status = [&] { return manager.get_components_status().at(info.name).state.id(); };
  rclcpp_lifecycle::State active(State::PRIMARY_STATE_ACTIVE, "active");
  require(status() == State::PRIMARY_STATE_UNCONFIGURED, "manager import state differed");
  require(manager.set_component_state(info.name, active) == Return::OK, "manager could not activate");
  auto all = manager.available_command_interfaces();
  require(all.size() == 60, "manager did not register all twelve actuators");
  require(manager.prepare_command_mode_switch(all, {}) && manager.perform_command_mode_switch(all, {}),
          "manager rejected pre-fault controller switch");
  const auto dt = rclcpp::Duration::from_seconds(.002);
  const auto start_loss = start_dropout(freeze_both);
  hardware_interface::HardwareReadWriteStatus fault;
  do {
    std::this_thread::sleep_for(milliseconds(5));
    sent.clear();
    fault = manager.read(clock->now(), dt);
    require(steady_clock::now() - start_loss < seconds(1), "manager never detected receipt expiry");
  } while (fault.result == Return::OK);
  const auto trip_ms = duration_cast<milliseconds>(steady_clock::now() - start_loss).count();
  std::cout << "manager_read_result=" << int(fault.result) << " lifecycle_after_read=" << int(status()) << " failed_count=" << fault.failed_hardware_names.size() << "\n";
  require(fault.result == Return::ERROR && fault.failed_hardware_names == std::vector<std::string>{info.name},
          "manager did not flag the failed hardware");
  require(status() == State::PRIMARY_STATE_UNCONFIGURED, "manager did not return driver to unconfigured");
  neutral_only();
  require(manager.available_command_interfaces().empty(), "failed hardware still advertised commands");
  do {
    manager.write(clock->now(), dt);
    manager.read(clock->now(), dt);
    require(status() == State::PRIMARY_STATE_UNCONFIGURED, "manager reactivated during loss");
    std::this_thread::sleep_for(milliseconds(5));
  } while (steady_clock::now() - start_loss < seconds(3));
  const auto loss_ms = duration_cast<milliseconds>(steady_clock::now() - start_loss).count();
  mode = 0;
  std::this_thread::sleep_for(milliseconds(40));
  manager.read(clock->now(), dt);
  manager.write(clock->now(), dt);
  require(status() == State::PRIMARY_STATE_UNCONFIGURED && manager.available_command_interfaces().empty(),
          "fresh reports alone armed unavailable hardware");
  neutral_only();
  std::cout << "profile=" << (strict ? "strict" : "supported_rehome") << " dropout_ms=" << loss_ms
            << " watchdog_observed_ms=" << trip_ms << " interfaces_withdrawn=" << all.size() << '\n';
  const auto start = steady_clock::now();
  require(manager.set_component_state(info.name, active) == Return::OK, "real manager could not reconfigure and home existing system");
  const auto elapsed = duration_cast<milliseconds>(steady_clock::now()-start).count();
  sent.clear();
  require(manager.read(clock->now(), dt).result == Return::OK && status() == State::PRIMARY_STATE_ACTIVE,
          "manager did not recover feedback");
  if (!strict) neutral_only();
  require(manager.prepare_command_mode_switch(all,{}) && manager.perform_command_mode_switch(all,{}),
          "manager rejected explicit new ownership");
  require(manager.read(clock->now(),dt).result == Return::OK && sent.back().flags[0] == 1,
          "manager failed to send newly authorized enable");
  std::cout << "resource_manager=1 same_import=1 recovered_active_ms=" << elapsed << '\n';
}

// Exercise the normal ResourceManager read/write and switch entry points. Only the
// 100 Hz IMU and the physical SPI exchange are replaced by memory interfaces.
void damping_case(hardware_interface::HardwareInfo &info, const std::string &scenario) {
  using namespace std::chrono;
  using Return = hardware_interface::return_type;
  using State = lifecycle_msgs::msg::State;
  int argc = 0;
  rclcpp::init(argc, nullptr);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  hardware_interface::ResourceManager manager(clock, rclcpp::get_logger("fc_supported_bench"));
  info.hardware_parameters["fc_imu_recovery"] = "supported_damping";
  info.rw_rate = 500;
  recovery_joints(info);
  manager.import_component(std::make_unique<control_board_hardware_interface::ControlBoardHardwareInterface>(), info);
  rclcpp_lifecycle::State active(State::PRIMARY_STATE_ACTIVE, "active");
  require(manager.set_component_state(info.name, active) == Return::OK, "supported home failed");
  const auto all = manager.available_command_interfaces();
  std::vector<std::string> lease;
  for (const auto &name : all)
    if (name.substr(name.rfind('/')+1) == "effort") lease.push_back(name);
  require(all.size()==60 && lease.size()==12, "wrong board handles");
  const auto dt = rclcpp::Duration::from_seconds(.005);
  auto read = [&] {return manager.read(clock->now(), dt).result;};
  require(read() == Return::OK, "idle neutral exchange");
  require(!manager.prepare_command_mode_switch(all,{}), "gait acquired bench hardware");
  require(!manager.prepare_command_mode_switch({lease.front()},{}), "partial bench lease");
  require(manager.prepare_command_mode_switch(lease,{}) && manager.perform_command_mode_switch(lease,{}), "bench cue failed");
  for (size_t leg = 0; leg < 4; ++leg) {
    data.qd_abad[leg] = 0.5f;
    data.qd_hip[leg] = -2.5f;
    data.qd_knee[leg] = 0;
  }
  require(read() == Return::OK, "first enable failed");
  spi_command_t expected{};
  for (auto &flag : expected.flags) flag=1;
  require(std::memcmp(&sent.back(),&expected,sizeof(expected))==0, "first enable carried torque");
  const float kd=(.50 - 225.0/4095)/(65.+130./4095);
  for (size_t leg = 0; leg < 4; ++leg)
    expected.kd_abad[leg]=expected.kd_hip[leg]=expected.kd_knee[leg]=kd;
  const bool stop = scenario=="damping_stop", bad_spi=scenario=="damping_spi", bad_q=scenario=="damping_joint";
  const bool gap=scenario=="damping_gap", batch=scenario=="damping_batch", flap=scenario=="damping_flap";
  const auto duration = scenario=="damping_gyro" ? seconds(1) : scenario=="damping_quat" ? seconds(2) : seconds(3);
  const auto lost = start_dropout(scenario=="damping_gyro" ? freeze_gyro : scenario=="damping_quat" ? freeze_quat : freeze_both);
  auto orientation=manager.claim_state_interface("imu_sensor/orientation.w");
  milliseconds trip{0}, disabled{0};
  size_t powered=0;
  bool injected=false, recovered=false;
  while (true) {
    const auto elapsed=steady_clock::now()-lost;
    if (elapsed >= duration && duration<seconds(3)) mode=0;
    if (flap && elapsed>milliseconds(200)) {
      // Advancing gyro reports alone cannot clear or renew the original outage.
      mode=freeze_quat;
    }
    if (trip.count() && !injected && (bad_spi || bad_q || stop || gap || batch)) {
      if (gap) std::this_thread::sleep_for(milliseconds(70));
      if (batch) {mode=6; std::this_thread::sleep_for(milliseconds(20));}
      if (bad_spi) spi_replies=1;
      if (bad_q) data.qd_knee[2]=std::numeric_limits<float>::infinity();
      if (stop) {
        sent.clear();
        require(manager.prepare_command_mode_switch({},lease) && manager.perform_command_mode_switch({},lease),
                "physical gateway ownership release failed");
        neutral_only(); // synchronous at the switch, no future IMU sample required
      }
      injected=true;
    }
    command.tau_knee_ff[0]=99; command.kp_abad[3]=99; command.q_des_hip[2]=99;
    manager.write(clock->now(),dt);
    const auto result=read();
    if (result==Return::ERROR) {
      disabled=duration_cast<milliseconds>(steady_clock::now()-lost);
      spi_command_t zero{};
      require(std::memcmp(&sent.back(),&zero,sizeof(zero))==0,"fault left powered packet");
      break;
    }
    require(result==Return::OK && std::memcmp(&sent.back(),&expected,sizeof(expected))==0,
            "unsupported target, gain, effort or flag reached wire");
    if (!std::isfinite(orientation.get_value())) {
      if (!trip.count()) trip=duration_cast<milliseconds>(steady_clock::now()-lost);
      ++powered;
    }
    if (duration<seconds(3) && trip.count() && elapsed>=duration && std::isfinite(orientation.get_value())) {
      require(!manager.prepare_command_mode_switch(all,{}), "fresh IMU allowed gait");
      require(manager.get_components_status().at(info.name).state.id()==State::PRIMARY_STATE_ACTIVE,
              "bench lost lease on short dropout");
      recovered=true;
      disabled=duration_cast<milliseconds>(steady_clock::now()-lost);
      break;
    }
    require(elapsed < seconds(4), "bench window exceeded");
    std::this_thread::sleep_for(milliseconds(5));
  }
  require(powered>0 && trip>=milliseconds(100), "no real receipt-age transition");
  if (recovered) {
    sent.clear();
    require(manager.prepare_command_mode_switch({},lease) && manager.perform_command_mode_switch({},lease),
            "post-recovery Stop failed");
    neutral_only();
    std::cout<<"bench="<<scenario<<" trip_ms="<<trip.count()<<" balance_restored_ms="<<disabled.count()
             <<" powered_loss_packets="<<powered<<" host_kd="<<kd<<" continuous_bench_lease=1"<<std::endl;
    return;
  }
  require(manager.get_components_status().at(info.name).state.id()==State::PRIMARY_STATE_UNCONFIGURED &&
          manager.available_command_interfaces().empty(), "expired bench retained authority");
  mode=0; sent.clear();
  std::this_thread::sleep_for(milliseconds(30));
  read(); manager.write(clock->now(),dt);
  require(manager.available_command_interfaces().empty(), "new report rearmed a lease");
  for(const auto &packet:sent){spi_command_t zero{};require(std::memcmp(&packet,&zero,sizeof(zero))==0,"queued action revived");}
  std::cout<<"bench="<<scenario<<" trip_ms="<<trip.count()<<" disabled_ms="<<disabled.count()
           <<" powered_loss_packets="<<powered<<" host_kd="<<kd<<" handles_withdrawn="<<all.size()<<std::endl;
}

std::array<uint64_t,24> receipt() {
  std::array<uint64_t,24> words{};
  std::ifstream f(receipt_path,std::ios::binary);f.read(reinterpret_cast<char *>(words.data()),sizeof(words));
  require(f && words[0]>0 && !(words[0]&1) && words[1]==0x314d494346505550ULL,"invalid native receipt");
  return words;
}

void configure_mechanical_stops(const hardware_interface::HardwareInfo &info) {
  float *positions[] = {data.q_abad, data.q_hip, data.q_knee};
  for (const auto &joint : info.joints) {
    const auto &p = joint.parameters;
    const int axis = std::stoi(p.at("can_id")) - 1, channel = std::stoi(p.at("can_channel")) - 1;
    require(axis >= 0 && axis < 3 && channel >= 0 && channel < 4, "invalid mechanical fixture CAN mapping");
    const bool positive = std::stod(p.at("homing_velocity")) > 0;
    mechanical_stops[axis][channel] = {std::stof(p.at("homed_position")), positive};
    positions[axis][channel] = std::stof(p.at(positive ? "position_min" : "position_max"));
  }
}

struct RecordedState {
  double seconds;
  std::array<double,12> position{}, velocity{};
};

std::vector<RecordedState> load_trace(const std::string &path, const hardware_interface::HardwareInfo &info) {
  std::ifstream csv(path);
  std::string line;
  require(bool(std::getline(csv, line)), "standing recording missing");
  std::string header = "seconds_from_stand";
  for (const auto &joint : info.joints) header += "," + joint.name;
  for (const auto &joint : info.joints) header += ",velocity_" + joint.name;
  require(line == header, "recording joint order does not match enrolled description");
  std::vector<RecordedState> rows;
  while (std::getline(csv, line)) {
    std::istringstream row(line);
    const auto number = [&] { std::string field; require(bool(std::getline(row, field, ',')), "short trace row");
                              return std::stod(field); };
    RecordedState state{number()};
    for (auto &q : state.position) q = number();
    for (auto &qd : state.velocity) qd = number();
    require(rows.empty() || state.seconds > rows.back().seconds, "recording clock stalled");
    rows.push_back(state);
  }
  require(rows.size() > 50 && rows.front().seconds < 0 && rows.back().seconds > 2, "standing recording incomplete");
  return rows;
}

void enrolled_standing(hardware_interface::ResourceManager &manager,
                       const hardware_interface::HardwareInfo &info, rclcpp::Clock &clock) {
  using namespace std::chrono;
  using Return = hardware_interface::return_type;
  namespace ci = controller_interface;
  const auto dt = rclcpp::Duration::from_seconds(.002);
  const auto all = manager.available_command_interfaces();
  require(all.size() == 60, "expected a complete twelve-joint claim");
  const auto read = [&] { return manager.read(clock.now(), dt).result; };
  require(read() == Return::OK && receipt()[4] > 0, "course Home failed");
  const auto home = receipt()[4];
  const auto cycle = [&] { require(read() == Return::OK && receipt()[4] == home, "unexpected board fault or Home");
                           std::this_thread::sleep_for(milliseconds(2)); };
  const auto until = steady_clock::now() + milliseconds(650);
  while (steady_clock::now() < until) cycle();
  const auto stand = [&] {
    require(manager.prepare_command_mode_switch(all,{}) && manager.perform_command_mode_switch(all,{}),
            "fresh field Stand claim refused");
    sent.clear(); cycle();
    spi_command_t first{};
    for (auto &flag : first.flags) flag = 1;
    require(sent.size() == 1 && std::memcmp(&sent.back(), &first, sizeof first) == 0,
            "initial claim did not start with a blank enabled exchange");
  };
  const auto stop = [&] {
    require(manager.prepare_command_mode_switch({},all) && manager.perform_command_mode_switch({},all),
            "release failed");
    sent.clear(); cycle(); neutral_only();
    require(!manager.perform_command_mode_switch(all,{}), "Stop allowed an unprepared rearm");
  };
  const char *trace_file = std::getenv("FC_TEST_STAND_RECORDING");
  if (!trace_file) {
    stand(); stop(); stand(); stop();
    std::cout << "course Home and two independently prepared Stand/Stop cycles preserved generation " << home << '\n';
    return;
  }
  const bool stuck = std::getenv("FC_TEST_STUCK_REAR") != nullptr;
  const auto trace = load_trace(trace_file, info);
  const char *config = std::getenv("FC_TEST_WALK_CONFIG");
  require(config, "supply selected FC walker configuration");
  const char *description = std::getenv("FC_TEST_FIELD_URDF");
  std::ifstream urdf(description);
  const std::string xml{std::istreambuf_iterator<char>(urdf), std::istreambuf_iterator<char>()};
  pluginlib::ClassLoader<ci::ControllerInterface> loader("controller_interface","controller_interface::ControllerInterface");
  auto walker = loader.createSharedInstance("neural_controller/NeuralController");
  auto options = walker->define_custom_node_options();
  options.arguments({"--ros-args","--params-file",config});
  require(walker->init("gemini_walk", xml, 520, "", options) == ci::return_type::OK, "walker init failed");
  require(walker->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "walker configure failed");
  // Home used finite travel against twelve mechanical stops. From this point, all
  // encoder positions and velocities are independent of every outgoing command.
  follow_return = false;
  float *position[] = {data.q_abad,data.q_hip,data.q_knee};
  float *velocity[] = {data.qd_abad,data.qd_hip,data.qd_knee};
  std::vector<std::pair<int,int>> mapping;
  for (const auto &joint : info.joints) {
    const auto &p=joint.parameters;
    mapping.emplace_back(std::stoi(p.at("can_id"))-1,std::stoi(p.at("can_channel"))-1);
  }
  auto inject = [&](size_t first,size_t second,double blend) {
    for (size_t i=0;i<mapping.size();++i) {
      const auto [a,c]=mapping[i];
      const bool rear=stuck && (info.joints[i].name=="leg_back_r_3" || info.joints[i].name=="leg_back_l_3");
      position[a][c]=rear ? (info.joints[i].name=="leg_back_r_3" ? .710763 : -.703134) :
                trace[first].position[i]+blend*(trace[second].position[i]-trace[first].position[i]);
      velocity[a][c]=rear ? 0 : trace[first].velocity[i]+blend*(trace[second].velocity[i]-trace[first].velocity[i]);
    }
  };
  // Bridge Home to the pre-claim recorded pose while disabled. Position chatter
  // on the historical start is real; the mechanical reference uses the driver's
  // synthetic Home coordinates until this passive interpolation is complete.
  for (int k=0;k<120;++k) {
    for (size_t i=0;i<mapping.size();++i) {
      const auto [a,c]=mapping[i];
      const double target=stuck && i==8 ? .710763 : stuck && i==11 ? -.703134 : trace.front().position[i];
      position[a][c] += std::clamp(target-position[a][c],-.005,.005);
    }
    cycle();
  }
  require(!manager.perform_command_mode_switch(all,{}), "unprepared Stand was accepted");
  stand();
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  std::vector<hardware_interface::LoanedStateInterface> states;
  for (const auto &key : all) commands.emplace_back(manager.claim_command_interface(key));
  for (const auto &key : manager.available_state_interfaces()) states.emplace_back(manager.claim_state_interface(key));
  walker->assign_interfaces(std::move(commands),std::move(states));
  require(walker->get_node()->activate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "walker activation failed");
  size_t index=0, updates=0;
  double elapsed=0;
  const auto started=steady_clock::now();
  bool tripped=false;
  while ((elapsed=duration<double>(steady_clock::now()-started).count()) < 2.12) {
    while (index+2<trace.size() && trace[index+1].seconds<elapsed) ++index;
    const auto &a=trace[index], &b=trace[index+1];
    inject(index,index+1,std::clamp((elapsed-a.seconds)/(b.seconds-a.seconds),0.,1.));
    if (read()!=Return::OK) { tripped=true; break; }
    require(walker->update(walker->get_node()->now(),dt)==ci::return_type::OK, "native walker update failed");
    manager.write(clock.now(),dt);
    ++updates;
    std::this_thread::sleep_for(milliseconds(2));
  }
  require(updates>1, "native walker did not produce its initialization trajectory");
  if (stuck) {
    const spi_command_t zero{};
    require(tripped && elapsed<2.0 && receipt()[7]==4 && !sent.empty() &&
            std::memcmp(&sent.back(),&zero,sizeof zero)==0, "frozen rear reached gait or stayed enabled");
  } else {
    require(!tripped && receipt()[4]==home,"healthy historical trajectory refused");
    require(std::abs(data.q_knee[3]+.52)<.20 && std::abs(data.q_knee[2]-.52)<.20,
            "rear knees never reached the recorded standing pose");
    require(walker->get_node()->deactivate().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
            "walker deactivation failed");
    stop(); stand(); stop();
  }
  std::cout << "recording=" << trace_file << " stuck=" << stuck << " elapsed_s=" << elapsed
            << " driver_result=" << (tripped ? "neutral_fault":"standing_and_stopped")
            << " walker_updates=" << updates << " home=" << home << std::endl;
}

void field_redundant_case(hardware_interface::HardwareInfo &info,const std::string &scenario) {
  using namespace std::chrono;
  using Return=hardware_interface::return_type;
  using State=lifecycle_msgs::msg::State;
  char directory[]="/tmp/fc-imu-XXXXXX";
  require(mkdtemp(directory)!=nullptr,"fixture directory");
  receipt_path=std::string(directory)+"/fc-board-health.bin";
  setenv("PUPPER_FC_BOARD_HEALTH_PATH",receipt_path.c_str(),1);
  int argc=0;rclcpp::init(argc,nullptr);
  auto clock=std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  hardware_interface::ResourceManager manager(clock,rclcpp::get_logger("field_imu_fixture"));
  info.hardware_parameters["fc_imu_recovery"]="field_redundant_imu";
  const bool enrolled = scenario == "floor_enrolled_standing";
  if (enrolled) {
    const char *path = std::getenv("FC_TEST_FIELD_URDF");
    require(path, "supply the field description generated by the FC launcher");
    std::ifstream urdf(path);
    const std::string xml{std::istreambuf_iterator<char>(urdf), std::istreambuf_iterator<char>()};
    const auto systems = hardware_interface::parse_control_resources_from_urdf(xml);
    require(systems.size()==1 && systems.front().joints.size()==12, "expected the enrolled robot's twelve axes");
    info = systems.front();
    require(info.hardware_parameters.at("fc_imu_recovery") == "field_redundant_imu", "wrong board profile");
    follow_return = true;
  } else {
    recovery_joints(info);
  }
  info.rw_rate=500;
  manager.import_component(std::make_unique<control_board_hardware_interface::ControlBoardHardwareInterface>(),info);
  rclcpp_lifecycle::State active(State::PRIMARY_STATE_ACTIVE,"active");
  if (enrolled) {
    configure_mechanical_stops(info);
    require(manager.set_component_state(info.name, active) == Return::OK, "enrolled supported Home failed");
    enrolled_standing(manager, info, *clock);
    std::filesystem::remove_all(directory);
    return;
  }
  if (scenario=="floor_home_reset" || scenario=="floor_home_bias") {
    std::thread reset;
    if (scenario=="floor_home_reset")
      reset=std::thread([]{std::this_thread::sleep_for(milliseconds(180));aux_fault=4;});
    else mode=3;
    const auto activated=manager.set_component_state(info.name,active);
    if(reset.joinable()) reset.join();
    require(activated==Return::ERROR && receipt()[4]==0 && receipt()[7]==4,"failed startup completed Home");
    neutral_only();
    std::ifstream trace(std::string(directory)+"/native-startup-trace.csv");
    std::string meta,columns,row,last;
    std::getline(trace,meta); std::getline(trace,columns);
    while(std::getline(trace,row)) last=row;
    require(meta.find("startup_passed=0")!=std::string::npos && columns.find("source_reason,alternate_ready")!=std::string::npos &&
            (scenario=="floor_home_reset" ? last.find(",source_error,0,")!=std::string::npos :
             last.find(",gyro_norm,")!=std::string::npos && last.find(",ready,1,")!=std::string::npos),
            "default failure trace omitted primary/alternate gate evidence");
    std::filesystem::remove_all(directory);
    return;
  }
  if (scenario=="floor_home_late_fault" || scenario=="floor_home_trace") {
    const auto trace_path=std::string(directory)+"/native-startup-trace.csv";
    const bool success=scenario=="floor_home_trace";
    if(success) setenv("PUPPER_IMU_STARTUP_TRACE",trace_path.c_str(),1);
    else lose_during_enable=true;
    require(manager.set_component_state(info.name,active)==(success ? Return::OK : Return::ERROR),"unexpected Home result");
    if(!success) require(!sent.empty() && sent.back().flags[0]==0 && receipt()[7]==4,"late fault without neutral");
    std::ifstream trace(trace_path);
    std::string meta; std::getline(trace,meta);
    require(meta.find("startup_passed=1")!=std::string::npos &&
            meta.find(success ? "later_home_fault=0" : "later_home_fault=1")!=std::string::npos,
            "successful primary gate trace lost on explicit request or subsequent failure");
    std::filesystem::remove_all(directory);
    return;
  }
  const bool delayed_backup=scenario=="floor_first_claim" || scenario=="floor_first_packet";
  if (delayed_backup) aux_fault=6;
  require(manager.set_component_state(info.name,active)==Return::OK,"field candidate supported Home");
  auto all=manager.available_command_interfaces();
  const auto dt=rclcpp::Duration::from_seconds(.005);
  auto read=[&] {return manager.read(clock->now(),dt).result;};
  auto mode_handle=manager.claim_state_interface("imu_sensor/fc_balance_mode");
  auto incident=manager.claim_state_interface("imu_sensor/fc_balance_incident");
  auto home=manager.claim_state_interface("imu_sensor/fc_balance_generation");
  if (delayed_backup) {
    require(read()==Return::OK,"neutral read after Home");
    const auto home_id=receipt()[4];
    require(home_id>0 && !std::filesystem::exists(std::string(directory)+"/native-startup-trace.csv"),
            "supported Home must complete without default synchronous trace I/O");
    sent.clear();
    require(read()==Return::OK && !manager.prepare_command_mode_switch(all,{}) &&
            !manager.perform_command_mode_switch(all,{}),"unqualified alternate admitted a leg claim");
    neutral_only();
    auto qualify=[&] {
      aux_fault=0;
      const auto end=steady_clock::now()+milliseconds(650);
      while(steady_clock::now()<end) {require(read()==Return::OK,"warmup failed"); std::this_thread::sleep_for(milliseconds(5));}
      require(manager.prepare_command_mode_switch(all,{}),"new qualified evidence did not release prepare");
    };
    qualify();
    std::this_thread::sleep_for(milliseconds(45));
    require(!manager.perform_command_mode_switch(all,{}),"expired cached proof allowed perform");
    require(read()==Return::OK && !manager.perform_command_mode_switch(all,{}),"expired prepare reused");
    qualify();
    if (scenario=="floor_first_packet") {
      require(manager.perform_command_mode_switch(all,{}),"fresh claim refused");
      aux_fault=6; std::this_thread::sleep_for(milliseconds(20)); sent.clear();
      require(read()==Return::ERROR && receipt()[7]==4,"lost agreement allowed first packet");
      neutral_only();
    } else {
      aux_fault=6; std::this_thread::sleep_for(milliseconds(20));
      require(read()==Return::OK && !manager.perform_command_mode_switch(all,{}),"low quality allowed perform");
      neutral_only();
      qualify();
      // Give the same advancing primary a short known batch-age budget. Its
      // proof expires with no read between prepare and renewal on fresh data.
      aux_fault=7;
      std::this_thread::sleep_for(milliseconds(15));
      const auto until=steady_clock::now()+milliseconds(100);
      bool prepared=false;
      while (!prepared && steady_clock::now()<until) {
        require(read()==Return::OK,"late batch receipt rejected primary");
        prepared=manager.prepare_command_mode_switch(all,{});
        if (!prepared) std::this_thread::sleep_for(milliseconds(1));
      }
      require(prepared,"fixture never observed a qualified short-life batch");
      aux_fault=0;
      std::this_thread::sleep_for(milliseconds(16));
      require(read()==Return::OK && !manager.perform_command_mode_switch(all,{}),
              "a newly healthy read revived a previously expired claim");
      require(manager.prepare_command_mode_switch(all,{}) && manager.perform_command_mode_switch(all,{}) &&
              read()==Return::OK,"fresh qualified claim rejected");
      spi_command_t first{};for(auto &flag:first.flags)flag=1;
      require(std::memcmp(&sent.back(),&first,sizeof first)==0 && receipt()[4]==home_id,
              "first packet contained old commands or changed Home");
    }
    std::filesystem::remove_all(directory);
    return;
  }
  require(read()==Return::OK && manager.prepare_command_mode_switch(all,{}) &&
          manager.perform_command_mode_switch(all,{}) && read()==Return::OK,"field start");
  const auto archive_receipt=[&](const char *phase){
    const char *dir=std::getenv("FC_TEST_RECEIPT_DIRECTORY");
    if (dir && scenario=="floor_gyro1")
      std::filesystem::copy_file(receipt_path,std::string(dir)+"/"+phase+".bin",
                                 std::filesystem::copy_options::overwrite_existing);
  };
  const auto initial=receipt();
  archive_receipt("primary");
  require(initial[4]>0 && home.get_value()==static_cast<double>(initial[4]) &&
          initial[7]==1 && initial[15]==5 && initial[16]==2 && initial[21]==3 && initial[22]==3,"invalid primary provenance");
  const bool returning=scenario=="floor_gyro1" || scenario=="floor_quat2" || scenario=="floor_twice" || scenario=="floor_disagree";
  const int duration_ms=scenario=="floor_quat2"?2000:1000;
  for (int cycle=1;cycle<=(scenario=="floor_twice"?2:1);++cycle) {
    // The manager must keep receiving timely sensor and joint exchanges through an actual running activation.
    const auto prime=steady_clock::now();
    while (steady_clock::now()-prime<milliseconds(650)) {
      require(read()==Return::OK,"primary warm-up");std::this_thread::sleep_for(milliseconds(5));
    }
    const auto lost=start_dropout(scenario=="floor_gyro1" || scenario=="floor_twice"?freeze_gyro:
                                   scenario=="floor_quat2"?freeze_quat:freeze_both);
    if(scenario=="floor_total")aux_fault=1;
    int64_t held_at=-1;
    bool restored=false;
    while (true) {
      const auto elapsed=duration_cast<milliseconds>(steady_clock::now()-lost).count();
      if(returning && elapsed>=duration_ms){mode=0;if(scenario=="floor_disagree")aux_fault=2;}
      if((scenario=="floor_queued" || scenario=="floor_rehome") && held_at>=0) aux_fault=3;
      if(scenario=="floor_spi" && held_at>=0) spi_replies=1;
      if(scenario=="floor_stop" && held_at>=0) {
        sent.clear();
        require(manager.prepare_command_mode_switch({},all) && manager.perform_command_mode_switch({},all),
                "physical Stop release refused");
        neutral_only();
        require(receipt()[7]==4 && !manager.prepare_command_mode_switch(all,{}),"Stop did not invalidate old authority");
      }
      if(scenario=="floor_jump" && held_at>=0) data.q_abad[0]+=1.f;
      if(scenario=="floor_reset" && held_at>=0) aux_fault=4;
      if(scenario=="floor_gap" && held_at>=0) aux_fault=5;
      manager.write(clock->now(),dt);
      const auto result=read();
      const auto health=receipt();
      require(health[3]==initial[3] && health[4]==initial[4],"instance or Home changed through outage");
      if(result==Return::ERROR){
        require((!returning || scenario=="floor_disagree") && health[7]==4 && health[5]>initial[5] &&
                manager.available_command_interfaces().empty(),"terminal outcome or handle withdrawal wrong");
        spi_command_t zero{};require(std::memcmp(&sent.back(),&zero,sizeof zero)==0,"terminal left enabled");
        if(scenario=="floor_total")require(held_at<0 && elapsed<200,"total outage accepted backup");
        if(scenario=="floor_long")require(held_at>=0 && elapsed>=2990,"long hold duration");
        std::cout<<"floor="<<scenario<<" held_ms="<<held_at<<" terminal_ms="<<elapsed<<" incident="<<health[6]<<" withdrawn=60"<<std::endl;
        break;
      }
      if(health[7]==2){
        if(held_at<0){
          held_at=elapsed;
          archive_receipt("hold");
          require(!manager.prepare_command_mode_switch(all,all),"lease handoff during loss");
          spi_command_t blank{};for(auto &flag:blank.flags)flag=1;
          require(std::memcmp(&sent.back(),&blank,sizeof blank)==0,"preincident target crossed hold boundary");
        }
        require(health[15]==8 && health[16]==7 && health[6]==static_cast<unsigned>(cycle) &&
                incident.get_value()==cycle && mode_handle.get_value()==2 &&
                health[8]-health[9]+health[11]<=40000000 && health[8]-health[10]+health[12]<=40000000,
                "backup authenticity or freshness");
      }else if(held_at>=0 && health[7]==3){
        require(returning && mode==0 && elapsed>=duration_ms+100 && incident.get_value()==cycle &&
                mode_handle.get_value()==3 && health[6]==static_cast<unsigned>(cycle) && health[15]==5 && health[16]==2,
                "restoration reused old data or lost incident");
        require(manager.available_command_interfaces().size()==60,"lost board authority despite credible backup");
        std::cout<<"floor="<<scenario<<" cycle="<<cycle<<" held_ms="<<held_at<<" restored_ms="<<elapsed
                 <<" same_home=1 same_board=1 authority_still_paused=1"<<std::endl;
        archive_receipt("restored");
        restored=true;break;
      }
      require(elapsed<3600,"field hold did not resolve");
      std::this_thread::sleep_for(milliseconds(5));
    }
    if(!restored)break;
  }
  if (scenario=="floor_gyro1") {
    const auto end=steady_clock::now()+milliseconds(160);
    while (steady_clock::now()<end) {
      require(read()==Return::OK && receipt()[7]==3 && mode_handle.get_value()==3 && incident.get_value()==1,
              "restored state reopened ordinary primary authority on same Home");
      std::this_thread::sleep_for(milliseconds(5));
    }
  }
  if (scenario=="floor_rehome") {
    const auto fault=receipt();
    aux_fault=0;mode=0;
    std::this_thread::sleep_for(milliseconds(30));
    require(manager.available_command_interfaces().empty(),"report reopened terminal hardware");
    require(manager.set_component_state(info.name,active)==Return::OK && read()==Return::OK,"fresh deliberate Home failed");
    const auto new_home=receipt();
    require(new_home[3]==initial[3] && new_home[4]>initial[4] && new_home[5]==fault[5] &&
            new_home[6]==0 && new_home[7]==1 && new_home[4]==home.get_value() && incident.get_value()==0,
            "new Home failed to fence old incident and generation");
    std::cout<<"floor=floor_rehome new_home=1 same_board=1 incident_reset=1"<<std::endl;
  }
  std::filesystem::remove_all(directory);
}

void unit_health() {
  using namespace std::chrono;
  auto now=steady_clock::time_point{}+seconds(100);
  auto o=output(now);
  require(!pupper_imu::feedback_error(o,now),"fresh rejected");
  o.gyro_receipt_time-=milliseconds(101);
  require(pupper_imu::feedback_error(o,now),"fresh q hid stale gyro");
  o=output(now);o.quaternion_receipt_time-=milliseconds(101);
  require(pupper_imu::feedback_error(o,now),"fresh gyro hid stale q");
  o=output(now);o.gyro_receipt_time+=milliseconds(1);
  require(pupper_imu::feedback_error(o,now),"future receipt accepted");
  o=output(now);o.gyro.x()=std::numeric_limits<float>::infinity();
  require(pupper_imu::feedback_error(o,now),"infinite gyro accepted");
  o=output(now);o.quat.w()=.5;
  require(pupper_imu::feedback_error(o,now),"bad quaternion norm accepted");
  o=output(now);o.gyro_minimum_age_at_receipt=milliseconds(90);
  require(!pupper_imu::feedback_error(o,now),"90ms report rejected before age limit");
  require(pupper_imu::feedback_error(o,now+milliseconds(20)),"receipt freshened90ms plus20ms cached gyro");
  o=output(now);o.quaternion_minimum_age_at_receipt=seconds(78);
  require(pupper_imu::feedback_error(o,now),"fresh gyro hid queued quaternion");
  o=output(now);o.gyro_minimum_age_at_receipt=microseconds(-1);
  require(pupper_imu::feedback_error(o,now),"negative age bound accepted");
  pupper_imu::StartupWindow queued;
  for(int i=0;i<=400;++i){auto t=now+milliseconds(i*10);auto s=output(t);s.gyro_minimum_age_at_receipt=seconds(78);require(!queued.observe(s,t),"old queue manufactured startup dwell");}
  pupper_imu::StartupWindow stable;
  for(int i=0;i<=300;++i){auto t=now+milliseconds(i*10);auto s=output(t);bool ready=stable.observe(s,t);require(ready==(i==300),"stationary window timing wrong");}
  pupper_imu::StartupWindow bias;
  for(int i=0;i<=1600;++i){auto t=now+milliseconds(i*10);auto s=output(t);s.gyro.x()=.13;require(!bias.observe(s,t),"stationary bias accepted");}
  pupper_imu::StartupWindow drift;
  for(int i=0;i<=1600;++i){auto t=now+milliseconds(i*10);auto s=output(t);s.quat=Eigen::Quaternionf(Eigen::AngleAxisf(i*.0001f,Eigen::Vector3f::UnitX()));require(!drift.observe(s,t),"gravity drift accepted");}
  pupper_imu::StartupWindow yaw;
  for(int i=0;i<=300;++i){auto t=now+milliseconds(i*10);auto sample=output(t);sample.quat=Eigen::Quaternionf(Eigen::AngleAxisf(i*.001f,Eigen::Vector3f::UnitZ()));require(yaw.observe(sample,t)==(i==300),"yaw-only rotation changed gravity gate");}
  pupper_imu::StartupWindow modest_bias;
  for(int i=0;i<=300;++i){auto t=now+milliseconds(i*10);auto sample=output(t);sample.gyro.x()=.0144f;require(modest_bias.observe(sample,t)==(i==300),"allowed gyro mean rejected");}
  pupper_imu::StartupWindow coherent_bias;
  for(int i=0;i<=1600;++i){auto t=now+milliseconds(i*10);auto sample=output(t);sample.gyro.x()=.03f;pupper_imu::StartupObservation d;require(!coherent_bias.observe(sample,t,&d),"coherent gyro mean accepted");if(i==300)require(std::strcmp(d.reason,"gyro_mean")==0,"mean rejection reason missing");if(i==301)require(d.gyro_samples==1,"mean rejection did not clear window");}
  pupper_imu::StartupWindow distinct;
  for(int i=0;i<=600;++i){auto t=now+milliseconds(i*5);auto sample=output(t);sample.gyro.x()=.0144f;sample.gyro_receipt_time=now+milliseconds((i/2)*10);pupper_imu::StartupObservation d;require(distinct.observe(sample,t,&d)==(i==600),"distinct-receipt dwell timing wrong");require(d.gyro_samples==static_cast<std::size_t>(i/2+1),"cached gyro counted twice in mean");}
  pupper_imu::StartupWindow duplicated;
  o=output(now);
  for(int i=0;i<=400;++i)require(!duplicated.observe(o,now+milliseconds(i*10)),"cached sample advanced startup");
  pupper_imu::StartupWindow gap;
  require(!gap.observe(output(now),now),"initial window accepted");
  auto later=now+milliseconds(3100);
  require(!gap.observe(output(later),later),"unobserved scheduler gap counted as dwell");
  pupper_imu::StartupWindow sign;
  for(int i=0;i<=300;++i){auto t=now+milliseconds(i*10);auto s=output(t);if(i%2)s.quat.coeffs()*=-1;require(sign.observe(s,t)==(i==300),"q/-q treated as rotation");}
  pupper_imu::StartupWindow spike;
  for(int i=0;i<=300;++i){auto t=now+milliseconds(i*10);auto s=output(t);if(i==250)s.gyro.x()=.06;require(!spike.observe(s,t),"gyro spike did not restart window");}
}
}
void init_spi(){command={};data={};}
void close_spi(){}
spi_command_t *get_spi_command(){return &command;}
spi_data_t *get_spi_data(){return &data;}
void spi_driver_run() {
  sent.push_back(command);
  if (simulate_empty_wakeup && sent.size() == 1) { data.spi_driver_status = 0; return; }
  if (lose_reply_during_homing && command.kp_abad[0] > 0) spi_replies = 1;
  data.spi_driver_status = spi_replies;
  if (lose_during_enable && command.flags[0]) mode = 1;
  if (follow_without_endstop) {
    for (int i = 0; i < 4; ++i) {
      data.q_abad[i] = command.q_des_abad[i];
      data.q_hip[i] = command.q_des_hip[i];
      data.q_knee[i] = command.q_des_knee[i];
    }
  }
  if (follow_return) {
    float *positions[] = {data.q_abad, data.q_hip, data.q_knee};
    const float *targets[] = {command.q_des_abad, command.q_des_hip, command.q_des_knee};
    const float *gains[] = {command.kp_abad, command.kp_hip, command.kp_knee};
    for (int i = 0; i < 4; ++i) {
      for (int axis = 0; axis < 3; ++axis) {
        if (gains[axis][i] <= 0) continue;
        const auto stop = mechanical_stops[axis][i];
        positions[axis][i] = stop.positive ? std::min(stop.position, targets[axis][i])
                                          : std::max(stop.position, targets[axis][i]);
      }
    }
  }
}
void BNO055::enable_redundant_reports(uint32_t) {}
BNO055::BNO055(int,uint32_t){}
BNO055::~BNO055(){}
BNO055::Output BNO055::sample(){return output(std::chrono::steady_clock::now());}
extern "C" int __real_open(const char *,int,...);
extern "C" int __wrap_open(const char *p,int flags,...){
  if (std::strcmp(p,"/tmp/control_board_hardware.lock")==0) return lock_fd;
  const auto trace=std::filesystem::path(receipt_path).parent_path()/"native-startup-trace.csv";
  require(!receipt_path.empty() && trace==p && flags==(O_CREAT|O_EXCL|O_WRONLY),"unexpected open");
  return __real_open(p,flags,0600);
}
extern "C" int __wrap_open64(const char *p,int f,...){return __wrap_open(p,f);}
extern "C" int __wrap_flock(int fd,int){require(fd==lock_fd,"unexpected flock");return 0;}
extern "C" int __real_close(int);
extern "C" int __wrap_close(int fd){if(fd!=lock_fd && !receipt_path.empty())return __real_close(fd);require(fd==lock_fd,"unexpected close");return 0;}
extern "C" int __wrap_unlink(const char *p){require(std::strcmp(p,"/tmp/control_board_hardware.lock")==0,"unexpected unlink");return 0;}
int main(int argc,char **argv){try{
  require(argc==2,"scenario required");std::string scenario=argv[1];
  if(scenario=="health"){unit_health();std::cout<<"PASS health\n";return 0;}
  clocked_reports = scenario == "supported_standby" || scenario == "supported_rehome" ||
                    scenario == "resource_manager" || scenario == "resource_manager_strict";
  auto driver=std::make_unique<control_board_hardware_interface::ControlBoardHardwareInterface>();
  hardware_interface::HardwareInfo info{};info.is_async=false;info.name="imu_guard_fake";info.type="system";
  hardware_interface::ComponentInfo sensor;sensor.name="imu_sensor";sensor.parameters={{"roll","0"},{"pitch","0"},{"yaw","0"}};info.sensors.push_back(sensor);
  if (scenario.rfind("mechanical_", 0) == 0) {
    recovery_joints(info);
    follow_without_endstop = scenario == "mechanical_no_stop";
    follow_return = scenario == "mechanical_return_valid";
    if (follow_return || scenario == "mechanical_return_frozen") {
      for (auto &joint : info.joints) {
        auto &p = joint.parameters;
        p["position_min"] = "-.25";
        p["position_max"] = ".25";
        p["post_homing_position"] = "-.15";
        p["homing_torque_threshold"] = ".02";
        p["homing_kd"] = "0";
      }
    }
    if (scenario == "mechanical_return_frozen") {
      // All encoder positions stay zero. Only joint7 needs to move after its reference.
      for (auto &joint : info.joints)
        if (joint.name != "joint7") joint.parameters["post_homing_position"] = "0";
    }
    if (follow_without_endstop) {
      for (auto &joint : info.joints) {
        auto &p = joint.parameters;
        p["position_min"] = "-.01";
        p["position_max"] = ".01";
        p["homing_velocity"] = ".1";
        p["homing_torque_threshold"] = ".01";
        p["homing_kd"] = "0";
      }
    }
    if (follow_return) {
      // Optionally exercise the exact installed geometry across every stage.
      if (const auto path = std::getenv("FC_TEST_MECHANICAL_URDF")) {
        std::ifstream input(path);
        require(input.good(), "selected mechanical test URDF unavailable");
        const std::string xml{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
        const auto systems = hardware_interface::parse_control_resources_from_urdf(xml);
        require(systems.size()==1 && systems.front().joints.size()==12, "expected twelve enrolled axes");
        info.joints = systems.front().joints;
      }
    }
    require(driver->on_init(info) == hardware_interface::CallbackReturn::SUCCESS, "mechanical test initialization");
    if (follow_return) configure_mechanical_stops(info);
    if (scenario == "mechanical_missing") spi_replies = 1;
    if (scenario == "mechanical_nonfinite") data.q_abad[0] = std::numeric_limits<float>::infinity();
    lose_reply_during_homing = scenario == "mechanical_lost";
    const auto original_logger = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(capture_fault);
    const auto activated = driver->on_activate({});
    rcutils_logging_set_output_handler(original_logger);
    if (scenario == "mechanical_return_frozen") {
      require(travel_evidence.size() == 1, "return timeout did not identify exactly one blocked joint");
      const auto &line = travel_evidence.front();
      require(line.find("phase=return axis=7 joint=joint7") != std::string::npos &&
              line.find("referenced=1 command_rad=-0.150000 measured_rad=0.000000 rest_rad=-0.150000 ") != std::string::npos &&
              line.find("measured_minus_rest_rad=0.150000 tolerance_rad=0.020000 target_reached=1") != std::string::npos,
              "return diagnostic lost the pre-disable target, encoder position, or tolerance");
      require(mechanical_refusal.find("phase=return first_pending_joint=joint7") != std::string::npos &&
              mechanical_refusal.find("pending_axes=1; neutral sent") != std::string::npos,
              "operator-facing refusal did not identify the return failure");
      std::cout << line << "\n" << mechanical_refusal << "\n";
    }
    if (follow_return) {
      require(activated == hardware_interface::CallbackReturn::SUCCESS, "valid measured Home failed");
      std::cout << "PASS " << scenario << "\n";
      return 0;
    }
    require(activated == hardware_interface::CallbackReturn::ERROR, "invalid mechanical Home succeeded");
    const spi_command_t zero{};
    require(!sent.empty() && std::memcmp(&sent.back(), &zero, sizeof zero) == 0, "mechanical fault did not send neutral");
    if (scenario == "mechanical_missing") neutral_only();
    if (scenario == "mechanical_missing" || scenario == "mechanical_nonfinite") {
      for (auto packet : sent) {
        for (auto &flag : packet.flags) flag = 0;
        require(std::memcmp(&packet, &zero, sizeof zero) == 0, "target escaped invalid initial feedback");
      }
    }
    if (follow_without_endstop) {
      float peak = 0.f;
      for (const auto &packet : sent)
        peak = std::max(peak, std::abs(packet.q_des_abad[0]));
      std::cout << "no stop peak target excursion radians=" << peak << "\n";
      // Two complete ideal traversals, including the configured stop preload.
      require(peak <= .061f, "single missing stop exceeded the configured joint travel budget");
    }
    std::cout << "PASS " << scenario << "\n";
    return 0;
  }
  if(scenario.rfind("floor_",0)==0) {
    clocked_reports=true;field_redundant_case(info,scenario);return 0;
  }
  if (scenario.rfind("damping_", 0)==0) {
    clocked_reports=true;
    damping_case(info, scenario);
    return 0;
  }
  if (scenario == "resource_manager" || scenario == "resource_manager_strict") {
    const bool strict = scenario == "resource_manager_strict";
    if (!strict) info.hardware_parameters["fc_imu_recovery"] = "supported_rehome";
    recovery_joints(info);
    resource_manager_case(info, strict);
    return 0;
  }
  const bool recovery = scenario == "supported_standby" || scenario == "supported_rehome";
  if (scenario == "strict_rehome") recovery_joints(info);
  if (recovery) { info.hardware_parameters["fc_imu_recovery"] = scenario; recovery_joints(info); }
  if(scenario=="init_missing")mode=5;
  const auto initialized=driver->on_init(info);
  if(scenario=="init_missing"){
    require(initialized==hardware_interface::CallbackReturn::ERROR,"missing IMU initialized");neutral_only();
  }else{
    require(initialized==hardware_interface::CallbackReturn::SUCCESS,"fresh init failed");require(sent.empty(),"init sent a command");
    if (scenario == "strict_rehome") {
      const auto dt = rclcpp::Duration::from_seconds(.005);
      auto states = driver->export_state_interfaces();
      data.q_abad[0] = 2;
      require(driver->on_activate({}) == hardware_interface::CallbackReturn::SUCCESS, "initial strict home failed");
      mode = 1;
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
      require(driver->read(rclcpp::Time(0), dt) == hardware_interface::return_type::ERROR, "strict fault ignored");
      driver->on_error({});
      mode = 0;
      data.q_abad[0] = 6;
      driver->on_configure({});
      require(driver->on_activate({}) == hardware_interface::CallbackReturn::SUCCESS, "repeated strict home failed");
      require(driver->read(rclcpp::Time(0), dt) == hardware_interface::return_type::OK &&
              std::abs(states[0].get_value()) < .001, "strict home reused stale encoder reference");
    } else if (recovery) recovery_case(*driver, scenario == "supported_standby");
    else if(scenario=="read_gyro"||scenario=="read_quat"||scenario=="read_norm"){
      mode=scenario=="read_gyro"?1:scenario=="read_quat"?2:4;
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      for(int i=0;i<4;++i)command.flags[i]=1;
      command.kp_hip[0]=2;
      rcutils_logging_set_output_handler(capture_fault);
      require(driver->read(rclcpp::Time(0),rclcpp::Duration::from_seconds(.01))==hardware_interface::return_type::ERROR,"invalid feedback accepted by read");neutral_only();
      require(neutral_at_log, "fault diagnostic preceded neutral SPI packet");
      const char *reason = scenario == "read_gyro" ? "stale gyro receipt" :
                           scenario == "read_quat" ? "stale quaternion receipt" : "invalid quaternion norm";
      require(fault_message.find(std::string("phase=read reason=") + reason) != std::string::npos,
              "missing component-specific failure");
      require(fault_message.find("quaternion={seq=17 valid=1 receipt_mono_us=") != std::string::npos &&
              fault_message.find("gyro={seq=31 valid=1 receipt_mono_us=") != std::string::npos &&
              fault_message.find(" age_us=") != std::string::npos, "missing per-component receipt evidence");
      if(scenario!="read_norm"){
        mode=scenario=="read_gyro"?6:7;std::this_thread::sleep_for(std::chrono::milliseconds(30));
        sent.clear();for(int i=0;i<4;++i)command.flags[i]=1;
        require(driver->read(rclcpp::Time(0),rclcpp::Duration::from_seconds(.01))==hardware_interface::return_type::ERROR,"queued report allowed SPI command");neutral_only();
      }
    }else if(scenario=="activate_bias"){
      mode=3;std::this_thread::sleep_for(std::chrono::milliseconds(30));
      require(driver->on_activate(rclcpp_lifecycle::State())==hardware_interface::CallbackReturn::ERROR,"biased IMU enabled");neutral_only();
    }else if(scenario=="homing_stale"){
      lose_during_enable=true;
      require(driver->on_activate(rclcpp_lifecycle::State())==hardware_interface::CallbackReturn::ERROR,"lost IMU during homing accepted");
      require(!sent.empty() && sent.back().flags[0]==0,"homing fault did not disable");
    }else if(scenario=="activate_fresh"){
      simulate_empty_wakeup=true;
      const auto start=std::chrono::steady_clock::now();
      require(driver->on_activate(rclcpp_lifecycle::State())==hardware_interface::CallbackReturn::SUCCESS,"stationary startup rejected");
      require(std::chrono::steady_clock::now()-start>=pupper_imu::stationary_window,"startup skipped window");
      const spi_command_t disabled{};
      require(sent.size()>2 && std::memcmp(&sent[0], &disabled, sizeof disabled)==0 &&
              std::memcmp(&sent[1], &disabled, sizeof disabled)==0 && sent[2].flags[0]==1,
              "fresh Home did not wait for the first verified neutral response");
    }else throw std::runtime_error("unknown scenario");
  }
  driver.reset();std::cout<<"PASS "<<scenario<<" fake_SPI_packets="<<sent.size()<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
