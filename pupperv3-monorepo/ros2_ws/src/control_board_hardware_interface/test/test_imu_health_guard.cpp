// Real driver read/activation callbacks with fake SPI/BNO and isolated locks.
// No device implementation is linked and every open except the fake lock fails.
#include "control_board_hardware_interface/control_board_hardware_interface.hpp"
#include "control_board_hardware_interface/imu_health.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include <atomic>
#include <cstring>
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
bool neutral_at_log = false;
void capture_fault(const rcutils_log_location_t *, int, const char *, rcutils_time_point_value_t,
                   const char *format, va_list *args) {
  char message[1024];
  std::vsnprintf(message, sizeof(message), format, *args);
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
int spi_replies = SPI_BOTH_REPLIES_VALID;
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
spi_command_t *get_spi_command(){return &command;}
spi_data_t *get_spi_data(){return &data;}
void spi_driver_run(){sent.push_back(command);data.spi_driver_status = spi_replies;if(lose_during_enable && command.flags[0])mode=1;}
BNO055::BNO055(int,uint32_t){}
BNO055::~BNO055(){}
BNO055::Output BNO055::sample(){return output(std::chrono::steady_clock::now());}
extern "C" int __wrap_open(const char *p,int,...){require(std::strcmp(p,"/tmp/control_board_hardware.lock")==0,"unexpected open");return lock_fd;}
extern "C" int __wrap_open64(const char *p,int f,...){return __wrap_open(p,f);}
extern "C" int __wrap_flock(int fd,int){require(fd==lock_fd,"unexpected flock");return 0;}
extern "C" int __wrap_close(int fd){require(fd==lock_fd,"unexpected close");return 0;}
extern "C" int __wrap_unlink(const char *p){require(std::strcmp(p,"/tmp/control_board_hardware.lock")==0,"unexpected unlink");return 0;}
int main(int argc,char **argv){try{
  require(argc==2,"scenario required");std::string scenario=argv[1];
  if(scenario=="health"){unit_health();std::cout<<"PASS health\n";return 0;}
  clocked_reports = scenario == "supported_standby" || scenario == "supported_rehome" ||
                    scenario == "resource_manager" || scenario == "resource_manager_strict";
  auto driver=std::make_unique<control_board_hardware_interface::ControlBoardHardwareInterface>();
  hardware_interface::HardwareInfo info{};info.is_async=false;info.name="imu_guard_fake";info.type="system";
  hardware_interface::ComponentInfo sensor;sensor.name="imu_sensor";sensor.parameters={{"roll","0"},{"pitch","0"},{"yaw","0"}};info.sensors.push_back(sensor);
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
      const auto start=std::chrono::steady_clock::now();
      require(driver->on_activate(rclcpp_lifecycle::State())==hardware_interface::CallbackReturn::SUCCESS,"stationary startup rejected");
      require(std::chrono::steady_clock::now()-start>=pupper_imu::stationary_window,"startup skipped window");
      require(!sent.empty()&&sent.front().flags[0]==1,"fresh activation never enabled fake SPI");
    }else throw std::runtime_error("unknown scenario");
  }
  driver.reset();std::cout<<"PASS "<<scenario<<" fake_SPI_packets="<<sent.size()<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
