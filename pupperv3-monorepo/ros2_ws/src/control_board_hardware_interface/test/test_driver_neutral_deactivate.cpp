// Actual lifecycle callbacks, fake SPI/BNO, no ROS node and no device/lock access.
#include "control_board_hardware_interface/control_board_hardware_interface.hpp"
#include <atomic>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
spi_command_t command{};
spi_data_t data{};
std::vector<spi_command_t> sent;
int init_count = 0;
int owner_fd=-1, spi_closes=0, sensor_closes=0;
std::string private_lock;
std::atomic<bool> sensor_live{false};
constexpr int lock_fd = 19321;
void require(bool value, const char *message) { if (!value) throw std::runtime_error(message); }
void seed_command() {
  float *fields[] = {command.q_des_abad, command.q_des_hip, command.q_des_knee,
    command.qd_des_abad, command.qd_des_hip, command.qd_des_knee,
    command.kp_abad, command.kp_hip, command.kp_knee,
    command.kd_abad, command.kd_hip, command.kd_knee,
    command.tau_abad_ff, command.tau_hip_ff, command.tau_knee_ff};
  for (int field = 0; field < 15; ++field)
    for (int leg = 0; leg < 4; ++leg) fields[field][leg] = 1.25f + field * 4 + leg;
  for (int leg = 0; leg < 4; ++leg) command.flags[leg] = 1 + leg;
}
void check(size_t expected) {
  require(sent.size() == expected, "unexpected SPI iteration count");
  const spi_command_t zero{};
  static_assert(sizeof(spi_command_t) == 256, "command layout changed");
  // Includes every float and flag for channels 0/1 AND 2/3 (both board groups).
  for (const auto &packet : sent)
    require(std::memcmp(&packet, &zero, sizeof zero) == 0, "outgoing command retained a nonzero field or enable flag");
  require(std::memcmp(&command, &zero, sizeof zero) == 0, "host command was not neutralized");
}
}
// Deliberately replace only this test executable's device implementation.
void init_spi() { ++init_count; command = {}; data = {}; }
spi_command_t *get_spi_command() { return &command; }
spi_data_t *get_spi_data() { return &data; }
void spi_driver_run() {
  require(private_lock.empty() || owner_fd>=0,"board traffic after releasing lock");
  sent.push_back(command);
}
void close_spi() {
  require(private_lock.empty() || (owner_fd>=0 && !sensor_live), "SPI released outside owned shutdown");
  ++spi_closes;
}
void BNO055::enable_redundant_reports(uint32_t) {}
BNO055::BNO055(int,uint32_t) {
  require(!private_lock.empty() && owner_fd>=0 && !sensor_live.exchange(true),"IMU initialized outside lock ownership");
}
BNO055::~BNO055() {
  require(owner_fd>=0,"IMU descriptor closed after board ownership release");
  sensor_live=false; ++sensor_closes;
}
BNO055::Output BNO055::sample() {
  Output value{};
  value.quat=Eigen::Quaternionf::Identity(); value.gyro=Eigen::Vector3f::Zero();
  value.gyro_valid=value.quaternion_valid=true;
  value.gyro_receipt_time=value.quaternion_receipt_time=std::chrono::steady_clock::now();
  return value;
}
// The production on_init/destructor use a lock file. Isolate it even when tests
// run on a Pi: never create, lock, close, or unlink the real driver's lock path.
extern "C" int __real_open(const char *,int,...);
extern "C" int __real_flock(int,int);
extern "C" int __real_close(int);
extern "C" int __real_unlink(const char *);
extern "C" int __wrap_open(const char *path, int flags, ...) {
  require(std::strcmp(path,"/tmp/control_board_hardware.lock")==0,"unexpected file access");
  if(private_lock.empty()) return lock_fd;
  require((flags&O_CLOEXEC)!=0,"board lock would leak across exec");
  return __real_open(private_lock.c_str(),flags,0600);
}
extern "C" int __wrap_open64(const char *path, int flags, ...) { return __wrap_open(path, flags); }
extern "C" int __wrap_flock(int fd,int operation) {
  if(private_lock.empty()) { require(fd==lock_fd,"unexpected flock"); return 0; }
  const int rc=__real_flock(fd,operation);
  if(rc==0) {
    require(operation==(LOCK_EX|LOCK_NB),"lock released before device shutdown");
    owner_fd=fd;
  }
  return rc;
}
extern "C" int __wrap_close(int fd) {
  if(private_lock.empty()) { require(fd==lock_fd,"unexpected close"); return 0; }
  if(fd==owner_fd) {
    require(!sensor_live && spi_closes==init_count && sensor_closes==init_count,
            "board lock released while devices remained open");
    owner_fd=-1;
  }
  return __real_close(fd);
}
extern "C" int __wrap_unlink(const char *) {
  throw std::runtime_error("removing the board lock inode splits ownership across different files");
}
int main(int argc, char **argv) {
  try {
    require(argc == 2, "one scenario required"); const std::string scenario = argv[1];
    auto driver = std::make_unique<control_board_hardware_interface::ControlBoardHardwareInterface>();
    hardware_interface::HardwareInfo info; info.name = "neutral_packet_fake"; info.type = "system";
    if(scenario=="lock_contention") {
      char path[]="/tmp/board-lock-fixture-XXXXXX";
      const int tmp=mkstemp(path);
      require(tmp>=0 && __real_close(tmp)==0,"create private lock");
      private_lock=path;
      hardware_interface::ComponentInfo imu; imu.name="imu_sensor";
      imu.parameters={{"roll","0"},{"pitch","0"},{"yaw","0"}}; info.sensors.push_back(imu);
      using Driver=control_board_hardware_interface::ControlBoardHardwareInterface;
      using Result=hardware_interface::CallbackReturn;
      require(driver->on_init(info)==Result::SUCCESS,"first owner failed");
      seed_command();
      for(int i=0;i<2;++i) {
        auto contender=std::make_unique<Driver>();
        require(contender->on_init(info)==Result::ERROR,"overlapping initialization accepted");
        contender.reset();
        require(init_count==1 && sent.empty() && owner_fd>=0 && sensor_live,
                "rejected contender affected active board owner");
      }
      driver.reset(); check(1);
      require(owner_fd<0 && !sensor_live && spi_closes==1,"owner did not release cleanly");
      auto replacement=std::make_unique<Driver>();
      require(replacement->on_init(info)==Result::SUCCESS,"replacement could not acquire stable inode");
      replacement.reset(); check(2);
      require(__real_unlink(path)==0,"persistent test lock inode disappeared");
      std::cout<<"PASS lock contention, final neutral and device close before lock handoff\n";
      return 0;
    }
    info.hardware_parameters["use_imu"] = "false";
    require(driver->on_init(info) == hardware_interface::CallbackReturn::SUCCESS, "fake init failed");
    require(init_count == 1 && sent.empty(), "initialization submitted a command");
    seed_command(); size_t expected = 0;
    if (scenario == "deactivate" || scenario == "repeated") {
      require(driver->on_deactivate({}) == hardware_interface::CallbackReturn::SUCCESS, "deactivate callback failed");
      check(++expected);
      if (scenario == "repeated") { seed_command(); driver->on_deactivate({}); check(++expected); }
    } else if (scenario == "error") {
      require(driver->on_error({}) == hardware_interface::CallbackReturn::SUCCESS, "error callback failed"); check(++expected);
    } else require(scenario == "destroy", "unknown scenario");
    // Even a stale nonzero packet immediately before destruction is cleared.
    seed_command(); driver.reset(); check(++expected);
    std::cout << "PASS " << scenario << " neutral_iterations=" << expected
              << " host_bytes=256 board_channel_groups=2 enable_packets=0\n";
    return 0;
  } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
