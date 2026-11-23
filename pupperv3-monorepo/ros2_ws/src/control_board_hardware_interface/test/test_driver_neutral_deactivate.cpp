// Actual lifecycle callbacks, fake SPI/BNO, no ROS node and no device/lock access.
#include "control_board_hardware_interface/control_board_hardware_interface.hpp"
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
spi_command_t command{};
spi_data_t data{};
std::vector<spi_command_t> sent;
// Ordered device/lock effects, so ownership release can be checked after I/O.
std::vector<std::string> events;
int init_count = 0, close_spi_count = 0, next_lock_fd = 19321;
constexpr int contended_lock_fd = 19322;
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
hardware_interface::HardwareInfo fake_info() {
  hardware_interface::HardwareInfo info; info.name = "neutral_packet_fake"; info.type = "system";
  info.hardware_parameters["use_imu"] = "false";
  return info;
}
}
// Deliberately replace only this test executable's device implementation.
// The fake transport always replies, so it reports both board groups valid.
void init_spi() { ++init_count; command = {}; data = {}; }
void close_spi() { ++close_spi_count; events.push_back("close_spi"); }
spi_command_t *get_spi_command() { return &command; }
spi_data_t *get_spi_data() { return &data; }
void spi_driver_run() {
  sent.push_back(command); events.push_back("packet");
  data.spi_driver_status = static_cast<int32_t>(static_cast<uint32_t>(sent.size()) << 16) | SPI_BOTH_REPLIES_VALID;
}
BNO055::BNO055(int, uint32_t) { throw std::runtime_error("unexpected IMU initialization"); }
BNO055::~BNO055() {}
BNO055::Output BNO055::sample() { throw std::runtime_error("unexpected IMU sampling"); }
// The production on_init/destructor use a lock file. Isolate it even when tests
// run on a Pi: never create, lock, close, or unlink the real driver's lock path.
extern "C" int __wrap_open(const char *path, int flags, ...) {
  require(std::strcmp(path, "/tmp/control_board_hardware.lock") == 0, "unexpected file access");
  require((flags & O_CLOEXEC) != 0, "lock descriptor would leak into child processes");
  return next_lock_fd++;
}
extern "C" int __wrap_open64(const char *path, int flags, ...) { return __wrap_open(path, flags); }
extern "C" int __wrap_flock(int fd, int) {
  require(fd >= 19321 && fd < next_lock_fd, "unexpected flock");
  if (fd == contended_lock_fd) { errno = EWOULDBLOCK; return -1; }
  return 0;
}
extern "C" int __wrap_close(int fd) {
  require(fd >= 19321 && fd < next_lock_fd, "unexpected close"); events.push_back("close:" + std::to_string(fd)); return 0;
}
extern "C" int __wrap_unlink(const char *) {
  throw std::runtime_error("the shared lock inode was unlinked; a concurrent owner could be bypassed");
}
int main(int argc, char **argv) {
  try {
    require(argc == 2, "one scenario required"); const std::string scenario = argv[1];
    unsetenv("PUPPER_FC_BOARD_HEALTH_PATH"); unsetenv("PUPPER_IMU_STARTUP_TRACE");
    if (scenario == "contention") {
      auto owner = std::make_unique<control_board_hardware_interface::ControlBoardHardwareInterface>();
      require(owner->on_init(fake_info()) == hardware_interface::CallbackReturn::SUCCESS, "owner init failed");
      auto second = std::make_unique<control_board_hardware_interface::ControlBoardHardwareInterface>();
      require(second->on_init(fake_info()) == hardware_interface::CallbackReturn::ERROR, "second owner initialized");
      require(events == std::vector<std::string>{"close:19322"}, "contended instance must close only its own descriptor once");
      seed_command(); events.clear();
      second.reset();
      require(events.empty() && sent.empty() && close_spi_count == 0 && init_count == 1,
              "instance without the lock sent a packet or released the owner's resources");
      owner.reset();
      require(events == (std::vector<std::string>{"packet", "close_spi", "close:19321"}),
              "owner must send neutral and close SPI before releasing its lock");
      check(1);
      std::cout << "PASS contention second_owner_packets=0 owner_release=neutral,close_spi,unlock unlink=0\n";
      return 0;
    }
    if (scenario == "init_failed_before_spi") {
      setenv("PUPPER_FC_BOARD_HEALTH_PATH", "relative-path-is-refused", 1);
      auto driver = std::make_unique<control_board_hardware_interface::ControlBoardHardwareInterface>();
      require(driver->on_init(fake_info()) == hardware_interface::CallbackReturn::ERROR, "invalid receipt path accepted");
      require(init_count == 0 && events.empty(), "failed initialization reached SPI");
      driver.reset();
      require(events == std::vector<std::string>{"close:19321"} && sent.empty() && close_spi_count == 0,
              "failed initialization sent a packet or closed SPI it never opened");
      std::cout << "PASS init_failed_before_spi packets=0 close_spi=0 lock_released=1\n";
      return 0;
    }
    auto driver = std::make_unique<control_board_hardware_interface::ControlBoardHardwareInterface>();
    require(driver->on_init(fake_info()) == hardware_interface::CallbackReturn::SUCCESS, "fake init failed");
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
    seed_command(); events.clear(); driver.reset(); check(++expected);
    require(events == (std::vector<std::string>{"packet", "close_spi", "close:19321"}),
            "destructor released the lock before its final neutral packet and SPI close");
    std::cout << "PASS " << scenario << " neutral_iterations=" << expected
              << " host_bytes=256 board_channel_groups=2 enable_packets=0\n";
    return 0;
  } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
