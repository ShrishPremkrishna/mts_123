// Actual lifecycle callbacks, fake SPI/BNO, no ROS node and no device/lock access.
#include "control_board_hardware_interface/control_board_hardware_interface.hpp"
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
spi_command_t command{};
spi_data_t data{};
std::vector<spi_command_t> sent;
int init_count = 0;
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
void spi_driver_run() { sent.push_back(command); }
BNO055::BNO055(int, uint32_t) { throw std::runtime_error("unexpected IMU initialization"); }
BNO055::~BNO055() {}
BNO055::Output BNO055::sample() { throw std::runtime_error("unexpected IMU sampling"); }
// The production on_init/destructor use a lock file. Isolate it even when tests
// run on a Pi: never create, lock, close, or unlink the real driver's lock path.
extern "C" int __wrap_open(const char *path, int, ...) {
  require(std::strcmp(path, "/tmp/control_board_hardware.lock") == 0, "unexpected file access"); return lock_fd;
}
extern "C" int __wrap_open64(const char *path, int flags, ...) { return __wrap_open(path, flags); }
extern "C" int __wrap_flock(int fd, int) { require(fd == lock_fd, "unexpected flock"); return 0; }
extern "C" int __wrap_close(int fd) { require(fd == lock_fd, "unexpected close"); return 0; }
extern "C" int __wrap_unlink(const char *path) {
  require(std::strcmp(path, "/tmp/control_board_hardware.lock") == 0, "unexpected unlink"); return 0;
}
int main(int argc, char **argv) {
  try {
    require(argc == 2, "one scenario required"); const std::string scenario = argv[1];
    auto driver = std::make_unique<control_board_hardware_interface::ControlBoardHardwareInterface>();
    hardware_interface::HardwareInfo info; info.name = "neutral_packet_fake"; info.type = "system";
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
