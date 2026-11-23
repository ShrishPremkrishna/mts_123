// FC_BOARD_HEALTH_V1 strict receipts from the actual driver callbacks.
// Fake SPI/BNO only. The receipt is a regular file in a private temporary
// directory; every other path, and the real driver lock, is refused.
#include "control_board_hardware_interface/control_board_hardware_interface.hpp"
#include "control_board_hardware_interface/board_health.hpp"

#include <fcntl.h>
#include <rcutils/logging.h>
#include <sys/stat.h>
#include <unistd.h>

#include <array>
#include <cmath>
#include <atomic>
#include <cerrno>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <limits>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

extern "C" int __real_open(const char *, int, ...);
extern "C" int __real_close(int);
extern "C" int __real_unlink(const char *);

namespace {
using Row = std::array<uint64_t, 24>;
constexpr uint64_t kMagic = 0x314d494346505550ULL;  // "PUPFCIM1"
constexpr int kLockFd = 19323;
std::string directory, receipt_path;
spi_command_t command{};
spi_data_t data{};
std::vector<spi_command_t> sent;
std::atomic<int> reply_mask{SPI_BOTH_REPLIES_VALID};
std::atomic<int> one_shot_mask{-1};  // >= 0: mask of the next exchange only (a transient glitch)
std::atomic<bool> imu_stale{false};
std::atomic<int> imu_sequence{0};
int fail_after_homing_packets = -1, nan_after_homing_packets = -1, homing_packets = 0, close_spi_count = 0;
int max_exchanges = -1;  // >= 0: simulated exchange bound for this scenario (not wall time)
// Fake encoder for joint 0 (can_channel 1, can_id 1). A valid reply carries
// these values; an invalid reply carries kFabricated, the worst case of a
// transport that decodes before it knows validity. Neither may reach state.
float fake_q = 0.f, fake_qd = 0.f;
constexpr float kFabricated = 9.5f;
std::vector<Row> during_homing;
std::mutex log_mutex;
std::vector<std::pair<int, std::string>> logs;

void require(bool ok, const std::string &why) { if (!ok) throw std::runtime_error(why); }

bool under_directory(const char *path) {
  const std::string prefix = directory + "/";
  return !directory.empty() && std::strncmp(path, prefix.c_str(), prefix.size()) == 0 &&
         std::strstr(path, "..") == nullptr;
}

// The same seqlock discipline as the laptop/Pi reader.
Row read_receipt() {
  const int fd = __real_open(receipt_path.c_str(), O_RDONLY | O_CLOEXEC);
  require(fd >= 0, "receipt missing");
  struct stat info {};
  require(fstat(fd, &info) == 0 && S_ISREG(info.st_mode) && info.st_size == 192 &&
              info.st_nlink == 1 && (info.st_mode & 0777) == 0400,
          "receipt is not a single read-only 24-word file");
  for (int attempt = 0; attempt < 100000; ++attempt) {
    uint64_t before = 0, after = 0;
    Row row{};
    require(pread(fd, &before, 8, 0) == 8 && pread(fd, row.data(), 192, 0) == 192 &&
                pread(fd, &after, 8, 0) == 8, "short receipt read");
    if (before == row[0] && after == row[0] && row[0] && !(row[0] & 1)) {
      __real_close(fd);
      return row;
    }
    std::this_thread::yield();
  }
  __real_close(fd);
  throw std::runtime_error("receipt never consistent");
}

void require_schema(const Row &row) {
  require(row[1] == kMagic && row[2] == static_cast<uint64_t>(getpid()) && row[3] != 0 &&
              row[22] == 1 && row[23] == 0 && row[21] <= 3 && row[15] == 5 && row[16] == 2 &&
              row[3] <= row[8],
          "receipt identity/schema/profile mismatch");
  require(row[13] <= 256 && row[14] <= 256, "report sequence out of range");
}

bool neutral(const spi_command_t &packet) {
  const spi_command_t zero{};
  return std::memcmp(&packet, &zero, sizeof zero) == 0;
}

// Every packet exchanged from index `from` on is the all-zero neutral packet.
void require_neutral_from(size_t from, const std::string &why) {
  require(from <= sent.size(), why + ": bad index");
  for (size_t i = from; i < sent.size(); ++i) require(neutral(sent[i]), why);
}

void require_neutral_last() {
  const spi_command_t zero{};
  require(!sent.empty() && std::memcmp(&sent.back(), &zero, sizeof zero) == 0,
          "last packet was not the neutral disable packet");
}

bool logged(int severity, const std::string &needle) {
  std::lock_guard<std::mutex> lock(log_mutex);
  for (const auto &entry : logs)
    if (entry.first == severity && entry.second.find(needle) != std::string::npos) return true;
  return false;
}

std::string last_error() {
  std::lock_guard<std::mutex> lock(log_mutex);
  for (auto it = logs.rbegin(); it != logs.rend(); ++it)
    if (it->first == RCUTILS_LOG_SEVERITY_ERROR) return it->second;
  return {};
}

void capture(const rcutils_log_location_t *, int severity, const char *name,
             rcutils_time_point_value_t, const char *format, va_list *args) {
  if (!name || std::string(name) != "ControlBoardHardwareInterface") return;
  va_list copy;
  va_copy(copy, *args);
  char buffer[4096];
  std::vsnprintf(buffer, sizeof buffer, format, copy);
  va_end(copy);
  std::lock_guard<std::mutex> lock(log_mutex);
  logs.emplace_back(severity, buffer);
}

hardware_interface::HardwareInfo fake_info(bool use_imu) {
  hardware_interface::HardwareInfo info;
  info.name = "board_health_fake";
  info.type = "system";
  if (!use_imu) info.hardware_parameters["use_imu"] = "false";
  hardware_interface::ComponentInfo sensor;
  sensor.name = "imu_sensor";
  sensor.parameters = {{"roll", "0"}, {"pitch", "0"}, {"yaw", "0"}};
  info.sensors.push_back(sensor);
  // One joint whose fake encoder never moves: the homing torque estimate
  // reaches its threshold after a few exchanges, exercising the real loop.
  hardware_interface::ComponentInfo joint;
  joint.name = "fake_joint";
  joint.parameters = {{"can_channel", "1"}, {"can_id", "1"}, {"position_min", "-1"},
                      {"position_max", "1"}, {"velocity_max", "10"}, {"effort_max", "10"},
                      {"kp_max", "20"}, {"kd_max", "2"}, {"homing_stage", "0"},
                      {"homing_velocity", "1.0"}, {"homing_kp", "1.0"}, {"homing_kd", "0.0"},
                      {"homed_position", "0.0"}, {"post_homing_position", "0.05"},
                      {"homing_torque_threshold", "0.05"}};
  info.joints.push_back(joint);
  return info;
}

using Driver = control_board_hardware_interface::ControlBoardHardwareInterface;
using CR = hardware_interface::CallbackReturn;
using RT = hardware_interface::return_type;
hardware_interface::return_type read_once(Driver &driver) {
  return driver.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01));
}
hardware_interface::return_type write_once(Driver &driver) {
  return driver.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01));
}

// Joint 0 position/velocity as the broadcaster and controllers see them: the
// exported state interfaces point at the driver's own state arrays.
std::pair<double, double> joint_state(Driver &driver) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  double position = std::numeric_limits<double>::quiet_NaN(), velocity = position;
  for (auto &handle : driver.export_state_interfaces()) {
    if (handle.get_prefix_name() != "fake_joint") continue;
    if (handle.get_interface_name() == "position") position = handle.get_value();
    if (handle.get_interface_name() == "velocity") velocity = handle.get_value();
  }
#pragma GCC diagnostic pop
  return {position, velocity};
}

// Handle::set_value returns void in some hardware_interface releases and
// [[nodiscard]] bool in others; a template keeps the unused branch uninstantiated.
template <typename Handle>
bool assign(Handle &handle, double value) {
  if constexpr (std::is_void_v<decltype(handle.set_value(value))>) {
    handle.set_value(value);
    return true;
  } else {
    return handle.set_value(value);
  }
}

// A controller's write to joint 0: the exported command interfaces point at
// the driver's own command arrays.
void set_command(Driver &driver, const std::string &interface, double value) {
  size_t written = 0;
  for (auto &handle : driver.export_command_interfaces())
    if (handle.get_prefix_name() == "fake_joint" && handle.get_interface_name() == interface)
      written += assign(handle, value);
  require(written == 1, "command interface " + interface + " not written");
}

std::string text(double value) {
  std::ostringstream out;
  out << value;
  return out.str();
}

// The command fields of transmitted packet `index` for joint 0.
std::string sent_fields(size_t index) {
  if (index >= sent.size()) return "nothing";
  const auto &p = sent[index];
  return "q_des=" + text(p.q_des_abad[0]) + " qd_des=" + text(p.qd_des_abad[0]) + " tau_ff=" +
         text(p.tau_abad_ff[0]) + " kp=" + text(p.kp_abad[0]) + " kd=" + text(p.kd_abad[0]);
}

std::unique_ptr<Driver> configured_driver(bool use_imu = true) {
  auto driver = std::make_unique<Driver>();
  require(driver->on_init(fake_info(use_imu)) == CR::SUCCESS, "init failed");
  require(driver->on_configure(rclcpp_lifecycle::State()) == CR::SUCCESS, "configure failed");
  return driver;
}

size_t count_logged(int severity, const std::string &needle) {
  std::lock_guard<std::mutex> lock(log_mutex);
  size_t n = 0;
  for (const auto &entry : logs) n += entry.first == severity && entry.second.find(needle) != std::string::npos;
  return n;
}

// fc_operator.board_error shows the text before " sampled_mono_us=".
std::string operator_view(const std::string &line) { return line.substr(0, line.find(" sampled_mono_us=")); }

// Activate, take the first (valid) read, then run one live write/read cycle
// that carries an enabled, nonzero packet. Returns the Home generation.
uint64_t activate_and_actuate(Driver &driver) {
  reply_mask = SPI_BOTH_REPLIES_VALID;
  fake_q = fake_qd = 0.f;
  require(driver.on_activate(rclcpp_lifecycle::State()) == CR::SUCCESS, "activation failed");
  require(read_once(driver) == RT::OK, "first read after Home failed");
  const auto row = read_receipt();
  require(row[7] == 1 && row[4] > 0, "valid first read not healthy");
  require(write_once(driver) == RT::OK && read_once(driver) == RT::OK, "live cycle failed");
  require(sent.back().flags[0] == 1 && sent.back().kp_abad[0] > 0,
          "homed write/read did not carry the live actuator packet");
  fake_q = 0.02f;
  fake_qd = 0.5f;
  require(write_once(driver) == RT::OK && read_once(driver) == RT::OK, "live cycle failed");
  const auto state = joint_state(driver);
  require(state.first == double(0.02f) && state.second == 0.5, "valid feedback not copied");
  return row[4];
}

// Homed write() of an invalid command. Returns "" when write() refuses it
// before any transmission: exactly one exchange, the neutral packet, a
// terminal receipt and one operator-parseable ERROR line. Otherwise returns
// what the controller manager's next read() transmitted (the unguarded path).
std::string refuse_command(Driver &driver, const std::string &reason, const std::string &token,
                           uint64_t generation, uint64_t faults) {
  const size_t before = sent.size();
  const size_t errors = count_logged(RCUTILS_LOG_SEVERITY_ERROR, "");
  if (write_once(driver) != RT::ERROR) {
    std::string outcome;
    const size_t next_index = sent.size();
    try {
      const RT next = read_once(driver);
      outcome = "next read() transmitted " + sent_fields(next_index) + " and returned " + (next == RT::OK ? "OK" : "ERROR");
    } catch (const std::exception &e) {
      outcome = std::string("fake transport: ") + e.what();
    }
    return "write() admitted it; " + outcome;
  }
  require(sent.size() == before + 1 && neutral(sent.back()),
          token + ": refusal did not send exactly one neutral packet");
  const auto row = read_receipt();
  require_schema(row);
  require(row[7] == 4 && row[5] == faults && row[4] == generation, token + ": command fault not terminal");
  require(count_logged(RCUTILS_LOG_SEVERITY_ERROR, "") == errors + 1, token + ": expected exactly one ERROR line");
  const auto error = last_error();
  require(operator_view(error) == "Command fault phase=write reason=" + reason &&
              error.find(" commands={fake_joint={") != std::string::npos &&
              error.find(token) != std::string::npos &&
              error.find(" generation=" + std::to_string(generation)) != std::string::npos,
          token + ": command fault diagnostic incomplete: " + error);
  return {};
}

// After a command fault the invalid value is still in the command interface:
// on_error and further cycles send only neutral, with no second fault and no
// health, until an explicit activation and Home.
void stays_neutral(Driver &driver, const std::string &why, uint64_t generation, uint64_t faults) {
  const size_t errors = count_logged(RCUTILS_LOG_SEVERITY_ERROR, "");
  const size_t from = sent.size();
  require(driver.on_error(rclcpp_lifecycle::State()) == CR::SUCCESS, why + ": on_error failed");
  for (int i = 0; i < 3; ++i)
    require(write_once(driver) == RT::OK && read_once(driver) == RT::OK, why + ": stale invalid command faulted again");
  require(sent.size() >= from + 4, why + ": neutral exchanges missing");
  require_neutral_from(from, why + ": nonneutral packet after the command fault");
  const auto row = read_receipt();
  require(row[7] == 4 && row[5] == faults && row[4] == generation, why + ": health recovered or fault recounted");
  require(count_logged(RCUTILS_LOG_SEVERITY_ERROR, "") == errors, why + ": stale invalid command logged another ERROR");
}

void publisher_race() {
  // Standalone publisher: a terminal snapshot must survive a healthy publisher
  // running concurrently on another thread, including one interrupted mid-write.
  pupper_imu::BoardHealth health;
  require(health.open(receipt_path.c_str()), "receipt open failed");
  BNO055::Output sample;
  sample.quaternion_receipt_time = sample.gyro_receipt_time = std::chrono::steady_clock::now();
  health.homes = 7;
  std::atomic<bool> stop{false};
  std::atomic<uint64_t> healthy_publishes{0};
  std::thread healthy([&] {
    while (!stop) {
      health.publish(1, sample, sample, 3, false);
      ++healthy_publishes;
      std::this_thread::sleep_for(std::chrono::microseconds(5));
    }
  });
  struct Join {
    std::atomic<bool> &stop; std::thread &thread;
    ~Join() { stop = true; if (thread.joinable()) thread.join(); }
  } join{stop, healthy};
  constexpr int trials = 300;
  for (int trial = 0; trial < trials; ++trial) {
    health.terminal = false;  // As a new activation does; the writer resumes healthy.
    const auto seen = healthy_publishes.load();
    while (healthy_publishes.load() < seen + 2) std::this_thread::yield();
    health.terminal.exchange(true);
    ++health.faults;
    health.publish(4, sample, sample, 3, false);
    // Every consistent snapshot after the terminal publish returns is terminal.
    for (int i = 0; i < 10; ++i) {
      const auto row = read_receipt();
      require(row[7] == 4 && row[5] == static_cast<uint64_t>(trial + 1),
              "stale healthy publisher overwrote the terminal snapshot");
    }
  }
  stop = true;
  healthy.join();
  const auto row = read_receipt();
  require_schema(row);
  require(row[7] == 4 && row[5] == trials && row[4] == 7, "final snapshot not terminal");
  std::cout << "PASS publisher_race trials=" << trials << " healthy_publishes=" << healthy_publishes.load() << "\n";
}
}  // namespace

// Fake device implementation for this executable only.
void init_spi() { command = {}; data = {}; }
void close_spi() { ++close_spi_count; }
spi_command_t *get_spi_command() { return &command; }
spi_data_t *get_spi_data() { return &data; }
void spi_driver_run() {
  // Safety assertion: no command value transmitted to the boards is NaN/Inf.
  const auto *values = reinterpret_cast<const float *>(&command);
  for (size_t i = 0; i < offsetof(spi_command_t, flags) / sizeof(float); ++i)
    require(std::isfinite(values[i]), "nonfinite actuator command transmitted");
  // Simulated exchanges, not wall time: a Home that ignores a refused command
  // keeps exchanging its last packet with NaN torque estimates and never ends.
  require(max_exchanges < 0 || sent.size() < static_cast<size_t>(max_exchanges),
          "Home kept exchanging after a nonfinite homing command");
  sent.push_back(command);
  // kp > 0 first occurs in Home; scenarios inspect these rows right after activation.
  const bool homing = command.kp_abad[0] > 0;
  if (homing) {
    during_homing.push_back(read_receipt());
    ++homing_packets;
    if (fail_after_homing_packets >= 0 && homing_packets > fail_after_homing_packets)
      reply_mask = 1;  // Board group 2 stops replying mid-Home.
    if (nan_after_homing_packets >= 0 && homing_packets > nan_after_homing_packets) {
      fake_q = std::numeric_limits<float>::quiet_NaN();  // Checksum-valid NaN mid-Home.
      // Simulated control steps, not wall time: a search that ignores the NaN
      // never sees the fake stop (reached in under 10 steps) and would not end.
      require(homing_packets < 1000, "Home kept searching on NaN feedback");
    }
  }
  int mask = one_shot_mask.exchange(-1);
  if (mask < 0) mask = reply_mask.load();
  data.q_abad[0] = mask == SPI_BOTH_REPLIES_VALID ? fake_q : kFabricated;
  data.qd_abad[0] = mask == SPI_BOTH_REPLIES_VALID ? fake_qd : -kFabricated;
  data.spi_driver_status = static_cast<int32_t>(static_cast<uint32_t>(sent.size()) << 16) | mask;
}
BNO055::BNO055(int, uint32_t) {}
BNO055::~BNO055() {}
BNO055::Output BNO055::sample() {
  BNO055::Output o;
  const auto now = std::chrono::steady_clock::now();
  o.quat = Eigen::Quaternionf::Identity();
  o.gyro = Eigen::Vector3f::Zero();
  o.quaternion_valid = o.gyro_valid = true;
  const int sequence = imu_sequence++ & 0xff;
  o.quaternion_report_sequence = o.gyro_report_sequence = sequence;
  o.quaternion_receipt_time = o.gyro_receipt_time = imu_stale ? now - std::chrono::seconds(1) : now;
  o.packet_timestamp = o.measurement_timestamp = std::chrono::high_resolution_clock::now();
  return o;
}
extern "C" int __wrap_open(const char *path, int flags, ...) {
  if (std::strcmp(path, "/tmp/control_board_hardware.lock") == 0) return kLockFd;
  require(under_directory(path), std::string("unexpected open ") + path);
  return __real_open(path, flags, 0600);
}
extern "C" int __wrap_open64(const char *path, int flags, ...) { return __wrap_open(path, flags); }
extern "C" int __wrap_flock(int fd, int) { require(fd == kLockFd, "unexpected flock"); return 0; }
extern "C" int __wrap_close(int fd) { return fd == kLockFd ? 0 : __real_close(fd); }
extern "C" int __wrap_unlink(const char *path) {
  require(under_directory(path), std::string("unexpected unlink ") + path);
  return __real_unlink(path);
}

int main(int argc, char **argv) {
  char base[] = "/tmp/pupper-board-health-test.XXXXXX";
  try {
    require(argc == 2, "scenario required");
    const std::string scenario = argv[1];
    require(mkdtemp(base) != nullptr, "mkdtemp failed");
    directory = base;
    receipt_path = directory + "/fc-board-health.bin";
    unsetenv("PUPPER_IMU_STARTUP_TRACE");
    setenv("PUPPER_FC_BOARD_HEALTH_PATH", receipt_path.c_str(), 1);
    require(rcutils_logging_initialize() == RCUTILS_RET_OK, "logging init failed");
    rcutils_logging_set_output_handler(capture);

    if (scenario == "publisher_race") {
      publisher_race();
    } else if (scenario == "lifecycle") {
      auto driver = std::make_unique<Driver>();
      require(driver->on_init(fake_info(true)) == hardware_interface::CallbackReturn::SUCCESS, "init failed");
      auto row = read_receipt();
      require_schema(row);
      require(row[7] == 0 && row[4] == 0 && row[5] == 0 && row[6] == 0, "initial receipt not unknown");
      require(logged(RCUTILS_LOG_SEVERITY_INFO, "FC_BOARD_HEALTH_V1 strict receipt profile=1"), "marker not logged");
      // Configured but not activated: fresh exchange and IMU still are not health.
      require(read_once(*driver) == hardware_interface::return_type::OK, "inactive read failed");
      row = read_receipt();
      require(row[7] == 0 && row[4] == 0, "unhomed read published health");

      require(driver->on_activate(rclcpp_lifecycle::State()) == hardware_interface::CallbackReturn::SUCCESS, "activation failed");
      const size_t first_home_packets = during_homing.size();
      require(first_home_packets > 0, "Home sent no homing packets");
      for (const auto &r : during_homing)
        require(r[7] == 5 && r[4] == 0, "receipt was not homing/no-generation during Home");
      row = read_receipt();
      require(row[7] == 5 && row[4] == 0, "Home completion published health without a read");

      require(read_once(*driver) == hardware_interface::return_type::OK, "healthy read failed");
      row = read_receipt();
      require_schema(row);
      const uint64_t generation = row[4];
      require(row[7] == 1 && generation > 0 && row[5] == 0 && row[21] == 3, "valid read not healthy");
      require(row[9] && row[10] && row[9] <= row[8] && row[10] <= row[8], "receipt times invalid");
      require(row[9] == row[17] && row[10] == row[18] && row[11] == row[19] && row[12] == row[20],
              "strict raw and selected evidence differ");
      require(logged(RCUTILS_LOG_SEVERITY_INFO, "FC_BOARD_HEALTH_V1 Home generation=" + std::to_string(generation)),
              "generation not logged");

      // Each valid homed read renews the receipt (invalid replies: active_fault).
      const auto before_valid = read_receipt();
      require(read_once(*driver) == hardware_interface::return_type::OK, "second read failed");
      row = read_receipt();
      require(row[0] > before_valid[0] && row[7] == 1 && row[4] == generation, "valid exchange did not renew health");

      require(driver->on_deactivate(rclcpp_lifecycle::State()) == hardware_interface::CallbackReturn::SUCCESS, "deactivate failed");
      require_neutral_last();
      row = read_receipt();
      require(row[7] == 4 && row[5] == 1 && row[4] == generation, "deactivation not terminal");
      require(read_once(*driver) == hardware_interface::return_type::OK, "inactive read failed");
      require(read_receipt()[7] == 4, "inactive read overwrote the terminal snapshot");

      // Reactivation performs a real Home search again and a new generation.
      during_homing.clear();
      require(driver->on_activate(rclcpp_lifecycle::State()) == hardware_interface::CallbackReturn::SUCCESS, "reactivation failed");
      // The fake encoder is deterministic, so a complete new search repeats exactly.
      require(during_homing.size() == first_home_packets, "reactivation skipped the mechanical Home search");
      for (const auto &r : during_homing)
        require(r[7] == 5 && r[5] == 1, "reactivation Home receipt wrong");
      require(read_once(*driver) == hardware_interface::return_type::OK, "reactivated read failed");
      row = read_receipt();
      const uint64_t second = row[4];
      require(row[7] == 1 && second > generation && row[5] == 1, "new Home did not produce a new generation");

      imu_stale = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      require(read_once(*driver) == hardware_interface::return_type::ERROR, "stale IMU accepted");
      require_neutral_last();
      row = read_receipt();
      require(row[7] == 4 && row[5] == 2 && row[4] == second, "IMU fault not terminal");
      const auto error = last_error();
      require(error.find("IMU fault phase=read reason=stale IMU component receipt sampled_mono_us=") == 0 &&
                  error.find(" generation=" + std::to_string(second)) != std::string::npos &&
                  error.find("fault_spi_mask=3") != std::string::npos &&
                  error.find("quaternion={seq=") != std::string::npos,
              "fault diagnostic lacks predicate, receipts or generation: " + error);
      driver.reset();
      row = read_receipt();
      require(row[7] == 4 && row[5] == 2 && close_spi_count == 1, "destruction changed the terminal record");
      std::cout << "PASS lifecycle generations=" << generation << "," << second << " faults=2\n";
    } else if (scenario == "homing_reply") {
      fail_after_homing_packets = 3;
      auto driver = std::make_unique<Driver>();
      require(driver->on_init(fake_info(true)) == hardware_interface::CallbackReturn::SUCCESS, "init failed");
      require(driver->on_activate(rclcpp_lifecycle::State()) == hardware_interface::CallbackReturn::ERROR, "Home completed on a missing reply");
      require_neutral_last();
      const auto row = read_receipt();
      require(row[7] == 4 && row[4] == 0 && row[5] == 1, "failed Home produced a generation");
      const auto error = last_error();
      require(error.find("Board fault phase=homing reason=missing or corrupt motor board reply sampled_mono_us=") == 0 &&
                  error.find("fault_spi_mask=1") != std::string::npos && error.find("generation=0") != std::string::npos,
              "homing diagnostic incomplete: " + error);
      require(!logged(RCUTILS_LOG_SEVERITY_INFO, "Successfully activated!"), "failed Home reported success");
      std::cout << "PASS homing_reply generation=0 faults=1\n";
    } else if (scenario == "probe_refused") {
      reply_mask = 0;
      auto driver = std::make_unique<Driver>();
      require(driver->on_init(fake_info(true)) == hardware_interface::CallbackReturn::SUCCESS, "init failed");
      require(driver->on_activate(rclcpp_lifecycle::State()) == hardware_interface::CallbackReturn::ERROR, "enabled without board replies");
      const spi_command_t zero{};
      for (const auto &packet : sent)
        require(std::memcmp(&packet, &zero, sizeof zero) == 0, "enable or nonzero packet sent without replies");
      const auto row = read_receipt();
      require(row[7] == 4 && row[4] == 0, "refused probe not terminal");
      const std::regex consumer(R"(^Home refused before motor enable: (missing motor board reply \(mask=[0-3]\))$)");
      require(std::regex_match(last_error(), consumer), "persistent_session cannot parse: " + last_error());
      std::cout << "PASS probe_refused packets=" << sent.size() << " all_neutral=1\n";
    } else if (scenario == "imu_disabled") {
      auto driver = std::make_unique<Driver>();
      require(driver->on_init(fake_info(false)) == hardware_interface::CallbackReturn::SUCCESS, "init failed");
      require(driver->on_activate(rclcpp_lifecycle::State()) == hardware_interface::CallbackReturn::SUCCESS, "activation failed");
      for (int i = 0; i < 3; ++i) require(read_once(*driver) == hardware_interface::return_type::OK, "read failed");
      const auto row = read_receipt();
      require_schema(row);
      require(row[7] == 0 && row[4] > 0 && row[9] == 0 && row[10] == 0, "IMU-disabled board published health");
      std::cout << "PASS imu_disabled status=0 generation=" << row[4] << "\n";
    } else if (scenario == "no_joints") {
      auto info = fake_info(true);
      info.joints.clear();
      auto driver = std::make_unique<Driver>();
      require(driver->on_init(info) == hardware_interface::CallbackReturn::SUCCESS, "init failed");
      require(driver->on_activate(rclcpp_lifecycle::State()) == hardware_interface::CallbackReturn::SUCCESS,
              "joint-less activation changed");
      require(read_once(*driver) == hardware_interface::return_type::OK, "read failed");
      const auto row = read_receipt();
      require(row[7] == 0 && row[4] == 0, "Home generation without any mechanical search");
      // Without a homed session the enable flags from activation are not resent.
      require(neutral(sent.back()), "unhomed read sent the enable packet");
      std::cout << "PASS no_joints status=0 generation=0 read_neutral=1\n";
    } else if (scenario == "active_fault") {
      // Homed and actuating: the first missing, one-sided or corrupt reply faults
      // to neutral before its data is copied, and only a new Home recovers.
      auto driver = configured_driver();
      uint64_t faults = 0, previous_generation = 0;
      for (int mask : {0, 1, 2}) {
        const uint64_t generation = activate_and_actuate(*driver);
        require(generation > previous_generation, "activation did not produce a new generation");
        previous_generation = generation;
        // One invalid exchange; the boards reply validly again from the next one.
        one_shot_mask = mask;
        require(write_once(*driver) == RT::OK, "write failed");
        const size_t live = sent.size();
        require(read_once(*driver) == RT::ERROR, "invalid reply accepted while homed");
        require(sent.size() > live + 1 && !neutral(sent[live]), "fault exchange sequence unexpected");
        require_neutral_from(live + 1, "nonneutral packet after the board fault");
        const auto state = joint_state(*driver);
        require(state.first == double(0.02f) && state.second == 0.5,
                "invalid reply replaced the last accepted joint state");
        auto row = read_receipt();
        require_schema(row);
        // Word 21 of the terminal row is the latest (neutral) exchange's mask; the
        // faulting mask is in the ERROR line.
        require(row[7] == 4 && row[5] == ++faults && row[4] == generation && row[21] == 3,
                "board fault not terminal");
        const auto error = last_error();
        require(operator_view(error) == "Board fault phase=read reason=missing or corrupt motor board reply" &&
                    error.find("fault_spi_mask=" + std::to_string(mask) + " neutral_spi_mask=3") != std::string::npos &&
                    error.find(" generation=" + std::to_string(generation)) != std::string::npos &&
                    error.find("kept_state={positions=[0.02") != std::string::npos,
                "board fault diagnostic incomplete: " + error);
        require(driver->on_error(rclcpp_lifecycle::State()) == CR::SUCCESS, "on_error failed");
        require_neutral_last();
        require(read_receipt()[5] == faults, "on_error counted a second fault");
        // Transport already valid again: inactive handling only. No health, no actuation.
        const size_t after = sent.size();
        for (int i = 0; i < 3; ++i)
          require(write_once(*driver) == RT::OK && read_once(*driver) == RT::OK, "recovered inactive read failed");
        require_neutral_from(after, "actuation resumed without a new Home");
        row = read_receipt();
        require(row[7] == 4 && row[5] == faults && row[4] == generation, "health recovered without a new Home");
        require(driver->on_configure(rclcpp_lifecycle::State()) == CR::SUCCESS, "reconfigure failed");
      }
      const uint64_t last = activate_and_actuate(*driver);
      require(last > previous_generation && read_receipt()[5] == faults, "new Home did not recover");
      std::cout << "PASS active_fault masks=0,1,2 faults=" << faults << " recovered_by_home=1\n";
    } else if (scenario == "nonfinite") {
      // A checksum-valid reply with NaN or +/-Inf position or velocity.
      const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
      const std::pair<float, float> cases[] = {{nan, 0.f}, {inf, 0.f}, {-inf, 0.f}, {0.f, nan}, {0.f, inf}, {0.f, -inf}};
      auto driver = configured_driver();
      uint64_t faults = 0;
      for (const auto &bad : cases) {
        const uint64_t generation = activate_and_actuate(*driver);
        fake_q = bad.first;
        fake_qd = bad.second;
        require(write_once(*driver) == RT::OK, "write failed");
        const size_t live = sent.size();
        require(read_once(*driver) == RT::ERROR, "nonfinite feedback accepted while homed");
        require(sent.size() > live + 1, "no neutral packet attempted after nonfinite feedback");
        require_neutral_from(live + 1, "nonneutral packet after nonfinite feedback");
        const auto state = joint_state(*driver);
        require(state.first == double(0.02f) && state.second == 0.5, "nonfinite feedback reached joint state");
        const auto row = read_receipt();
        require(row[7] == 4 && row[5] == ++faults && row[4] == generation && row[21] == 3,
                "nonfinite fault not terminal with a checksum-valid mask");
        const auto error = last_error();
        require(operator_view(error) == "Board fault phase=read reason=nonfinite motor feedback" &&
                    error.find("nonfinite={fake_joint={board_rad=") != std::string::npos,
                "nonfinite diagnostic incomplete: " + error);
        require(driver->on_error(rclcpp_lifecycle::State()) == CR::SUCCESS, "on_error failed");
        require(driver->on_configure(rclcpp_lifecycle::State()) == CR::SUCCESS, "reconfigure failed");
      }
      std::cout << "PASS nonfinite cases=6 faults=" << faults << "\n";
    } else if (scenario == "post_home_fault") {
      // The first read after Home is invalid: fault, never healthy, no residual.
      auto driver = configured_driver();
      uint64_t faults = 0;
      for (int variant = 0; variant < 2; ++variant) {
        reply_mask = SPI_BOTH_REPLIES_VALID;
        fake_q = fake_qd = 0.f;
        require(driver->on_activate(rclcpp_lifecycle::State()) == CR::SUCCESS, "activation failed");
        auto row = read_receipt();
        // Home completion publishes nothing; the new generation appears with the next row.
        require(row[7] == 5, "Home completion changed the homing receipt");
        const uint64_t before = row[4];
        const auto homed_state = joint_state(*driver);
        if (variant == 0) reply_mask = 0; else fake_q = std::numeric_limits<float>::quiet_NaN();
        const size_t first = sent.size();
        require(read_once(*driver) == RT::ERROR, "invalid first read after Home accepted");
        require(sent.size() > first + 1 && !neutral(sent[first]),
                "post-Home fault: unexpected exchange sequence (Home packet, then neutral)");
        require_neutral_from(first + 1, "nonneutral packet after post-Home fault");
        require(joint_state(*driver) == homed_state, "invalid first read replaced the Home state");
        row = read_receipt();
        // The terminal row carries the new generation with status 4; consumers ignore generation unless status 1.
        require(row[7] == 4 && row[4] > before && row[5] == ++faults, "post-Home fault not terminal");
        require(logged(RCUTILS_LOG_SEVERITY_INFO, "FC_BOARD_HEALTH_V1 Home generation=" + std::to_string(row[4])),
                "terminal row does not carry this Home's generation");
        require(operator_view(last_error()) == std::string("Board fault phase=post_home reason=") +
                    (variant == 0 ? "missing or corrupt motor board reply" : "nonfinite motor feedback"),
                "post-Home diagnostic: " + last_error());
        require(count_logged(RCUTILS_LOG_SEVERITY_INFO, "FC_HOME_RESIDUAL_V1") == 0,
                "residual reported for an invalid first read");
        require(driver->on_error(rclcpp_lifecycle::State()) == CR::SUCCESS, "on_error failed");
        reply_mask = SPI_BOTH_REPLIES_VALID;
        fake_q = 0.f;
        require(read_once(*driver) == RT::OK && read_receipt()[7] == 4, "health recovered without a new Home");
        require(count_logged(RCUTILS_LOG_SEVERITY_INFO, "FC_HOME_RESIDUAL_V1") == 0, "residual after a fault");
        require(driver->on_configure(rclcpp_lifecycle::State()) == CR::SUCCESS, "reconfigure failed");
      }
      activate_and_actuate(*driver);
      require(count_logged(RCUTILS_LOG_SEVERITY_INFO, "FC_HOME_RESIDUAL_V1") == 1, "valid Home residual missing");
      std::cout << "PASS post_home_fault variants=mask0,nan faults=" << faults << "\n";
    } else if (scenario == "inactive") {
      // Not homed: invalid replies are tolerated, never copied, and every packet
      // is neutral even with a stale prepared payload in the command buffer.
      auto driver = configured_driver();
      const auto stale = [] {
        for (auto &flag : command.flags) flag = 1;
        command.kp_abad[0] = 5.f;
        command.q_des_abad[0] = 0.4f;
        command.tau_abad_ff[0] = 0.3f;
      };
      const auto tolerate = [&](const std::string &stage, std::pair<double, double> kept) {
        const size_t from = sent.size();
        reply_mask = 0;
        stale();
        require(read_once(*driver) == RT::OK, stage + ": invalid reply faulted while not homed");
        reply_mask = 1;
        require(write_once(*driver) == RT::OK && read_once(*driver) == RT::OK, stage + ": one-sided reply faulted");
        reply_mask = SPI_BOTH_REPLIES_VALID;
        fake_q = std::numeric_limits<float>::infinity();
        stale();
        require(read_once(*driver) == RT::OK, stage + ": nonfinite feedback faulted while not homed");
        require_neutral_from(from, stage + ": nonneutral packet while not homed");
        require(joint_state(*driver) == kept, stage + ": rejected feedback reached joint state");
      };
      tolerate("before activation", {0.0, 0.0});
      require(read_receipt()[7] == 0 && read_receipt()[5] == 0, "inactive rejection changed the receipt");
      require(logged(RCUTILS_LOG_SEVERITY_WARN, "Board feedback rejected while not homed (state kept) reason="),
              "inactive rejection not diagnosed");
      // Legitimate inactive feedback is still copied.
      fake_q = 0.01f;
      fake_qd = 0.f;
      require(read_once(*driver) == RT::OK && joint_state(*driver).first == double(0.01f), "valid inactive feedback not copied");
      const uint64_t first = activate_and_actuate(*driver);
      require(driver->on_deactivate(rclcpp_lifecycle::State()) == CR::SUCCESS, "deactivate failed");
      const auto kept = joint_state(*driver);
      tolerate("after deactivation", kept);
      require(read_receipt()[7] == 4 && read_receipt()[5] == 1, "inactive rejection changed the terminal receipt");
      const uint64_t second = activate_and_actuate(*driver);
      require(second > first, "reactivation after inactive rejections did not Home");
      std::cout << "PASS inactive tolerated=6 generations=" << first << "," << second << "\n";
    } else if (scenario == "stale_payload") {
      // After deactivate or a fault, write() then read() sends only neutral,
      // although the command arrays still hold the session's targets and gains.
      auto driver = configured_driver();
      activate_and_actuate(*driver);
      require(driver->on_deactivate(rclcpp_lifecycle::State()) == CR::SUCCESS, "deactivate failed");
      size_t from = sent.size();
      for (int i = 0; i < 3; ++i) require(write_once(*driver) == RT::OK && read_once(*driver) == RT::OK, "cycle failed");
      require_neutral_from(from, "stale payload sent after deactivate");
      activate_and_actuate(*driver);
      reply_mask = 0;
      require(read_once(*driver) == RT::ERROR, "fault not raised");
      reply_mask = SPI_BOTH_REPLIES_VALID;
      from = sent.size();
      for (int i = 0; i < 3; ++i) require(write_once(*driver) == RT::OK && read_once(*driver) == RT::OK, "cycle failed");
      require_neutral_from(from, "stale payload sent after fault");
      std::cout << "PASS stale_payload deactivate=neutral fault=neutral\n";
    } else if (scenario == "home_residual") {
      // Home with a fake encoder at 0.3 rad, so the zero offset is nonzero; then
      // report the first valid read: commanded (packet), rest, measured.
      auto driver = configured_driver();
      const std::regex line(R"(^FC_HOME_RESIDUAL_V1 generation=(\d+) frame=joint_after_home_offset units=rad )"
                            R"(sampled_mono_us=\d+ joints=1 fake_joint=\{commanded=(\S+) rest=(\S+) measured=(\S+) )"
                            R"(measured_minus_commanded=(\S+) measured_minus_rest=(\S+) commanded_kp=(\S+)\} assessment=none$)");
      std::vector<uint64_t> generations;
      for (int home = 0; home < 2; ++home) {
        reply_mask = SPI_BOTH_REPLIES_VALID;
        fake_q = 0.3f;
        fake_qd = 0.f;
        require(driver->on_activate(rclcpp_lifecycle::State()) == CR::SUCCESS, "activation failed");
        require(count_logged(RCUTILS_LOG_SEVERITY_INFO, "FC_HOME_RESIDUAL_V1") == size_t(home),
                "residual reported before the first read");
        fake_q = 0.34f;
        for (int i = 0; i < 4; ++i) require(read_once(*driver) == RT::OK, "read failed");
        require(count_logged(RCUTILS_LOG_SEVERITY_INFO, "FC_HOME_RESIDUAL_V1") == size_t(home + 1),
                "residual not reported exactly once per Home");
        std::string text;
        {
          std::lock_guard<std::mutex> lock(log_mutex);
          for (auto it = logs.rbegin(); it != logs.rend() && text.empty(); ++it)
            if (it->second.rfind("FC_HOME_RESIDUAL_V1", 0) == 0) text = it->second;
        }
        std::smatch m;
        require(std::regex_match(text, m, line), "residual line not parseable: " + text);
        const auto near = [](const std::string &v, double want) { return std::abs(std::stod(v) - want) < 1e-6; };
        const uint64_t generation = std::stoull(m[1]);
        require(generation == read_receipt()[4], "residual generation differs from the receipt");
        require(near(m[2], 0.05) && near(m[3], 0.05) && near(m[4], 0.04) && near(m[5], -0.01) &&
                    near(m[6], -0.01) && near(m[7], 1.0),
                "residual values wrong: " + text);
        generations.push_back(generation);
        require(driver->on_deactivate(rclcpp_lifecycle::State()) == CR::SUCCESS, "deactivate failed");
      }
      require(generations[1] > generations[0], "second Home reused the generation");
      std::cout << "PASS home_residual lines=2 measured=0.04 commanded=0.05 rest=0.05\n";
    } else if (scenario == "homing_nonfinite") {
      // A checksum-valid NaN encoder value mid-Home stops the Home with neutral;
      // the fake transport asserts that no NaN/Inf command is ever transmitted.
      nan_after_homing_packets = 3;
      auto driver = configured_driver();
      require(driver->on_activate(rclcpp_lifecycle::State()) == CR::ERROR, "Home completed on NaN feedback");
      require_neutral_last();
      const auto row = read_receipt();
      require(row[7] == 4 && row[4] == 0 && row[5] == 1, "failed Home produced a generation");
      require(operator_view(last_error()) == "Board fault phase=homing reason=nonfinite motor feedback",
              "homing nonfinite diagnostic: " + last_error());
      require(std::isfinite(joint_state(*driver).first), "NaN reached joint state during Home");
      std::cout << "PASS homing_nonfinite generation=0 faults=1\n";
    } else if (scenario == "command_nonfinite") {
      // Homed: NaN, +Inf or -Inf in any controller command field is refused by
      // write() before any transmission, whatever magnitude a clamp would give it.
      const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
      auto driver = configured_driver();
      uint64_t faults = 0;
      size_t cases = 0;
      std::vector<std::string> admitted;
      for (const char *field : {"position", "velocity", "effort", "kp", "kd"}) {
        for (double bad : {nan, inf, -inf}) {
          const std::string token = std::string(field) + "=" + text(bad);
          const uint64_t generation = activate_and_actuate(*driver);
          set_command(*driver, field, bad);
          const std::string outcome = refuse_command(*driver, "nonfinite actuator command", token, generation, ++faults);
          ++cases;
          if (!outcome.empty()) {
            admitted.push_back(token + ": " + outcome);
            driver->on_error(rclcpp_lifecycle::State());
            faults = read_receipt()[5];
          } else {
            stays_neutral(*driver, token, generation, faults);
          }
          require(driver->on_configure(rclcpp_lifecycle::State()) == CR::SUCCESS, "reconfigure failed");
        }
      }
      if (!admitted.empty()) {
        std::string all;
        for (const auto &line : admitted) all += "\n  " + line;
        throw std::runtime_error(std::to_string(admitted.size()) + "/" + std::to_string(cases) +
                                 " nonfinite commands admitted:" + all);
      }
      // Only an explicit activation and Home restore valid control.
      const uint64_t generation = activate_and_actuate(*driver);
      require(read_receipt()[7] == 1 && read_receipt()[5] == faults && generation > 0, "new Home did not recover");
      std::cout << "PASS command_nonfinite cases=" << cases << " faults=" << faults << " recovered_by_home=1\n";
    } else if (scenario == "command_encoded") {
      // A finite command whose packet value the float wire format cannot hold:
      // with kp=0 there is no position clamp, so the cast would produce +/-Inf.
      auto driver = configured_driver();
      uint64_t faults = 0;
      std::vector<std::string> admitted;
      for (double far : {1e39, -1e39, std::numeric_limits<double>::max()}) {
        const uint64_t generation = activate_and_actuate(*driver);
        set_command(*driver, "kp", 0.0);
        set_command(*driver, "position", far);
        const std::string outcome =
            refuse_command(*driver, "nonfinite encoded actuator command", "position=" + text(far), generation, ++faults);
        if (!outcome.empty()) {
          admitted.push_back("position=" + text(far) + " kp=0: " + outcome);
          driver->on_error(rclcpp_lifecycle::State());
          faults = read_receipt()[5];
        } else {
          stays_neutral(*driver, "position=" + text(far), generation, faults);
        }
        require(driver->on_configure(rclcpp_lifecycle::State()) == CR::SUCCESS, "reconfigure failed");
      }
      if (!admitted.empty()) {
        std::string all;
        for (const auto &line : admitted) all += "\n  " + line;
        throw std::runtime_error(std::to_string(admitted.size()) + "/3 unencodable commands admitted:" + all);
      }
      // Unchanged: with kp > 0 the existing position limit clamps the same value.
      activate_and_actuate(*driver);
      set_command(*driver, "kp", 1.0);
      set_command(*driver, "position", 1e39);
      require(write_once(*driver) == RT::OK && read_once(*driver) == RT::OK, "limit-clamped command refused");
      require(sent.back().q_des_abad[0] == 1.f && sent.back().kp_abad[0] == 1.f && read_receipt()[7] == 1,
              "position limit clamp changed: " + sent_fields(sent.size() - 1));
      std::cout << "PASS command_encoded cases=3 faults=" << faults << " clamped_kp_positive=1\n";
    } else if (scenario == "command_inactive") {
      // Not homed (before activation, after deactivation): invalid commands are
      // never prepared or sent, and do not fault, log or change the receipt.
      const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
      auto driver = configured_driver();
      const auto poison = [&] {
        set_command(*driver, "position", nan);
        set_command(*driver, "velocity", inf);
        set_command(*driver, "effort", -inf);
        set_command(*driver, "kp", inf);
        set_command(*driver, "kd", nan);
      };
      const auto inactive = [&](const std::string &stage, uint64_t status, uint64_t faults) {
        const size_t errors = count_logged(RCUTILS_LOG_SEVERITY_ERROR, "");
        const size_t from = sent.size();
        poison();
        for (int i = 0; i < 3; ++i)
          require(write_once(*driver) == RT::OK && read_once(*driver) == RT::OK, stage + ": invalid inactive command faulted");
        require(sent.size() == from + 3, stage + ": unexpected exchange count");
        require_neutral_from(from, stage + ": nonneutral packet while not homed");
        const auto row = read_receipt();
        require(row[7] == status && row[5] == faults, stage + ": inactive command changed the receipt");
        require(count_logged(RCUTILS_LOG_SEVERITY_ERROR, "") == errors, stage + ": inactive command logged an ERROR");
      };
      inactive("before activation", 0, 0);
      // on_activate discards the invalid values; Home and control work as before.
      const uint64_t first = activate_and_actuate(*driver);
      require(driver->on_deactivate(rclcpp_lifecycle::State()) == CR::SUCCESS, "deactivate failed");
      inactive("after deactivation", 4, 1);
      const uint64_t second = activate_and_actuate(*driver);
      require(second > first && read_receipt()[5] == 1, "reactivation after invalid inactive commands failed");
      std::cout << "PASS command_inactive stages=2 neutral=1 generations=" << first << "," << second << "\n";
    } else if (scenario == "command_homing") {
      // Home computes its own commands; a nonfinite one (here from an infinite
      // homing velocity) stops the Home with neutral before it is transmitted.
      auto info = fake_info(true);
      info.joints[0].parameters["homing_velocity"] = "inf";
      max_exchanges = 1000;  // The fault is expected within the first Home steps (~14 exchanges).
      auto driver = std::make_unique<Driver>();
      require(driver->on_init(info) == CR::SUCCESS, "init failed");
      require(driver->on_activate(rclcpp_lifecycle::State()) == CR::ERROR, "Home transmitted a nonfinite command");
      // Deterministic fake: 1 neutral probe, 10 enable exchanges and the 2 setup
      // steps. The refused first search step exchanges nothing, not even the
      // previously admitted packet, before the fault's single neutral packet.
      require(sent.size() == 14 && neutral(sent[13]) && !neutral(sent[12]),
              "refused Home step was exchanged before neutral: " + std::to_string(sent.size()) + " exchanges");
      const auto row = read_receipt();
      require(row[7] == 4 && row[4] == 0 && row[5] == 1, "failed Home produced a generation");
      require(operator_view(last_error()) == "Command fault phase=homing reason=nonfinite actuator command" &&
                  last_error().find("position=inf") != std::string::npos,
              "homing command diagnostic: " + last_error());
      require(!logged(RCUTILS_LOG_SEVERITY_INFO, "Successfully activated!"), "failed Home reported success");
      std::cout << "PASS command_homing generation=0 faults=1\n";
    } else {
      throw std::runtime_error("unknown scenario");
    }
    __real_unlink(receipt_path.c_str());
    rmdir(base);
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    if (!receipt_path.empty()) __real_unlink(receipt_path.c_str());
    rmdir(base);
    return 1;
  }
}
