// Exercise the actual production serializer only. Never initialize SPI or send.
#include "rt/rt_spi.h"
#include <cstring>
#include <iostream>
#include <stdexcept>
extern void spi_to_spine(spi_command_t *, spine_cmd_t *, int);
extern "C" int __wrap_open(const char *, int, ...) { throw std::runtime_error("unexpected device open"); }
extern "C" int __wrap_open64(const char *, int, ...) { throw std::runtime_error("unexpected device open64"); }
extern "C" int __wrap_ioctl(int, unsigned long, ...) { throw std::runtime_error("unexpected device ioctl"); }
int main() {
  try {
    spi_command_t zero{};
    for (int first_channel : {0, 2}) {
      spine_cmd_t encoded; std::memset(&encoded, 0x55, sizeof encoded);
      spi_to_spine(&zero, &encoded, first_channel);
      const float *fields[] = {encoded.q_des_abad, encoded.q_des_hip, encoded.q_des_knee,
        encoded.qd_des_abad, encoded.qd_des_hip, encoded.qd_des_knee,
        encoded.kp_abad, encoded.kp_hip, encoded.kp_knee,
        encoded.kd_abad, encoded.kd_hip, encoded.kd_knee,
        encoded.tau_abad_ff, encoded.tau_hip_ff, encoded.tau_knee_ff};
      // Signed zero from coordinate conversion is numerically neutral; wire
      // bytes/checksum need not be all-zero. Do not infer firmware acceptance.
      for (const auto *field : fields)
        for (int leg = 0; leg < 2; ++leg)
          if (field[leg] != 0.0f) throw std::runtime_error("nonzero encoded command");
      for (int flag : encoded.flags) if (flag != 0) throw std::runtime_error("encoded enable flag");
      uint32_t words[33]; static_assert(sizeof words == sizeof encoded, "board layout changed");
      std::memcpy(words, &encoded, sizeof words); uint32_t checksum = 0;
      for (int i = 0; i < 32; ++i) checksum ^= words[i];
      if (words[32] != checksum) throw std::runtime_error("incorrect board checksum");
    }
    std::cout << "PASS actual_serializer board_groups=2 all_command_values_zero=1 enable_flags=0\n";
    return 0;
  } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
