// Exercise the actual production serializer only. Never initialize SPI or send.
#include "rt/rt_spi.h"
#include <cstring>
#include <iostream>
#include <algorithm>
#include <array>
#include <cstdarg>
#include <linux/spi/spidev.h>
#include <stdexcept>
extern void spi_to_spine(spi_command_t *, spine_cmd_t *, int);
extern "C" int __wrap_open(const char *, int, ...) { throw std::runtime_error("unexpected device open"); }
extern "C" int __wrap_open64(const char *, int, ...) { throw std::runtime_error("unexpected device open64"); }
namespace {
enum class Reply { good, short_transfer, failed, corrupt, empty };
std::array<Reply, 2> replies;
unsigned calls = 0;
std::array<unsigned,2> closes{};
}
extern int spi_1_fd,spi_2_fd;
extern "C" int __wrap_close(int fd) {
  if(fd!=61 && fd!=62) throw std::runtime_error("unexpected close");
  ++closes[fd-61]; return 0;
}
extern "C" int __wrap_ioctl(int, unsigned long request, ...) {
  if (request != SPI_IOC_MESSAGE(1)) throw std::runtime_error("unexpected device ioctl");
  va_list args; va_start(args, request);
  auto *transfer = va_arg(args, spi_ioc_transfer *);
  va_end(args);
  spine_data_t response{};
  auto mode = replies.at(calls++);
  if (mode != Reply::empty) {
    response.q_abad[0] = 1.0f;
    response.qd_abad[0] = -0.5f;
    uint32_t words[15];
    std::memcpy(words, &response, sizeof(response));
    for (int i=0;i<14;++i) response.checksum ^= words[i];
    if (mode == Reply::corrupt) ++response.checksum;
  }
  uint16_t words[30];
  std::memcpy(words, &response, sizeof(response));
  auto *out = reinterpret_cast<uint16_t *>(transfer->rx_buf);
  for (size_t i=0;i<30;++i) out[i]=__builtin_bswap16(words[i]);
  if (mode == Reply::failed) return -1;
  if (mode == Reply::short_transfer) return transfer->len - 1;
  return transfer->len;
}
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
    spi_data_t data{};
    for (auto missing : {Reply::good, Reply::short_transfer, Reply::failed, Reply::corrupt, Reply::empty}) {
      for (int board : {0, 1}) {
        replies.fill(Reply::good);
        replies[board] = missing;
        calls=0;
        spi_send_receive(&zero,&data);
        const int expected=missing==Reply::good ? 3 : 3^(1<<board);
        if (calls!=2 || (data.spi_driver_status & 3) != expected)
          throw std::runtime_error("SPI receipt accepted invalid exchange or retained old good bit");
      }
    }
    spi_1_fd=61; spi_2_fd=62;
    close_spi(); close_spi();
    if(closes[0]!=1 || closes[1]!=1 || spi_1_fd!=-1 || spi_2_fd!=-1)
      throw std::runtime_error("SPI descriptors leaked or closed twice");
    std::cout << "PASS actual_serializer board_groups=2 all_command_values_zero=1 enable_flags=0\n";
    return 0;
  } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
