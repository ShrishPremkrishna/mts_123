// Actual rt_spi exchange and reply-validity bits against scripted fake spidev
// replies. open/ioctl/close are wrapped; no device is ever opened.
#include "rt/rt_spi.h"

#include <array>
#include <cerrno>
#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

extern uint32_t spi_driver_iterations;

namespace {
enum class Reply { valid, zero, corrupt, short_transfer, failed, nonfinite, moved };
constexpr int kBoard1 = 41, kBoard2 = 42;
std::array<Reply, 2> script{Reply::valid, Reply::valid};
std::vector<int> closes;
int opens = 0;
void require(bool ok, const char *why) { if (!ok) throw std::runtime_error(why); }

spine_data_t reply_for(int board) {
  spine_data_t d{};
  d.q_abad[0] = 0.5f + board;
  d.q_abad[1] = -0.25f;
  d.qd_knee[1] = 2.0f;
  d.flags[0] = 1;
  if (script[board] == Reply::moved) d.q_knee[0] = 0.125f + board;
  if (script[board] == Reply::nonfinite) {
    d.q_abad[0] = std::numeric_limits<float>::quiet_NaN();
    d.qd_hip[1] = board ? -std::numeric_limits<float>::infinity() : std::numeric_limits<float>::infinity();
  }
  uint32_t words[15];
  std::memcpy(words, &d, sizeof words);
  uint32_t checksum = 0;
  for (int i = 0; i < 14; ++i) checksum ^= words[i];
  d.checksum = static_cast<int32_t>(checksum);
  return d;
}

uint16_t swap16(uint16_t v) { return static_cast<uint16_t>((v >> 8) | ((v & 0xff) << 8)); }

// Board group g owns legs 2g and 2g+1.
bool group_unchanged(const spi_data_t &a, const spi_data_t &b, int group) {
  for (int leg = 2 * group; leg < 2 * group + 2; ++leg) {
    const float fa[] = {a.q_abad[leg], a.q_hip[leg], a.q_knee[leg], a.qd_abad[leg], a.qd_hip[leg], a.qd_knee[leg]};
    const float fb[] = {b.q_abad[leg], b.q_hip[leg], b.q_knee[leg], b.qd_abad[leg], b.qd_hip[leg], b.qd_knee[leg]};
    if (std::memcmp(fa, fb, sizeof fa) != 0 || a.flags[leg] != b.flags[leg]) return false;
  }
  return true;
}

int run(std::array<Reply, 2> replies) {
  script = replies;
  spi_driver_run();
  return get_spi_data()->spi_driver_status & SPI_BOTH_REPLIES_VALID;
}
}  // namespace

extern "C" int __wrap_open(const char *path, int flags, ...) {
  require((flags & O_CLOEXEC) && (flags & O_RDWR), "SPI descriptor flags");
  ++opens;
  if (std::strcmp(path, "/dev/spidev0.0") == 0) return kBoard1;
  if (std::strcmp(path, "/dev/spidev0.1") == 0) return kBoard2;
  throw std::runtime_error("unexpected open");
}
extern "C" int __wrap_open64(const char *path, int flags, ...) { return __wrap_open(path, flags); }
extern "C" int __wrap_close(int fd) {
  require(fd == kBoard1 || fd == kBoard2, "unexpected close");
  closes.push_back(fd);
  return 0;
}
extern "C" int __wrap_ioctl(int fd, unsigned long request, ...) {
  require(fd == kBoard1 || fd == kBoard2, "unexpected ioctl descriptor");
  if (request != SPI_IOC_MESSAGE(1)) return 0;  // Mode/speed/word configuration.
  va_list args;
  va_start(args, request);
  auto *message = va_arg(args, struct spi_ioc_transfer *);
  va_end(args);
  const int board = fd == kBoard1 ? 0 : 1;
  auto *rx = reinterpret_cast<uint16_t *>(static_cast<uintptr_t>(message->rx_buf));
  const Reply reply = script[board];
  if (reply == Reply::failed) { errno = EIO; return -1; }
  if (reply != Reply::zero) {
    spine_data_t d = reply_for(board);
    if (reply == Reply::corrupt) d.q_hip[0] = 1.0f;  // Payload changed after checksum.
    uint16_t words[30];
    std::memcpy(words, &d, sizeof words);
    for (int i = 0; i < 30; ++i) rx[i] = swap16(words[i]);
  }
  return reply == Reply::short_transfer ? static_cast<int>(message->len) - 2 : static_cast<int>(message->len);
}

int main() {
  try {
    init_spi();
    require(opens == 2 && closes.empty(), "init did not open exactly both SPI devices");
    require(run({Reply::valid, Reply::valid}) == SPI_BOTH_REPLIES_VALID, "two valid replies rejected");
    const auto *data = get_spi_data();
    require(data->q_abad[0] == -0.5f && data->q_abad[2] == -1.5f && data->qd_knee[3] == -2.0f,
            "valid reply decoding changed");
    require((static_cast<uint32_t>(data->spi_driver_status) >> 16) == spi_driver_iterations,
            "iteration counter bits changed");
    // A rejected group keeps its last valid decoded values; the accepted group updates.
    spi_data_t before = *data;
    require(run({Reply::moved, Reply::zero}) == 1, "all-zero reply accepted (XOR alone passes zeros)");
    require(!group_unchanged(*data, before, 0) && data->q_knee[0] == -0.125f, "valid group did not update");
    require(group_unchanged(*data, before, 1) && data->q_abad[2] == -1.5f,
            "all-zero reply replaced the group's last valid values");
    before = *data;
    require(run({Reply::corrupt, Reply::moved}) == 2, "corrupt checksum accepted");
    require(group_unchanged(*data, before, 0), "corrupt reply was decoded into data");
    require(data->q_knee[2] == -1.125f, "valid group did not update");
    before = *data;
    require(run({Reply::short_transfer, Reply::failed}) == 0, "short or failed transfer accepted");
    require(group_unchanged(*data, before, 0) && group_unchanged(*data, before, 1),
            "short or failed transfer was decoded into data");
    require(run({Reply::valid, Reply::valid}) == SPI_BOTH_REPLIES_VALID, "validity was sticky after failure");
    before = *data;
    require(run({Reply::failed, Reply::failed}) == 0, "validity was sticky after success");
    require(group_unchanged(*data, before, 0) && group_unchanged(*data, before, 1),
            "failed transfers changed data");

    // Checksum validity says nothing about finiteness: the transport accepts and
    // decodes the frame, and the driver must reject it (joint_feedback_error).
    require(run({Reply::nonfinite, Reply::nonfinite}) == SPI_BOTH_REPLIES_VALID, "nonfinite frame changed transport mask");
    require(std::isnan(data->q_abad[0]) && std::isinf(data->qd_hip[1]) && std::isinf(data->qd_hip[3]),
            "nonfinite values not decoded as such");

    // Unsigned counter: shifting past bit 31 and wrapping are defined.
    spi_driver_iterations = 0xFFFFu;
    require(run({Reply::valid, Reply::valid}) == SPI_BOTH_REPLIES_VALID, "wrap changed validity bits");
    require((static_cast<uint32_t>(get_spi_data()->spi_driver_status) & 0xFFFF0000u) == 0, "high bits not wrapped");
    spi_driver_iterations = 0xFFFFFFFFu;
    require(run({Reply::valid, Reply::zero}) == 1 && spi_driver_iterations == 0, "iteration counter did not wrap");

    // Reinitialization releases the earlier descriptors; close is idempotent.
    init_spi();
    require(opens == 4 && closes == (std::vector<int>{kBoard1, kBoard2}), "reinit leaked descriptors");
    close_spi();
    close_spi();
    require(closes == (std::vector<int>{kBoard1, kBoard2, kBoard1, kBoard2}), "close_spi not idempotent");
    std::cout << "PASS spi_reply_status masks=3,1,2,0,3,0,3 rejected_groups_kept=1 nonfinite_passed_to_driver=1 wrap=defined cloexec=1 close_idempotent=1\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
