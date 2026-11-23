#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace pupper_imu {

// One writer, read-only consumers. The sequence protects the entire snapshot.
// A new driver replaces the path with a new inode and monotonic instance ID.
class BoardHealth {
 public:
  using Clock = std::chrono::steady_clock;
  static uint64_t ns(Clock::time_point t) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
  }
  bool open(const char *path) {
    if (!path || path[0]!='/') return false;
    std::string tmp = std::string(path)+".XXXXXX";
    int fd = mkstemp(tmp.data());
    if (fd < 0) return false;
    bool ok = ftruncate(fd, length) == 0;
    if (ok) {
      auto *m = mmap(nullptr, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
      if (m != MAP_FAILED) words = static_cast<uint64_t *>(m);
      ok = words && mlock(words, length)==0;
    }
    if (ok) {
      for (unsigned i=0;i<24;++i) words[i]=0;
      ok = fchmod(fd, 0400)==0 && rename(tmp.c_str(),path)==0;
    }
    ::close(fd);
    if (!ok) { unlink(tmp.c_str()); if (words) { munmap(words,length); words=nullptr; } }
    return ok;
  }
  ~BoardHealth() { if (words) munmap(words,length); }
  template<class Output>
  void publish(unsigned status, const Output &raw, const Output &selected, unsigned spi, bool alternative) {
    if (!words) return;
    if (writing.test_and_set(std::memory_order_acquire)) {
      // Stop/error can interrupt an ordinary publisher. Revoke its mapped
      // authority now, including if that thread gets descheduled mid-snapshot.
      // An odd sequence or a terminal status both fail closed for consumers.
      if (terminal.load()) {
        __atomic_store_n(words+7,uint64_t{4},__ATOMIC_RELEASE);
        __atomic_store_n(words+5,faults.load(),__ATOMIC_RELEASE);
      }
      return;
    }
    const auto seq=__atomic_load_n(words,__ATOMIC_RELAXED);
    __atomic_store_n(words,seq+1,__ATOMIC_SEQ_CST);
    const auto report = [](int n) {return static_cast<uint64_t>(n<0?256:n);};
    const auto age = [](std::chrono::microseconds t) {return t.count()<0 ? UINT64_MAX : uint64_t(t.count())*1000;};
    const uint64_t payload[] = {
      0x314d494346505550ULL, uint64_t(getpid()), instance, homes.load(), faults.load(), incidents.load(),
      terminal.load() ? 4u : status, ns(Clock::now()), ns(selected.quaternion_receipt_time), ns(selected.gyro_receipt_time),
      age(selected.quaternion_minimum_age_at_receipt), age(selected.gyro_minimum_age_at_receipt),
      report(selected.quaternion_report_sequence), report(selected.gyro_report_sequence), alternative?8u:5u,alternative?7u:2u,
      ns(raw.quaternion_receipt_time), ns(raw.gyro_receipt_time), age(raw.quaternion_minimum_age_at_receipt),
      age(raw.gyro_minimum_age_at_receipt), spi, profile, 0
    };
    for (unsigned i=0;i<23;++i) __atomic_store_n(words+i+1,payload[i],__ATOMIC_RELAXED);
    // A terminal publisher which lost the writer flag has already revoked the
    // snapshot. Never let a previously constructed primary payload undo that.
    if (terminal.load()) {
      __atomic_store_n(words+7,uint64_t{4},__ATOMIC_RELAXED);
      __atomic_store_n(words+5,faults.load(),__ATOMIC_RELAXED);
    }
    __atomic_store_n(words,seq+2,__ATOMIC_RELEASE);
    writing.clear(std::memory_order_release);
  }
  unsigned profile=1;
  std::atomic<uint64_t> homes{0},faults{0},incidents{0};
  std::atomic<bool> terminal{false};
 private:
  static constexpr size_t length=24*sizeof(uint64_t);
  uint64_t *words=nullptr;
  const uint64_t instance=ns(Clock::now());
  std::atomic_flag writing=ATOMIC_FLAG_INIT;
};
}
