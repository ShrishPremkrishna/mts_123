#pragma once

// Pure SH-2/SHTP parsing. No Linux, ROS, I2C or motor dependencies.
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace pupper_bno {

struct Packet {
  std::array<uint8_t, 4> header{};
  std::vector<uint8_t> payload;
};
enum class ReadResult { packet, empty, invalid, timed_out };

inline unsigned packet_length(const uint8_t *p) {
  return (unsigned(p[0]) | (unsigned(p[1]) << 8)) & 0x7fff;
}

// read(dst,n) returns the actual byte count, now() monotonic microseconds.
// The deadline bounds retries, not a kernel read that itself blocks indefinitely.
// A header-only I2C read does not consume payload; subsequent reads repeat a
// header. A zero repeated header means no payload is currently available, even
// when the syscall returned the requested count. Never consume its padding.
template <class Read, class Now, class Wait>
ReadResult read_packet(Packet &out, Read read, Now now, Wait wait,
                       uint64_t budget_us = 25000, unsigned max_payload = 4096) {
  out = Packet{};
  const uint64_t started = now();
  auto expired = [&]() { return now() - started >= budget_us; };
  std::array<uint8_t, 4> first{};
  if (read(first.data(), first.size()) != 4) return ReadResult::invalid;
  const unsigned length = packet_length(first.data());
  if (length == 0) return ReadResult::empty;
  if (length < 4 || length-4 > max_payload || first[2] > 5) return ReadResult::invalid;
  const bool orphan = (first[1] & 0x80) != 0;
  Packet pending;
  pending.header = first;
  pending.payload.reserve(length-4);
  unsigned remaining = length-4;
  uint8_t last_sequence = first[3];
  while (remaining) {
    if (expired()) return ReadResult::timed_out;
    const unsigned count = std::min(remaining, 28u);
    std::array<uint8_t, 32> chunk{};
    if (read(chunk.data(), count+4) != int(count+4)) return ReadResult::invalid;
    if (packet_length(chunk.data()) == 0) {
      // Wait for the exact remaining continuation. The extra header read also
      // prevents an empty padded transfer being mistaken for report bytes.
      for (;;) {
        if (expired()) return ReadResult::timed_out;
        wait();
        if (expired()) return ReadResult::timed_out;
        std::array<uint8_t, 4> resumed{};
        if (read(resumed.data(), 4) != 4) return ReadResult::invalid;
        const unsigned resumed_length = packet_length(resumed.data());
        if (resumed_length == 0) continue;
        if (resumed_length != remaining+4 || resumed[2] != first[2]
            || !(resumed[1] & 0x80) || resumed[3] != uint8_t(last_sequence+1)) return ReadResult::invalid;
        last_sequence = resumed[3];
        break;
      }
      continue;
    }
    if (packet_length(chunk.data()) != remaining+4 || chunk[2] != first[2]
        || !(chunk[1] & 0x80) || chunk[3] != uint8_t(last_sequence+1)) return ReadResult::invalid;
    last_sequence = chunk[3];
    pending.payload.insert(pending.payload.end(), chunk.begin()+4, chunk.begin()+4+count);
    remaining -= count;
  }
  if (expired()) return ReadResult::timed_out;
  // A continuation encountered without its beginning is drained but never
  // parsed. No guessing at an arbitrary payload offset after a prior failure.
  if (orphan) return ReadResult::invalid;
  out = std::move(pending);
  return ReadResult::packet;
}

inline uint16_t u16(const uint8_t *p) { return uint16_t(p[0]) | uint16_t(p[1]) << 8; }
inline int16_t i16(const uint8_t *p) {
  const unsigned value = u16(p);
  return int16_t(value >= 32768 ? int(value)-65536 : int(value));
}
inline int32_t i32(const uint8_t *p) {
  const uint32_t value = uint32_t(p[0]) | uint32_t(p[1]) << 8
                         | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
  const int64_t signed_value = value >= 0x80000000u ? int64_t(value)-0x100000000ll : value;
  return int32_t(signed_value);
}

struct ReportSequence {
  bool seen = false;
  uint8_t last = 0;
  bool accept(uint8_t next) {
    const uint8_t advance = uint8_t(next-last);
    if (seen && (advance == 0 || advance >= 128)) return false;
    last = next; seen = true; return true;
  }
};

struct Report {
  uint8_t id = 0, sequence = 0, accuracy = 0;
  uint16_t delay_ticks = 0;
  int64_t reference_delta_ticks = 0;
  bool timing_valid = false;
  std::array<int16_t, 4> values{};
  uint16_t rotation_accuracy = 0;
  uint8_t stability = 0;
};

// Parse an entire known batch atomically. Unknown/truncated records cannot
// shift the cursor or partially update cached sensor state. Report timestamps
// remain protocol deltas; without captured HINT they are not absolute times.
inline bool parse_reports(const std::vector<uint8_t> &data, std::vector<Report> &out) {
  out.clear();
  std::vector<Report> pending;
  size_t cursor = 0;
  int64_t reference = 0;
  bool have_reference = false, timing_valid = false;
  while (cursor < data.size()) {
    const uint8_t id = data[cursor];
    unsigned size = 0;
    switch (id) {
      case 0xfb: case 0xfa: size = 5; break;
      case 0x02: size = 10; break;
      case 0x05: size = 14; break;
      case 0x08: size = 12; break;
      case 0x13: size = 6; break;
      default: return false;
    }
    if (cursor+size > data.size()) return false;
    const uint8_t *r = data.data()+cursor;
    if (id == 0xfb) {
      const int32_t delta = i32(r+1);
      reference = -int64_t(delta);
      have_reference = true;
      timing_valid = delta != 0x7fffffff;
    } else if (id == 0xfa) {
      if (!have_reference) return false;
      reference += i32(r+1);
    } else {
      if (!have_reference) return false;
      Report report;
      report.id = id; report.sequence = r[1]; report.accuracy = r[2] & 3;
      report.delay_ticks = uint16_t(r[2] & 0xfc) << 6 | r[3];
      report.reference_delta_ticks = reference;
      report.timing_valid = timing_valid;
      if (id == 0x13) report.stability = r[4];
      else {
        for (unsigned k=0; k<(id == 0x02 ? 3u : 4u); ++k) report.values[k] = i16(r+4+k*2);
        if (id == 0x05) report.rotation_accuracy = u16(r+12);
      }
      pending.push_back(report);
    }
    cursor += size;
  }
  out = std::move(pending);
  return have_reference && !out.empty();
}

}  // namespace pupper_bno
