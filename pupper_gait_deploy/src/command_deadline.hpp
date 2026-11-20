#pragma once
// Source-deadline transport for the explicitly selected, same-host FC velocity
// path. Parsing/state admission run outside update(); update sees an immutable
// sample. This contract bounds requested velocity, not physical stopping.
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <time.h>
#include <json.hpp>  // Existing RTNeural/modules/json include path; no new dependency.

namespace neural_controller {
constexpr const char *kCommandDeadlineContract = "fc_deadline_v1";
constexpr const char *kCommandDeadlineSchema = "pupper_body_command_deadline_v1";
constexpr const char *kCommandDeadlineTopic = "/fc/body_command/admitted";
constexpr std::int64_t kCommandDeadlineMaxNs = 150000000;

inline std::int64_t command_monotonic_ns() noexcept {
  timespec time{};
  if (clock_gettime(CLOCK_MONOTONIC, &time) != 0 || time.tv_sec < 0 ||
      time.tv_sec > (std::numeric_limits<std::int64_t>::max() - time.tv_nsec) / 1000000000)
    return 0;  // Fail closed at point of use; no exception escapes update().
  return static_cast<std::int64_t>(time.tv_sec) * 1000000000 + time.tv_nsec;
}
inline std::string command_clock_id() {
  std::ifstream input("/proc/sys/kernel/random/boot_id");
  std::string boot; std::getline(input, boot);
  struct stat info{};
  bool valid_boot = boot.size() == 36;
  for (std::size_t i = 0; i < boot.size(); ++i) {
    const bool dash = i == 8 || i == 13 || i == 18 || i == 23;
    if (dash ? boot[i] != '-' : !((boot[i] >= '0' && boot[i] <= '9') || (boot[i] >= 'a' && boot[i] <= 'f')))
      valid_boot = false;
  }
  if (!valid_boot || stat("/proc/self/ns/time", &info) != 0 || info.st_ino == 0)
    throw std::runtime_error("same-host monotonic clock identity unavailable");
  return "linux-monotonic:" + boot + ":" + std::to_string(info.st_ino);
}

struct CommandDeadline {
  std::string clock_id, source_instance;
  std::int64_t seq = 0, issued = 0, expires = 0;
  double vx = 0., vy = 0., wz = 0.;
  bool allowed = false;
  bool zero() const { return vx == 0. && vy == 0. && wz == 0.; }
  bool same_origin(const CommandDeadline &other) const {
    return clock_id == other.clock_id && source_instance == other.source_instance &&
      seq == other.seq && issued == other.issued && expires == other.expires &&
      vx == other.vx && vy == other.vy && wz == other.wz;
  }
};

inline bool decode_command_deadline(const std::string &text, CommandDeadline &out) {
  if (text.empty() || text.size() > 1024) return false;
  if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xef &&
      static_cast<unsigned char>(text[1]) == 0xbb && static_cast<unsigned char>(text[2]) == 0xbf) return false;
  try {
    bool malformed = false;
    std::set<std::string> keys;
    auto callback = [&](int depth, nlohmann::json::parse_event_t event, nlohmann::json &value) {
      using Event = nlohmann::json::parse_event_t;
      if (event == Event::array_start || (event == Event::object_start && depth != 0)) malformed = true;
      if (event == Event::key && !keys.insert(value.get<std::string>()).second) malformed = true;
      return true;
    };
    const auto value = nlohmann::json::parse(text, callback);
    const std::set<std::string> expected{"schema", "clock_id", "source_instance", "seq",
      "issued_mono_ns", "expires_mono_ns", "vx", "vy", "wz", "allowed"};
    if (malformed || !value.is_object() || keys != expected || value.size() != 10 ||
        value.at("schema") != kCommandDeadlineSchema || !value.at("clock_id").is_string() ||
        !value.at("source_instance").is_string() || !value.at("allowed").is_boolean()) return false;
    CommandDeadline next;
    next.clock_id = value.at("clock_id").get<std::string>();
    next.source_instance = value.at("source_instance").get<std::string>();
    if (next.clock_id.empty() || next.clock_id.size() > 128 || next.source_instance.size() != 32) return false;
    for (const unsigned char c : next.clock_id) if (c > 127) return false;
    for (const char c : next.source_instance) if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    auto integer = [&](const char *key, std::int64_t &destination) {
      const auto &item = value.at(key);
      if (!item.is_number_integer() || item.is_boolean()) return false;
      if (item.is_number_unsigned() && item.get<std::uint64_t>() > static_cast<std::uint64_t>(INT64_MAX)) return false;
      destination = item.get<std::int64_t>();
      return destination > 0;
    };
    if (!integer("seq", next.seq) || !integer("issued_mono_ns", next.issued) ||
        !integer("expires_mono_ns", next.expires) || next.expires <= next.issued ||
        next.expires - next.issued > kCommandDeadlineMaxNs) return false;
    auto number = [&](const char *key, double &destination) {
      const auto &item = value.at(key);
      if (!item.is_number() || item.is_boolean()) return false;
      destination = item.get<double>(); return std::isfinite(destination);
    };
    if (!number("vx", next.vx) || !number("vy", next.vy) || !number("wz", next.wz) ||
        std::abs(next.vx) > .3 || std::abs(next.vy) > .3 || std::abs(next.wz) > 1. ||
        std::hypot(next.vx, next.vy) > .3 + 1e-12) return false;
    next.allowed = value.at("allowed").get<bool>();
    out = std::move(next); return true;
  } catch (const std::exception &) { return false; }
}

// Caller serializes this non-RT state with activation/revocation. A malformed
// current packet retires authority on the current origin without erasing its
// highwater; replay of that origin cannot recover it.
struct CommandDeadlineState {
  enum class Result { ignore, revoke, store };
  std::string clock_id;
  std::int64_t activation = 0, issued_highwater = 0;
  bool bound = false, veto = false;
  CommandDeadline latest;
  void reset(const std::string &clock, std::int64_t now) {
    clock_id = clock; activation = now; issued_highwater = 0; bound = false; veto = false; latest = {};
  }
  Result invalidate() { veto = true; return Result::revoke; }
  Result admit(const CommandDeadline &next, std::int64_t now) {
    if (next.clock_id != clock_id || next.issued < activation ||
        (bound && next.source_instance != latest.source_instance)) return Result::ignore;
    // A reordered older origin has no authority even to revoke a newer one.
    if (bound && next.seq < latest.seq) return Result::ignore;
    if (next.issued > now) return invalidate();
    if (!bound) {
      if (!next.zero() || now >= next.expires) return Result::ignore;
      bound = true; latest = next; issued_highwater = next.issued;
      veto = !next.allowed; return Result::store;
    }
    if (next.seq == latest.seq) {
      if (!next.same_origin(latest)) return invalidate();
      veto = veto || !next.allowed || now >= next.expires;
      latest.allowed = latest.allowed && next.allowed;
      return Result::store;
    }
    if (next.issued <= issued_highwater) {
      latest = next;  // Retire the higher sequence, without lowering issue time.
      return invalidate();
    }
    latest = next; issued_highwater = next.issued;
    veto = !next.allowed || now >= next.expires;
    return Result::store;  // Expired higher sequences retire, never bind/authorize.
  }
};
}  // namespace neural_controller
