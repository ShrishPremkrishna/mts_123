#pragma once
// Serialized joystick and external controller-switch authority. The ROS adapter
// implements this port non-blockingly in one mutually-exclusive callback group.
#include <chrono>
#include <deque>
#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace pupper_stop {
struct Request {
  bool stop;
  uint64_t epoch;
  std::vector<std::string> activate;
  std::vector<std::string> deactivate;
  uint64_t command_id = 0;
  int strictness = 0; // zero selects backend BEST_EFFORT for joystick operations
  bool needs_clear = true;
  bool motion = true;
  std::chrono::steady_clock::time_point queue_deadline{};
};
struct Completion { uint64_t command_id; bool ok; };
struct Port {
  virtual ~Port() = default;
  virtual void emergency() = 0;
  virtual bool ready() = 0;
  virtual uint64_t start(const Request &) = 0;
  virtual std::optional<bool> poll(uint64_t token) = 0;
  // Removes local response tracking ONLY; does not recall remote execution.
  virtual void forget(uint64_t token) noexcept = 0;
};
struct Status {
  bool latched, fault, stop_required, in_flight, shutdown;
  uint64_t epoch;
  std::string last_face;
  bool unknown_remote_submission;
};
class StopArbiter {
 public:
  using Time = std::chrono::steady_clock::time_point;
  StopArbiter(std::shared_ptr<Port> port, std::vector<std::string> names,
              std::vector<size_t> buttons, size_t stop_button = 12,
              size_t release_button = 9,
              std::chrono::milliseconds timeout = std::chrono::milliseconds(1500),
              bool initially_latched = false)
      : port_(std::move(port)), names_(std::move(names)), buttons_(std::move(buttons)),
        stop_button_(stop_button), release_button_(release_button), timeout_(timeout),
        previous_(buttons_.size(), false) {
    if (!port_ || names_.empty() || buttons_.empty() || buttons_.size() > names_.size()
        || stop_button_ == release_button_ || timeout_.count() <= 0)
      throw std::invalid_argument("invalid arbiter configuration");
    for (size_t i = 0; i < buttons_.size(); ++i) {
      if (buttons_[i] == stop_button_ || buttons_[i] == release_button_)
        throw std::invalid_argument("stop/release overlaps face button");
      for (size_t j = 0; j < i; ++j)
        if (buttons_[i] == buttons_[j]) throw std::invalid_argument("duplicate button");
    }
    for (size_t i = 0; i < names_.size(); ++i) {
      if (names_[i].empty()) throw std::invalid_argument("empty controller");
      for (size_t j = 0; j < i; ++j)
        if (names_[i] == names_[j]) throw std::invalid_argument("duplicate controller");
    }
    last_face_ = names_.front();
    if (initially_latched) latch_stop(false);
  }
  ~StopArbiter() { shutdown(); }

  // Submit a public SwitchController request. Zero means rejected before any
  // backend dispatch. Accepted requests receive one correlated completion.
  uint64_t submit(Request request, Time now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_ || unknown_submission_ || external_.size() >= 8) return 0;
    const auto motor = [&](const std::string &name) {
      return std::find(names_.begin(), names_.end(), name) != names_.end();
    };
    const auto allowed = [&](const std::string &name) {
      return motor(name) || name == "joint_state_broadcaster" || name == "imu_sensor_broadcaster";
    };
    const auto valid = [&](const std::vector<std::string> &names) {
      for (size_t i=0; i<names.size(); ++i) {
        if (!allowed(names[i])) return false;
        for (size_t j=0; j<i; ++j) if (names[i] == names[j]) return false;
      }
      return true;
    };
    if (!valid(request.activate) || !valid(request.deactivate)) return 0;
    for (const auto &name : request.activate)
      if (std::find(request.deactivate.begin(), request.deactivate.end(), name) != request.deactivate.end()) return 0;
    const auto motor_activations = std::count_if(request.activate.begin(), request.activate.end(), motor);
    if (motor_activations > 1) return 0;
    request.needs_clear = motor_activations > 0;
    request.motion = request.needs_clear || std::any_of(request.deactivate.begin(), request.deactivate.end(), motor);
    if (request.needs_clear && (latched_ || fault_ || stop_required_ || inflight_ || pending_ || !external_.empty())) return 0;
    if (request.motion && fault_) return 0;
    request.stop = false; request.epoch = epoch_; request.command_id = ++next_command_;
    request.queue_deadline = now + timeout_;
    external_.push_back(request);
    return request.command_id;
  }

  std::vector<Completion> take_completions() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = std::move(completions_); completions_.clear(); return result;
  }

  void joy(const std::vector<int32_t> &values) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_) return;
    const auto pressed = [&](size_t index) { return index < values.size() && values[index] == 1; };
    const bool stop = pressed(stop_button_), release = pressed(release_button_);
    const bool stop_edge = stop && !previous_stop_, release_edge = release && !previous_release_;
    std::vector<bool> face_edges(buttons_.size());
    for (size_t i = 0; i < buttons_.size(); ++i) {
      const bool value = pressed(buttons_[i]);
      face_edges[i] = value && !previous_[i];
      previous_[i] = value; // consume edges even while stop is held/latched
    }
    previous_stop_ = stop; previous_release_ = release;
    if (stop) {
      if (stop_edge) latch_stop(false);
      return; // stop always wins the entire Joy frame
    }
    if (fault_) return;
    if (release_edge) {
      // A fresh deliberate release is required AFTER confirmed deactivation.
      // An early release edge is consumed, never deferred for automatic rearm.
      if (!stop_required_ && !inflight_) {
        latched_ = false;
        cancel_external_motion();
        pending_ = Pending{last_face_, epoch_};
      }
      return;
    }
    if (latched_) return;
    // Multiple face edges choose the lowest configured index deterministically.
    for (size_t i = 0; i < face_edges.size(); ++i) if (face_edges[i]) {
      last_face_ = names_[i];
      cancel_external_motion();
      pending_ = Pending{last_face_, epoch_};
      return;
    }
  }

  void tick(Time now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_) return;
    for (auto it=external_.begin(); it!=external_.end();) {
      if (now >= it->queue_deadline) { completions_.push_back({it->command_id, false}); it=external_.erase(it); }
      else ++it;
    }
    // Reassert the software stop at a bounded 10 Hz while a remote operation is
    // uncertain. This is still not proof of physical stopping or RPC recall.
    if (latched_ && (stop_required_ || inflight_ || unknown_submission_) && now >= next_emergency_) {
      next_emergency_ = now + std::chrono::milliseconds(100);
      try { port_->emergency(); } catch (...) { fault_ = true; }
    }
    if (unknown_submission_) return; // no trackable completion: operator recovery
    try {
      if (inflight_) {
        const auto answer = port_->poll(inflight_->token);
        if (answer.has_value()) {
          const Flight done = *inflight_;
          port_->forget(done.token); inflight_.reset();
          if (done.request.command_id)
            completions_.push_back({done.request.command_id, *answer &&
              (!done.request.needs_clear || (!latched_ && !fault_ && done.request.epoch == epoch_))});
          if (!*answer) {
            if (done.request.motion) latch_stop(true);
            if (done.request.stop) ++failed_stops_;
          } else if (done.request.stop) {
            if (done.request.epoch == epoch_) { stop_required_ = false; failed_stops_ = 0; }
          } else if (done.request.needs_clear && (latched_ || fault_ || done.request.epoch != epoch_)) {
            // Even a late successful activation must be followed by deactivation.
            stop_required_ = true;
          }
        } else {
          if (!inflight_->timed_out && now >= inflight_->deadline) {
            inflight_->timed_out = true;
            if (inflight_->request.command_id) {
              completions_.push_back({inflight_->request.command_id, false});
              inflight_->request.command_id = 0;
            }
            latch_stop(true);
          }
          return; // bounded polling; retain at most one unresolved remote request
        }
      }
      if (stop_required_) {
        if (failed_stops_ >= 3 || !port_->ready()) return;
        dispatch(Request{true, epoch_, {}, names_, 0, 0, false, true}, now);
        return;
      }
      if (!external_.empty()) {
        auto request = external_.front();
        if (request.needs_clear && (latched_ || fault_ || request.epoch != epoch_)) {
          completions_.push_back({request.command_id, false}); external_.pop_front();
          return;
        }
        if (!port_->ready()) return;
        external_.pop_front(); dispatch(std::move(request), now); return;
      }
      if (!latched_ && !fault_ && pending_ && pending_->epoch == epoch_ && port_->ready()) {
        auto selected = *pending_; pending_.reset();
        std::vector<std::string> deactivate;
        for (const auto &name : names_) if (name != selected.name) deactivate.push_back(name);
        dispatch(Request{false, epoch_, {selected.name}, deactivate}, now);
      }
    } catch (...) {
      // Adapter errors cannot rearm the state machine or dispatch another actor.
      if (!fault_) latch_stop(true);
      if (inflight_ && inflight_->request.command_id) {
        completions_.push_back({inflight_->request.command_id, false});
        inflight_->request.command_id = 0;
      }
      // Poll ambiguity retains the outstanding request. Repeated uncertainty
      // must not consume the retry budget needed for eventual corrective stop.
      if (!inflight_) ++failed_stops_;
    }
  }

  Status status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {latched_, fault_, stop_required_, bool(inflight_) || bool(pending_) || !external_.empty(), shutdown_, epoch_, last_face_, unknown_submission_};
  }
  void shutdown() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (shutdown_) return;
    shutdown_ = true; latched_ = true; pending_.reset();
    for (const auto &request : external_) completions_.push_back({request.command_id, false});
    external_.clear();
    if (inflight_ && inflight_->request.command_id) completions_.push_back({inflight_->request.command_id, false});
    // Best-effort signal only. Shutdown cannot prove a physical stop or recall
    // an already-submitted request. No callback retains this arbiter instance.
    try { port_->emergency(); } catch (...) {}
    if (inflight_) { port_->forget(inflight_->token); inflight_.reset(); }
  }
 private:
  struct Pending { std::string name; uint64_t epoch; };
  struct Flight { uint64_t token; Request request; Time deadline; bool timed_out; };
  void cancel_external_motion() {
    for (auto it=external_.begin(); it!=external_.end();) {
      if (it->needs_clear) { completions_.push_back({it->command_id, false}); it=external_.erase(it); }
      else ++it;
    }
  }
  void latch_stop(bool fault) {
    latched_ = true; fault_ = fault_ || fault; stop_required_ = true;
    ++epoch_; pending_.reset(); cancel_external_motion();
    try { port_->emergency(); } catch (...) { fault_ = true; }
  }
  void dispatch(Request request, Time now) {
    try {
      auto token = port_->start(request);
      inflight_ = Flight{token, std::move(request), now + timeout_, false};
    } catch (...) {
      // async_send_request can fail after an ambiguous remote submission. Do
      // not invent a negative acknowledgement or send an unordered retry.
      unknown_submission_ = true;
      if (request.command_id) completions_.push_back({request.command_id, false});
      for (const auto &queued : external_) completions_.push_back({queued.command_id, false});
      external_.clear();
      latch_stop(true);
    }
  }
  std::shared_ptr<Port> port_;
  std::vector<std::string> names_;
  std::vector<size_t> buttons_;
  size_t stop_button_, release_button_;
  std::chrono::milliseconds timeout_;
  mutable std::mutex mutex_;
  std::vector<bool> previous_;
  bool previous_stop_ = false, previous_release_ = false;
  bool latched_ = false, fault_ = false, stop_required_ = false, shutdown_ = false;
  bool unknown_submission_ = false;
  Time next_emergency_{};
  uint64_t epoch_ = 0;
  unsigned failed_stops_ = 0;
  std::string last_face_;
  uint64_t next_command_ = 0;
  std::deque<Request> external_;
  std::vector<Completion> completions_;
  std::optional<Pending> pending_;
  std::optional<Flight> inflight_;
};
} // namespace pupper_stop
