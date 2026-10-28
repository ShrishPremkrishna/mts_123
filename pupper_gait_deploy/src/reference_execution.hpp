#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <string>
#include <stdexcept>

namespace neural_controller {
inline constexpr const char* kReferenceStatusProtocol = "pupper_reference_v1";
inline constexpr const char* kReferenceRequestPrefix = "pupper_reference_v1:";
inline bool valid_reference_id(const std::string& id) {
  return id.size()==32 && std::all_of(id.begin(),id.end(),[](unsigned char c) {
    return (c>='0' && c<='9') || (c>='a' && c<='f');
  });
}
inline std::string reference_request_id(const std::string& label) {
  const std::string prefix=kReferenceRequestPrefix;
  if(label.compare(0,prefix.size(),prefix)!=0) return {};
  const auto id=label.substr(prefix.size());
  return valid_reference_id(id) ? id : std::string{};
}
// Acknowledges controller playback, never a measured jump or landing. The latest
// rejection does not cancel an already running overlay. Completion is emitted
// only after the controller successfully applies its next gait-only action.
struct ReferenceExecution {
  std::string active_id, status_id, phase, reason;
  double end_s=0.;
  bool running=false;
  // Bounded outbox preserves a busy rejection and active completion produced in
  // one update. Under a request flood, oldest receipts may be lost; callers must
  // treat a missing receipt as unknown, never infer success.
  std::deque<std::string> pending;
  void enqueue() {
    const auto value=message(); if(value.empty()) return;
    if(pending.size()>=16) pending.pop_front();
    pending.push_back(value);
  }
  std::string next_message() const { return pending.empty() ? message() : pending.front(); }
  void published() { if(!pending.empty()) pending.pop_front(); }
  void reset() { *this=ReferenceExecution{}; }
  bool duplicate(const std::string& id) const { return !id.empty() && id==active_id; }
  void accept(const std::string& id,double start_s,double duration_s) {
    if(!std::isfinite(start_s)||!std::isfinite(duration_s)||duration_s<=0.)
      throw std::invalid_argument("invalid reference execution clock");
    active_id=id;end_s=start_s+duration_s;running=true;
    status_id=id;phase="playing";reason="ok";enqueue();
  }
  void reject(const std::string& id,const std::string& why) {
    if(!valid_reference_id(id)) return;
    status_id=id;phase="rejected";reason=why;enqueue();
  }
  void complete_after_apply(double now_s) {
    if(running && now_s>=end_s) {
      running=false;status_id=active_id;phase="completed";reason="ok";enqueue();
    }
  }
  void interrupt(const std::string& why) {
    if(!running) return;
    running=false;status_id=active_id;phase="interrupted";reason=why;enqueue();
  }
  std::string message() const {
    if(!valid_reference_id(status_id)) return {};
    return std::string(kReferenceStatusProtocol)+" "+status_id+" "+phase+" "+reason;
  }
};
} // namespace neural_controller
