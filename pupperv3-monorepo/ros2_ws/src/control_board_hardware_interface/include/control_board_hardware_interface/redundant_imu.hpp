#pragma once

#include "rt/rt_bno055.h"
#include "control_board_hardware_interface/imu_health.hpp"
#include <array>
#include <cmath>
#include <string_view>

namespace pupper_imu {

// Host receipt qualification; SH-2 reference/delay provide minimum known age.
// No synthetic orientation, integration across a gap, or motor re-enable.
class RedundantImu {
 public:
  using Clock=std::chrono::steady_clock;
  using Time=Clock::time_point;
  using Output=BNO055::Output;
  int mode=1;
  bool qualified=false;

  Time claim_deadline() const { return claim_until; }

  enum class Gate {source_error, primary_feedback, alternate_feedback, alternate_accuracy, alternate_age,
                   alternate_skew, overlap, disagreement, waiting_for_advance, accumulating, ready, count};
  static constexpr std::array<const char *, static_cast<size_t>(Gate::count)> gate_names{
    "source_error", "primary_feedback", "alternate_feedback", "alternate_accuracy", "alternate_age",
    "alternate_skew", "overlap", "disagreement", "waiting_for_advance", "accumulating", "ready"};

  const char *select(const Output &raw, Time now, bool allow_hold, Output &selected, Gate *gate=nullptr) {
    using namespace std::chrono;
    if (gate) *gate=Gate::source_error;
    claim_until={};
    selected=raw;
    const auto &a=raw.alternative;
    if (!initialized) {base_reset=a.reset_count; last_gaps=a.gap_count; initialized=true;}
    if (a.reset_count!=base_reset) return "BNO reported a reset since qualification";
    if (a.gap_count!=last_gaps) {
      if (mode==2) return "alternative report sequence or receipt discontinuity during hold";
      reset_agreement();
      last_gaps=a.gap_count;
    }
    const char *primary_error=feedback_error(raw,now);
    const char *alt_feedback=feedback_error(a,now);
    const bool alt_accuracy=a.quaternion_accuracy>=2 && a.gyro_accuracy>=2;
    const bool alt_age=recent(a.quaternion_receipt_time,a.quaternion_minimum_age_at_receipt,now,milliseconds(40)) &&
                       recent(a.gyro_receipt_time,a.gyro_minimum_age_at_receipt,now,milliseconds(40));
    const bool alt_skew=std::abs(duration_cast<milliseconds>(a.quaternion_receipt_time-a.gyro_receipt_time).count())<=20;
    const bool alternative_ok=!alt_feedback && alt_accuracy && alt_age && alt_skew;
    const bool q_live=recent(raw.quaternion_receipt_time,raw.quaternion_minimum_age_at_receipt,now,max_receipt_age);
    const bool g_live=recent(raw.gyro_receipt_time,raw.gyro_minimum_age_at_receipt,now,max_receipt_age);
    if ((known_q!=Time{} && raw.quaternion_receipt_time<known_q) ||
        (known_g!=Time{} && raw.gyro_receipt_time<known_g)) return "primary receipt moved backwards";
    if (q_live && raw.quaternion_valid) known_q=raw.quaternion_receipt_time;
    if (g_live && raw.gyro_valid) known_g=raw.gyro_receipt_time;
    const bool matched=alternative_ok && !primary_error &&
        recent(raw.quaternion_receipt_time,raw.quaternion_minimum_age_at_receipt,now,milliseconds(30)) &&
        recent(raw.gyro_receipt_time,raw.gyro_minimum_age_at_receipt,now,milliseconds(30)) &&
        std::max({raw.quaternion_receipt_time,raw.gyro_receipt_time,a.quaternion_receipt_time,a.gyro_receipt_time}) -
        std::min({raw.quaternion_receipt_time,raw.gyro_receipt_time,a.quaternion_receipt_time,a.gyro_receipt_time})
            <= milliseconds(20);
    const std::array<Time,4> receipts{raw.quaternion_receipt_time,raw.gyro_receipt_time,
                                      a.quaternion_receipt_time,a.gyro_receipt_time};
    bool advanced=matched;
    for (size_t i=0;i<receipts.size();++i) advanced=advanced && receipts[i]>credited[i];
    if (gate) {
      if (primary_error) *gate=Gate::primary_feedback;
      else if (alt_feedback) *gate=Gate::alternate_feedback;
      else if (!alt_accuracy) *gate=Gate::alternate_accuracy;
      else if (!alt_age) *gate=Gate::alternate_age;
      else if (!alt_skew) *gate=Gate::alternate_skew;
      else if (!matched) *gate=Gate::overlap;
      else *gate=advanced?Gate::accumulating:Gate::waiting_for_advance;
    }
    if (mode!=2 && !primary_error) {
      // GameRV uses an arbitrary yaw origin. Acquire a yaw-only world-frame
      // transform after a continuous period of overlapping, agreeing measurements.
      if (!alternative_ok || (last_match!=Time{} && now-last_match>milliseconds(qualified?130:40))) reset_agreement();
      if (matched) {
        if (agreement_since==Time{}) {
          const auto difference=raw.quat.normalized()*a.quat.normalized().conjugate();
          const double yaw=std::atan2(2*(difference.w()*difference.z()+difference.x()*difference.y()),
                                     1-2*(difference.y()*difference.y()+difference.z()*difference.z()));
          alignment=Eigen::AngleAxisf(float(yaw),Eigen::Vector3f::UnitZ());
        }
        if (!consistent(raw, true, true)) {
          if (gate) *gate=Gate::disagreement;
          reset_agreement();
        }
        else if (advanced) {
          if (last_match!=Time{} && now-last_match>milliseconds(40)) {
            reset_agreement();
          }
          credited=receipts;
          if (agreement_since==Time{}) agreement_since=now;
          last_match=now;
          if (now-agreement_since>=milliseconds(500)) qualified=true;
        }
      }
      if (qualified && matched) {
        // A duplicate may consume the remaining proof but cannot renew it.
        claim_until=std::min({last_match+milliseconds(40),
            raw.quaternion_receipt_time-raw.quaternion_minimum_age_at_receipt+milliseconds(30),
            raw.gyro_receipt_time-raw.gyro_minimum_age_at_receipt+milliseconds(30),
            a.quaternion_receipt_time-a.quaternion_minimum_age_at_receipt+milliseconds(40),
            a.gyro_receipt_time-a.gyro_minimum_age_at_receipt+milliseconds(40)});
        if (gate && claim_until>now) *gate=Gate::ready;
      }
      return nullptr;
    }
    if (!allow_hold) return primary_error ? primary_error : "hold outside a running field session";
    if (!alternative_ok) return "redundant report missing, stale, invalid, or low accuracy";
    // Explicitly reject malformed primaries. Only already-observed reports aging
    // out may open the hold. A fresh angular rate alone cannot renew orientation.
    if (primary_error) {
      std::string_view why(primary_error);
      if (why!="stale quaternion receipt" && why!="stale gyro receipt" &&
          why!="quaternion report known too old" && why!="gyro report known too old") return primary_error;
      if (!raw.quaternion_valid || !raw.gyro_valid || !raw.quat.coeffs().allFinite() || !raw.gyro.allFinite() ||
          raw.quat.norm()<.9 || raw.quat.norm()>1.1 ||
          (!q_live && (raw.quaternion_receipt_time!=known_q || !valid_clock(raw.quaternion_receipt_time,raw.quaternion_minimum_age_at_receipt,now))) ||
          (!g_live && (raw.gyro_receipt_time!=known_g || !valid_clock(raw.gyro_receipt_time,raw.gyro_minimum_age_at_receipt,now))))
        return "unknown primary clock or invalid report";
    }
    if (!consistent(raw,q_live,g_live)) return "returning primary disagrees with qualified backup";
    if (mode!=2) {
      if (!qualified || now-last_match>milliseconds(130)) return "alternative had no recent agreement";
      const auto q_time=raw.quaternion_receipt_time-raw.quaternion_minimum_age_at_receipt;
      const auto g_time=raw.gyro_receipt_time-raw.gyro_minimum_age_at_receipt;
      deadline=std::min(q_time,g_time)+seconds(3);
      mode=2;
      restore_since={};
    }
    if (now>=deadline) return "redundant IMU hold deadline";
    if (!matched) restore_since={};
    else if (advanced) {
      if (last_restore!=Time{} && now-last_restore>milliseconds(40)) restore_since={};
      credited=receipts;
      last_restore=now;
      if (restore_since==Time{}) restore_since=now;
      if (now-restore_since>=milliseconds(120)) {
        mode=3;
        last_match=now;
        agreement_since=now;
        return nullptr;
      }
    }
    selected.quat=alignment*a.quat.normalized();
    selected.gyro=a.gyro;
    selected.quaternion_valid=selected.gyro_valid=true;
    selected.quaternion_accuracy=a.quaternion_accuracy;
    selected.gyro_accuracy=a.gyro_accuracy;
    selected.quaternion_report_sequence=a.quaternion_report_sequence;
    selected.gyro_report_sequence=a.gyro_report_sequence;
    selected.quaternion_receipt_time=a.quaternion_receipt_time;
    selected.gyro_receipt_time=a.gyro_receipt_time;
    selected.quaternion_minimum_age_at_receipt=a.quaternion_minimum_age_at_receipt;
    selected.gyro_minimum_age_at_receipt=a.gyro_minimum_age_at_receipt;
    return nullptr;
  }
 private:
  static bool valid_clock(Time t,std::chrono::microseconds age,Time now) {
    return t!=Time{} && t<=now && age.count()>=0 && age<=max_receipt_age;
  }
  static bool recent(Time t,std::chrono::microseconds age,Time now,std::chrono::milliseconds bound) {
    return valid_clock(t,age,now) && now-t+age<=bound;
  }
  bool consistent(const Output &r,bool q,bool g) const {
    // Only compare a primary if it is contemporary with the backup. In the
    // last few cycles before expiry its old orientation cannot be compared to a moving body.
    auto &a=r.alternative;
    auto nearby=[](Time x,Time y){return std::abs(std::chrono::duration<double>(x-y).count())<=.025;};
    if (q && nearby(r.quaternion_receipt_time,a.quaternion_receipt_time) &&
        r.quat.normalized().angularDistance(alignment*a.quat.normalized())>.060f) return false;
    return !g || !nearby(r.gyro_receipt_time,a.gyro_receipt_time) || (r.gyro-a.gyro).norm()<=.15f;
  }
  void reset_agreement() {qualified=false; agreement_since={};}
  bool initialized=false;
  uint64_t base_reset=0,last_gaps=0;
  Eigen::Quaternionf alignment=Eigen::Quaternionf::Identity();
  std::array<Time,4> credited{};
  Time claim_until{},last_restore{},agreement_since{},last_match{},deadline{},restore_since{},known_q{},known_g{};
};
}
