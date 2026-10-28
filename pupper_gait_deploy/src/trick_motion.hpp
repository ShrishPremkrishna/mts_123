#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

namespace neural_controller {

/** A selected clip may start away from the network's normalization pose.
 * Initialize/fade from that clip pose without changing the observation default.
 * Return the effective *unclipped* affine action for next-step history. */
inline double checked_activation_anchor(double anchor, double default_position,
                                        double scale, double lower, double upper) {
  if (!std::isfinite(anchor) || !std::isfinite(default_position) || !std::isfinite(scale) ||
      !std::isfinite(lower) || !std::isfinite(upper) || lower > upper ||
      anchor < lower - 1e-6 || anchor > upper + 1e-6) {
    throw std::runtime_error("activation anchor is nonfinite or outside controller joint bounds");
  }
  anchor = std::min(std::max(anchor, lower), upper);  // export rounding only
  if (scale == 0.0 && anchor != default_position) {
    throw std::runtime_error("nondefault activation anchor requires nonzero action scale");
  }
  return anchor;
}

inline float faded_position_action(float raw_action, float scale, float default_position,
                                   float anchor, float fade) {
  // Retain exact legacy arithmetic for walking and default-pose first clips.
  if (anchor == default_position) return fade * raw_action;
  if (scale == 0.0) throw std::runtime_error("cannot normalize nondefault zero-scale anchor");
  return (1.0f - fade) * ((anchor - default_position) / scale) + fade * raw_action;
}

/**
 * A trick's reference motion, from a tracking ("trick") policy JSON's "motion"
 * block (exported by pupper-mjlab's export_trick.py). One row per frame at
 * `fps`: the reference joint angles and velocities, and the reference body
 * orientation. The policy tracks it frame by frame, as in training.
 */
/**
 * A clip: frames [start, end) of the motion. Clips that follow each other share
 * their boundary pose, so playback jumps from one clip's last frame to the next
 * clip's first frame without a discontinuity. A looping clip repeats until
 * another clip is requested; otherwise `next` plays (or, with none, the clip's
 * last frame holds).
 */
struct TrickClip {
  std::string name;
  int start = 0;
  int end = 0;
  bool loop = false;
  int next = -1;  // index into TrickMotion::clips, -1 = hold the last frame
};

struct TrickMotion {
  int frames = 0;
  double fps = 50.0;
  std::vector<float> joint_pos;    // frames x n_joints, absolute radians
  std::vector<float> joint_vel;    // frames x n_joints, rad/s
  std::vector<double> anchor_quat; // frames x 4, (w, x, y, z)
  // Clips from the JSON's "clips" block; without one, the whole motion is a
  // single clip that holds its last frame (the original single-motion tricks).
  std::vector<TrickClip> clips;

  int clip_index(const std::string &name) const {
    for (std::size_t i = 0; i < clips.size(); i++) {
      if (clips[i].name == name) return static_cast<int>(i);
    }
    return -1;
  }

  /** The frame playing `t` seconds into the trick; holds the last frame after the end. */
  int frame_at(double t) const {
    if (t <= 0.0) return 0;
    return std::min(static_cast<int>(t * fps + 1e-6), frames - 1);  // +eps: 9.2 s * 50 = 459.999...
  }
  double duration() const { return frames / fps; }
};

template <typename Json>
inline void parse_trick_motion(const Json &trick, const Json &motion, int n_joints,
                               TrickMotion &out) {
  out.fps = trick.at("fps");
  if (!std::isfinite(out.fps) || out.fps <= 0.0 || n_joints <= 0) {
    throw std::runtime_error("trick motion: fps and joint count must be positive and finite");
  }
  const auto &jp = motion.at("joint_pos");
  const auto &jv = motion.at("joint_vel");
  const auto &aq = motion.at("anchor_quat_wxyz");
  out.frames = static_cast<int>(jp.size());
  if (out.frames < 2 || jv.size() != jp.size() || aq.size() != jp.size()) {
    throw std::runtime_error("trick motion: joint_pos, joint_vel and anchor_quat_wxyz must "
                             "have the same number of frames (at least two)");
  }
  out.joint_pos.clear();
  out.joint_vel.clear();
  out.anchor_quat.clear();
  for (int f = 0; f < out.frames; f++) {
    if (static_cast<int>(jp[f].size()) != n_joints || static_cast<int>(jv[f].size()) != n_joints ||
        aq[f].size() != 4) {
      throw std::runtime_error("trick motion: frame " + std::to_string(f) + " has the wrong width");
    }
    for (int i = 0; i < n_joints; i++) {
      const float position = jp[f][i], velocity = jv[f][i];
      if (!std::isfinite(position) || !std::isfinite(velocity)) {
        throw std::runtime_error("trick motion: nonfinite joint position or velocity");
      }
      out.joint_pos.push_back(position);
      out.joint_vel.push_back(velocity);
    }
    double norm_squared = 0.0;
    for (int k = 0; k < 4; k++) {
      const double value = aq[f][k];
      if (!std::isfinite(value)) throw std::runtime_error("trick motion: nonfinite quaternion");
      norm_squared += value * value;
      out.anchor_quat.push_back(value);
    }
    if (std::abs(norm_squared - 1.0) > 2e-3) {
      throw std::runtime_error("trick motion: anchor quaternion must have unit norm");
    }
  }

  out.clips.clear();
  if (trick.contains("clips")) {
    for (const auto &c : trick.at("clips")) {
      TrickClip clip;
      clip.name = c.at("name");
      clip.start = c.at("start");
      clip.end = c.at("end");
      clip.loop = c.value("loop", false);
      if (clip.name.empty() || out.clip_index(clip.name) >= 0) {
        throw std::runtime_error("trick clip names must be nonempty and unique");
      }
      if (clip.start < 0 || clip.end > out.frames || clip.end - clip.start < 2) {
        throw std::runtime_error("trick clip '" + clip.name + "' has a bad frame range");
      }
      out.clips.push_back(clip);
    }
    if (out.clips.empty()) throw std::runtime_error("trick clips must not be empty");
    // Resolve "next" names once all clips are known.
    std::size_t i = 0;
    for (const auto &c : trick.at("clips")) {
      if (c.contains("next") && !c.at("next").is_null()) {
        const std::string next = c.at("next");
        out.clips[i].next = out.clip_index(next);
        if (out.clips[i].next < 0) {
          throw std::runtime_error("trick clip '" + out.clips[i].name + "' continues to unknown clip '" +
                                   next + "'");
        }
      }
      i++;
    }
  } else {
    out.clips.push_back(TrickClip{"all", 0, out.frames, false, -1});
  }
}

// The wire token identifies one explicit clip request, independently of the
// activation token. Legacy single-name requests remain supported natively.
inline constexpr const char* kInteractiveClipStatusProtocol = "pupper_clip_v1";
inline bool valid_clip_request_id(const std::string& id) {
  return id.size()==32 && std::all_of(id.begin(),id.end(),[](unsigned char c) {
    return (c>='0' && c<='9') || (c>='a' && c<='f');
  });
}
inline bool parse_clip_request(const std::string& payload,std::string& name,std::string& id) {
  name.clear();id.clear();
  if(payload.compare(0,12,"pupper_clip_")!=0) {
    name=payload;return !name.empty();
  }
  std::istringstream stream(payload);
  std::string protocol,parsed_id,parsed_name,extra;
  if(!(stream>>protocol>>parsed_id>>parsed_name) || (stream>>extra) ||
      protocol!=kInteractiveClipStatusProtocol || !valid_clip_request_id(parsed_id)) return false;
  name=parsed_name;id=parsed_id;return true;
}

/**
 * Plays a trick's clips on the controller's clock. frame(t) returns the frame to
 * track at trick time t (seconds since the trick started playing); clips switch
 * at clip boundaries, taking queued requests first, then looping or following
 * `next`. Kept free of ROS so the host test can drive it.
 */
class ClipPlayer {
 public:
  /** Start over from clip `start` (the first clip by default). */
  void reset(int start = 0) {
    clip_ = start;
    t0_ = 0.0;
    holding_ = false;
    queue_.clear();
    request_id_.clear();
    remembered_ids_.clear();
  }

  /** Queue a clip to play when the current one reaches its end. */
  bool request(const TrickMotion &m, const std::string &name, const std::string &id = {}) {
    const int idx = m.clip_index(name);
    if (idx < 0 || queue_.size() >= kMaxQueue) return false;
    if (!id.empty()) {
      if (!valid_clip_request_id(id) ||
          std::find(remembered_ids_.begin(),remembered_ids_.end(),id)!=remembered_ids_.end()) return false;
      if (remembered_ids_.size()>=kRememberedRequests) remembered_ids_.pop_front();
      remembered_ids_.push_back(id);
    }
    queue_.push_back({idx,id});
    return true;
  }

  int frame(const TrickMotion &m, double t) {
    if (t < 0.0) return m.clips[clip_].start;
    if (holding_) {
      if (queue_.empty()) return m.clips[clip_].end - 1;
      clip_ = queue_.front().clip;  // a request after a hold starts now
      request_id_ = queue_.front().id;
      queue_.pop_front();
      t0_ = t;
      holding_ = false;
    }
    for (int guard = 0; guard < 1000; guard++) {
      const TrickClip &c = m.clips[clip_];
      const double length = (c.end - c.start - 1) / m.fps;  // time to reach the last frame
      if (t - t0_ < length - 1e-6) break;  // switch on the boundary itself
      int next;
      if (!queue_.empty()) {
        next = queue_.front().clip;
        request_id_ = queue_.front().id;
        queue_.pop_front();
      } else if (c.loop) {
        next = clip_;
      } else if (c.next >= 0) {
        next = c.next;
      } else {
        holding_ = true;  // nothing to play: hold the last frame
        return c.end - 1;
      }
      t0_ += length;
      clip_ = next;
    }
    const TrickClip &c = m.clips[clip_];
    const int f = c.start + static_cast<int>((t - t0_) * m.fps + 1e-6);
    return std::min(f, c.end - 1);
  }

  int clip() const { return clip_; }
  bool holding() const { return holding_; }
  std::size_t queued() const { return queue_.size(); }
  // Persists through automatic next/loop transitions. A legacy request clears it
  // only when that request starts, never while it is merely waiting in the queue.
  const std::string& request_id() const { return request_id_; }

 private:
  static constexpr std::size_t kMaxQueue = 4;
  // Bounded idempotency window, not unbounded replay protection. Reset with the
  // activation; API callers always generate fresh UUIDs and do not auto-retry.
  static constexpr std::size_t kRememberedRequests = 64;
  struct Request { int clip; std::string id; };
  int clip_ = 0;
  double t0_ = 0.0;
  bool holding_ = false;
  std::deque<Request> queue_;
  std::string request_id_;
  std::deque<std::string> remembered_ids_;
};

// --- quaternion helpers, (w, x, y, z) -----------------------------------------
using Quat = std::array<double, 4>;

inline Quat quat_mul(const Quat &a, const Quat &b) {
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
          a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
          a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
          a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

inline Quat quat_conj(const Quat &q) { return {q[0], -q[1], -q[2], -q[3]}; }

inline double quat_yaw(const Quat &q) {
  return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
}

inline Quat yaw_quat(double yaw) { return {std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2)}; }

/**
 * motion_anchor_ori_b: the reference body orientation expressed in the robot's
 * body frame, as the first two columns of its rotation matrix, row-major
 * (R00, R01, R10, R11, R20, R21) -- mjlab's matrix_from_quat(q)[..., :2].
 *
 * `robot` is the IMU orientation; `reference` the motion's anchor orientation
 * already rotated by the yaw offset captured when the trick started, so the
 * reference starts facing wherever the robot faces (training resets the robot
 * onto the reference, so the policy expects them aligned at the start).
 */
inline void relative_orientation_6d(const Quat &robot, const Quat &reference, float *out) {
  Quat q = quat_mul(quat_conj(robot), reference);
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (double &v : q) v /= n;
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y - w * z);
  const double r10 = 2 * (x * y + w * z), r11 = 1 - 2 * (x * x + z * z);
  const double r20 = 2 * (x * z - w * y), r21 = 2 * (y * z + w * x);
  out[0] = static_cast<float>(r00);
  out[1] = static_cast<float>(r01);
  out[2] = static_cast<float>(r10);
  out[3] = static_cast<float>(r11);
  out[4] = static_cast<float>(r20);
  out[5] = static_cast<float>(r21);
}

}  // namespace neural_controller
