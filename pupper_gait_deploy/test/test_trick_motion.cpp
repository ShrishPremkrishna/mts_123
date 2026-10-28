// Parity test for trick (motion-tracking) policies' on-robot observations.
//
// motion_anchor_ori_b must equal what mjlab computed in training
// (subtract_frame_transforms + matrix_from_quat[..., :2]); a transposed or
// conjugated quaternion here would feed the policy a mirrored body orientation.
// trick_golden.json is generated from mjlab's own functions. Also checks the
// frame clock and the parser on a small motion block.
//
// Standalone (no gtest), like the other host tests.

#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

#include "json.hpp"
#include "neural_controller/trick_motion.hpp"

using namespace neural_controller;

int main() {
  std::ifstream stream(TRICK_GOLDEN_JSON);
  if (!stream) throw std::runtime_error("could not open trick_golden.json");
  nlohmann::json golden;
  stream >> golden;
  int failures = 0;

  // Walk -> selected Spider exit starts at a low pose although the tracking
  // network's normalization/action default is standing. Initialization must
  // hold that exit frame; fading must retain the pre-clamp effective action.
  for (const float anchor : {-1.037f, .709f, -.52f}) {
    const float default_pos = -.52f, scale = .75f, raw = .4f;
    checked_activation_anchor(anchor, default_pos, scale, -2., 2.);
    for (int step = 0; step <= 100; ++step) {
      const float fade = step / 100.f;
      const float effective = faded_position_action(raw, scale, default_pos, anchor, fade);
      const float target = default_pos + scale * effective;
      const float expected = (1-fade)*anchor + fade*(default_pos+scale*raw);
      if (std::abs(target-expected)>2e-7f) ++failures;
      if (anchor==default_pos && effective!=fade*raw) ++failures; // exact legacy path
      if (step==100 && effective!=raw) ++failures;
    }
  }
  if (faded_position_action(8.f, .75f, -.52f, .709f, 0.f) == 0.f) ++failures;
  if (faded_position_action(8.f, 0.f, -.52f, -.52f, .5f) != 4.f) ++failures;
  for (int invalid=0; invalid<3; ++invalid) {
    bool rejected=false;
    try {
      checked_activation_anchor(invalid==0?3.:.709, -.52,
                                invalid==1?0.:.75, -2., invalid==2?NAN:2.);
    } catch(const std::runtime_error&) { rejected=true; }
    if(!rejected) ++failures;
  }
  std::printf("activation fade: selected exit, default first clip, walking arithmetic, bounds and zero-scale checks\n");

  double worst = 0.0;
  for (const auto &c : golden["relative_orientation"]) {
    const Quat robot{c["robot_wxyz"][0], c["robot_wxyz"][1], c["robot_wxyz"][2], c["robot_wxyz"][3]};
    const Quat ref{c["reference_wxyz"][0], c["reference_wxyz"][1], c["reference_wxyz"][2],
                   c["reference_wxyz"][3]};
    float got[6];
    relative_orientation_6d(robot, ref, got);
    for (int k = 0; k < 6; k++) worst = std::max(worst, std::abs(got[k] - double(c["ori_6d"][k])));
  }
  std::printf("relative orientation: %zu cases, worst error %.2e\n",
              golden["relative_orientation"].size(), worst);
  if (worst > 1e-5) failures++;

  TrickMotion motion;
  motion.fps = golden["frame_at"]["fps"];
  motion.frames = golden["frame_at"]["frames"];
  for (const auto &c : golden["frame_at"]["cases"]) {
    const int got = motion.frame_at(double(c[0]));
    if (got != int(c[1])) {
      std::printf("frame_at(%.3f) = %d, expected %d\n", double(c[0]), got, int(c[1]));
      failures++;
    }
  }

  // Yaw alignment: rotating the reference by the captured offset makes the first
  // frame face the robot's heading, so the relative orientation starts at identity.
  const Quat robot_start = quat_mul(yaw_quat(1.1), Quat{0.9990, 0.0, -0.0447, 0.0});
  const Quat ref_start = yaw_quat(-0.4);
  const double offset = quat_yaw(robot_start) - quat_yaw(ref_start);
  float aligned[6];
  relative_orientation_6d(yaw_quat(quat_yaw(robot_start)), quat_mul(yaw_quat(offset), ref_start),
                          aligned);
  const float identity[6] = {1, 0, 0, 1, 0, 0};
  for (int k = 0; k < 6; k++) {
    if (std::abs(aligned[k] - identity[k]) > 1e-6) {
      std::printf("yaw alignment: component %d is %f, expected %f\n", k, aligned[k], identity[k]);
      failures++;
    }
  }

  nlohmann::json trick = {{"name", "t"}, {"fps", 50}};
  nlohmann::json block = {{"joint_pos", {std::vector<double>(12, 0.1), std::vector<double>(12, 0.2)}},
                          {"joint_vel", {std::vector<double>(12, 0.0), std::vector<double>(12, 1.0)}},
                          {"anchor_quat_wxyz", {{1, 0, 0, 0}, {1, 0, 0, 0}}}};
  parse_trick_motion(trick, block, 12, motion);
  if (motion.frames != 2 || std::abs(motion.joint_pos[12] - 0.2f) > 1e-6 ||
      std::abs(motion.joint_vel[23] - 1.0f) > 1e-6) {
    std::printf("parse_trick_motion: unexpected contents\n");
    failures++;
  }

  // Malformed clocks/clip tables used to parse successfully and then divide by
  // zero or index an empty clip array during controller activation.
  auto rejects = [&](const nlohmann::json &t, const nlohmann::json &b, const char *name) {
    try {
      TrickMotion invalid;
      parse_trick_motion(t, b, 12, invalid);
      std::printf("invalid trick accepted: %s\n", name);
      failures++;
    } catch (const std::exception &) {}
  };
  auto invalid = trick;
  invalid["fps"] = 0;
  rejects(invalid, block, "zero fps");
  invalid = trick;
  invalid["clips"] = nlohmann::json::array();
  rejects(invalid, block, "empty clips");
  invalid["clips"] = {{{"name", "same"}, {"start", 0}, {"end", 2}},
                       {{"name", "same"}, {"start", 0}, {"end", 2}}};
  rejects(invalid, block, "duplicate clips");
  auto invalid_motion = block;
  invalid_motion["anchor_quat_wxyz"][0] = {0, 0, 0, 0};
  rejects(trick, invalid_motion, "zero quaternion");
  invalid_motion = block;
  invalid_motion["joint_pos"][0][0] = 1e300;
  rejects(trick, invalid_motion, "overflowing motor target");

  // Clip sequencing: A -> B (loops) ; requests C (-> B) and D (holds) ; a request
  // after the hold starts at once. 10 fps, clips share boundary frames.
  {
    nlohmann::json t = {{"name", "clips"}, {"fps", 10},
                        {"clips", {{{"name", "A"}, {"start", 0}, {"end", 10}, {"next", "B"}},
                                   {{"name", "B"}, {"start", 9}, {"end", 15}, {"loop", true}},
                                   {{"name", "C"}, {"start", 14}, {"end", 20}, {"next", "B"}},
                                   {{"name", "D"}, {"start", 19}, {"end", 25}}}}};
    std::vector<std::vector<double>> jp(25, std::vector<double>(12, 0.0));
    std::vector<std::vector<double>> aq(25, std::vector<double>{1, 0, 0, 0});
    nlohmann::json b = {{"joint_pos", jp}, {"joint_vel", jp}, {"anchor_quat_wxyz", aq}};
    TrickMotion m;
    parse_trick_motion(t, b, 12, m);
    ClipPlayer p;
    struct Step { double t; const char *request; int frame; const char *clip; };
    const Step steps[] = {
        {0.00, nullptr, 0, "A"},  {0.85, nullptr, 8, "A"},  {0.90, nullptr, 9, "B"},
        {1.20, "C", 12, "B"},     {1.40, nullptr, 14, "C"}, {1.85, nullptr, 18, "C"},
        {1.90, nullptr, 9, "B"},  {2.00, "D", 10, "B"},     {2.40, nullptr, 19, "D"},
        {2.95, nullptr, 24, "D"}, {3.40, nullptr, 24, "D"}, {3.50, "C", 14, "C"},
        {3.70, nullptr, 16, "C"},
    };
    for (const auto &st : steps) {
      if (st.request && !p.request(m, st.request)) {
        std::printf("clip request %s refused\n", st.request);
        failures++;
      }
      const int f = p.frame(m, st.t);
      const std::string clip = m.clips[p.clip()].name;
      if (f != st.frame || clip != st.clip) {
        std::printf("t=%.2f: frame %d in %s, expected %d in %s\n", st.t, f, clip.c_str(), st.frame,
                    st.clip);
        failures++;
      }
    }
    if (p.request(m, "nope")) {
      std::printf("unknown clip was accepted\n");
      failures++;
    }
    // start_clip: an activation that starts from clip C plays C, then its next (B).
    p.reset(m.clip_index("C"));
    const int f0 = p.frame(m, 0.0), f1 = p.frame(m, 0.55);
    if (f0 != 14 || m.clips[p.clip()].name != "B" || f1 != 9) {
      std::printf("reset(C): frames %d then %d in %s, expected 14 then 9 in B\n", f0, f1,
                  m.clips[p.clip()].name.c_str());
      failures++;
    }
  }

  // Request identity changes no frame-clock arithmetic. Tokens follow queued
  // selection through auto-next and looping, not the instant of enqueueing.
  {
    TrickMotion m;
    m.fps=10;m.frames=25;
    m.clips={{"A",0,10,false,1},{"B",9,15,true,-1},
             {"C",14,20,false,1},{"D",19,25,false,-1}};
    auto check=[&](bool ok,const char* what) {
      if(!ok) { std::printf("clip correlation failed: %s\n",what);++failures; }
    };
    const std::string a(32,'a'),b(32,'b'),c(32,'c');
    std::string name,id;
    check(parse_clip_request("C",name,id) && name=="C" && id.empty(),"legacy name");
    check(parse_clip_request("pupper_clip_v1 "+a+" C",name,id) && name=="C" && id==a,"structured request");
    for(const std::string& bad:std::vector<std::string>{"pupper_clip_v1","pupper_clip_v1 bad C",
        "pupper_clip_v1 "+a+" C extra","pupper_clip_v2 "+a+" C",
        "pupper_clip_v1 "+std::string(32,'A')+" C"}) {
      check(!parse_clip_request(bad,name,id) && name.empty() && id.empty(),"malformed parse");
    }
    ClipPlayer p;p.reset(1);
    check(p.request_id().empty() && p.frame(m,0.)==9,"initial chain none");
    check(p.request(m,"C",a) && p.request_id().empty(),"queued token not prematurely active");
    check(!p.request(m,"D",a) && p.queued()==1,"queued duplicate refused");
    check(p.frame(m,.4)==13 && p.request_id().empty(),"current loop owns receipt");
    check(p.frame(m,.5)==14 && p.request_id()==a,"token activates at boundary");
    check(p.frame(m,1.)==9 && p.request_id()==a,"token follows automatic next");
    check(p.frame(m,1.2)==11 && p.request_id()==a,"token survives loop");
    check(!p.request(m,"D",a) && p.queued()==0,"active duplicate refused");
    check(!p.request(m,"D","bad") && !p.request(m,"unknown",b) && p.queued()==0,"invalid request no mutation");
    check(p.request(m,"D",b),"new request accepted");
    check(p.frame(m,1.5)==19 && p.request_id()==b,"new token replaces at selection");
    check(p.frame(m,2.)==24 && p.holding() && p.request_id()==b,"hold retains token");
    check(p.request(m,"C") && p.frame(m,2.1)==14 && p.request_id().empty(),"legacy selection clears token");
    check(!p.request(m,"D",a),"recent completed duplicate refused");
    p.reset();check(p.request_id().empty() && p.request(m,"C",a),"activation resets identity");

    // Compare every frame, clip, hold and queue state with the legacy two-arg
    // request API across automatic boundaries, queueing and held reactivation.
    ClipPlayer legacy,tagged;int request_number=0;
    for(int step=0;step<=400;++step) {
      const double t=step/100.;
      if(step==120 || step==200 || step==350) {
        const std::string clip=step==200?"D":"C";
        const std::string token(32,static_cast<char>('a'+request_number++));
        check(legacy.request(m,clip)==tagged.request(m,clip,token),"legacy request acceptance parity");
      }
      check(legacy.frame(m,t)==tagged.frame(m,t) && legacy.clip()==tagged.clip() &&
            legacy.holding()==tagged.holding() && legacy.queued()==tagged.queued(),"legacy trajectory parity");
    }
    p.reset(3);
    for(int k=0;k<4;++k) check(p.request(m,"D",std::string(32,static_cast<char>('a'+k))),"queue capacity");
    const std::string e(32,'e');
    check(!p.request(m,"D",e) && p.queued()==4,"full queue refuses without reserving nonce");
    p.frame(m,1.);check(p.request(m,"D",e),"full-queue rejected nonce can retry");

    // Document the bounded 64-successful-request idempotency window exactly.
    p.reset(3);
    for(int k=0;k<65;++k) {
      const std::string token=std::string(30,'0')+"0123456789abcdef"[k/16]+"0123456789abcdef"[k%16];
      check(p.request(m,"D",token),"unique remembered request");
      p.frame(m,2.*k+1.);
    }
    check(p.request(m,"D",std::string(32,'0')),"old nonce leaves bounded window");
    check(!p.request(m,"D",std::string(30,'0')+"40"),"recent nonce remains remembered");
  }

  std::printf(failures ? "FAILED (%d)\n" : "trick motion: all checks passed\n", failures);
  return failures ? 1 : 0;
}
