#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include "json.hpp"
#include "neural_controller/trick_motion.hpp"
#include "neural_controller/obs_spec.hpp"
using namespace neural_controller;
using Json=nlohmann::json;
int checks=0,failures=0;
void check(bool v,const char* message) {++checks;if(!v){++failures;std::cerr<<"FAIL "<<message<<'\n';}}
int candidate_frame(ClipPlayer& p,const TrickMotion& m,double t) {
#ifdef STATIC_LOOP_CANDIDATE
 return p.frame(m,t,true);
#else
 return p.frame(m,t);
#endif
}
TrickMotion parse(const Json& p) {TrickMotion m;parse_trick_motion(p.at("trick"),p.at("motion"),12,m);return m;}
int main(int argc,char**argv) {
 if(argc!=2)throw std::runtime_error("exact Sit policy path required");
 std::ifstream f(argv[1]);Json policy;f>>policy;const auto motion=parse(policy);
 const int idle=motion.clip_index("sit_idle");const std::string id(32,'a'),nextid(32,'b');
 for(const std::string name:{"stand_up","shake_left","shake_right"}) {
  const int target=motion.clip_index(name);const auto &c=motion.clips[target];
  const double duration=(c.end-c.start-1)/motion.fps;
  for(double phase:{.01,.25,.50,.99}) {
   ClipPlayer p,old;p.reset(idle);old.reset(idle);
   check(p.request(motion,name,id),"explicit request accepted");old.request(motion,name,id);
   check(candidate_frame(p,motion,phase)==c.start,"request starts now at exact first frame");
   check(p.request_id()==id && p.queued()==0 && !p.holding(),"causal ID/queue at early start");
   check(old.frame(motion,phase)<motion.clips[idle].end && old.clip()==idle,"generic default retains boundary wait");
   const int boundary=old.frame(motion,1.);check(boundary==c.start,"old boundary reaches same target first frame");
   for(int j=0;j<12;++j) {
    const int source=motion.clips[idle].start*12+j;
    check(motion.joint_pos[c.start*12+j]==motion.joint_pos[source],"exact q continuity");
    check(motion.joint_pos[c.start*12+j]-motion.joint_pos[source]==motion.joint_pos[boundary*12+j]-motion.joint_pos[source],"position delta unchanged");
    check(motion.joint_vel[c.start*12+j]-motion.joint_vel[source]==motion.joint_vel[boundary*12+j]-motion.joint_vel[source],"velocity delta unchanged, including nonzero paw dq");
   }
   for(int k=0;k<4;++k) check(motion.anchor_quat[c.start*4+k]==motion.anchor_quat[motion.clips[idle].start*4+k],"exact anchor quaternion continuity");
   // Every target sample is preserved on its own full clock, including the
   // nonzero paw starting velocity. No resampling or shortened playback.
   for(int step=0;step<175;++step) {
    const double dt=step/motion.fps;
    check(candidate_frame(p,motion,phase+dt)==c.start+step,"complete unchanged target sequence");
   }
   check(!p.holding() && p.clip()==target,"no early terminal result");
   candidate_frame(p,motion,phase+duration);
   check(p.request_id()==id,"request ID persists at terminal/automatic return");
   check(name=="stand_up" ? p.holding() && p.clip()==target : !p.holding() && p.clip()==idle,"exact full-duration completion semantics");
  }
 }
 // Full FIFO: first action starts early, the next cannot skip its duration.
 ClipPlayer p;p.reset(idle);p.request(motion,"shake_left",id);p.request(motion,"stand_up",nextid);
 candidate_frame(p,motion,.25);check(p.clip()==motion.clip_index("shake_left") && p.queued()==1,"front only");
 candidate_frame(p,motion,.75);check(p.clip()==motion.clip_index("shake_left") && p.request_id()==id,"second request cannot interrupt moving clip");
 candidate_frame(p,motion,3.75);check(p.clip()==motion.clip_index("stand_up") && p.request_id()==nextid,"second starts at original finite boundary");
 check(!p.request(motion,"stand_up",nextid),"duplicate request rejected");
 // Startup fade, unknown request, reset/cancellation, and queued loop front.
 p.reset(idle);p.request(motion,"stand_up",id);candidate_frame(p,motion,-.01);
 check(p.clip()==idle && p.queued()==1 && p.request_id().empty(),"fade does not consume queued transition");
 p.reset(idle);candidate_frame(p,motion,.25);check(p.clip()==idle && p.queued()==0,"reset cancels pending request");
 check(!p.request(motion,"missing",id),"unknown request rejected");
 p.request(motion,"sit_idle",id);p.request(motion,"stand_up",nextid);candidate_frame(p,motion,.25);
 check(p.clip()==idle && p.queued()==2,"loop front not skipped to eligible later request");
 // Each consumed source field independently revokes static eligibility. Tiny
 // representable changes are rejected without a tolerance.
 for(int mutation=0;mutation<6;++mutation) {
  auto changed=policy;const int start=motion.clips[idle].start;
  if(mutation==0)changed["motion"]["joint_pos"][start+5][0]=double(motion.joint_pos[(start+5)*12])+1e-5;
  if(mutation==1)changed["motion"]["joint_vel"][start+5][0]=1e-8;
  if(mutation==2) {for(int k=0;k<4;++k)changed["motion"]["anchor_quat_wxyz"][start+5][k]=-double(changed["motion"]["anchor_quat_wxyz"][start+5][k]);}
  if(mutation==3)changed["motion"]["joint_pos"][735][0]=double(changed["motion"]["joint_pos"][735][0])+1e-5;
  if(mutation==4) {for(int k=0;k<4;++k)changed["motion"]["anchor_quat_wxyz"][735][k]=-double(changed["motion"]["anchor_quat_wxyz"][735][k]);}
  if(mutation==5)changed["trick"]["clips"][4]["loop"]=true;
  const auto m=parse(changed);p.reset(idle);p.request(m,"stand_up",id);candidate_frame(p,m,.25);
  check(p.clip()==idle && p.queued()==1 && p.request_id().empty(),"ineligible source/target retains boundary");
  candidate_frame(p,m,1.);check(p.clip()==m.clip_index("stand_up"),"ineligible source/target still uses existing boundary");
 }
#ifdef STATIC_LOOP_CANDIDATE

 check(has_only_dense_layers(policy),"exact four-dense Sit actor eligible");
 for(int bad=0;bad<6;++bad) {
  auto changed=policy;
  if(bad==0)changed.erase("layers");
  if(bad==1)changed["layers"]=Json::array();
  if(bad==2)changed["layers"][0]["type"]="lstm";
  if(bad==3)changed["layers"][0].erase("type");
  if(bad==4)changed["layers"][0]["type"]=7;
  if(bad==5)changed["layers"]=Json::object();
  check(!has_only_dense_layers(changed),"unknown/stateful/malformed layer metadata rejects early path");
  check(!supports_static_loop_requests(parse_obs_spec(policy.at("obs_spec"),12),1,false,false,has_only_dense_layers(changed)),"ineligible architecture keeps boundary scheduling");
 }
 auto layout=parse_obs_spec(policy.at("obs_spec"),12);
 check(supports_static_loop_requests(layout,1,false,false,has_only_dense_layers(policy)),"actual Sit layout opts in");
 check(!supports_static_loop_requests(layout,2,false,false,true),"longer history stays default");
 check(!supports_static_loop_requests(layout,1,true,false,true),"gait path stays default");
 check(!supports_static_loop_requests(layout,1,false,true,true),"jump path stays default");
 for(int ObsLayout::*member:{&ObsLayout::frame_size,&ObsLayout::motion_command,&ObsLayout::motion_anchor_ori_b,&ObsLayout::ang_vel,&ObsLayout::joint_pos_rel,&ObsLayout::joint_vel_rel,&ObsLayout::last_action,&ObsLayout::projected_gravity,&ObsLayout::command,&ObsLayout::desired_world_z,&ObsLayout::gait_reference}) {
  auto changed=layout;changed.*member+=1;check(!supports_static_loop_requests(changed,1,false,false,true),"different observation contract stays default");
 }
#endif
 std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;
}
