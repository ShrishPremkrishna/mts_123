// Offline integration diagnostic. Uses the production inference, clip, quaternion,
// gait and command-filter headers; the ROS lifecycle/update plumbing is transcribed
// explicitly here, not claimed to be an execution of the ROS controller itself.
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <RTNeural/RTNeural.h>
#include "trick_motion.hpp"
#include "gait_reference_json.hpp"
#include "command_filter.hpp"
using Json = nlohmann::json;
using namespace neural_controller;
struct Controller {
  Json metadata;
  std::unique_ptr<RTNeural::Model<float>> model;
  TrickMotion motion;
  ClipPlayer player;
  GaitReference gait;
  CommandClip command_clip;
  HeadingHold heading;
  std::map<std::string, int> offsets;
  std::vector<float> observation;
  std::vector<double> defaults, initial, anchor, lower, upper;
  std::vector<float> target, before_clip, raw, model_input, reference_offset;
  std::vector<double> effective_command;
  int width=0, history=1, repeat=-1, frame=-1;
  double init_s=0, fade_s=0, yaw_offset=0, scale=0;
  bool yaw_set=false, clip_anchor=false;
  explicit Controller(const char* path) {
    std::ifstream file(path); file>>metadata;
    std::ifstream network(path); model=RTNeural::json_parser::parseJson<float>(network,false);
    defaults=metadata.at("default_joint_pos").get<std::vector<double>>();
    lower=metadata.at("joint_lower_limits").get<std::vector<double>>();
    upper=metadata.at("joint_upper_limits").get<std::vector<double>>();
    scale=metadata.at("action_scale");
    if (!(std::isfinite(scale)&&scale>0)) throw std::runtime_error("positive action scale required");
    history=metadata.at("observation_history");
    for (const auto& part:metadata.at("obs_spec").at("components")) {
      offsets[part.at("name")]=width; width+=part.at("size").get<int>();
    }
    if (model->getInSize()!=width*history || model->getOutSize()!=12) throw std::runtime_error("network dimensions");
    if(metadata.contains("motion")) parse_trick_motion(metadata.at("trick"),metadata.at("motion"),12,motion);
    if(metadata.contains("gait_reference")) parse_gait_reference(metadata.at("gait_reference"),12,gait);
    if(metadata.contains("command_clip")) parse_command_clip(metadata.at("command_clip"),command_clip);
    if(metadata.contains("heading_hold")) parse_heading_hold(metadata.at("heading_hold"),heading);
    if(metadata.contains("jump_slot")) throw std::runtime_error("jump-slot controller outside bounded Spider diagnostic");
  }
  void activate(const Json& request) {
    initial=request.at("joint_pos").get<std::vector<double>>();
    init_s=request.at("init_s"); fade_s=request.at("fade_s");
    clip_anchor=request.value("clip_anchor",false);
    int start=0;
    const std::string selected=request.value("start_clip","");
    if(!selected.empty()) { start=motion.clip_index(selected); if(start<0) throw std::runtime_error("unknown starting clip"); }
    player.reset(start); heading.reset(); repeat=-1; yaw_set=false; frame=-1;
    anchor=defaults;
    if(clip_anchor && motion.frames) {
      for(int j=0;j<12;++j) anchor[j]=checked_activation_anchor(motion.joint_pos[12*motion.clips[start].start+j],defaults[j],scale,lower[j],upper[j]);
    }
    observation.assign(width*history,0);
    if(offsets.count("projected_gravity")) for(int h=0;h<history;++h) observation[h*width+offsets.at("projected_gravity")+2]=-1;
    target.assign(initial.begin(),initial.end()); before_clip=target; raw.assign(12,0);
  }
  Json step(const Json& r) {
    const double elapsed=r.at("elapsed_s");
    if(elapsed<init_s) {
      for(int j=0;j<12;++j) target[j]=initial[j]*(1-elapsed/init_s)+anchor[j]*elapsed/init_s;
      before_clip=target;
      return {{"target",target},{"before_clip",before_clip},{"kp",7.5},{"kd",.25},{"inference",false},{"stage","initialization"},{"frame",frame}};
    }
    repeat=(repeat+1)%10;
    const double fade_time=elapsed-init_s, alpha=std::min(fade_time/fade_s,1.0);
    if(repeat==0) {
      const auto q=r.at("joint_pos").get<std::vector<double>>();
      const auto dq=r.at("joint_vel").get<std::vector<double>>();
      const auto angular=r.at("angular_velocity").get<std::vector<double>>();
      const auto quat=r.at("quaternion").get<Quat>();
      auto command=r.at("command").get<std::vector<double>>();
      command_clip.apply(command[0],command[1],command[2]);
      command[2]=heading.apply(quat_yaw(quat),command[0],command[1],command[2]);
      auto write=[&](const std::string& name,const auto& values) {
        if(offsets.count(name)) for(std::size_t j=0;j<values.size();++j) observation[offsets.at(name)+j]=values[j];
      };
      const double w=quat[0],x=quat[1],y=quat[2],z=quat[3];
      write("projected_gravity",std::vector<double>{2*(w*y-x*z),-2*(y*z+w*x),-(1-2*(x*x+y*y))});
      write("base_ang_vel",angular); write("command",command);
      write("desired_world_z",std::vector<double>{0,0,1});
      std::vector<double> rel(12); for(int j=0;j<12;++j) rel[j]=q[j]-defaults[j];
      write("joint_pos_rel",rel); write("joint_vel_rel",dq);
      if(motion.frames) {
        frame=player.frame(motion,fade_time-fade_s);
        std::vector<double> mc(24);
        for(int j=0;j<12;++j) { mc[j]=motion.joint_pos[frame*12+j]; mc[j+12]=motion.joint_vel[frame*12+j]; }
        write("motion_command",mc);
        if(!yaw_set) { Quat ref0; for(int k=0;k<4;++k) ref0[k]=motion.anchor_quat[k]; yaw_offset=quat_yaw(quat)-quat_yaw(ref0); yaw_set=true; }
        Quat ref; for(int k=0;k<4;++k) ref[k]=motion.anchor_quat[frame*4+k];
        if(offsets.count("motion_anchor_ori_b")) relative_orientation_6d(quat,quat_mul(yaw_quat(yaw_offset),ref),observation.data()+offsets.at("motion_anchor_ori_b"));
      }
      if(gait.valid()) compute_gait_reference_offset(gait,defaults,fade_time,command[0],command[1],command[2],observation.data()+offsets.at("gait_reference"));
      for(float& v:observation) v=std::clamp(v,-100.f,100.f);
      // Read-only audit copy before history rotation and last-action feedback.
      // This lets the caller distinguish reference, actor and clamp transitions.
      model_input=observation; effective_command=command;
      reference_offset.clear();
      if(gait.valid()) reference_offset.assign(observation.begin()+offsets.at("gait_reference"),observation.begin()+offsets.at("gait_reference")+12);
      model->forward(observation.data());
      std::rotate(observation.rbegin(),observation.rbegin()+width,observation.rend());
      for(int j=0;j<12;++j) {
        raw[j]=model->getOutputs()[j];
        const float effective=clip_anchor ? faded_position_action(raw[j],scale,defaults[j],anchor[j],alpha) : float(alpha)*raw[j];
        before_clip[j]=float(defaults[j])+float(scale)*effective;
        if(offsets.count("last_action")) observation[offsets.at("last_action")+j]=effective;
        target[j]=std::clamp(before_clip[j],float(lower[j]),float(upper[j]));
      }
    }
    Json result={{"target",target},{"before_clip",before_clip},{"raw_action",raw},{"kp",metadata.at("kp")},{"kd",metadata.at("kd")},{"inference",repeat==0},{"stage",alpha<1?"fade":"policy"},{"frame",frame}};
    if(repeat==0 && r.value("audit_inference",false)) {
      result["model_input"]=model_input; result["gait_reference_offset"]=reference_offset;
      result["effective_command"]=effective_command; result["reference_time_s"]=fade_time;
    }
    if(motion.frames) { result["clip"]=motion.clips[player.clip()].name; result["holding"]=player.holding(); }
    return result;
  }
};
int main(int argc,char** argv) {
  try {
    if(argc!=3) return 2;
    Controller trick(argv[1]),walk(argv[2]); Controller* active=&trick;
    std::string line;
    while(std::getline(std::cin,line)) {
      const auto request=Json::parse(line);
      if(request.at("op")=="activate") { active=request.at("controller")=="trick"?&trick:&walk;active->activate(request);std::cout<<"{\"activated\":true}"<<std::endl; }
      else std::cout<<active->step(request).dump()<<std::endl;
    }
  } catch(const std::exception& e) { std::cerr<<e.what()<<std::endl; return 1; }
}
