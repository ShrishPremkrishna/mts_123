// Offline exported-policy locomotion screen. Production inference/reference/filter
// headers; lifecycle/history plumbing is transcribed, not ROS execution.
#include <fstream>
#include <iostream>
#include <memory>
#include <RTNeural/RTNeural.h>
#include "gait_reference_json.hpp"
#include "command_filter.hpp"
#include "trick_motion.hpp"
using Json = nlohmann::json;
using namespace neural_controller;

struct Controller {
  Json metadata;
  std::unique_ptr<RTNeural::Model<float>> network;
  GaitReference gait;
  JumpSlot jump;
  CommandClip clip;
  HeadingHold heading;
  std::vector<float> history, before, target, raw, effective, input;
  std::vector<double> defaults, lower, upper;
  int frames;
  float scale;
  bool has_jump;
  double previous_time = -1;
  const double scheduled_slot_start;
  explicit Controller(const char* path, double slot_start = std::numeric_limits<double>::infinity())
      : scheduled_slot_start(slot_start) {
    std::ifstream f(path); f >> metadata;
    std::ifstream n(path); network = RTNeural::json_parser::parseJson<float>(n, false);
    if (metadata.at("contract_version") != 2 || metadata.at("single_observation_size") != 48 ||
        metadata.contains("motion")) throw std::runtime_error("only v2 48-dimensional locomotion exports supported");
    const Json expected = Json::array({{{"name","base_ang_vel"},{"size",3}},
      {{"name","projected_gravity"},{"size",3}},{{"name","command"},{"size",3}},
      {{"name","desired_world_z"},{"size",3}},{{"name","joint_pos_rel"},{"size",12}},
      {{"name","last_action"},{"size",12}},{{"name","gait_reference"},{"size",12}}});
    if (metadata.at("obs_spec").at("components") != expected ||
        metadata.at("obs_spec").at("frame_order") != "newest_first")
      throw std::runtime_error("unsupported observation layout");
    if ((metadata.contains("action_types") && metadata.at("action_types") != Json(std::vector<std::string>(12,"position"))) ||
        (metadata.contains("action_type") && metadata.at("action_type") != "position") ||
        metadata.contains("position_target_slew") ||
        (metadata.contains("required_controller_features") && !metadata.at("required_controller_features").empty()))
      throw std::runtime_error("native host supports legacy position actions without additional controller features only");
    defaults = metadata.at("default_joint_pos").get<std::vector<double>>();
    lower = metadata.at("joint_lower_limits").get<std::vector<double>>();
    upper = metadata.at("joint_upper_limits").get<std::vector<double>>();
    frames = metadata.at("observation_history"); scale = metadata.at("action_scale");
    if (frames < 1 || !std::isfinite(scale) || scale <= 0 || defaults.size()!=12 ||
        lower.size()!=12 || upper.size()!=12 || network->getInSize()!=48*frames || network->getOutSize()!=12)
      throw std::runtime_error("invalid locomotion dimensions or scale");
    parse_gait_reference(metadata.at("gait_reference"),12,gait);
    parse_command_clip(metadata.at("command_clip"),clip);
    parse_heading_hold(metadata.at("heading_hold"),heading);
    has_jump=metadata.contains("jump_slot");
    if(has_jump) parse_jump_slot(metadata.at("jump_slot"),12,jump);
    if(std::isfinite(scheduled_slot_start) &&
       (!has_jump || !jump.valid() || scheduled_slot_start < 2.0 ||
        std::abs(scheduled_slot_start/jump.grid_s - std::round(scheduled_slot_start/jump.grid_s)) > 1e-9))
      throw std::runtime_error("scheduled jump requires a valid slot and a grid-aligned start after fade-in");
    reset();
  }
  void reset() {
    history.assign(48*frames,0); for(int h=0;h<frames;++h) history[h*48+5]=-1;
    raw.assign(12,0);effective=raw;before.assign(defaults.begin(),defaults.end());target=before;
    heading.reset(); previous_time=-1;
  }
  Json infer(const Json& r) {
    const double elapsed=r.at("elapsed_s");
    if(!std::isfinite(elapsed) || elapsed<2 || elapsed<=previous_time)
      throw std::runtime_error("inference must follow two-second initialization on an increasing clock");
    previous_time=elapsed;
    const double phase_time=elapsed-2;
    const float fade=std::min(phase_time/2,1.0);
    auto command=r.at("command").get<std::vector<double>>();
    const auto q=r.at("joint_pos").get<std::vector<double>>();
    const auto gyro=r.at("angular_velocity").get<std::vector<double>>();
    const auto quat=r.at("quaternion").get<Quat>();
    if(command.size()!=3 || q.size()!=12 || gyro.size()!=3) throw std::runtime_error("state dimensions");
    clip.apply(command[0],command[1],command[2]);
    // gemini_walk's explicit 1.5m/s cap, with game buttons/reference requests disabled.
    command[0]=std::clamp(command[0],-1.5,1.5);
    if(has_jump) command[1]=std::clamp(command[1],-1.5,1.5);
    command[2]=heading.apply(quat_yaw(quat),command[0],command[1],command[2]);
    if(std::isfinite(scheduled_slot_start) && jump.has_hop() && command[0] >= jump.hop_speed)
      throw std::runtime_error("scheduled jump probe excludes commands that select the production hop branch");
    const double w=quat[0],x=quat[1],y=quat[2],z=quat[3];
    for(int k=0;k<3;++k) {history[k]=gyro[k];history[6+k]=command[k];history[9+k]=(k==2);}
    history[3]=2*(w*y-x*z);history[4]=-2*(y*z+w*x);history[5]=-(1-2*(x*x+y*y));
    for(int j=0;j<12;++j) history[12+j]=q[j]-defaults[j];
    if(has_jump) compute_mixed_jump_reference_offset(gait,jump,defaults,phase_time,
        command[0],command[1],command[2],scheduled_slot_start,false,history.data()+36);
    else compute_gait_reference_offset(gait,defaults,phase_time,command[0],command[1],command[2],history.data()+36);
    for(float& v:history) {if(!std::isfinite(v)) throw std::runtime_error("nonfinite observation");v=std::clamp(v,-100.f,100.f);}
    input=history;
    network->forward(history.data());
    std::rotate(history.rbegin(),history.rbegin()+48,history.rend());
    for(int j=0;j<12;++j) {
      raw[j]=network->getOutputs()[j];effective[j]=fade*raw[j];
      history[24+j]=effective[j];before[j]=effective[j]*scale+float(defaults[j]);
      target[j]=std::clamp(before[j],float(lower[j]),float(upper[j]));
      if(!std::isfinite(target[j])) throw std::runtime_error("nonfinite target");
    }
    Json result={{"target",target},{"before_clip",before},{"raw_action",raw},
      {"effective_action",effective},{"effective_command",command},{"fade",fade},
      {"reference",std::vector<float>(input.begin()+36,input.begin()+48)}};
    if(std::isfinite(scheduled_slot_start)) {
      result["scheduled_slot_start_s"]=scheduled_slot_start;
      result["jump_reference_active"]=phase_time>=scheduled_slot_start && phase_time<scheduled_slot_start+jump.active_s;
    }
    if(r.value("debug",false)) result["observation"]=input;
    return result;
  }
};

int main(int argc,char** argv) {
  try {
    double slot_start=std::numeric_limits<double>::infinity();
    if(argc==5 && std::string(argv[3])=="--scheduled-jump") {
      std::size_t parsed=0;slot_start=std::stod(argv[4],&parsed);
      if(parsed!=std::string(argv[4]).size() || !std::isfinite(slot_start))
        throw std::runtime_error("scheduled jump start must be finite");
    } else if(argc!=4 || std::string(argv[3])!="--no-jump")
      throw std::runtime_error("usage: export_locomotion_controller POLICY REPLICAS --no-jump OR --scheduled-jump SLOT_START; without an explicit schedule jump capability unmeasured");
    const int count=std::stoi(argv[2]);if(count<1 || count>32) throw std::runtime_error("replica count outside 1..32");
    std::vector<std::unique_ptr<Controller>> controllers;
    for(int i=0;i<count;++i) controllers.emplace_back(new Controller(argv[1],slot_start));
    std::string line;
    while(std::getline(std::cin,line)) {
      const auto r=Json::parse(line);
      if(r.at("op")=="reset") {for(auto& c:controllers)c->reset();std::cout<<"{\"reset\":true}"<<std::endl;continue;}
      if(r.at("op")!="infer" || r.at("states").size()!=controllers.size()) throw std::runtime_error("invalid batch");
      Json result=Json::array();
      for(int i=0;i<count;++i) result.push_back(r.at("states")[i].is_null()?Json(nullptr):controllers[i]->infer(r.at("states")[i]));
      std::cout<<result.dump()<<std::endl;
    }
  } catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;return 1;}
}
