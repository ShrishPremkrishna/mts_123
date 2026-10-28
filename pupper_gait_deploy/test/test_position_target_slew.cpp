#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <json.hpp>
#include "neural_controller/position_target_slew.hpp"
using namespace neural_controller;
using Json=nlohmann::json;

Json policy() {
  return {{"required_controller_features",{"position_target_slew_v1"}},
    {"action_types",std::vector<std::string>(12,"position")},
    {"position_target_slew",{{"contract","pupper_position_target_slew_v1"},{"max_rate_rad_s",15.},{"max_step_s",.02},
      {"joint_names",position_slew_joint_names()},{"lower_limits",std::vector<double>(12,-1.)},
      {"upper_limits",std::vector<double>(12,1.)},{"driver_limits_sha256",std::string(64,'a')}}}};
}
PositionTargetSlew parse(const Json& p) {
  return parse_position_target_slew(p,std::vector<double>(12,-2.),std::vector<double>(12,2.),position_slew_joint_names(),std::vector<std::string>(12,"position"));
}
template<class F> void refuses(F f) { bool rejected=false;try{f();}catch(const std::exception&){rejected=true;}assert(rejected); }

int main() {
  assert(!parse(Json::object()).enabled);
  auto slew=parse(policy());std::array<double,12> measured{},request{};
  measured.fill(2.);request.fill(-100.);slew.reset(measured);
  assert(slew.previous[0]==1.);assert(slew.step(request,0)[0]==1.);
  assert(std::abs(slew.step(request,1)[0]-.7)<1e-12); // jitter is capped to20ms
  for(int i=0;i<100;++i) slew.step(request,.001);
  assert(std::abs(slew.previous[0]+.8)<1e-12);
  for(int i=0;i<20;++i) slew.step(request,.001);
  assert(slew.previous[0]==-1.);
  auto saved=slew.previous;
  refuses([&]{slew.step(request,-.1);});assert(slew.previous==saved);
  refuses([&]{slew.step(request,std::numeric_limits<double>::infinity());});assert(slew.previous==saved);
  request[11]=std::numeric_limits<double>::quiet_NaN();
  refuses([&]{slew.step(request,.02);});assert(slew.previous==saved);
  measured[11]=std::numeric_limits<double>::infinity();
  refuses([&]{slew.reset(measured);});assert(slew.previous==saved);
  // Reset must remove prior trajectory state and start at the measured bounded pose.
  measured.fill(.2);slew.reset(measured);assert(slew.previous[0]==.2);
  request.fill(1.);double previous=.2;
  for(int i=0;i<30;++i) {
    double next=slew.step(request,1./520)[0];assert(next-previous<=15./520+1e-12);previous=next;
  }
  assert(std::abs(previous-1.)<1e-12); // advances between held inference requests
  for(const auto& key: {"max_rate_rad_s","max_step_s"}) {
    auto p=policy();p["position_target_slew"][key]=0;refuses([&]{parse(p);});
    p["position_target_slew"][key]=true;refuses([&]{parse(p);});
  }
  auto p=policy();p["position_target_slew"]["max_rate_rad_s"]=15.001;refuses([&]{parse(p);});
  p=policy();p["position_target_slew"]["max_step_s"]=.02001;refuses([&]{parse(p);});
  p=policy();p["position_target_slew"]["lower_limits"][0]=-2.1;refuses([&]{parse(p);});
  p=policy();p["position_target_slew"]["joint_names"][0]="other";refuses([&]{parse(p);});
  p=policy();p["position_target_slew"]["driver_limits_sha256"]="invalid";refuses([&]{parse(p);});
  p=policy();p.erase("required_controller_features");refuses([&]{parse(p);});
  p=policy();p.erase("position_target_slew");refuses([&]{parse(p);});
  p=policy();p["required_controller_features"].push_back("unknown");refuses([&]{parse(p);});
  p=policy();p["action_types"][0]="effort";refuses([&]{parse(p);});
  p=policy();p.erase("action_types");refuses([&]{parse(p);});
  std::cout<<"Position target slew tests passed\n";
}
