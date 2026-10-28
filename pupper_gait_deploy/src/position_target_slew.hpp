#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace neural_controller {

inline const std::vector<std::string>& position_slew_joint_names() {
  static const std::vector<std::string> names={
    "leg_front_r_1","leg_front_r_2","leg_front_r_3",
    "leg_front_l_1","leg_front_l_2","leg_front_l_3",
    "leg_back_r_1","leg_back_r_2","leg_back_r_3",
    "leg_back_l_1","leg_back_l_2","leg_back_l_3"};
  return names;
}

// Opt-in physical command contract. Desired actor output/history is independent
// of this state. Call at EVERY command update, even while inference is held.
struct PositionTargetSlew {
  bool enabled=false, initialized=false;
  double max_rate_rad_s=0, max_step_s=0;
  std::string driver_limits_sha256;
  std::array<double,12> lower{},upper{},previous{};

  template<class Values> void reset(const Values& measured) {
    if(!enabled) return;
    if(measured.size()!=12) throw std::runtime_error("slew reset requires12 named joints");
    std::array<double,12> candidate{};
    for(int j=0;j<12;++j) {
      if(!std::isfinite(measured[j])) throw std::runtime_error("nonfinite slew reset position");
      candidate[j]=std::clamp(double(measured[j]),lower[j],upper[j]);
    }
    previous=candidate; initialized=true;
  }

  template<class Values> std::array<double,12> step(const Values& request,double dt) {
    if(!enabled || !initialized) throw std::runtime_error("slew contract is not active/reset");
    if(!std::isfinite(dt) || dt<0 || request.size()!=12) throw std::runtime_error("invalid slew command clock/dimensions");
    std::array<double,12> candidate{};
    const double distance=max_rate_rad_s*std::min(dt,max_step_s);
    for(int j=0;j<12;++j) {
      if(!std::isfinite(request[j])) throw std::runtime_error("nonfinite slew request");
      const double desired=std::clamp(double(request[j]),lower[j],upper[j]);
      candidate[j]=previous[j]+std::clamp(desired-previous[j],-distance,distance);
      candidate[j]=std::clamp(candidate[j],lower[j],upper[j]);
    }
    previous=candidate; return previous;
  }
};

template<class Json,class Values>
inline PositionTargetSlew parse_position_target_slew(const Json& policy,
    const Values& model_lower,const Values& model_upper,
    const std::vector<std::string>& names,const std::vector<std::string>& action_types) {
  PositionTargetSlew slew;
  const bool present=policy.contains("position_target_slew");
  bool required=false;
  if(policy.contains("required_controller_features")) {
    const auto& features=policy.at("required_controller_features");
    if(!features.is_array()) throw std::runtime_error("required_controller_features must be an array");
    for(const auto& feature:features) {
      if(!feature.is_string() || feature.template get<std::string>()!="position_target_slew_v1" || required)
        throw std::runtime_error("unknown or duplicate required controller feature");
      required=true;
    }
  }
  if(present!=required) throw std::runtime_error("slew metadata and required controller feature must appear together");
  if(!present) return slew; // Exact legacy path when neither field is present.
  const auto& block=policy.at("position_target_slew");
  if(!block.is_object() || block.at("contract")!="pupper_position_target_slew_v1")
    throw std::runtime_error("unknown position target slew contract");
  if(names!=position_slew_joint_names() || block.at("joint_names").template get<std::vector<std::string>>()!=names)
    throw std::runtime_error("slew named joint order differs from controller");
  if(action_types.size()!=12 || std::any_of(action_types.begin(),action_types.end(),[](const auto& t){return t!="position";}))
    throw std::runtime_error("slew contract requires position actions only");
  if(!policy.contains("action_types") || policy.at("action_types").template get<std::vector<std::string>>()!=action_types)
    throw std::runtime_error("slew policy action types must explicitly match position configuration");
  if(!block.at("max_rate_rad_s").is_number() || !block.at("max_step_s").is_number())
    throw std::runtime_error("slew rate and step must be numeric");
  slew.max_rate_rad_s=block.at("max_rate_rad_s");slew.max_step_s=block.at("max_step_s");
  if(!std::isfinite(slew.max_rate_rad_s) || slew.max_rate_rad_s<=0 || slew.max_rate_rad_s>15 ||
     !std::isfinite(slew.max_step_s) || slew.max_step_s<=0 || slew.max_step_s>.02)
    throw std::runtime_error("slew limits must be finite, positive and no greater than15rad/s and0.02s");
  if(model_lower.size()!=12 || model_upper.size()!=12 || !block.at("lower_limits").is_array() ||
     !block.at("upper_limits").is_array() || block.at("lower_limits").size()!=12 || block.at("upper_limits").size()!=12)
    throw std::runtime_error("slew limits require12 bounds");
  for(int j=0;j<12;++j) {
    if(!block.at("lower_limits").at(j).is_number() || !block.at("upper_limits").at(j).is_number())
      throw std::runtime_error("slew bounds must be numeric");
    slew.lower[j]=block.at("lower_limits").at(j);slew.upper[j]=block.at("upper_limits").at(j);
    if(!std::isfinite(slew.lower[j]) || !std::isfinite(slew.upper[j]) || !std::isfinite(model_lower[j]) ||
       !std::isfinite(model_upper[j]) || slew.lower[j]>=slew.upper[j] || slew.lower[j]<model_lower[j] || slew.upper[j]>model_upper[j])
      throw std::runtime_error("slew bounds must be finite, ordered and contained in model limits");
  }
  slew.driver_limits_sha256=block.at("driver_limits_sha256").template get<std::string>();
  if(slew.driver_limits_sha256.size()!=64 || !std::all_of(slew.driver_limits_sha256.begin(),slew.driver_limits_sha256.end(),[](char c){return (c>='0'&&c<='9')||(c>='a'&&c<='f');}))
    throw std::runtime_error("slew driver-limits SHA256 is invalid");
  slew.enabled=true;return slew;
}
} // namespace neural_controller
