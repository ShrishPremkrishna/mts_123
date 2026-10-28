// Exercise production NeuralController::update against memory-backed ROS
// command interfaces. No controller manager, driver or physical robot is used.
#include <gtest/gtest.h>
#include <fstream>
#include <cstdio>
#include <unistd.h>
#include "neural_controller/neural_controller.hpp"

namespace {
class MemoryController : public neural_controller::NeuralController {
 public:
  static constexpr std::array<const char*, 5> fields{"position", "velocity", "effort", "kp", "kd"};
  std::array<std::array<double, 5>, 12> values{};
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> handles;
  std::array<double,7> imu_values{{0.,0.,0.,std::sqrt(.5),0.,0.,std::sqrt(.5)}};
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> imu_handles;

  explicit MemoryController(bool slew) {
    params_.joint_names = neural_controller::position_slew_joint_names();
    params_.action_types.assign(12, "position");
    params_.init_kps.assign(12, 7.5);
    params_.init_kds.assign(12, .25);
    params_.estop_kd = .7;
    params_.init_duration = 1.;
    params_.fade_in_duration = 1.;
    params_.repeat_action = 10;
    init_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    slew_last_command_time_ = init_time_;
    init_joint_pos_.fill(1.);
    activation_joint_pos_.fill(0.);
    action_.fill(-1.);
    for (int j = 0; j < 12; ++j) {
      values[j].fill(8.);
      for (int k = 0; k < 5; ++k) {
        handles.push_back(std::make_shared<hardware_interface::CommandInterface>(
            params_.joint_names[j], fields[k], &values[j][k]));
        command_interfaces_.emplace_back(*handles.back());
      }
    }
    for (auto &handle : command_interfaces_)
      command_interfaces_map_[handle.get_prefix_name()].insert_or_assign(
          handle.get_interface_name(), std::ref(handle));
    if (slew) {
      position_target_slew_.enabled = true;
      position_target_slew_.max_rate_rad_s = .4;
      position_target_slew_.max_step_s = .02;
      position_target_slew_.lower.fill(-1.2);
      position_target_slew_.upper.fill(1.2);
      position_target_slew_.reset(init_joint_pos_);
    }
  }
  ~MemoryController() override { command_interfaces_.clear(); state_interfaces_.clear(); }
  void add_tilted_imu() {
    const std::array<const char*,7> names{{"angular_velocity.x","angular_velocity.y","angular_velocity.z",
      "orientation.x","orientation.y","orientation.z","orientation.w"}};
    for(int k=0;k<7;++k) {
      imu_handles.push_back(std::make_shared<hardware_interface::StateInterface>("imu_sensor",names[k],&imu_values[k]));
      state_interfaces_.emplace_back(*imu_handles.back());
    }
    for(auto &handle:state_interfaces_)
      state_interfaces_map_[handle.get_prefix_name()].insert_or_assign(handle.get_interface_name(),std::ref(handle));
    repeat_action_counter_=-1;
  }
  void stop() { estop_active_ = true; }
  void set_held_cycle() { repeat_action_counter_ = 0; }
  auto tick(double t) {
    return update(rclcpp::Time(static_cast<int64_t>(std::llround(t * 1e9)), RCL_ROS_TIME),
                  rclcpp::Duration::from_seconds(1./520));
  }
};

TEST(ProductionSafety, StopPreemptsInitializationForLegacyAndSlew) {
  for (bool slew : {false, true}) {
    MemoryController controller(slew);
    controller.stop();
    ASSERT_EQ(controller.tick(.25), controller_interface::return_type::OK);
    for (const auto &joint : controller.values) {
      for (int i = 0; i < 4; ++i) EXPECT_DOUBLE_EQ(joint[i], 0.);
      EXPECT_DOUBLE_EQ(joint[4], .7);
    }
  }
}

TEST(ProductionSafety, StopPreemptsHeldInferenceForLegacyAndSlew) {
  for (bool slew : {false, true}) {
    MemoryController controller(slew);
    controller.set_held_cycle();
    controller.stop();
    ASSERT_EQ(controller.tick(2.), controller_interface::return_type::OK);
    for (const auto &joint : controller.values) {
      for (int i = 0; i < 4; ++i) EXPECT_DOUBLE_EQ(joint[i], 0.);
      EXPECT_DOUBLE_EQ(joint[4], .7);
    }
  }
}

TEST(ProductionSafety, SlewLimitsInitializationAndContinuesHeldRequests) {
  MemoryController controller(true);
  ASSERT_EQ(controller.tick(.25), controller_interface::return_type::OK);
  EXPECT_NEAR(controller.values[0][0], 1.-.4*.02, 1e-12);
  EXPECT_DOUBLE_EQ(controller.values[0][3], 7.5);
  EXPECT_DOUBLE_EQ(controller.values[0][4], .25);
  controller.set_held_cycle();
  ASSERT_EQ(controller.tick(2.), controller_interface::return_type::OK);
  const double previous = controller.values[0][0];
  EXPECT_NEAR(previous, 1.-2*.4*.02, 1e-12);
  ASSERT_EQ(controller.tick(2.+1./520), controller_interface::return_type::OK);
  EXPECT_NEAR(controller.values[0][0], previous-.4/520, 5e-10);
}

TEST(ProductionSafety, LegacyInitializationAndHeldCommandAreUnchanged) {
  MemoryController controller(false);
  ASSERT_EQ(controller.tick(.25), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(controller.values[0][0], .75);
  controller.set_held_cycle();
  ASSERT_EQ(controller.tick(2.), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(controller.values[0][0], .75);
}
class ProductionRuntime : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { int argc=0; rclcpp::init(argc,nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  nlohmann::json make_policy(bool slew) {
    using Json=nlohmann::json;
    Json policy={{"in_shape",{nullptr,36}},{"observation_history",1},{"action_scale",.75},
      {"kp",5.5},{"kd",.25},{"default_joint_pos",std::vector<double>(12,0.)},
      {"joint_lower_limits",std::vector<double>(12,-1.5)},{"joint_upper_limits",std::vector<double>(12,1.5)},
      {"layers",Json::array({{{"type","dense"},{"activation",""},{"shape",{nullptr,12}},
        {"weights",Json::array({std::vector<std::vector<double>>(36,std::vector<double>(12,0.)),std::vector<double>(12,0.)})}}})}};
    if(slew) {
      policy["required_controller_features"]={"position_target_slew_v1"};
      policy["action_types"]=std::vector<std::string>(12,"position");
      policy["position_target_slew"]={{"contract","pupper_position_target_slew_v1"},{"max_rate_rad_s",15.},{"max_step_s",.02},
        {"joint_names",neural_controller::position_slew_joint_names()},{"lower_limits",std::vector<double>(12,-1.2)},
        {"upper_limits",std::vector<double>(12,1.2)},{"driver_limits_sha256",std::string(64,'a')}};
    }
    return policy;
  }
  controller_interface::return_type initialize(MemoryController &controller,const nlohmann::json &policy,bool spoof) {
    const std::string path="/tmp/cs123-controller-policy-"+std::to_string(getpid())+".json";
    {std::ofstream output(path);output<<policy.dump();}
    auto options=rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("model_path",path),rclcpp::Parameter("joint_names",neural_controller::position_slew_joint_names()),
      rclcpp::Parameter("action_types",std::vector<std::string>(12,"position")),
      rclcpp::Parameter("init_kps",std::vector<double>(12,7.5)),rclcpp::Parameter("init_kds",std::vector<double>(12,.25)),
      rclcpp::Parameter("init_duration",1.),rclcpp::Parameter("estop_kd",.7),
      rclcpp::Parameter("position_target_slew_active",spoof),
      rclcpp::Parameter("position_target_slew_contract_json",std::string("forged"))});
    auto result=controller.init("slew_runtime_test","",520,"",options);
    std::remove(path.c_str());return result;
  }
};

TEST_F(ProductionRuntime, RuntimeCapabilityIsReadOnlyAndIgnoresOverrides) {
  for(bool slew:{false,true}) {
    MemoryController controller(false);const auto policy=make_policy(slew);
    ASSERT_EQ(initialize(controller,policy,!slew),controller_interface::return_type::OK);
    auto node=controller.get_node();
    EXPECT_EQ(node->get_parameter("position_target_slew_active").as_bool(),slew);
    const auto actual=nlohmann::json::parse(node->get_parameter("position_target_slew_contract_json").as_string());
    EXPECT_EQ(actual,slew?policy.at("position_target_slew"):nlohmann::json::object());
    EXPECT_TRUE(node->describe_parameter("position_target_slew_active").read_only);
    EXPECT_TRUE(node->describe_parameter("position_target_slew_contract_json").read_only);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("position_target_slew_active",!slew)).successful);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("position_target_slew_contract_json",std::string("{}"))).successful);
  }
}

TEST_F(ProductionRuntime, TiltAppliesDampingInTheDetectionUpdate) {
  MemoryController controller(false);
  ASSERT_EQ(initialize(controller,make_policy(false),false),controller_interface::return_type::OK);
  controller.add_tilted_imu();
  ASSERT_EQ(controller.tick(2.),controller_interface::return_type::OK);
  for(const auto &joint:controller.values) {
    for(int i=0;i<4;++i) EXPECT_DOUBLE_EQ(joint[i],0.);
    EXPECT_DOUBLE_EQ(joint[4],.7);
  }
}
} // namespace
