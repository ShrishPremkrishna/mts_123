#include <thread>
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
  std::array<double,12> joint_state{};
  void add_flat_feedback() {
    imu_values={0.,0.,0.,0.,0.,0.,1.};add_tilted_imu();
    for(int j=0;j<12;++j) {
      imu_handles.push_back(std::make_shared<hardware_interface::StateInterface>(params_.joint_names[j],"position",&joint_state[j]));
      state_interfaces_.emplace_back(*imu_handles.back());
    }
  }
  auto activate_in_memory() {
    const auto result=on_activate(rclcpp_lifecycle::State());
    init_time_=rclcpp::Time(0,0,RCL_ROS_TIME);repeat_action_counter_=-1;
    return result;
  }
  std::string clip_receipt_buffer() const { return rt_clip_status_publisher_->msg_.data; }
  auto execution_id() const {return execution_request_id_;}
  auto clip_request_id() const {return clip_player_.request_id();}
  auto queued_clips() const {return clip_player_.queued();}
  void queue_clip(const std::string& payload) {
    auto message=std::make_shared<std_msgs::msg::String>();message->data=payload;
    rt_clip_request_ptr_.writeFromNonRT(message);repeat_action_counter_=-1;
  }
  auto tick_actor(double t) {repeat_action_counter_=-1;return tick(t);}
  void spoil_terminal_motion() { for(int j=12;j<24;++j)trick_.joint_pos[j]=std::numeric_limits<float>::quiet_NaN(); }
  void enable_reference(std::shared_ptr<std_msgs::msg::Float32MultiArray> message, bool busy=false) {
    use_jump_slot_=true;params_.reference_tricks=true;
    jump_slot_.n_samples=2;jump_slot_.n_joints=12;
    jump_slot_.playback_s=.94;jump_slot_.active_s=.8;jump_slot_.cross_fade_s=.06;
    jump_slot_.grid_s=.75;jump_slot_.busy_s=1.;jump_slot_.jump_table.assign(24,0.);
    slot_starts_.fill(std::numeric_limits<double>::infinity());
    if(busy) slot_starts_[0]=1.;
    rt_reference_trick_ptr_.writeFromNonRT(message);repeat_action_counter_=-1;
  }
  auto reference_state() const { return reference_execution_; }
  auto retained_reference() const { return last_reference_trick_; }
  void stop() { estop_active_ = true; }
  void set_held_cycle() { repeat_action_counter_ = 0; }
  controller_interface::return_type tick(double t) {
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
  nlohmann::json make_trick_policy() {
    using Json=nlohmann::json;
    auto policy=make_policy(false);
    policy["in_shape"]={nullptr,24};
    policy["layers"][0]["weights"][0]=std::vector<std::vector<double>>(24,std::vector<double>(12,0.));
    policy["obs_spec"]={{"frame_order","newest_first"},{"components",Json::array({{{"name","motion_command"},{"size",24}}})}};
    policy["trick"]={{"fps",50.},{"frames",2},{"duration_s",.04}};
    policy["motion"]={{"joint_pos",std::vector<std::vector<double>>(2,std::vector<double>(12,0.))},
      {"joint_vel",std::vector<std::vector<double>>(2,std::vector<double>(12,0.))},
      {"anchor_quat_wxyz",{{1.,0.,0.,0.},{1.,0.,0.,0.}}}};
    return policy;
  }
  controller_interface::return_type initialize(MemoryController &controller,const nlohmann::json &policy,bool spoof) {
    const std::string path="/tmp/cs123-controller-policy-"+std::to_string(getpid())+".json";
    {std::ofstream output(path);output<<policy.dump();}
    auto options=rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("model_path",path),rclcpp::Parameter("joint_names",neural_controller::position_slew_joint_names()),
      rclcpp::Parameter("action_types",std::vector<std::string>(12,"position")),
      rclcpp::Parameter("init_kps",std::vector<double>(12,7.5)),rclcpp::Parameter("init_kds",std::vector<double>(12,.25)),
      rclcpp::Parameter("init_duration",1.),rclcpp::Parameter("fade_in_duration",1.),rclcpp::Parameter("estop_kd",.7),
      rclcpp::Parameter("reference_trick_status_protocol",std::string("forged")),
      rclcpp::Parameter("reference_trick_status_active",true),
      rclcpp::Parameter("policy_execution_status_protocol",std::string("forged")),
      rclcpp::Parameter("policy_execution_status_active",true),
      rclcpp::Parameter("interactive_clip_status_protocol",std::string("forged")),
      rclcpp::Parameter("interactive_clip_status_active",true),
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

TEST_F(ProductionRuntime, ReferenceCapabilityCannotBeSpoofedOrChanged) {
  MemoryController controller(false);
  ASSERT_EQ(initialize(controller,make_policy(false),false),controller_interface::return_type::OK);
  auto node=controller.get_node();
  EXPECT_EQ(node->get_parameter("reference_trick_status_protocol").as_string(),"pupper_reference_v1");
  EXPECT_FALSE(node->get_parameter("reference_trick_status_active").as_bool());
  auto result=node->set_parameter(rclcpp::Parameter("reference_trick_status_protocol",std::string("forged")));
  EXPECT_FALSE(result.successful);
}

std::shared_ptr<std_msgs::msg::Float32MultiArray> reference_message() {
  auto message=std::make_shared<std_msgs::msg::Float32MultiArray>();
  message->data.assign(27,0.f);message->data[0]=.94f;message->data[1]=.8f;message->data[2]=.06f;
  message->layout.dim.resize(1);message->layout.dim[0].label=std::string("pupper_reference_v1:")+std::string(32,'a');
  return message;
}

TEST_F(ProductionRuntime, ReferenceAcceptedThenObservationErrorNeverConfirmsCompletion) {
  MemoryController controller(false);
  ASSERT_EQ(initialize(controller,make_policy(false),false),controller_interface::return_type::OK);
  auto message=reference_message();controller.enable_reference(message);
  // Deliberately omit observation state interfaces. A real accepted request can
  // still fail during update; success must wait until actions were applied.
  EXPECT_EQ(controller.tick(2.1),controller_interface::return_type::ERROR);
  EXPECT_EQ(controller.reference_state().phase,"playing");
  EXPECT_EQ(controller.retained_reference(),message);
  controller.stop();EXPECT_EQ(controller.tick(2.2),controller_interface::return_type::OK);
  EXPECT_EQ(controller.reference_state().phase,"interrupted");
}

TEST_F(ProductionRuntime, MalformedReservedReferenceLabelsNeverFallBackToLegacyPlayback) {
  for(const std::string& label:std::vector<std::string>{"pupper_reference_v1:bad",
      "pupper_reference_v2:"+std::string(32,'a'),"pupper_reference_v1:"+std::string(32,'A'),
      "pupper_reference_","pupper_reference_v1:"}) {
    MemoryController controller(false);
    ASSERT_EQ(initialize(controller,make_policy(false),false),controller_interface::return_type::OK);
    auto message=reference_message();message->layout.dim.front().label=label;
    controller.enable_reference(message);
    // Missing observation interfaces fail later, after the parser is exercised.
    EXPECT_EQ(controller.tick(2.1),controller_interface::return_type::ERROR);
    EXPECT_FALSE(controller.reference_state().running);
    EXPECT_TRUE(controller.reference_state().phase.empty());
  }
  for(const std::string& label:std::vector<std::string>{"","legacy_jump_table"}) {
    MemoryController controller(false);
    ASSERT_EQ(initialize(controller,make_policy(false),false),controller_interface::return_type::OK);
    auto message=reference_message();message->layout.dim.front().label=label;
    controller.enable_reference(message);
    EXPECT_EQ(controller.tick(2.1),controller_interface::return_type::ERROR);
    EXPECT_TRUE(controller.reference_state().running);
    EXPECT_EQ(controller.reference_state().phase,"playing");
    EXPECT_TRUE(controller.reference_state().active_id.empty());
  }
}

TEST_F(ProductionRuntime, ReferenceBusyAndNonfinitePayloadAreExplicitlyRejected) {
  for(bool busy:{false,true}) {
    MemoryController controller(false);
    ASSERT_EQ(initialize(controller,make_policy(false),false),controller_interface::return_type::OK);
    auto message=reference_message();if(!busy)message->data.back()=std::numeric_limits<float>::quiet_NaN();
    controller.enable_reference(message,busy);
    EXPECT_EQ(controller.tick(2.1),controller_interface::return_type::ERROR);
    EXPECT_EQ(controller.reference_state().phase,"rejected");
    EXPECT_EQ(controller.reference_state().reason,busy ? "busy" : "invalid_payload");
    EXPECT_FALSE(controller.reference_state().running);
  }
}

TEST_F(ProductionRuntime, PolicyExecutionCapabilityReflectsTheLoadedModel) {
  for(bool trick:{false,true}) {
    MemoryController controller(false);
    ASSERT_EQ(initialize(controller,trick?make_trick_policy():make_policy(false),false),controller_interface::return_type::OK);
    auto node=controller.get_node();
    EXPECT_EQ(node->get_parameter("policy_execution_status_protocol").as_string(),"pupper_policy_v1");
    EXPECT_EQ(node->get_parameter("policy_execution_status_active").as_bool(),trick);
    EXPECT_TRUE(node->describe_parameter("policy_execution_status_protocol").read_only);
    EXPECT_TRUE(node->describe_parameter("policy_execution_status_active").read_only);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("policy_execution_status_active",!trick)).successful);
  }
}

TEST_F(ProductionRuntime, MalformedExecutionIdRefusesActivationBeforeCommands) {
  MemoryController controller(false);
  ASSERT_EQ(initialize(controller,make_trick_policy(),false),controller_interface::return_type::OK);
  ASSERT_TRUE(controller.get_node()->set_parameter(rclcpp::Parameter("execution_request_id",std::string("bad"))).successful);
  EXPECT_EQ(controller.activate_in_memory(),controller_interface::CallbackReturn::ERROR);
  for(const auto &joint:controller.values)for(double value:joint)EXPECT_DOUBLE_EQ(value,8.);
}

TEST_F(ProductionRuntime, FiniteReceiptUsesActivationIdAndWaitsForAppliedAction) {
  for(bool fail:{false,true}) {
    MemoryController controller(false);
    ASSERT_EQ(initialize(controller,make_trick_policy(),false),controller_interface::return_type::OK);
    auto node=controller.get_node();
    ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("execution_request_id",std::string(32,'a'))).successful);
    controller.add_flat_feedback();
    ASSERT_EQ(controller.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
    ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("execution_request_id",std::string(32,'b'))).successful);
    EXPECT_EQ(controller.execution_id(),std::string(32,'a'));
    if(fail)controller.spoil_terminal_motion();
    const auto result=controller.tick(2.1);
    if(fail) {
      EXPECT_EQ(result,controller_interface::return_type::ERROR);
      EXPECT_TRUE(controller.clip_receipt_buffer().empty());
      for(const auto &joint:controller.values)for(double value:joint)EXPECT_DOUBLE_EQ(value,8.);
    } else {
      EXPECT_EQ(result,controller_interface::return_type::OK);
      EXPECT_EQ(controller.clip_receipt_buffer(),"all holding frame 1 queued 0 execution "+std::string(32,'a')+" request none");
      for(const auto &joint:controller.values)EXPECT_DOUBLE_EQ(joint[0],0.);
    }
  }
}

TEST_F(ProductionRuntime, InteractiveCapabilityIsReadOnlyAndCannotBeSpoofed) {
  for(bool trick:{false,true}) {
    MemoryController controller(false);
    ASSERT_EQ(initialize(controller,trick?make_trick_policy():make_policy(false),false),controller_interface::return_type::OK);
    auto node=controller.get_node();
    EXPECT_EQ(node->get_parameter("interactive_clip_status_protocol").as_string(),"pupper_clip_v1");
    EXPECT_EQ(node->get_parameter("interactive_clip_status_active").as_bool(),trick);
    EXPECT_TRUE(node->describe_parameter("interactive_clip_status_protocol").read_only);
    EXPECT_TRUE(node->describe_parameter("interactive_clip_status_active").read_only);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("interactive_clip_status_active",!trick)).successful);
  }
}

TEST_F(ProductionRuntime, CorrelatedClipRequestUsesAppliedFrameAndMalformedRequestDoesNotMutate) {
  for(bool named_activation:{false,true}) {
    MemoryController controller(false);
    ASSERT_EQ(initialize(controller,make_trick_policy(),false),controller_interface::return_type::OK);
    const std::string activation=named_activation?std::string(32,'a'):"none",request(32,'b');
    if(named_activation) { ASSERT_TRUE(controller.get_node()->set_parameter(rclcpp::Parameter("execution_request_id",activation)).successful); }
    controller.add_flat_feedback();ASSERT_EQ(controller.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);
    EXPECT_EQ(controller.clip_receipt_buffer(),"all holding frame 1 queued 0 execution "+activation+" request none");
    controller.queue_clip("pupper_clip_v1 "+request+" all");
    ASSERT_EQ(controller.tick_actor(2.2),controller_interface::return_type::OK);
    EXPECT_EQ(controller.clip_request_id(),request);
    EXPECT_EQ(controller.queued_clips(),0u);
    controller.queue_clip("pupper_clip_v1 bad all");
    ASSERT_EQ(controller.tick_actor(2.3),controller_interface::return_type::OK);
    EXPECT_EQ(controller.clip_request_id(),request);EXPECT_EQ(controller.queued_clips(),0u);
    controller.queue_clip("pupper_clip_v1 "+request+" all");
    ASSERT_EQ(controller.tick_actor(2.4),controller_interface::return_type::OK);
    EXPECT_EQ(controller.clip_request_id(),request);EXPECT_EQ(controller.queued_clips(),0u);
    // Once the publisher is available, the held receipt retains both causal IDs.
    for(int attempt=0;attempt<20;++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      ASSERT_EQ(controller.tick_actor(2.5+attempt*.01),controller_interface::return_type::OK);
      if(controller.clip_receipt_buffer()=="all holding frame 1 queued 0 execution "+activation+" request "+request) break;
    }
    EXPECT_EQ(controller.clip_receipt_buffer(),"all holding frame 1 queued 0 execution "+activation+" request "+request);
    controller.queue_clip("all");ASSERT_EQ(controller.tick_actor(3.),controller_interface::return_type::OK);
    EXPECT_TRUE(controller.clip_request_id().empty());
  }
}

} // namespace
