#include <thread>
#include <chrono>
#include <cmath>
#include <limits>
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
  void add_full_tracking_feedback() {
    add_flat_feedback();
    for(int j=0;j<12;++j) {
      imu_handles.push_back(std::make_shared<hardware_interface::StateInterface>(params_.joint_names[j],"velocity",&joint_state[j]));
      state_interfaces_.emplace_back(*imu_handles.back());
    }
  }
  auto selected_clip() const { return clip_player_.clip(); }
  auto clip_holding() const { return clip_player_.holding(); }
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
  // /cmd_vel: receipts go through the production callback against a fake
  // steady clock, so expiry is exact without sleeping.
  std::chrono::steady_clock::time_point fake_now{std::chrono::seconds(100)};
  std::chrono::steady_clock::time_point cmd_vel_now() const override { return fake_now; }
  std::int64_t command_deadline_now_ns() const override {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(fake_now.time_since_epoch()).count();
  }
  auto deadline_callback() const { return command_deadline_callback_; }
  auto deadline_generation() const { return cmd_vel_generation_.load(); }
  void delayed_authority(std::uint64_t generation, std::int64_t activation,
                         std::int64_t sampled, const std::string &source) {
    publish_command_authority({command_clock_id_,source,generation,activation,sampled});
  }
  void commit_parsed(const neural_controller::CommandDeadline &value, std::uint64_t generation) {
    commit_command_deadline(&value, generation);
  }
  bool strict_subscriptions() const {
    return command_deadline_subscriber_ && !cmd_vel_subscriber_ && !cmd_pose_subscriber_ &&
      !joy_subscriber_ && !reference_trick_subscriber_;
  }
  nlohmann::json envelope(std::int64_t seq, double vx=0., double vy=0., double wz=0.) const {
    const auto now = command_deadline_now_ns();
    return {{"schema",neural_controller::kCommandDeadlineSchema},{"clock_id",command_clock_id_},
      {"source_instance",std::string(32,'a')},{"seq",seq},{"issued_mono_ns",now},
      {"expires_mono_ns",now+150000000},{"vx",vx},{"vy",vy},{"wz",wz},{"allowed",true}};
  }
  void send_deadline(const nlohmann::json &value) {
    auto message=std::make_shared<std_msgs::msg::String>(); message->data=value.dump();
    command_deadline_callback_(message);
  }
  void advance(std::chrono::nanoseconds d) {
    fake_now += std::chrono::duration_cast<std::chrono::steady_clock::duration>(d);
  }
  auto cmd_vel_callback() const { return cmd_vel_callback_; }
  auto received_cmd_vel() { return *rt_cmd_vel_ptr_.readFromRT(); }
  void deliver_late_sample(std::shared_ptr<const CmdVelSample> sample) {
    rt_cmd_vel_ptr_.writeFromNonRT(sample);
  }
  bool has_cmd_vel_subscription() const { return cmd_vel_subscriber_ != nullptr; }
  void send_cmd_vel(double vx, double vy, double wz, double vz = 0.) {
    auto message=std::make_shared<geometry_msgs::msg::Twist>();
    message->linear.x=vx;message->linear.y=vy;message->angular.z=wz;message->linear.z=vz;
    cmd_vel_callback_(message);
  }
  std::array<float,3> observed_command() const {
    const int c=obs_layout_.command;
    return {observation_.at(c),observation_.at(c+1),observation_.at(c+2)};
  }
  float observed_gait() const { return observation_.at(obs_layout_.gait_reference); }
  auto deactivate_in_memory() {
    const auto result=on_deactivate(rclcpp_lifecycle::State());
    // on_deactivate releases the loans; hand the same memory back for reactivation.
    for(auto &handle:handles) command_interfaces_.emplace_back(*handle);
    for(auto &handle:imu_handles) state_interfaces_.emplace_back(*handle);
    return result;
  }
  int repeat_counter() const { return repeat_action_counter_; }
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
  nlohmann::json make_static_loop_policy(bool moving=false) {
    using Json=nlohmann::json;
    auto policy=make_trick_policy();
    policy["in_shape"]={nullptr,69};
    policy["layers"][0]["weights"][0]=std::vector<std::vector<double>>(69,std::vector<double>(12,0.));
    policy["obs_spec"]={{"frame_order","newest_first"},{"components",Json::array({
      {{"name","motion_command"},{"size",24}},{{"name","motion_anchor_ori_b"},{"size",6}},
      {{"name","base_ang_vel"},{"size",3}},{{"name","joint_pos_rel"},{"size",12}},
      {{"name","joint_vel_rel"},{"size",12}},{{"name","last_action"},{"size",12}}})}};
    policy["trick"]={{"fps",2.},{"clips",Json::array({
      {{"name","idle"},{"start",0},{"end",3},{"loop",true}},
      {{"name","exit"},{"start",3},{"end",6}}})}};
    policy["motion"]={{"joint_pos",std::vector<std::vector<double>>(6,std::vector<double>(12,0.))},
      {"joint_vel",std::vector<std::vector<double>>(6,std::vector<double>(12,0.))},
      {"anchor_quat_wxyz",std::vector<std::vector<double>>(6,{1.,0.,0.,0.})}};
    policy["motion"]["joint_vel"][3][0]=.125; // same existing boundary velocity step
    policy["motion"]["joint_pos"][4][0]=.1;policy["motion"]["joint_pos"][5][0]=.2;
    if(moving)policy["motion"]["joint_pos"][1][0]=.001;
    return policy;
  }
  // Minimal velocity-gait policy: a parsed gait reference plus the command
  // component it reads vx/vy/wz from. Constant tables and zero weights, no real model.
  nlohmann::json make_gait_policy() {
    using Json=nlohmann::json;
    auto policy=make_policy(false);
    policy["in_shape"]={nullptr,21};
    policy["layers"][0]["weights"][0]=std::vector<std::vector<double>>(21,std::vector<double>(12,0.));
    policy["obs_spec"]={{"frame_order","newest_first"},{"components",Json::array({
      {{"name","base_ang_vel"},{"size",3}},{{"name","projected_gravity"},{"size",3}},
      {{"name","command"},{"size",3}},{{"name","gait_reference"},{"size",12}}})}};
    policy["gait_reference"]={{"n_samples",2},{"frequency",2.},{"blend_speed",.5},{"gallop_speed",10.},
      {"gallop_freq_mult",1.},{"dir_threshold",.05},
      {"trot_table",std::vector<std::vector<double>>(2,std::vector<double>(12,.1))},
      {"gallop_table",std::vector<std::vector<double>>(2,std::vector<double>(12,.1))}};
    return policy;
  }
  // Target-conditioned layout: a command component with no gait reference.
  // Twist is a target here, so it must not be treated as an expiring velocity.
  nlohmann::json make_target_policy() {
    using Json=nlohmann::json;
    auto policy=make_policy(false);
    policy["in_shape"]={nullptr,9};
    policy["layers"][0]["weights"][0]=std::vector<std::vector<double>>(9,std::vector<double>(12,0.));
    policy["obs_spec"]={{"frame_order","newest_first"},{"components",Json::array({
      {{"name","base_ang_vel"},{"size",3}},{{"name","projected_gravity"},{"size",3}},
      {{"name","command"},{"size",3}}})}};
    return policy;
  }
  // Initialized, fed and activated; the first post-fade tick runs inference.
  void activate_gait(MemoryController &controller) {
    ASSERT_EQ(initialize(controller,make_gait_policy(),false),controller_interface::return_type::OK);
    controller.add_flat_feedback();
    ASSERT_EQ(controller.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
  }
  void activate_deadline(MemoryController &controller, nlohmann::json policy = nullptr) {
    ASSERT_EQ(initialize(controller,policy.is_null()?make_gait_policy():policy,false,
      {rclcpp::Parameter("cmd_vel_contract",std::string("fc_deadline_v1")),
       rclcpp::Parameter("game_buttons",false)}),controller_interface::return_type::OK);
    controller.add_flat_feedback();
    ASSERT_EQ(controller.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
    ASSERT_TRUE(controller.strict_subscriptions());
  }
  controller_interface::return_type initialize(MemoryController &controller,const nlohmann::json &policy,bool spoof,
                                               std::vector<rclcpp::Parameter> extra={}) {
    const std::string path="/tmp/cs123-controller-policy-"+std::to_string(getpid())+".json";
    {std::ofstream output(path);output<<policy.dump();}
    std::vector<rclcpp::Parameter> overrides{
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
      rclcpp::Parameter("position_target_slew_contract_json",std::string("forged")),
      rclcpp::Parameter("cmd_vel_expiry_supported",!spoof),
      rclcpp::Parameter("cmd_vel_expiry_timeout_s",9.)};
    overrides.insert(overrides.end(),extra.begin(),extra.end());
    auto options=rclcpp::NodeOptions().parameter_overrides(overrides);
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

TEST_F(ProductionRuntime, StaticLoopRequestPreservesFadeFullClockAndAppliedReceipt) {
  MemoryController controller(false);
  ASSERT_EQ(initialize(controller,make_static_loop_policy(),false),controller_interface::return_type::OK);
  const std::string activation(32,'a'),request(32,'b');
  ASSERT_TRUE(controller.get_node()->set_parameter(rclcpp::Parameter("execution_request_id",activation)).successful);
  controller.add_full_tracking_feedback();ASSERT_EQ(controller.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
  controller.queue_clip("pupper_clip_v1 "+request+" exit");
  ASSERT_EQ(controller.tick_actor(1.5),controller_interface::return_type::OK);
  EXPECT_EQ(controller.selected_clip(),0);EXPECT_EQ(controller.queued_clips(),1u);
  EXPECT_TRUE(controller.clip_request_id().empty());
  ASSERT_EQ(controller.tick_actor(2.25),controller_interface::return_type::OK);
  EXPECT_EQ(controller.selected_clip(),1);EXPECT_EQ(controller.queued_clips(),0u);
  EXPECT_EQ(controller.clip_request_id(),request);EXPECT_EQ(controller.execution_id(),activation);
  EXPECT_FALSE(controller.clip_holding());
  for(int attempt=0;attempt<20;++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    ASSERT_EQ(controller.tick_actor(2.25),controller_interface::return_type::OK);
    if(controller.clip_receipt_buffer()=="exit playing frame 3 queued 0 execution "+activation+" request "+request)break;
  }
  EXPECT_EQ(controller.clip_receipt_buffer(),"exit playing frame 3 queued 0 execution "+activation+" request "+request);
  ASSERT_EQ(controller.tick_actor(3.24),controller_interface::return_type::OK);EXPECT_FALSE(controller.clip_holding());
  for(int attempt=0;attempt<20;++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    ASSERT_EQ(controller.tick_actor(3.25),controller_interface::return_type::OK);
    if(controller.clip_receipt_buffer()=="exit holding frame 5 queued 0 execution "+activation+" request "+request)break;
  }
  EXPECT_TRUE(controller.clip_holding());
  EXPECT_EQ(controller.clip_receipt_buffer(),"exit holding frame 5 queued 0 execution "+activation+" request "+request);
}

TEST_F(ProductionRuntime, MovingLoopKeepsBoundaryAndStopPreemptsStaticRequest) {
  for(bool stopped:{false,true}) {
    MemoryController controller(false);
    ASSERT_EQ(initialize(controller,make_static_loop_policy(!stopped),false),controller_interface::return_type::OK);
    controller.add_full_tracking_feedback();ASSERT_EQ(controller.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);
    controller.queue_clip("pupper_clip_v1 "+std::string(32,'b')+" exit");
    if(stopped)controller.stop();
    ASSERT_EQ(controller.tick_actor(2.25),controller_interface::return_type::OK);
    EXPECT_EQ(controller.selected_clip(),0);EXPECT_TRUE(controller.clip_request_id().empty());
    if(stopped) {
      for(const auto &joint:controller.values) {
        for(int k=0;k<4;++k)EXPECT_DOUBLE_EQ(joint[k],0.);
        EXPECT_DOUBLE_EQ(joint[4],.7);
      }
    } else {
      EXPECT_EQ(controller.queued_clips(),1u);
      ASSERT_EQ(controller.tick_actor(3.),controller_interface::return_type::OK);
      EXPECT_EQ(controller.selected_clip(),1);
    }
  }
}

using namespace std::chrono_literals;
void expect_command(MemoryController &controller,float vx,float vy) {
  const auto command=controller.observed_command();
  EXPECT_FLOAT_EQ(command[0],vx);EXPECT_FLOAT_EQ(command[1],vy);
}
void expect_command(MemoryController &controller,float vx,float vy,float wz) {
  expect_command(controller,vx,vy);EXPECT_FLOAT_EQ(controller.observed_command()[2],wz);
}
void expect_zero_command(MemoryController &controller) {
  for(float value:controller.observed_command())EXPECT_FLOAT_EQ(value,0.f);
}

TEST_F(ProductionRuntime, CmdVelExpiryContractIsReadOnlySnapshotAndCannotBeSpoofed) {
  for(double timeout:{.05,.25,1.}) {
    MemoryController controller(false);
    ASSERT_EQ(initialize(controller,make_gait_policy(),false,{rclcpp::Parameter("cmd_vel_timeout_s",timeout)}),
              controller_interface::return_type::OK);
    auto node=controller.get_node();
    EXPECT_TRUE(node->get_parameter("cmd_vel_expiry_supported").as_bool());
    EXPECT_DOUBLE_EQ(node->get_parameter("cmd_vel_expiry_timeout_s").as_double(),timeout);
    EXPECT_TRUE(node->describe_parameter("cmd_vel_expiry_supported").read_only);
    EXPECT_TRUE(node->describe_parameter("cmd_vel_expiry_timeout_s").read_only);
    EXPECT_TRUE(node->describe_parameter("cmd_vel_timeout_s").read_only);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("cmd_vel_expiry_supported",false)).successful);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("cmd_vel_expiry_timeout_s",.5)).successful);
  }
  // A later timeout change is refused and cannot move the loaded deadline.
  MemoryController controller(false);activate_gait(controller);
  ASSERT_FALSE(controller.get_node()->set_parameter(rclcpp::Parameter("cmd_vel_timeout_s",1.)).successful);
  controller.send_cmd_vel(.3,.1,.5);controller.advance(250ms);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);
  expect_zero_command(controller);
}

TEST_F(ProductionRuntime, InvalidCmdVelTimeoutRefusesLoad) {
  for(double timeout:{0.,-.25,.049,1.001,std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::infinity()}) {
    MemoryController controller(false);
    EXPECT_EQ(initialize(controller,make_gait_policy(),false,{rclcpp::Parameter("cmd_vel_timeout_s",timeout)}),
              controller_interface::return_type::ERROR) << timeout;
  }
}

TEST_F(ProductionRuntime, StartupBeforeAnyCommandIsUnchangedAndZero) {
  MemoryController controller(false);activate_gait(controller);
  EXPECT_TRUE(controller.has_cmd_vel_subscription());
  controller.send_cmd_vel(.3,.1,.5);
  // Initialization still interpolates with init gains; commands are not read.
  ASSERT_EQ(controller.tick(.5),controller_interface::return_type::OK);
  for(const auto &joint:controller.values) {
    EXPECT_DOUBLE_EQ(joint[0],0.);EXPECT_DOUBLE_EQ(joint[3],7.5);EXPECT_DOUBLE_EQ(joint[4],.25);
  }
  MemoryController quiet(false);activate_gait(quiet);
  ASSERT_EQ(quiet.tick_actor(2.1),controller_interface::return_type::OK);
  expect_zero_command(quiet);
}

TEST_F(ProductionRuntime, FreshCmdVelSurvivesUntilExactBoundary) {
  MemoryController controller(false);activate_gait(controller);
  controller.send_cmd_vel(.3,.1,.5);
  controller.advance(249999999ns);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);
  expect_command(controller,.3f,.1f);
  controller.advance(1ns);
  ASSERT_EQ(controller.tick_actor(2.2),controller_interface::return_type::OK);
  expect_zero_command(controller);
  // A fresh receipt restores the request.
  controller.send_cmd_vel(.2,0.,.5);
  ASSERT_EQ(controller.tick_actor(2.3),controller_interface::return_type::OK);
  expect_command(controller,.2f,0.f);
}

TEST_F(ProductionRuntime, ExpiryDuringHeldActionForcesInferenceWithZeroTargets) {
  MemoryController controller(false);activate_gait(controller);
  controller.send_cmd_vel(.3,.1,.5);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);
  ASSERT_EQ(controller.repeat_counter(),0);expect_command(controller,.3f,.1f);
  controller.advance(100ms);
  ASSERT_EQ(controller.tick(2.1+1./520),controller_interface::return_type::OK);
  EXPECT_EQ(controller.repeat_counter(),1);expect_command(controller,.3f,.1f);  // held, live
  controller.advance(150ms);
  ASSERT_EQ(controller.tick(2.1+2./520),controller_interface::return_type::OK);
  EXPECT_EQ(controller.repeat_counter(),0);expect_zero_command(controller);  // new observation
  ASSERT_EQ(controller.tick(2.1+3./520),controller_interface::return_type::OK);
  EXPECT_EQ(controller.repeat_counter(),1);  // remaining expired does not force every inference
  expect_zero_command(controller);
}

TEST_F(ProductionRuntime, ExpiryCannotBeUndoneByMalformedJumpSpeedCap) {
  MemoryController controller(false);
  auto policy=make_gait_policy();
  policy["jump_slot"]={{"n_samples",2},{"playback_s",.94},{"active_s",.8},
      {"cross_fade_s",.06},{"grid_s",.75},{"busy_s",1.},
      {"walk_speed_cap",-.49},{"run_speed_cap",-1.5},
      {"jump_table",std::vector<std::vector<double>>(2,std::vector<double>(12,0.))}};
  ASSERT_EQ(initialize(controller,policy,false),controller_interface::return_type::OK);
  controller.add_flat_feedback();
  ASSERT_EQ(controller.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
  controller.send_cmd_vel(.3,.1,.5);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);
  controller.advance(250ms);
  ASSERT_EQ(controller.tick(2.1+1./520),controller_interface::return_type::OK);
  expect_zero_command(controller);
  EXPECT_FLOAT_EQ(controller.observed_gait(),0.f);
}

TEST_F(ProductionRuntime, ExpiredZeroSurvivesClippingAndHeadingHoldIntoGaitObservation) {
  MemoryController controller(false);
  auto policy=make_gait_policy();
  policy["command_clip"]={{"vx",{.1,.5}},{"vy",{.1,.5}},{"wz",{.1,1.}}};
  policy["heading_hold"]={{"kp",1.5},{"clip",1.},{"yaw_threshold",.1},{"walk_threshold",.05}};
  ASSERT_EQ(initialize(controller,policy,false),controller_interface::return_type::OK);
  controller.add_flat_feedback();
  ASSERT_EQ(controller.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
  controller.send_cmd_vel(.3,.1,.5);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);
  EXPECT_GT(controller.observed_gait(),0.f);
  controller.advance(250ms);
  ASSERT_EQ(controller.tick(2.1+1./520),controller_interface::return_type::OK);
  expect_zero_command(controller);
  EXPECT_FLOAT_EQ(controller.observed_gait(),0.f);
}

TEST_F(ProductionRuntime, FutureReceiptCannotExtendCommandLifetime) {
  MemoryController controller(false);activate_gait(controller);
  controller.send_cmd_vel(.3,.1,.5);
  controller.advance(-1ns);  // injected bad clock/receipt ordering
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);
  expect_zero_command(controller);
}

TEST_F(ProductionRuntime, NonfiniteCmdVelRevokesPriorCommandImmediately) {
  const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
  for(int axis=0;axis<3;++axis) {
    MemoryController controller(false);activate_gait(controller);
    controller.send_cmd_vel(.3,.1,.5);
    ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);
    expect_command(controller,.3f,.1f);
    controller.send_cmd_vel(axis==0?nan:.3,axis==1?inf:.1,axis==2?-inf:.5);
    // The next (otherwise held) cycle infers with a zero request and no NaN error.
    ASSERT_EQ(controller.tick(2.1+1./520),controller_interface::return_type::OK);
    EXPECT_EQ(controller.repeat_counter(),0);expect_zero_command(controller);
  }
  // An axis the velocity path does not use does not revoke.
  MemoryController controller(false);activate_gait(controller);
  controller.send_cmd_vel(.3,.1,.5,nan);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);
  expect_command(controller,.3f,.1f);
}

TEST_F(ProductionRuntime, DeactivationClearsCmdVelAndLateOldCallbackCannotReplay) {
  MemoryController controller(false);activate_gait(controller);
  controller.send_cmd_vel(.3,.1,.5);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);
  expect_command(controller,.3f,.1f);
  const auto old_callback=controller.cmd_vel_callback();
  const auto old_sample=controller.received_cmd_vel();
  ASSERT_EQ(controller.deactivate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(controller.has_cmd_vel_subscription());
  // An in-flight delivery from the torn-down subscription, while inactive.
  auto late=std::make_shared<geometry_msgs::msg::Twist>();late->linear.x=.4;late->linear.y=.2;
  old_callback(late);
  ASSERT_EQ(controller.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(controller.has_cmd_vel_subscription());
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);
  expect_zero_command(controller);  // neither the prior nor the late command survives
  old_callback(late);  // and again after reactivation
  ASSERT_EQ(controller.tick_actor(2.2),controller_interface::return_type::OK);
  expect_zero_command(controller);
  // Simulate the old callback passing its first generation check before
  // deactivation, then completing its buffer write after reactivation.
  controller.deliver_late_sample(old_sample);
  ASSERT_EQ(controller.tick_actor(2.25),controller_interface::return_type::OK);
  expect_zero_command(controller);
  controller.send_cmd_vel(.2,-.1,.5);  // the new activation's receiver works
  ASSERT_EQ(controller.tick_actor(2.3),controller_interface::return_type::OK);
  expect_command(controller,.2f,-.1f);
}

TEST_F(ProductionRuntime, TargetAndTrickPoliciesAreNotTreatedAsExpiringVelocity) {
  MemoryController target(false);
  ASSERT_EQ(initialize(target,make_target_policy(),false),controller_interface::return_type::OK);
  EXPECT_FALSE(target.get_node()->get_parameter("cmd_vel_expiry_supported").as_bool());
  target.add_flat_feedback();
  ASSERT_EQ(target.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
  target.send_cmd_vel(.3,.1,.5);target.advance(5s);
  ASSERT_EQ(target.tick_actor(2.1),controller_interface::return_type::OK);
  const auto held=target.observed_command();
  EXPECT_FLOAT_EQ(held[0],.3f);EXPECT_FLOAT_EQ(held[1],.1f);EXPECT_FLOAT_EQ(held[2],.5f);
  for(const auto &policy:{make_trick_policy(),make_policy(false)}) {
    MemoryController controller(false);
    ASSERT_EQ(initialize(controller,policy,false),controller_interface::return_type::OK);
    EXPECT_FALSE(controller.get_node()->get_parameter("cmd_vel_expiry_supported").as_bool());
  }
}

} // namespace

TEST(CommandDeadlineParsing, BoundedExactSchemaAndNumericTypes) {
  using neural_controller::CommandDeadline;
  using neural_controller::decode_command_deadline;
  CommandDeadline parsed;
  nlohmann::json value={{"schema",neural_controller::kCommandDeadlineSchema},{"clock_id","clock"},
    {"source_instance",std::string(32,'a')},{"seq",1},{"issued_mono_ns",100},
    {"expires_mono_ns",150000100},{"vx",.2},{"vy",0.},{"wz",0.},{"allowed",true}};
  ASSERT_TRUE(decode_command_deadline(value.dump(),parsed));
  EXPECT_FALSE(decode_command_deadline(std::string(1025,' '),parsed));
  auto duplicate=value.dump();duplicate.insert(1,"\"seq\":1,");
  EXPECT_FALSE(decode_command_deadline(duplicate,parsed));
  for(const char *key:{"seq","issued_mono_ns","expires_mono_ns"}) {
    for(auto bad:std::vector<nlohmann::json>{true,0,-1,1.,"1",UINT64_MAX}) {
      auto mutated=value;mutated[key]=bad;EXPECT_FALSE(decode_command_deadline(mutated.dump(),parsed))<<key<<bad;
    }
  }
  for(const char *key:{"vx","vy","wz"}) {
    for(auto bad:std::vector<nlohmann::json>{true,"0",nullptr,nlohmann::json::array({0})}) {
      auto mutated=value;mutated[key]=bad;EXPECT_FALSE(decode_command_deadline(mutated.dump(),parsed));
    }
  }
  auto mutated=value;mutated["unknown"]=0;EXPECT_FALSE(decode_command_deadline(mutated.dump(),parsed));
  mutated=value;mutated["allowed"]=1;EXPECT_FALSE(decode_command_deadline(mutated.dump(),parsed));
  mutated=value;mutated["expires_mono_ns"]=150000101;EXPECT_FALSE(decode_command_deadline(mutated.dump(),parsed));
  mutated=value;mutated["issued_mono_ns"]=INT64_MAX;mutated["expires_mono_ns"]=INT64_MAX;
  EXPECT_FALSE(decode_command_deadline(mutated.dump(),parsed));
  mutated=value;mutated["vx"]="NaN";EXPECT_FALSE(decode_command_deadline(mutated.dump(),parsed));
  mutated=value;mutated["vx"] = .3/std::sqrt(2.); mutated["vy"] = .3/std::sqrt(2.);
  EXPECT_TRUE(decode_command_deadline(mutated.dump(),parsed));
  mutated["vx"] = .3/std::sqrt(2.)+1e-9;EXPECT_FALSE(decode_command_deadline(mutated.dump(),parsed));
}

TEST_F(ProductionRuntime, DeadlineModeRequiresGaitAndNoAlternateAuthority) {
  for(auto bad:std::vector<rclcpp::Parameter>{rclcpp::Parameter("game_buttons",true),
      rclcpp::Parameter("reference_tricks",true),rclcpp::Parameter("chord_partner",std::string("other"))}) {
    MemoryController controller(false);
    EXPECT_EQ(initialize(controller,make_gait_policy(),false,
      {rclcpp::Parameter("cmd_vel_contract",std::string("fc_deadline_v1")),
       rclcpp::Parameter("game_buttons",false),bad}),controller_interface::return_type::ERROR);
  }
  MemoryController target(false);
  EXPECT_EQ(initialize(target,make_target_policy(),false,
    {rclcpp::Parameter("cmd_vel_contract",std::string("fc_deadline_v1")),
     rclcpp::Parameter("game_buttons",false)}),controller_interface::return_type::ERROR);
}

TEST_F(ProductionRuntime, DeadlineRequiresFreshSourceZeroAndOriginalExpiryDuringHeldInference) {
  MemoryController controller(false);activate_deadline(controller);
  EXPECT_EQ(controller.get_node()->get_parameter("cmd_vel_loaded_contract").as_string(),"fc_deadline_v1");
  controller.send_deadline(controller.envelope(1,.2));
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  controller.send_deadline(controller.envelope(1));controller.advance(1ns);
  const auto moving=controller.envelope(2,.2);controller.send_deadline(moving);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_command(controller,.2f,0.f,0.f);
  controller.advance(149999999ns);controller.send_deadline(moving);  // Fresh receipt cannot extend origin.
  ASSERT_EQ(controller.tick(2.1),controller_interface::return_type::OK);expect_command(controller,.2f,0.f,0.f);
  controller.advance(1ns);
  ASSERT_EQ(controller.tick(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  EXPECT_EQ(controller.repeat_counter(),0);
  controller.send_deadline(moving);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
}

TEST_F(ProductionRuntime, DeadlineVetoAndSourceZeroBypassClipAndHeadingHold) {
  MemoryController controller(false);auto policy=make_gait_policy();
  policy["command_clip"]={{"vx",{.1,.5}},{"vy",{.1,.5}},{"wz",{.1,1.}}};
  policy["heading_hold"]={{"kp",1.5},{"clip",1.},{"yaw_threshold",.1},{"walk_threshold",.05}};
  activate_deadline(controller,policy);
  controller.send_deadline(controller.envelope(1));
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  controller.advance(1ns);auto moving=controller.envelope(2,.2);controller.send_deadline(moving);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);
  controller.imu_values[5]=std::sin(.3);controller.imu_values[6]=std::cos(.3);  // yaw only; no fall
  auto veto=moving;veto["allowed"]=false;controller.send_deadline(veto);
  ASSERT_EQ(controller.tick(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  EXPECT_FLOAT_EQ(controller.observed_gait(),0.f);EXPECT_EQ(controller.repeat_counter(),0);
  controller.send_deadline(moving);  // Delayed pre-veto duplicate cannot revive.
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  controller.advance(1ns);controller.send_deadline(controller.envelope(3,.2));
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);EXPECT_GT(controller.observed_command()[0],0.f);
  controller.advance(1ns);controller.send_deadline(controller.envelope(4));
  ASSERT_EQ(controller.tick(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
}

TEST_F(ProductionRuntime, DeadlineMalformedConflictingAndOldCommandsCannotRevive) {
  MemoryController controller(false);activate_deadline(controller);
  controller.send_deadline(controller.envelope(1));controller.advance(1ns);
  auto moving=controller.envelope(2,.2);controller.send_deadline(moving);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);
  auto malformed=std::make_shared<std_msgs::msg::String>();malformed->data="{\"seq\":true}";
  controller.deadline_callback()(malformed);
  ASSERT_EQ(controller.tick(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  controller.send_deadline(moving);ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  controller.advance(1ns);auto newer=controller.envelope(3,.2);controller.send_deadline(newer);
  auto conflict=newer;conflict["expires_mono_ns"]=conflict["expires_mono_ns"].get<int64_t>()-1;
  controller.send_deadline(conflict);controller.send_deadline(newer);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  controller.advance(1ns);auto zero=controller.envelope(4);controller.send_deadline(zero);
  controller.send_deadline(moving);ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
}

TEST_F(ProductionRuntime, DeadlineClockSourceAndActivationGenerationStayBound) {
  MemoryController controller(false);activate_deadline(controller);
  controller.send_deadline(controller.envelope(1));controller.advance(1ns);
  auto moving=controller.envelope(2,.2);controller.send_deadline(moving);
  auto old_callback=controller.deadline_callback();auto generation=controller.deadline_generation();
  neural_controller::CommandDeadline parsed;ASSERT_TRUE(neural_controller::decode_command_deadline(moving.dump(),parsed));
  ASSERT_EQ(controller.deactivate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
  controller.advance(1ns);ASSERT_EQ(controller.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
  // Model the callback paused after parsing while lifecycle resets receiver state.
  controller.commit_parsed(parsed,generation);
  auto message=std::make_shared<std_msgs::msg::String>();message->data=moving.dump();old_callback(message);
  controller.send_deadline(moving); // Pre-activation even through current callback.
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  auto wrong=controller.envelope(3);wrong["clock_id"]="other-boot";controller.send_deadline(wrong);
  controller.send_deadline(controller.envelope(3,.2)); // Wrong-clock zero cannot bind.
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  controller.send_deadline(controller.envelope(3));controller.advance(1ns);
  auto takeover=controller.envelope(4,.2);takeover["source_instance"]=std::string(32,'b');controller.send_deadline(takeover);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  controller.send_deadline(controller.envelope(4,.2));
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_command(controller,.2f,0.f,0.f);
  auto older_future=controller.envelope(3,.1);
  older_future["issued_mono_ns"]=controller.command_deadline_now_ns()+1;
  older_future["expires_mono_ns"]=controller.command_deadline_now_ns()+150000001;
  controller.send_deadline(older_future);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_command(controller,.2f,0.f,0.f);
}

TEST_F(ProductionRuntime, DeadlineRetiresExpiredOrNonIncreasingIssueSequences) {
  MemoryController controller(false);activate_deadline(controller);
  auto initial=controller.envelope(1);initial["allowed"]=false;controller.send_deadline(initial);
  controller.advance(1ns);auto moving=controller.envelope(2,.2);controller.send_deadline(moving);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_command(controller,.2f,0.f,0.f);
  auto bad=moving;bad["seq"]=3;controller.send_deadline(bad);controller.send_deadline(moving);
  ASSERT_EQ(controller.tick(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  controller.advance(1ns);auto retired=controller.envelope(3,.2);controller.send_deadline(retired);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  auto newest=controller.envelope(4,.2);controller.send_deadline(newest);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_command(controller,.2f,0.f,0.f);
  controller.advance(1ns);auto expired=controller.envelope(5,.2);controller.advance(150ms);
  controller.send_deadline(expired);controller.send_deadline(newest);
  ASSERT_EQ(controller.tick(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
}

TEST_F(ProductionRuntime, DeadlineActualDdsSubscriptionRejectsBareTwistAuthority) {
  MemoryController controller(false);activate_deadline(controller);
  auto publisher_node=std::make_shared<rclcpp::Node>("deadline_runtime_source");
  auto input=publisher_node->create_publisher<std_msgs::msg::String>(neural_controller::kCommandDeadlineTopic,1);
  auto bare=publisher_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller.get_node()->get_node_base_interface());executor.add_node(publisher_node);
  const auto wait_until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(input->get_subscription_count()!=1 && std::chrono::steady_clock::now()<wait_until) {
    executor.spin_some();std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(input->get_subscription_count(),1u);EXPECT_EQ(bare->get_subscription_count(),0u);
  auto publish=[&](const nlohmann::json &value) {
    std_msgs::msg::String message;message.data=value.dump();input->publish(message);
    const auto until=std::chrono::steady_clock::now()+30ms;
    while(std::chrono::steady_clock::now()<until) {executor.spin_some();std::this_thread::sleep_for(1ms);}
  };
  publish(controller.envelope(1));controller.advance(1ns);auto moving=controller.envelope(2,.2);publish(moving);
  ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_command(controller,.2f,0.f,0.f);
  geometry_msgs::msg::Twist raw;raw.linear.x=.3;bare->publish(raw);executor.spin_some();
  controller.advance(150ms);ASSERT_EQ(controller.tick(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  publish(moving);ASSERT_EQ(controller.tick_actor(2.1),controller_interface::return_type::OK);expect_zero_command(controller);
  executor.remove_node(publisher_node);executor.remove_node(controller.get_node()->get_node_base_interface());
}

TEST_F(ProductionRuntime, DeadlineAuthorityReportsBindingAndRejectsOldSnapshots) {
  MemoryController controller(false);activate_deadline(controller);
  auto observer=std::make_shared<rclcpp::Node>("deadline_authority_observer");
  std::vector<nlohmann::json> statuses;
  auto subscription=observer->create_subscription<std_msgs::msg::String>(
    "/neural_controller_default/command_authority",rclcpp::QoS(10).best_effort(),
    [&](std_msgs::msg::String::SharedPtr msg){statuses.push_back(nlohmann::json::parse(msg->data));});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller.get_node()->get_node_base_interface());executor.add_node(observer);
  auto pump=[&](){const auto until=std::chrono::steady_clock::now()+30ms;
    while(std::chrono::steady_clock::now()<until){executor.spin_some();std::this_thread::sleep_for(1ms);}};
  const auto discovery_until=std::chrono::steady_clock::now()+2s;
  while(observer->count_publishers("/neural_controller_default/command_authority")!=1 &&
        std::chrono::steady_clock::now()<discovery_until) pump();
  ASSERT_EQ(observer->count_publishers("/neural_controller_default/command_authority"),1u);
  pump();
  controller.send_deadline(controller.envelope(1,.2));pump();
  ASSERT_FALSE(statuses.empty());EXPECT_EQ(statuses.back().at("bound_source_instance"),"");
  const auto activation=statuses.back().at("activation_mono_ns").get<int64_t>();
  const auto old_generation=controller.deadline_generation();
  EXPECT_GT(activation,0);EXPECT_GE(old_generation,1u);
  const auto unbound_time=statuses.back().at("sampled_mono_ns").get<int64_t>();
  auto zero=controller.envelope(1);zero["allowed"]=false;controller.send_deadline(zero);pump();
  ASSERT_EQ(statuses.back().at("bound_source_instance"),std::string(32,'a'));
  const auto bound_count=statuses.size();
  // An older unbound snapshot cannot follow a newer binding, even if a test
  // clock has equal sample times. Real snapshots preserve their original time.
  controller.delayed_authority(old_generation,activation,unbound_time,"");pump();
  EXPECT_EQ(statuses.size(),bound_count);
  controller.advance(100ms);auto foreign=controller.envelope(2);foreign["source_instance"]=std::string(32,'b');
  controller.send_deadline(foreign);pump();
  ASSERT_GT(statuses.size(),bound_count);EXPECT_EQ(statuses.back().at("bound_source_instance"),std::string(32,'a'));
  EXPECT_EQ(statuses.back().at("generation"),old_generation);
  const auto rate_count=statuses.size();controller.send_deadline(foreign);pump();
  EXPECT_EQ(statuses.size(),rate_count); // Callback traffic is bounded to10Hz without changes.
  ASSERT_EQ(controller.deactivate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
  controller.advance(1ns);ASSERT_EQ(controller.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
  pump();statuses.clear();
  controller.delayed_authority(old_generation,activation,controller.command_deadline_now_ns(),std::string(32,'a'));pump();
  EXPECT_TRUE(statuses.empty());
  controller.send_deadline(foreign);pump(); // Pre-activation origin cannot bind, but reports current state.
  ASSERT_FALSE(statuses.empty());EXPECT_EQ(statuses.back().at("bound_source_instance"),"");
  EXPECT_GT(statuses.back().at("generation").get<uint64_t>(),old_generation);
  EXPECT_GT(statuses.back().at("activation_mono_ns").get<int64_t>(),activation);
  auto replacement=controller.envelope(3);replacement["source_instance"]=std::string(32,'b');
  controller.send_deadline(replacement);pump();
  EXPECT_EQ(statuses.back().at("bound_source_instance"),std::string(32,'b'));
  executor.remove_node(observer);executor.remove_node(controller.get_node()->get_node_base_interface());
}
