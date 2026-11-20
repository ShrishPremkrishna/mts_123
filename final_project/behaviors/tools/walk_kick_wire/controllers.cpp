#include "../../../../pupper_gait_deploy/test/test_neural_controller_safety.cpp"
#include <filesystem>
namespace {
class WireController : public MemoryController {
 public:
  WireController() : MemoryController(false) {}
  std::chrono::steady_clock::time_point cmd_vel_now() const override { return std::chrono::steady_clock::now(); }
  int64_t command_deadline_now_ns() const override { return neural_controller::command_monotonic_ns(); }
  float applied_forward() const {
    const auto offset = (params_.observation_history > 1 ? single_observation_size_ : 0) + obs_layout_.command;
    return observation_.at(offset);
  }
};
TEST_F(ProductionRuntime, SelectedWalkKickWire) {
  ASSERT_STREQ(std::getenv("PUPPER_ISOLATED_ROS_TEST"), "1");
  const std::filesystem::path dir("/evidence");
  WireController walk, kick;
  const auto options = rclcpp::NodeOptions().arguments({"--ros-args","--params-file","/evidence/persistent_controllers.yaml"});
  ASSERT_EQ(walk.init("gemini_walk","",520,"",options),controller_interface::return_type::OK);
  ASSERT_EQ(kick.init("trick_leg_kick","",520,"",options),controller_interface::return_type::OK);
  ASSERT_TRUE(walk.get_node()->get_parameter("cmd_vel_original_deadline_supported").as_bool());
  ASSERT_TRUE(kick.get_node()->get_parameter("policy_execution_status_active").as_bool());
  walk.add_full_tracking_feedback();
  kick.add_full_tracking_feedback();
  ASSERT_TRUE(kick.get_node()->set_parameter(rclcpp::Parameter("start_clip","kick_right")).successful);
  const std::string job(32,'b');
  ASSERT_TRUE(kick.get_node()->set_parameter(rclcpp::Parameter("execution_request_id",job)).successful);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(walk.get_node()->get_node_base_interface());
  executor.add_node(kick.get_node()->get_node_base_interface());
  ASSERT_EQ(walk.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(walk.strict_subscriptions());
  const auto began = std::chrono::steady_clock::now();
  auto switched = began;
  int stage = 0;
  bool first = false, second = false, clip = false;
  while (std::chrono::steady_clock::now()-began < std::chrono::seconds(25)) {
    executor.spin_some(std::chrono::milliseconds(2));
    if (stage == 0 && first && std::filesystem::exists(dir/"play")) {
      ASSERT_EQ(walk.deactivate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
      ASSERT_EQ(kick.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
      stage = 1; switched=std::chrono::steady_clock::now();
    } else if (stage == 1 && clip && std::filesystem::exists(dir/"restore")) {
      ASSERT_EQ(kick.deactivate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
      ASSERT_EQ(walk.activate_in_memory(),controller_interface::CallbackReturn::SUCCESS);
      ASSERT_TRUE(walk.strict_subscriptions());
      stage=2; switched=std::chrono::steady_clock::now();
    }
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-switched).count();
    if(stage==1) {
      ASSERT_EQ(kick.tick(elapsed),controller_interface::return_type::OK);
      const auto frame=kick.clip_receipt_buffer();
      if (!clip && frame.find("kick_right playing frame ")==0 && frame.find("execution "+job)!=std::string::npos) {
        std::ofstream(dir/"clip-playing")<<frame; clip=true;
      }
    } else {
      ASSERT_EQ(walk.tick(4.2+elapsed),controller_interface::return_type::OK);
      bool &seen=stage==0?first:second;
      if (!seen && walk.applied_forward()>.03f) {
        seen=true;
        std::ofstream(dir/(stage==0?"walk-before":"walk-after"))<<walk.applied_forward()<<" "<<walk.deadline_generation()<<std::endl;
      }
    }
    if (std::filesystem::exists(dir/"done")) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_TRUE(first);EXPECT_TRUE(clip);EXPECT_TRUE(second);
}
} // namespace
