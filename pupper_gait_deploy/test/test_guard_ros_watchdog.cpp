// Isolated ROS chain test. Reuses the production controller's existing memory
// interfaces and policy fixture; never run on a robot or a networked ROS graph.
// Run only ProductionRuntime.GuardDisappearanceExpiresRealRosReceipt, with the
// guarded mux source managed by the bounded offline orchestration.
#include "test_neural_controller_safety.cpp"
#include <cstdlib>
#include <filesystem>

namespace {
class SteadyMemoryController : public MemoryController {
 public:
  SteadyMemoryController() : MemoryController(false) {}
  std::chrono::steady_clock::time_point cmd_vel_now() const override {
    return std::chrono::steady_clock::now();
  }
};

TEST_F(ProductionRuntime, GuardDisappearanceExpiresRealRosReceipt) {
  ASSERT_STREQ(std::getenv("PUPPER_ISOLATED_ROS_TEST"), "1");
  const char *directory = std::getenv("PUPPER_NATIVE_GUARD_EVIDENCE");
  ASSERT_NE(directory, nullptr);
  const std::filesystem::path evidence(directory);
  ASSERT_TRUE(std::filesystem::is_directory(evidence));
  SteadyMemoryController controller;
  activate_gait(controller);
  ASSERT_TRUE(controller.has_cmd_vel_subscription());
  ASSERT_DOUBLE_EQ(controller.get_node()->get_parameter("cmd_vel_expiry_timeout_s").as_double(), .25);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller.get_node()->get_node_base_interface());
  const auto nanos = [](auto time) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
  };
  const auto began = std::chrono::steady_clock::now();
  nlohmann::json trace = nlohmann::json::array();
  bool seen_nonzero = false, final_nonzero = false, killed = false, saw_zero = false;
  int64_t last_receipt_ns = 0, first_zero_ns = 0;
  int64_t last_live_update_ns = 0;
  while (std::chrono::steady_clock::now() - began < std::chrono::seconds(12)) {
    executor.spin_some();
    const auto sample = controller.received_cmd_vel();
    if (sample) last_receipt_ns = nanos(sample->receipt);
    killed = killed || std::filesystem::exists(evidence / "guard-killed");
    const auto before = std::chrono::steady_clock::now();
    // ROS time is held fixed after initialization. Expiry must still follow
    // actual steady receipt age, and must interrupt a held inference cycle.
    ASSERT_EQ(controller.tick(2.1), controller_interface::return_type::OK);
    const auto after = std::chrono::steady_clock::now();
    const auto command = controller.observed_command();
    const bool nonzero = std::abs(command[0] - .2f) < 1e-6f;
    const bool zero = command[0] == 0.f && command[1] == 0.f && command[2] == 0.f;
    const auto update_ns = nanos(before);
    trace.push_back({{"before_update_ns", update_ns}, {"after_update_ns", nanos(after)},
                     {"last_callback_receipt_ns", last_receipt_ns}, {"command", command},
                     {"guard_killed", killed}, {"repeat_counter", controller.repeat_counter()}});
    if (nonzero) {
      last_live_update_ns = update_ns;
      if (!seen_nonzero) {
        seen_nonzero = true;
        std::ofstream(evidence / "native-nonzero-ready") << update_ns << '\n';
      }
      if (!final_nonzero && sample && sample->twist.linear.x == .2 &&
          std::filesystem::exists(evidence / "await-final-native-nonzero")) {
        final_nonzero = true;
        std::ofstream(evidence / "native-final-nonzero-ready") << update_ns << '\n';
      }
    }
    if (killed && final_nonzero && zero && !saw_zero) {
      saw_zero = true;
      first_zero_ns = nanos(after);
    }
    if (saw_zero && nanos(after) - first_zero_ns >= 50000000) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  executor.remove_node(controller.get_node()->get_node_base_interface());
  nlohmann::json result = {
      {"scope", "Real mux/guard DDS into production controller subscription/update; memory joints and synthetic gait policy"},
      {"clock", "steady-clock callback receipt; ROS update time held fixed at 2.1 seconds"},
      {"configured_timeout_ns", 250000000}, {"last_callback_receipt_ns", last_receipt_ns},
      {"last_live_update_ns", last_live_update_ns}, {"first_zero_update_ns", first_zero_ns},
      {"first_zero_receipt_age_ns", first_zero_ns ? first_zero_ns - last_receipt_ns : 0},
      {"seen_nonzero", seen_nonzero}, {"final_native_nonzero_confirmed", final_nonzero},
      {"guard_killed", killed}, {"saw_zero", saw_zero},
      {"memory_joint_kp", controller.values[0][3]}, {"memory_joint_kd", controller.values[0][4]},
      {"trace", trace}, {"physical_stop_proven", false}};
  std::ofstream(evidence / "native-watchdog.json") << result.dump(2) << '\n';
  EXPECT_TRUE(seen_nonzero);
  EXPECT_TRUE(final_nonzero);
  EXPECT_TRUE(killed);
  EXPECT_TRUE(saw_zero);
  EXPECT_GT(last_receipt_ns, 0);
  // The production predicate is age < timeout. A host scheduler may delay the
  // next update; the trace, rather than a hard wall-time promise, records that.
  EXPECT_GE(first_zero_ns - last_receipt_ns, 250000000);
  EXPECT_LT(last_live_update_ns - last_receipt_ns, 250000000);
  EXPECT_GT(controller.values[0][3], 0.);  // normal control, not deactivation
  EXPECT_GT(controller.values[0][4], 0.);
}
}  // namespace
