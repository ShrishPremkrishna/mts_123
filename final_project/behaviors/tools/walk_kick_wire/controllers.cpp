// Memory fixture manager: a labeled stand-in for ros2_control_node where the
// controller manager package is unavailable (the Mac Jazzy container).
// The production NeuralController code runs from the selected library with the
// generated release YAML and policies. Lifecycle transitions, interface loans,
// the update schedule and the IMU broadcast are fixture-driven here, not the
// real manager. There is no hardware component, driver or device access.
//
// Timing: every event carries CLOCK_MONOTONIC nanoseconds read in this process
// (the clock named by the native command_clock_id). The switch callback brackets
// each in-process on_activate/on_deactivate call, so here those are exact entry
// times. The real manager exposes no such hook; there only bounds are observable.
#include "../../../../pupper_gait_deploy/test/test_neural_controller_safety.cpp"
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>

namespace {
using controller_manager_msgs::srv::ListControllers;
using controller_manager_msgs::srv::SwitchController;
using Clock = std::chrono::steady_clock;
constexpr char kNeural[] = "neural_controller/NeuralController";
constexpr char kImu[] = "imu_sensor_broadcaster";

class FixtureController : public MemoryController {
 public:
  FixtureController() : MemoryController(false) {}
  std::chrono::steady_clock::time_point cmd_vel_now() const override { return Clock::now(); }
  int64_t command_deadline_now_ns() const override { return neural_controller::command_monotonic_ns(); }
  controller_interface::return_type step(double since_activation, double period) {
    return update(rclcpp::Time(static_cast<int64_t>(std::llround(since_activation * 1e9)), RCL_ROS_TIME),
                  rclcpp::Duration::from_seconds(period));
  }
  std::vector<std::string> configured_joints() const {
    return get_node()->get_parameter("joint_names").as_string_array();
  }
};

struct Slot {
  std::unique_ptr<FixtureController> controller;
  bool active = false;
  bool first_update = false;  // the next update is the first after an activation
  double period = 0.;
  Clock::time_point activated, due;
  std::shared_ptr<void> parameter_probe;
};

std::int64_t mono() { return neural_controller::command_monotonic_ns(); }

std::string required_env(const char *name) {
  const char *value = std::getenv(name);
  if (!value || !*value) throw std::runtime_error(std::string("missing ") + name);
  return value;
}

void report(const std::string &json) { std::cout << json << std::endl; }

std::string quoted(const std::vector<std::string> &names) {
  std::ostringstream out;
  out << "[";
  for (size_t i = 0; i < names.size(); ++i) out << (i ? "," : "") << "\"" << names[i] << "\"";
  out << "]";
  return out.str();
}

// One in-process lifecycle call: {"controller", "begin_ns", "end_ns", "ok"}.
std::string lifecycle_row(const std::string &name, std::int64_t begin, std::int64_t end, bool ok) {
  return "{\"controller\":\"" + name + "\",\"begin_ns\":" + std::to_string(begin) + ",\"end_ns\":" +
         std::to_string(end) + ",\"ok\":" + (ok ? "true" : "false") + "}";
}

std::string joined(const std::vector<std::string> &rows) {
  std::string out = "[";
  for (size_t i = 0; i < rows.size(); ++i) out += (i ? "," : "") + rows[i];
  return out + "]";
}

TEST_F(ProductionRuntime, FixtureManager) {
  ASSERT_STREQ(std::getenv("PUPPER_ISOLATED_ROS_TEST"), "1");
  const auto params = required_env("WIRE_PARAMS_FILE");
  const double manager_rate = std::stod(required_env("WIRE_MANAGER_RATE"));
  const double imu_rate = std::stod(required_env("WIRE_IMU_RATE"));
  std::vector<std::string> names;
  std::stringstream list(required_env("WIRE_CONTROLLERS"));
  for (std::string name; std::getline(list, name, ',');) names.push_back(name);
  ASSERT_FALSE(names.empty());

  std::map<std::string, Slot> slots;
  const auto options = rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", params});
  for (const auto &name : names) {
    auto &slot = slots[name];
    slot.controller = std::make_unique<FixtureController>();
    ASSERT_EQ(slot.controller->init(name, "", static_cast<unsigned>(manager_rate), "", options),
              controller_interface::return_type::OK) << name;
    // The memory interfaces carry the selected controller's own joint names.
    ASSERT_EQ(slot.controller->configured_joints(), neural_controller::position_slew_joint_names()) << name;
    const auto rate = slot.controller->get_node()->get_parameter("update_rate").as_int();
    ASSERT_GT(rate, 0);
    slot.period = 1. / static_cast<double>(rate);
    slot.controller->add_full_tracking_feedback();
    // Passive: reports when the controller node committed an activation parameter.
    slot.parameter_probe = slot.controller->get_node()->add_post_set_parameters_callback(
        [name](const std::vector<rclcpp::Parameter> &changed) {
          const auto at = mono();
          std::string fields;
          for (const auto &parameter : changed)
            if (parameter.get_name() == "start_clip" || parameter.get_name() == "execution_request_id")
              fields += (fields.empty() ? "\"" : ",\"") + parameter.get_name() + "\":\"" + parameter.value_to_string() + "\"";
          if (!fields.empty())
            report("{\"event\":\"param_set\",\"controller\":\"" + name + "\",\"mono_ns\":" + std::to_string(at) +
                   ",\"parameters\":{" + fields + "}}");
        });
  }
  auto manager = std::make_shared<rclcpp::Node>("controller_manager");
  auto imu = manager->create_publisher<sensor_msgs::msg::Imu>("/imu_sensor_broadcaster/imu", rclcpp::SystemDefaultsQoS());
  const auto &memory_imu = slots.begin()->second.controller->imu_values;  // upright, still

  auto list_service = manager->create_service<ListControllers>(
      "/controller_manager/list_controllers",
      [&](const std::shared_ptr<ListControllers::Request>, std::shared_ptr<ListControllers::Response> response) {
        for (const auto &[name, slot] : slots) {
          controller_manager_msgs::msg::ControllerState row;
          row.name = name; row.type = kNeural; row.state = slot.active ? "active" : "inactive";
          response->controller.push_back(row);
        }
        controller_manager_msgs::msg::ControllerState row;
        row.name = kImu; row.type = "imu_sensor_broadcaster/IMUSensorBroadcaster"; row.state = "active";
        response->controller.push_back(row);
      });
  // The launch remaps the manager's own service to this backend behind the Stop gateway.
  auto switch_service = manager->create_service<SwitchController>(
      "/controller_manager/switch_controller_backend",
      [&](const std::shared_ptr<SwitchController::Request> request, std::shared_ptr<SwitchController::Response> response) {
        const auto entered = mono();
        const bool strict = request->strictness == SwitchController::Request::STRICT;
        bool ok = true;
        for (const auto *group : {&request->deactivate_controllers, &request->activate_controllers})
          for (const auto &name : *group)
            if (!slots.count(name)) ok = false;
        if (!ok && strict) {
          response->ok = false;
          report("{\"event\":\"switch\",\"ok\":false,\"reason\":\"unknown controller\",\"entry_ns\":" +
                 std::to_string(entered) + ",\"response_ns\":" + std::to_string(mono()) + "}");
          return;
        }
        ok = true;
        std::vector<std::string> deactivated, activated;
        for (const auto &name : request->deactivate_controllers) {
          auto found = slots.find(name);
          if (found == slots.end() || !found->second.active) continue;
          const auto begin = mono();
          const bool done = found->second.controller->deactivate_in_memory() == controller_interface::CallbackReturn::SUCCESS;
          deactivated.push_back(lifecycle_row(name, begin, mono(), done));
          ok = done && ok;
          found->second.active = false;
        }
        for (const auto &name : request->activate_controllers) {
          auto found = slots.find(name);
          if (found == slots.end() || found->second.active) continue;
          // Every NeuralController claims all leg command interfaces.
          const bool claimed = std::any_of(slots.begin(), slots.end(), [](const auto &row) { return row.second.active; });
          if (claimed) {
            ok = false;
            continue;
          }
          const auto begin = mono();
          const bool done = found->second.controller->activate_in_memory() == controller_interface::CallbackReturn::SUCCESS;
          activated.push_back(lifecycle_row(name, begin, mono(), done));
          if (!done) {
            ok = false;
            continue;
          }
          found->second.active = found->second.first_update = true;
          found->second.activated = found->second.due = Clock::now();
        }
        response->ok = ok;
        report("{\"event\":\"switch\",\"activate\":" + quoted(request->activate_controllers) +
               ",\"deactivate\":" + quoted(request->deactivate_controllers) + ",\"strict\":" +
               (strict ? "true" : "false") + ",\"ok\":" + (ok ? "true" : "false") +
               ",\"entry_ns\":" + std::to_string(entered) + ",\"activations\":" + joined(activated) +
               ",\"deactivations\":" + joined(deactivated) + ",\"response_ns\":" + std::to_string(mono()) + "}");
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(manager);
  for (auto &[name, slot] : slots) executor.add_node(slot.controller->get_node()->get_node_base_interface());
  report("{\"event\":\"ready\",\"controllers\":" + quoted(names) + ",\"clock_id\":\"" +
         neural_controller::command_clock_id() + "\",\"mono_ns\":" + std::to_string(mono()) + "}");

  const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1. / manager_rate));
  const auto imu_period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1. / imu_rate));
  auto next = Clock::now(), imu_due = next;
  while (rclcpp::ok()) {
    executor.spin_some(std::chrono::milliseconds(0));
    const auto now = Clock::now();
    for (auto &[name, slot] : slots) {
      if (!slot.active || now < slot.due) continue;
      // A delayed cycle is skipped, not replayed in a burst.
      const auto step = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(slot.period));
      slot.due = std::max(slot.due + step, now);
      const double since = std::chrono::duration<double>(now - slot.activated).count();
      const auto begin = mono();
      if (slot.controller->step(since, slot.period) != controller_interface::return_type::OK) {
        // Like the manager, an update error removes the controller from the loop.
        slot.controller->deactivate_in_memory();
        slot.active = slot.first_update = false;
        report("{\"event\":\"update_error\",\"controller\":\"" + name + "\",\"mono_ns\":" +
               std::to_string(begin) + "}");
      } else if (slot.first_update) {
        slot.first_update = false;
        report("{\"event\":\"first_update\",\"controller\":\"" + name + "\",\"begin_ns\":" +
               std::to_string(begin) + ",\"end_ns\":" + std::to_string(mono()) + "}");
      }
    }
    if (now >= imu_due) {
      imu_due = std::max(imu_due + imu_period, now);
      sensor_msgs::msg::Imu sample;
      sample.header.stamp = manager->get_clock()->now();
      sample.header.frame_id = "base_link";
      sample.angular_velocity.x = memory_imu[0]; sample.angular_velocity.y = memory_imu[1];
      sample.angular_velocity.z = memory_imu[2];
      sample.orientation.x = memory_imu[3]; sample.orientation.y = memory_imu[4];
      sample.orientation.z = memory_imu[5]; sample.orientation.w = memory_imu[6];
      imu->publish(sample);
    }
    next = std::max(next + period, Clock::now());
    std::this_thread::sleep_until(next);
  }
  for (auto &[name, slot] : slots)
    if (slot.active) { slot.controller->deactivate_in_memory(); slot.active = false; }
  report("{\"event\":\"exit\",\"mono_ns\":" + std::to_string(mono()) + "}");
}
}  // namespace
