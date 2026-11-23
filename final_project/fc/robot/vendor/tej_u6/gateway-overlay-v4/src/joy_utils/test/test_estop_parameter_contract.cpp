// Exercise the actual node constructor/parameter interface, without spinning,
// publishing Joy, calling controller services, or loading any hardware plugin.
#define main joy_utils_production_main_not_executed
#include "../src/estop_controller.cpp"
#undef main
#include <iostream>
#include <stdexcept>

static void require(bool value, const std::string &message) {
  if (!value) throw std::runtime_error(message);
}

int main(int argc, char **argv) {
  const bool overrides = argc > 1 && std::string(argv[1]) == "overrides";
  const char *override_args[] = {"estop_parameter_contract_test", "--ros-args",
    "-p", "estop_index:=11", "-p", "estop_release_index:=8",
    "-p", "switch_button_indices:=[0,1,2,3]",
    "-p", "controller_names:=[neural_controller_default,neural_controller_three_legged,neural_controller,neural_controller_student,neural_controller_default_alt,gemini_walk,trick_sit_stay,walk_spider]",
    "-p", "stop_arbitration_protocol:=spoof", "-p", "stop_arbitration_instance_id:=spoof"};
  const char *default_args[] = {"estop_parameter_contract_test"};
  rclcpp::init(overrides ? 14 : 1, overrides ? override_args : default_args);
  int result = 0;
  try {
    auto node = std::make_shared<EStopController>();
    require(node->get_parameter("stop_arbitration_protocol").as_string() == "pupper_stop_v1", "protocol override must be ignored");
    const auto instance = node->get_parameter("stop_arbitration_instance_id").as_string();
    require(std::regex_match(instance, std::regex("[0-9a-f]{32}")), "fresh instance marker");
    for (const auto *name : {"stop_arbitration_protocol", "stop_arbitration_instance_id"}) {
      require(node->describe_parameter(name).read_only, "authority marker must be read-only");
      require(!node->set_parameter(rclcpp::Parameter(name, "spoof")).successful, "authority marker mutation refused");
    }
    const std::vector<std::string> expected_names = overrides
      ? std::vector<std::string>{"neural_controller_default", "neural_controller_three_legged",
          "neural_controller", "neural_controller_student", "neural_controller_default_alt",
          "gemini_walk", "trick_sit_stay", "walk_spider"}
      : std::vector<std::string>{"neural_controller", "neural_controller_three_legged"};
    const std::vector<int64_t> expected_buttons = overrides
      ? std::vector<int64_t>{0, 1, 2, 3} : std::vector<int64_t>{0, 1};
    const std::vector<rclcpp::Parameter> expected{
      rclcpp::Parameter("estop_index", overrides ? 11 : 12),
      rclcpp::Parameter("estop_release_index", overrides ? 8 : 9),
      rclcpp::Parameter("switch_button_indices", expected_buttons),
      rclcpp::Parameter("controller_names", expected_names)};
    const std::vector<rclcpp::Parameter> changes{
      rclcpp::Parameter("estop_index", 10), rclcpp::Parameter("estop_release_index", 7),
      rclcpp::Parameter("switch_button_indices", std::vector<int64_t>{3, 2, 1, 0}),
      rclcpp::Parameter("controller_names", std::vector<std::string>{"old_uncovered_actor"})};
    for (size_t i = 0; i < expected.size(); ++i) {
      const auto &parameter = expected[i];
      const auto descriptor = node->describe_parameter(parameter.get_name());
      require(descriptor.read_only, parameter.get_name() + " must be read-only");
      require(descriptor.type == static_cast<uint8_t>(parameter.get_type()), "descriptor type");
      require(node->get_parameter(parameter.get_name()) == parameter, "startup override/default mismatch");
      const auto individual = node->set_parameter(changes[i]);
      require(!individual.successful, "SetParameters must reject cached mapping mutation");
      require(node->get_parameter(parameter.get_name()) == parameter, "rejected mutation changed value");
    }
    const auto atomic = node->set_parameters_atomically(changes);
    require(!atomic.successful, "SetParametersAtomically must reject cached mapping mutation");
    for (const auto &parameter : expected)
      require(node->get_parameter(parameter.get_name()) == parameter, "atomic rejection changed value");
    std::cout << (overrides ? "startup overrides" : "source defaults")
              << ": four read-only descriptors and individual/atomic mutation rejection passed; no spin or motion\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
