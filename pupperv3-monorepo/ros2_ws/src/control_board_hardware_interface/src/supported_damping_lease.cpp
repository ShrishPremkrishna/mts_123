// The normal physical Stop gateway switches this controller. It never runs a
// policy or writes a motor command. The board owns every bounded bench packet.
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace control_board_hardware_interface {
class SupportedDampingLease : public controller_interface::ControllerInterface {
 public:
  controller_interface::CallbackReturn on_init() override {
    auto_declare<std::vector<std::string>>("joint_names", {});
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    joints_ = get_node()->get_parameter("joint_names").as_string_array();
    return joints_.size() == 12 ? controller_interface::CallbackReturn::SUCCESS :
                                 controller_interface::CallbackReturn::ERROR;
  }
  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration result{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    for (const auto &joint : joints_) result.names.push_back(joint + "/effort");
    return result;
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override {
    return controller_interface::return_type::OK;
  }
 private:
  std::vector<std::string> joints_;
};
}
PLUGINLIB_EXPORT_CLASS(control_board_hardware_interface::SupportedDampingLease, controller_interface::ControllerInterface)
