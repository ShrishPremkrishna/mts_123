cmake_minimum_required(VERSION 3.5)
project(joy_utils)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# Add the executable
add_executable(estop_controller src/estop_controller.cpp)

# Link against ROS 2 libraries
ament_target_dependencies(estop_controller
  rclcpp
  rcl_interfaces
  sensor_msgs
  std_msgs
  controller_manager_msgs
)

# Install the executable
install(TARGETS estop_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(estop_parameter_contract_test test/test_estop_parameter_contract.cpp)
  ament_target_dependencies(estop_parameter_contract_test
    rclcpp rcl_interfaces sensor_msgs std_msgs controller_manager_msgs)
  add_test(NAME estop_parameters_default COMMAND estop_parameter_contract_test)
  add_test(NAME estop_parameters_overrides COMMAND estop_parameter_contract_test overrides)
  set_tests_properties(estop_parameters_default estop_parameters_overrides PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=164;ROS_LOCALHOST_ONLY=1"
    TIMEOUT 20)
endif()

# Export package
ament_package()
