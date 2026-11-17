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
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "srv/StartMotion.srv")
rosidl_get_typesupport_target(start_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

# Add the executable
add_executable(estop_controller src/estop_controller.cpp)
target_include_directories(estop_controller PRIVATE include)
target_link_libraries(estop_controller "${start_typesupport}")

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
  target_include_directories(estop_parameter_contract_test PRIVATE include)
  target_link_libraries(estop_parameter_contract_test "${start_typesupport}")
  ament_target_dependencies(estop_parameter_contract_test
    rclcpp rcl_interfaces sensor_msgs std_msgs controller_manager_msgs)
  add_executable(stop_arbiter_test test/test_stop_arbiter.cpp)
  target_include_directories(stop_arbiter_test PRIVATE include)
  add_test(NAME stop_arbiter_deterministic COMMAND stop_arbiter_test)
  add_executable(stop_gateway_dds_test test/test_stop_gateway_dds.cpp)
  target_include_directories(stop_gateway_dds_test PRIVATE include)
  target_link_libraries(stop_gateway_dds_test "${start_typesupport}")
  ament_target_dependencies(stop_gateway_dds_test
    rclcpp rcl_interfaces sensor_msgs std_msgs controller_manager_msgs)
  foreach(scenario delayed timeout shutdown service_loss negative broadcaster_type)
    add_test(NAME stop_gateway_${scenario} COMMAND stop_gateway_dds_test ${scenario})
    set_tests_properties(stop_gateway_${scenario} PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=166;ROS_LOCALHOST_ONLY=1;LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}" TIMEOUT 15 RUN_SERIAL TRUE)
  endforeach()
  add_executable(inventory_barrier_test test/test_inventory_barrier.cpp)
  target_include_directories(inventory_barrier_test PRIVATE include)
  target_link_libraries(inventory_barrier_test "${start_typesupport}")
  ament_target_dependencies(inventory_barrier_test
    rclcpp rcl_interfaces sensor_msgs std_msgs controller_manager_msgs)
  add_test(NAME stop_inventory_freshness COMMAND inventory_barrier_test)
  set_tests_properties(stop_inventory_freshness PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=166;ROS_LOCALHOST_ONLY=1;LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}" TIMEOUT 10 RUN_SERIAL TRUE)
  add_executable(start_motion_test test/test_start_motion.cpp)
  target_include_directories(start_motion_test PRIVATE include)
  target_link_libraries(start_motion_test "${start_typesupport}")
  ament_target_dependencies(start_motion_test
    rclcpp rcl_interfaces sensor_msgs std_msgs controller_manager_msgs)
  add_test(NAME start_motion_transaction COMMAND start_motion_test)
  add_test(NAME start_motion_unknown_owner COMMAND start_motion_test unknown)
  set_tests_properties(start_motion_transaction start_motion_unknown_owner PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=166;ROS_LOCALHOST_ONLY=1;LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}" TIMEOUT 20 RUN_SERIAL TRUE)
  add_test(NAME estop_parameters_default COMMAND estop_parameter_contract_test)
  add_test(NAME estop_parameters_overrides COMMAND estop_parameter_contract_test overrides)
  set_tests_properties(estop_parameters_default estop_parameters_overrides PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=164;ROS_LOCALHOST_ONLY=1;LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}"
    TIMEOUT 20)
endif()

install(DIRECTORY include/ DESTINATION include)

# Export package
ament_export_dependencies(rosidl_default_runtime)
ament_package()
