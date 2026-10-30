cmake_minimum_required(VERSION 3.8)
project(control_board_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)

# add compile directive for shared library
add_library(control_board_hardware_interface SHARED
  src/control_board_hardware_interface.cpp
  src/rt/rt_spi.cpp
  src/rt/rt_bno055.cpp
  src/imu_manager.cpp
)

# add include directory
target_include_directories(control_board_hardware_interface
  PRIVATE
    include
)

ament_target_dependencies(
  control_board_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
)

pluginlib_export_plugin_description_file(hardware_interface control_board_hardware_interface.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY test/
  DESTINATION test
)

if(BUILD_TESTING)
  # Link the real thread owner to a fake BNO055; no hardware implementation is
  # linked or invoked by these lifecycle tests.
  add_executable(test_imu_manager_lifecycle test/test_imu_manager_lifecycle.cpp src/imu_manager.cpp)
  target_include_directories(test_imu_manager_lifecycle PRIVATE include)
  ament_target_dependencies(test_imu_manager_lifecycle rclcpp tf2)
  enable_testing()
  foreach(scenario never_started implicit_destroy explicit_stop_twice restart_after_stop destroy_during_sample)
    add_test(NAME imu_manager_${scenario} COMMAND test_imu_manager_lifecycle ${scenario})
    set_tests_properties(imu_manager_${scenario} PROPERTIES TIMEOUT 10)
  endforeach()

  # Actual driver callbacks with fake SPI/BNO and isolated lock syscalls.
  # Neither target opens a device, enables actuators, nor runs a ROS executor.
  add_executable(test_driver_neutral_deactivate test/test_driver_neutral_deactivate.cpp
    src/control_board_hardware_interface.cpp src/imu_manager.cpp)
  target_compile_features(test_driver_neutral_deactivate PRIVATE cxx_std_17)
  target_include_directories(test_driver_neutral_deactivate PRIVATE include)
  ament_target_dependencies(test_driver_neutral_deactivate hardware_interface pluginlib rclcpp rclcpp_lifecycle tf2)
  target_link_options(test_driver_neutral_deactivate PRIVATE
    -Wl,--wrap=open -Wl,--wrap=open64 -Wl,--wrap=close -Wl,--wrap=flock -Wl,--wrap=unlink)
  foreach(scenario deactivate error destroy repeated)
    add_test(NAME driver_neutral_${scenario} COMMAND test_driver_neutral_deactivate ${scenario})
    set_tests_properties(driver_neutral_${scenario} PROPERTIES TIMEOUT 10)
  endforeach()
  add_executable(test_spi_zero_encoding test/test_spi_zero_encoding.cpp src/rt/rt_spi.cpp)
  target_include_directories(test_spi_zero_encoding PRIVATE include)
  target_link_libraries(test_spi_zero_encoding pthread)
  target_link_options(test_spi_zero_encoding PRIVATE -Wl,--wrap=open -Wl,--wrap=open64 -Wl,--wrap=ioctl)
  add_test(NAME driver_neutral_board_encoding COMMAND test_spi_zero_encoding)
  set_tests_properties(driver_neutral_board_encoding PROPERTIES TIMEOUT 10)

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  transmission_interface
)
ament_package()