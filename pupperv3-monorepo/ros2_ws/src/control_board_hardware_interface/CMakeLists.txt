cmake_minimum_required(VERSION 3.8)
project(control_board_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)

# add compile directive for shared library
add_library(control_board_hardware_interface SHARED
  src/control_board_hardware_interface.cpp
  src/supported_damping_lease.cpp
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
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
)

pluginlib_export_plugin_description_file(controller_interface supported_damping_lease.xml)
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
  foreach(scenario never_started implicit_destroy explicit_stop_twice restart_after_stop destroy_during_sample invalid_after_good)
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
  foreach(scenario deactivate error destroy repeated lock_contention)
    add_test(NAME driver_neutral_${scenario} COMMAND test_driver_neutral_deactivate ${scenario})
    set_tests_properties(driver_neutral_${scenario} PROPERTIES TIMEOUT 10)
  endforeach()
  add_executable(test_spi_zero_encoding test/test_spi_zero_encoding.cpp src/rt/rt_spi.cpp)
  target_include_directories(test_spi_zero_encoding PRIVATE include)
  target_link_libraries(test_spi_zero_encoding pthread)
  target_link_options(test_spi_zero_encoding PRIVATE -Wl,--wrap=open -Wl,--wrap=open64 -Wl,--wrap=ioctl -Wl,--wrap=close)
  add_test(NAME driver_neutral_board_encoding COMMAND test_spi_zero_encoding)
  set_tests_properties(driver_neutral_board_encoding PROPERTIES TIMEOUT 10)

  add_executable(test_imu_health_guard test/test_imu_health_guard.cpp
    src/control_board_hardware_interface.cpp src/imu_manager.cpp)
  target_compile_features(test_imu_health_guard PRIVATE cxx_std_17)
  target_include_directories(test_imu_health_guard PRIVATE include)
  ament_target_dependencies(test_imu_health_guard controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle tf2)
  target_link_options(test_imu_health_guard PRIVATE
    -Wl,--wrap=open -Wl,--wrap=open64 -Wl,--wrap=close -Wl,--wrap=flock -Wl,--wrap=unlink)
  foreach(scenario mechanical_missing mechanical_lost mechanical_nonfinite mechanical_no_stop
      mechanical_return_valid mechanical_return_frozen
      health init_missing read_gyro read_quat read_norm activate_bias homing_stale activate_fresh
      supported_standby supported_rehome damping_gyro damping_quat damping_both damping_stop damping_spi damping_joint
      floor_first_claim floor_first_packet floor_home_reset floor_home_bias floor_home_late_fault floor_home_trace
      floor_gyro1 floor_quat2 floor_twice floor_total floor_long floor_queued floor_disagree floor_spi floor_reset
      floor_gap floor_jump floor_stop floor_rehome
      damping_flap damping_gap damping_batch
      resource_manager resource_manager_strict strict_rehome)
    add_test(NAME imu_health_${scenario} COMMAND test_imu_health_guard ${scenario})
    set_tests_properties(imu_health_${scenario} PROPERTIES TIMEOUT 25)
  endforeach()

  # In the repository and extracted Pi bundle, generate the same field geometry as Prepare.
  find_path(FC_HARDWARE_DIR lab6_session.launch.py
    PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../../../../foundation/hardware"
          "${CMAKE_CURRENT_SOURCE_DIR}/../../../foundation/hardware" NO_DEFAULT_PATH)
  if(FC_HARDWARE_DIR)
    add_test(NAME imu_health_enrolled_field_standing COMMAND python3 -c [=[
import os, pathlib, runpy, subprocess, sys, tempfile
hardware, executable = pathlib.Path(sys.argv[1]), sys.argv[2]
os.environ["PUPPER_FC_IMU_RECOVERY"] = "field_redundant_imu"
with tempfile.TemporaryDirectory() as directory:
    select = runpy.run_path(str(hardware / "lab6_session.launch.py"))["selected_robot_description"]
    field, _ = select(hardware / "floor_stock/baseline-expanded.urdf", pathlib.Path(directory), "fc_walk_kick")
    subprocess.run([executable, "floor_enrolled_standing"], check=True,
                   env={**os.environ, "FC_TEST_FIELD_URDF": str(field)})
]=] "${FC_HARDWARE_DIR}" $<TARGET_FILE:test_imu_health_guard>)
    set_tests_properties(imu_health_enrolled_field_standing PROPERTIES TIMEOUT 30)
  endif()

  add_executable(test_bno_protocol test/test_bno_protocol.cpp)
  target_compile_features(test_bno_protocol PRIVATE cxx_std_17)
  target_include_directories(test_bno_protocol PRIVATE include)
  foreach(scenario normal two delayed_tail large three probe2 short_read bad_channel bad_sequence bad_length
      missing_continuation orphan too_short too_large empty bounded_retry atomic_truncated atomic_unknown
      signed_timing unknown_time no_base game_and_stability sequence)
    add_test(NAME bno_protocol_${scenario} COMMAND test_bno_protocol ${scenario})
    set_tests_properties(bno_protocol_${scenario} PROPERTIES TIMEOUT 5)
  endforeach()

  add_executable(test_bno_cache test/test_bno_cache.cpp src/rt/rt_bno055.cpp src/imu_manager.cpp)
  target_compile_features(test_bno_cache PRIVATE cxx_std_17)
  target_include_directories(test_bno_cache PRIVATE include)
  ament_target_dependencies(test_bno_cache rclcpp tf2)
  target_link_options(test_bno_cache PRIVATE
    -Wl,--wrap=_ZNSt6chrono3_V212steady_clock3nowEv
    -Wl,--wrap=open -Wl,--wrap=open64 -Wl,--wrap=ioctl -Wl,--wrap=read -Wl,--wrap=write -Wl,--wrap=close)
  foreach(scenario initial gyro_only valid delayed_tail duplicate gyro_refresh control game_not_selected
      bad_quaternion truncated short_read open_failure overlap_deadline following_read_blocked stale_before_fresh
      manager_blocked_read redundant_wire)
    add_test(NAME bno_cache_${scenario} COMMAND test_bno_cache ${scenario})
    set_tests_properties(bno_cache_${scenario} PROPERTIES TIMEOUT 5)
  endforeach()

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
