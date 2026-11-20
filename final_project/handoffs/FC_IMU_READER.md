# Completed IMU reports

The board reader now publishes a usable quaternion/gyro pair as soon as either report advances. Previously the next I²C read could keep a completed pair private until it exceeded the 100 ms freshness limit. In the regression, the next read blocks for 120 ms. The controller can see the completed pair during the stall and sees its original timestamps expire if no more data arrives.

From a ROS Jazzy build of `control_board_hardware_interface` with testing enabled, run `ctest --test-dir build/control_board_hardware_interface -R '^bno_cache_' --output-on-failure`. The manager-level regression is `bno_cache_manager_blocked_read`.

For the next field investigation, record individual report sequences, sample ages and joint feedback at the first fault. The cause of the historical physical timeout is still unknown. Long-dropout recovery, retaining Home and physical standing have separate acceptance requirements on the match branch.
