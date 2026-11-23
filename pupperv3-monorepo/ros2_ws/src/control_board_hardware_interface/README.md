# control_board_hardware_interface

This repository provides a ros2_control hardware interface for the Pupper v3 control board.

## Prerequisites
- [Pupper v3 control board](https://oshwlab.com/gabrael_3548/pupper-v3-control-power-boards)
- Raspberry Pi 4 running Ubuntu 22.04 and ROS2 Humble [(tested with this image)](https://github.com/ros-realtime/ros-realtime-rpi4-image/releases/tag/22.04.1_v5.15.39-rt42-raspi_ros2_humble)
- Recommended: configure ros2_control for realtime operation as described [here](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html#determinism)
- Create a workspace `/home/pi/ros2_ws`

## How to configure
- An example URDF is contained in `test_state_publisher.urdf.xacro`.
```
<ros2_control name="control_board_hardware_interface" type="system">
    <hardware>
        <plugin>control_board_hardware_interface/ControlBoardHardwareInterface</plugin>
        <param name="imu_mounting_deg.yaw">0</param>
        <param name="imu_mounting_deg.pitch">0</param>
        <param name="imu_mounting_deg.roll">0</param>
    </hardware>

    <joint name="joint_1">
        <param name="can_channel">1</param>
        <param name="can_id">1</param>
        <param name="can_protocol">cheetah</param>

        <param name="position_scale">95.5</param>
        <param name="velocity_scale">30.0</param>
        <param name="effort_scale">18.0</param>
        <param name="kp_scale">500.0</param>
        <param name="kd_scale">5.0</param>
        <param name="axis_direction">-1</param>
        <param name="position_offset">0.0</param>

        <param name="position_min">-1.0</param>
        <param name="position_max">1.0</param>
        <param name="velocity_max">1.0</param>
        <param name="effort_max">1.0</param>
        <param name="kp_max">1.0</param>
        <param name="kd_max">1.0</param>

        <command_interface name="position"/>
        <command_interface name="velocity"/>
        <command_interface name="effort"/>
        <command_interface name="kp"/>
        <command_interface name="kd"/>

        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
    </joint>

    <sensor name="imu_sensor">
        <state_interface name="orientation.x"/>
        <state_interface name="orientation.y"/>
        <state_interface name="orientation.z"/>
        <state_interface name="orientation.w"/>
        <state_interface name="angular_velocity.x"/>
        <state_interface name="angular_velocity.y"/>
        <state_interface name="angular_velocity.z"/>
        <state_interface name="linear_acceleration.x"/>
        <state_interface name="linear_acceleration.y"/>
        <state_interface name="linear_acceleration.z"/>
    </sensor>
</ros2_control>
```
### Explanation of parameters
- `imu_mounting_deg.*`: Mounting angles for the IMU relative the the IMU link, measured in degrees
- `can_channel`: Which CANbus channel the actuator is connected to
- `can_id`: CANbus ID of the actuator (must be 1 or greater)

## Ownership and FC board health receipt (team addition)
- One process owns the boards through an `flock` on `/tmp/control_board_hardware.lock`. The lock file is never unlinked. An instance that fails to lock sends nothing and closes only its own descriptor. On destruction the owner sends the neutral packet, stops the IMU thread, closes I2C/SPI and only then releases the lock.
- `rt_spi` sets bits 0/1 of `spi_driver_status` when board group 0/1 returned a complete, nonempty, checksum-valid reply in the latest exchange (`SPI_BOTH_REPLIES_VALID`). Only then are that group's q/qd/flags in `spi_data_t` replaced; a missing, short or corrupt reply leaves the group's last valid values. This is transport evidence only; the firmware has no motor sample counter, and a checksum-valid reply can still carry NaN/Inf.
- Joint feedback is accepted only if the mask is 3 and every configured joint's position and velocity is finite (NaN, +Inf and -Inf are rejected). It is checked before being copied into joint state, so rejected feedback never reaches the state interfaces; they keep the last accepted values.
  - Homed (from Home completion, including the first `read()` after Home): the first rejected exchange calls the existing fault path: neutral packet, receipt status 4, one `Board fault phase=<post_home|read> reason=<missing or corrupt motor board reply|nonfinite motor feedback> sampled_mono_us=...` line, and `read()` returns `ERROR`. There is no numeric tolerance and no automatic recovery: a valid transport afterwards does not restore health or actuation; a new activation and Home are required.
  - Not homed (before activation, after deactivation or a fault): rejected feedback is counted (`unhomed_rejected_reads`, a WARN at most once per second) and `read()` returns `OK`.
- Whenever no Home of the current activation is in effect, `write()` prepares nothing and `read()` sends the all-zero neutral packet, whatever the command interfaces or an earlier session left in the buffer. When such a `read()` accepts feedback, it reports zero joint effort, since the packet carried no targets or gains. Invalid values left in the command interfaces therefore cannot fault it. This does not depend on ros2_control lifecycle behavior.
- Commands are admitted before any transmission: by a homed `write()`, and before every Home exchange. Admission reads each joint's position, velocity, effort, kp and kd command once and rejects any NaN, +Inf or -Inf before any clamp. No magnitude threshold is involved, so an infinite value never becomes a limit-clamped command. The packet is then staged and committed only if every final value fits the float wire type, checked after limits and zero offset. A finite position beyond float range with kp = 0 is therefore rejected rather than encoded as Inf. `rt_spi` encodes it by sign/offset only, so finite stays finite. On rejection the existing fault path runs: neutral packet, receipt status 4, one `Command fault phase=<write|homing> reason=<nonfinite actuator command|nonfinite encoded actuator command> sampled_mono_us=...` line, whose IMU and board fields are followed by `commands={<joint>={position=.. velocity=.. effort=.. kp=.. kd=.. zero_offset=..}}`,, and `write()` returns `ERROR` or Home stops. There is no automatic recovery; a new activation and Home are required.
- Activation probes both boards with the neutral packet for up to 100 ms before any enable command (`Home refused before motor enable: missing motor board reply (mask=N)`). Home applies the same joint-feedback check to every exchange, including the last enable exchange, before copying it; a rejection stops Home with neutral (`phase=homing`). Home has no travel/time bound (unchanged); its joint limits and gains are unchanged.
- On the first accepted `read()` after each Home the driver logs one INFO line, report only: `FC_HOME_RESIDUAL_V1 generation=G frame=joint_after_home_offset units=rad sampled_mono_us=T joints=N <joint>={commanded=.. rest=.. measured=.. measured_minus_commanded=.. measured_minus_rest=.. commanded_kp=..} ... assessment=none`. Commanded is the target in the packet that read exchanged, rest is the configured `post_homing_position`, measured is the accepted encoder position, all in the joint frame after this Home's zero offset. It has no threshold and does not mean the pose or robot is ready. A rejected first read logs a fault instead.
- If `PUPPER_FC_BOARD_HEALTH_PATH` is an absolute path, the driver publishes `FC_BOARD_HEALTH_V1`: a read-only 24×uint64 seqlock file (`board_health.hpp`, profile 1 strict). Status 0 = unknown, 5 = homing, 1 = healthy, 4 = terminal. Healthy requires this activation's completed Home, an enabled IMU whose sample passed `feedback_error`, and valid replies from both boards in the same `read()`. A homed read with rejected feedback faults (above). The Home generation changes only after a completed Home. Deactivation, errors and faults latch status 4 and increment the fault count. Word 21 (SPI mask) of a terminal row is the mask of the neutral exchange that followed the fault; the faulting mask is `fault_spi_mask` in the ERROR line.
- Fault lines have the form `<kind> fault phase=... reason=... sampled_mono_us=...` and include IMU component receipts/ages and board counters/generation. A rejected-feedback fault also lists the kept joint state and any nonfinite raw values. Focused fake-device tests: `test_board_health`, `test_spi_reply_status`, `test_driver_neutral_deactivate`. No test in this package has a wall-clock timeout (`TIMEOUT 0`).
