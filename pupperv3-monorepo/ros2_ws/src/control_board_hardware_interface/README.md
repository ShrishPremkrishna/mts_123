# control_board_hardware_interface

This repository provides a ros2_control hardware interface for the Pupper v3 control board.

## Pupper FC balance-feedback recovery candidates

The default FC profile uses the original hard fault behavior. The FC release preparer reads
`PUPPER_FC_IMU_RECOVERY` and records the selected mode and robot-description SHA-256 in its
runtime manifest. `strict` keeps the pinned description. Each supported profile produces a separate derived description.
Preparing a release or running the offline tests does not connect to hardware.

- `supported_standby` is for a robot physically held on its stand for the entire session.
  Its first activation performs the usual supported mechanical home. It then sends disabled,
  zero-torque packets, rejects every leg controller and keeps the same process through an IMU
  outage. Balance becomes unknown immediately and becomes available after three continuous
  seconds of distinct, fresh, stationary reports. Only telemetry resumes.
- `supported_rehome` is a candidate for a field session. An IMU fault during gait or kick
  sends a disabled packet before logging and returns a hardware error. The running game must
  stop. A fresh sensor report alone cannot send any motor command. With the robot supported,
  a deliberate hardware configure/activate transition on the same manager discards the old
  offsets, checks stationary balance and remeasures all twelve hard stops. The driver then
  sends neutral until a fresh, complete controller switch. Its first enabled packet has zero
  targets, gains and feed-forward torques. Starting a new match is a separate operator action.
  Stop must remain effective and the prior match and kick ownership must be cleared before the
  supported activation. If the manager or board cannot be recovered, use the ordinary cold home.

### Bench damping through a short IMU outage

`supported_damping` requires a stand that supports the torso for the entire session. Supported mechanical Home runs first, then the board sends disabled packets until a deliberate cue through the physical Stop gateway. A dedicated controller uses the existing gateway owner name `gemini_walk` and claims exactly the twelve effort handles. It reads no IMU, runs no policy and writes no actuator setpoint. The board rejects the sixty-handle gait and kick claims. The first enabled packet has zero gains. Every subsequent bench packet requests only a small derivative gain, zero velocity, zero stiffness and zero feed-forward. The motor drive damps its own velocity. A frozen Pi-side encoder sample cannot manufacture a persistent feed-forward torque. The legs can still move under gravity and the torso needs support.

At the 100 ms freshness threshold the driver marks balance unknown. An ordinary expiry can be bridged only for a component the driver previously saw fresh. Fresh, distinct quaternion and gyro pairs must advance together at least three times over at least 20 ms, before the original three-second deadline. Only the telemetry becomes available again. The same bench motor mode, owner and calibrated coordinates persist. A missing or malformed report, a newly queued ancient report, a recent loop interval over 50 ms, invalid joint position or velocity, or either invalid SPI exchange ends the powered experiment. The deadline is measured from the oldest expired report including its known batch age. A new healthy component on its own cannot extend it. A terminal fault sends a neutral disable packet, revokes control and requires supported rehome. During loss a Stop release sends neutral inside the switch callback, independent of an IMU update. A healthy Stop also sends neutral and requires a fresh cue to enable damping again.

The selected joint configuration permits 3 N·m feed-forward and 30 rad/s. The candidate chooses a 0.50 N·m **design envelope** for the drive-local term and protocol rounding. It reduces the gain for lower enrolled effort or damping limits. For the pinned twelve joints the requested gain is 0.00684 N·m·s/rad. The published reference board packs this as approximately 0.00611. This is a light damper. Host and reference board code give no measurement of actual motor current. The software timer runs in the Pi control loop. A stopped process, frozen bus or unpowered board cannot guarantee physical delivery of Stop or the three-second packet. A stand-side acceptance test must check drive response and the independent means of removing power.

**Why the floor still needs a different recovery.** The walker uses gravity direction and body angular velocity together with joints. After 100 ms it can already be using observations from approximately ten missed reports. A second or more can cover entire steps. Stored joint angles alone do not identify body orientation, floor contact or heading. In the published STM32 image, the returned flags are both zero and the encoder cache changes on successful CAN reads. A checksum-valid SPI reply can repeat old CAN state. Its return message has no board generation, per-actuator generation, sample sequence or measured current. A floor mode that kept calibrated coordinates across faults would need independent evidence of continuity plus a fresh full controller activation, fresh inputs and a new match/drive cue. The existing `supported_rehome` mode keeps the manager and remeasures the coordinates. The packed robot's installed board image and physical response remain unverified. [Published SPIneV1 image and flashing guide](https://pupper-v3-documentation.readthedocs.io/en/latest/guide/flashing_control_board.html) · [Trace, simulation and source identities](../../../../foundation/results/fc-supported-damping-20261120.json)

The native reader publishes a newly usable quaternion/gyro pair before its next I²C probe.
Each component retains its own sequence, host receipt time and known batch age. A real gap
still trips the 100 ms watchdog. The compiled reader identifies this behavior with
`FC_IMU_PUBLISH_PER_REPORT_V1` in its setup banner.

The current SPI firmware supplies no board boot counter or encoder generation. An unchanged
joint reading and a live Pi cannot establish that the motor coordinate survived a reset.
Field use still needs supported board-reset and interrupted-controller tests, followed by
supervised floor qualification. The candidates default off. A recovery-enabled release must pin the rebuilt target driver by its
own SHA-256 and verify it is the file mapped by the manager. Its enrollment banner contains
`FC_IMU_RECOVERY_CONTRACT_V1_SUPPORTED_REHOME` and the selected profile.

From the project root, rehearse the actual launch construction without ROS nodes or motors:

```sh
python -m pytest -q foundation/hardware/test_lab6_session_profile.py -k imu_profile
```

With the native ROS package built with `BUILD_TESTING=ON`, exercise the production reader,
memory-device callbacks and installed ROS ResourceManager against the same driver:

```sh
ctest --test-dir build/control_board_hardware_interface --no-tests=error -j3 --output-on-failure -R '^(imu_health_|imu_manager_|bno_cache_|bno_protocol_|driver_neutral_)'
```

For an independent run on the development Mac, start with the project root and its cached
ROS Jazzy Docker image. This builds from source and runs without a network or devices.

```sh
docker run --rm --network none \
  -v "$PWD/pupperv3-monorepo/ros2_ws/src/control_board_hardware_interface:/driver:ro" \
  cs123-ros-jazzy-slew-check bash -ec '
    source /opt/ros/jazzy/setup.bash
    cmake -S /driver -B /tmp/build -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
    cmake --build /tmp/build --parallel 3
    ctest --test-dir /tmp/build --no-tests=error -j3 --output-on-failure -R "^(imu_health_|imu_manager_|bno_cache_|bno_protocol_|driver_neutral_)"'
```

The thread test uses the real IMU manager and reader. Only the device syscalls are
intercepted. The dropout tests use the real driver with memory-backed SPI and a simulated
100 Hz sensor. They freeze individual receipt times and sequence numbers for one or two
seconds and both components for three seconds. The simulated bus captures full motor
packets. The [November 20 native results](../../../../foundation/results/fc-native-reliability-20261120.json)
include the observed trip, requalification and rehome times.

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
