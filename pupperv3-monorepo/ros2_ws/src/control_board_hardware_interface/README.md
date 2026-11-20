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

### Field hold when a primary report drops out

`field_redundant_imu` is an opt-in software candidate for the enclosed robot's BNO08x. During a partial outage an already running, compatible walker can keep receiving orientation and body rates while its native command fence requests a stationary stance. Match authority stays paused. This requires the release's corresponding walker and clip fences, the source receipt reader and a fresh operator resume. Home, controller switching into a new owner and cold startup always require healthy primaries. The stock profile keeps its original hard watchdog.

The driver requests four independent **report streams on one sensor**, all at 100 Hz. Its normal pair is Rotation Vector (0x05) and calibrated angular velocity (0x02). Its alternative pair is Game Rotation Vector (0x08) and uncalibrated angular velocity minus the separately encoded bias (0x07). A fixed yaw alignment reconciles the two orientation origins. Each of the four receipt clocks must advance to earn a new comparison. Initial qualification needs at least 500 ms of agreement. Further qualifications require gyro disagreement at most 0.15 rad/s, angular difference at most 0.060 rad, acceptable alternative accuracy (2 or 3), four-way receipt skew at most 20 ms and component age including known batch delay within the selection limits. A healthy primary can continue when no backup qualifies.

If a previously good primary ages past 100 ms during an already running match, both alternative reports must still be within 40 ms. The board clears the preceding motor command before the controller's next update, exposes hold mode and increments the incident. Return requires 120 ms of advancing agreement before the absolute 3 s deadline measured from the oldest primary receipt minus its known batch age. The entire window includes detection and requalification. An eight-bit report sequence may need additional time before a returning sample can be ordered. The board publishes restored-primary mode until a later incident or lifecycle transition. The operator can release the new game epoch when the existing controller and both teams' authorities satisfy their respective gates.

An executable-channel SH-2 reset notification invalidates both pairs and latches a fault. Old or duplicate sequences cannot refresh clocks. A jump exceeding four reports, an interruption exceeding 40 ms in either backup, low quality or incompatible values during hold, a missed or invalid SPI exchange, an implausible joint change or the hard deadline triggers neutral disable and loss of the 60 motor handles. A Stop release during hold sends neutral immediately and invalidates the incident. An entire sensor/bus outage follows the 100 ms watchdog. SPI validation checks both links and joint bounds. The firmware has no per-motor age, current or reset counter. A silent plausible coordinate reset or common-mode sensor error can escape these checks. The time in a report is a minimum known age relative to the unobserved interrupt. Exact physical sampling time and a shared-chip failure cannot be reconstructed from the host receipt. The packed robot still needs a supervised supported target test and floor qualification.

The three exported state handles are `imu_sensor/fc_balance_mode`, `imu_sensor/fc_balance_incident` and `imu_sensor/fc_balance_generation`. The positive Home generation uses monotonic microseconds and advances after a completed mechanical Home. A new Home resets its incident to zero. The terminal-fault counter persists across Home within that producer. The hardware also writes a 192-byte snapshot to the selected absolute `PUPPER_FC_BOARD_HEALTH_PATH`. It binds fresh source IDs, sequences and receipt times to the producer PID, process instance, Home and terminal fault generation. Consumers must read a stable even snapshot, verify its own-host monotonic age and bind the receipt to the release's boarded process. The ROS IMU broadcaster can repeat a cached message at the publication rate. The deployed target must map a verified board library containing `FC_FIELD_REDUNDANT_IMU_V1` and `FC_BOARD_HEALTH_V1`.

**Offline evidence, November 20.** The November 2 supported-sensor archive supplied 6,038 uncalibrated gyro and 6,031 Game Rotation Vector reports. Running the recorded payloads and receipt times through the native BNO reader and selector at a simulated 260 Hz qualified a backup on 15,528 of 15,675 observations over 60.286 s. Suppressing one stream five seconds into the capture gives the following results. All intervals below begin at the last usable primary measurement including its known batch age. The first control tick at or after the deadline accounts for sampling granularity.

| Captured reports suppressed | First hold | Native outcome |
| --- | --- | --- |
| Calibrated gyro for 1 s | 103.7 ms | Primary restored at 1.138 s |
| Rotation Vector for 2 s | 103.7 ms | Primary restored at 2.704 s |
| Rotation Vector for 2.5 s | 103.7 ms | Primary restored at 2.704 s |
| Both primary streams for 4 s | 103.7 ms | Disabled at 3.004 s |
| All four streams for 4 s | No hold | Disabled at 103.7 ms |

The loss was injected offline into a healthy, supported stationary capture. This establishes packet compatibility, selection timing and the shared-sensor limitation. No physical fall, standing performance or field reset was tested. [Original sensor session](../../../../foundation/results/workstream1-imu-sensor-session-20261102.json)

To replay the same capture with the native Jazzy testing build, run the following from the project root. The extractor verifies the archive digest. The binary accepts channel-3 payloads at their original monotonic receipt times.

```sh
driver=pupperv3-monorepo/ros2_ws/src/control_board_hardware_interface
python3 "$driver/test/extract_field_imu_trace.py" \
  artifacts/hardware/2026-11-02/imu-sensor-session/sensor-session-evidence.tar.gz /tmp/field-imu.tsv
for scenario in baseline gyro1 quat2 quat2_5 both4 total; do
  build/control_board_hardware_interface/test_bno_cache archive /tmp/field-imu.tsv "$scenario" || exit 1
done
```

### Bench damping through a short IMU outage

`supported_damping` requires a stand that supports the torso for the entire session. Supported mechanical Home runs first, then the board sends disabled packets until a deliberate cue through the physical Stop gateway. A dedicated controller uses the existing gateway owner name `gemini_walk` and claims exactly the twelve effort handles. It reads no IMU, runs no policy and writes no actuator setpoint. The board rejects the sixty-handle gait and kick claims. The first enabled packet has zero gains. Every subsequent bench packet requests only a small derivative gain, zero velocity, zero stiffness and zero feed-forward. The motor drive damps its own velocity. A frozen Pi-side encoder sample cannot manufacture a persistent feed-forward torque. The legs can still move under gravity and the torso needs support.

At the 100 ms freshness threshold the driver marks balance unknown. An ordinary expiry can be bridged only for a component the driver previously saw fresh. Fresh, distinct quaternion and gyro pairs must advance together at least three times over at least 20 ms, before the original three-second deadline. Only the telemetry becomes available again. The same bench motor mode, owner and calibrated coordinates persist. A missing or malformed report, a newly queued ancient report, a recent loop interval over 50 ms, invalid joint position or velocity, or either invalid SPI exchange ends the powered experiment. The deadline is measured from the oldest expired report including its known batch age. A new healthy component on its own cannot extend it. A terminal fault sends a neutral disable packet, revokes control and requires supported rehome. During loss a Stop release sends neutral inside the switch callback, independent of an IMU update. A healthy Stop also sends neutral and requires a fresh cue to enable damping again.

The selected joint configuration permits 3 N·m feed-forward and 30 rad/s. The candidate chooses a 0.50 N·m **design envelope** for the drive-local term and protocol rounding. It reduces the gain for lower enrolled effort or damping limits. For the pinned twelve joints the requested gain is 0.00684 N·m·s/rad. The published reference board packs this as approximately 0.00611. This is a light damper. Host and reference board code give no measurement of actual motor current. The software timer runs in the Pi control loop. A stopped process, frozen bus or unpowered board cannot guarantee physical delivery of Stop or the three-second packet. A stand-side acceptance test must check drive response and the independent means of removing power.

**A total blackout on the floor.** The walker uses gravity direction and body angular velocity together with joints. After 100 ms it can already be using observations from approximately ten missed reports. A second or more can cover entire steps. Stored joint angles alone do not identify body orientation, floor contact or heading. In the published STM32 image, the returned flags are both zero and the encoder cache changes on successful CAN reads. A checksum-valid SPI reply can repeat old CAN state. Its return message has no board generation, per-actuator generation, sample sequence or measured current. Keeping the existing calibration after losing drive power or terminally disabling the controller would require independent evidence of motor coordinate continuity. The existing `supported_rehome` mode keeps the manager and remeasures the coordinates. The packed robot's installed board image and physical response remain unverified. [Published SPIneV1 image and flashing guide](https://pupper-v3-documentation.readthedocs.io/en/latest/guide/flashing_control_board.html) · [Trace, simulation and source identities](../../../../foundation/results/fc-supported-damping-20261120.json)

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


### Mechanical Home travel timeout

After sending neutral, the board driver records the search or return phase and
each unresolved axis in the native run log. Each row identifies the joint,
reference status, commanded and measured positions, rest target, signed
measured-minus-rest error, and allowed settling error in radians. The main
failure message includes the first outstanding joint for the lobby. A joint
that has yet to find its reference reports its current encoder coordinate.
Reproduce a single axis frozen on return with the real driver and fake SPI
using `ctest --test-dir <board-build> -V -R imu_health_mechanical_return_frozen`.
