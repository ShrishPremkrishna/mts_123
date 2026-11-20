# Pupper FC handoff · November 20, 2026

Robot testing ended at the operator’s request. Blue’s last supported Home sent neutral before Stand. The owned shutdown recorded successful cleanup. Physical gamepad-free standing, stopping from a sustained stand, watchdog release of the physical legs and a floor match remain unverified.

| Branch | Contents |
| --- | --- |
| `codex/pupper-fc-lobby-perception` at `69d8a73` | Laptop lobby, teleop, discovery, camera settings, automatic field calibration, pursuit, match rehearsal, packaging and restored Pi eyes/touch Wi-Fi. Includes the preceding physical 12-joint Home receipt. |
| `codex/pupper-fc-match` | Browser Stop lease, native and UI diagnostics, and the exact installed diagnostic candidate `ef65bb7`. This handoff records the field qualification failure. |
| `codex/pupper-fc-stand-wip` at `6a79102` | Further UI warning correlation and a draft native replay with two archived Blue trajectories. The draft replay was not compiled. The proposed stock Home to guarded Stand transition is unimplemented. |

## What changed on Blue

The course configuration specifies a knee mechanical stop at +0.71 rad on the right and −0.71 rad on the left. The walking command bounds end 0.10 rad inside those stops. The FC feedback check used those command bounds when evaluating the first enabled exchange. A replay using Blue’s deployed geometry and healthy IMU/SPI reproduced the same fault at the prescribed Home pose. Moving only the four knee readings 0.12 rad inward allowed the exchange. The physical fault log omitted per-joint coordinates, so the exact channel that tripped on that run remains unknown.

On `0c07ad6` the laptop completed Home and activated the walker with no gamepad. The board reported invalid or discontinuous joint feedback about 1.5 ms later. Gyro and orientation receipts were 3.143 ms old. The gateway acknowledged Stop and the controller manager released the legs. This incident is separate from the earlier return timeout and from recorded IMU or field-camera losses.

On `ef65bb7` the generated Home pose instead asked all four knees to return to ±0.20 rad at the existing Home gains. During supported run `fc-pupper-1-514d12fd09b7` all 12 referenced their stops. At the 18.08 s mechanical deadline, the two remaining axes were:

| Joint | Commanded rad | Measured rad | Permitted settling error rad |
| --- | ---: | ---: | ---: |
| Rear right, leg_back_r_3 | +0.200000 | +0.710763 | 0.363636 |
| Rear left, leg_back_l_3 | −0.200000 | −0.703134 | 0.363636 |

The other ten joints passed the return check. The two final values do not reveal motor current, individual drive acknowledgment or physical obstruction. Older recordings from Blue show both rear encoders moving through the requested angles during the stock walker's two-second Stand ramp. In `fc-field-74798bf0`, rear right/left moved from +0.613/−0.608 at activation to +0.554/−0.546 around 0.15 s and −0.508/+0.514 around 2.10 s. Those runs used a different binary and commanded the bend during Stand. [Recorded trajectory fixtures at the handoff](https://github.com/ShrishPremkrishna/mts_123/tree/6a79102/pupperv3-monorepo/ros2_ws/src/control_board_hardware_interface/test)

## Software evidence and artifacts

Blue’s isolated ROS gateway, using a memory controller and the production SSH relay, requested native Stop about 0.73 s after the last responding browser heartbeat. Chrome exercised explicit Stop and a frozen browser. The final offline suite passed 195 tests and 34 subtests plus real ROS rehearsals of Blue, Red, both teams and a rolling ball. The real powered trials above ended before a sustained Stand. The last extracted laptop package reached the lobby in 5.015 s cold and reopened its running instance in 0.065 s. Its 49 source blobs, 19 pinned wheels and the Pi’s 753 source blobs were checked against release inputs.

| Diagnostic package for `ef65bb7` | SHA-256 |
| --- | --- |
| `pupper-fc-laptop.zip` | `432604742174a3de8750935f688a7baeb4515d532a4618b4200d9035a059cc93` |
| `pupper-fc-pi.tar.gz` | `9d3675eb75525650da94dc18b247e7cb4bea1e63f7659f914f4850ccebf43d88` |

The installed Pi release was `/home/pi/pupper-fc-releases/fc-gamepad-free-ef65bb7af26f`. Its normal idle-session rollback command from the matching Pi extraction is `python3 foundation/hardware/fc_walk_kick_release.py rollback`. Blue retains its saved earlier enrollment. Raw native logs, the final package verification and earlier position traces are retained on the release workstation in `outputs/pupper-fc-gamepad-free/verification/`. [Operator instructions](../hardware/FC_OPERATOR.md)
