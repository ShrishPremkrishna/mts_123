# Pupper FC portable release

The laptop and Pi archives are independently installable. The desktop app runs on the Mac. Enroll each robot once on its own Pi. The bundles include the application and selected walker and kick policy sources. They contain no stored credentials, host enrollment, virtual environment, ROS installation, Hailo device firmware or target binaries.

## Laptop

1. Expand `pupper-fc-laptop.zip` into a writable local folder. Keep the extracted `Pupper FC` folder together.
2. On an Apple Silicon Mac running macOS 15 or later with CPython 3.12 installed, double-click **Pupper FC.command**. The ZIP includes all 19 runtime wheels, pinned by SHA-256. The first launch creates `.venv-fc` and installs directly from the ZIP’s extracted wheels. Later launches reuse that environment. The full source checkout also supports installation from the package index.
3. Connect the Blue or Red team. Connect the field camera. You can choose the simulated preview in Camera settings to inspect the field without a robot. A live match needs an enrolled Pi, trusted SSH key, active homed walker and live calibrated camera.

For terminal use, run `./pupper-fc --no-open` and open the local URL it prints. Host overrides and the port are listed by `./pupper-fc --help`. The full-project developer tests and `pupper-fc-check` stay in the source checkout.

## Pi

Extract to a permanent, writable directory on each prepared robot. Keep the previous installation and native prefix for rollback. The previously used Pi already has the deadline controller, physical Stop gateway and Hailo runtime. This release requires the updated control-board driver even in strict mode. With the native session shut down, build **only the board package** once on that Pi. From the extracted `pupper-fc-pi` directory, in Bash:

```sh
source /home/pi/pupper-fc-releases/fc-platform-3bd1bc9f54db/environment.sh
(cd native && colcon build --base-paths src --packages-select control_board_hardware_interface --executor sequential --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release)
python3 foundation/hardware/fc_walk_kick_release.py enroll --robot-id pupper-1 --imu-prefix "$PWD/native/install/control_board_hardware_interface"
```

The first command names the recorded Pi's FC underlay. On a different Pi substitute its installed FC environment and use `pupper-2` for Red. If this exact driver is already installed, enroll with its existing prefix and skip the build. Enrollment checks the driver's per-report IMU contract, saves its SHA-256 and verifies the local controller, Stop gateway and detector selections. Strict mode is the default. Opt-in supported recovery requires `--imu-recovery supported_rehome` and its supervised qualification. The new overlay must stay at its enrolled path. Enrollment leaves the motors inactive.

For the daily three actions, Stop and recovery, see `foundation/hardware/FC_OPERATOR.md` in the tarball. To revert after supporting the robot and shutting down the owning session, from the extracted directory:

```sh
python3 foundation/hardware/fc_walk_kick_release.py rollback
```

On a new Pi, install the matching Pupper ROS/Hailo underlay and the approved detector HEF first. The archive includes source for the FC neural controller and gateway. If those two components also need building, add `neural_controller joy_utils` to the colcon selection and source the resulting `native/install/setup.bash` on top of the underlay. Pass the combined environment script to enrollment with `--platform-env FILE`.

To rehearse application staging only, choose a new target-local directory and use the same interpreter with PyYAML and the field runtime installed:

```sh
python3 foundation/hardware/fc_walk_kick_release.py stage --source . --destination /tmp/pupper-fc-staged-check
python3 foundation/hardware/fc_walk_kick_release.py verify --release /tmp/pupper-fc-staged-check
```

## Rebuild from the source checkout

Run `./pupper-fc-package`. Artifacts and `SHA256SUMS` appear in `outputs/pupper-fc-release`; `--output DIRECTORY` changes the destination. The builder acquires the locked wheels with pip and retains verified copies in the ignored `outputs/pupper-fc-wheels` cache. To seed a build from an existing wheelhouse, set `PIP_NO_INDEX=1` and `PIP_FIND_LINKS=/absolute/path/to/wheels` for the command. A clean Git commit is recorded inside both archives. With the same commit and Python compression toolchain, repeat builds produce identical archives. Compare the delivered digests with `shasum -a 256 -c SHA256SUMS` on macOS or `sha256sum -c SHA256SUMS` on Linux before extraction.
