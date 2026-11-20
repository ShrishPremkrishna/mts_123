# Pupper FC portable release

The laptop and Pi archives are independently installable. The desktop app runs on the Mac. Enroll each robot once on its own Pi. The bundles include the application and selected walker and kick policy sources. They contain no stored credentials, host enrollment, virtual environment, ROS installation, Hailo device firmware or target binaries.

## Laptop

1. Expand `pupper-fc-laptop.zip` into a writable local folder. Keep the extracted `Pupper FC` folder together.
2. On an Apple Silicon Mac running macOS 15 or later with CPython 3.12 installed, double-click **Pupper FC.command**. The ZIP includes all 19 runtime wheels, pinned by SHA-256. The first launch creates `.venv-fc` and installs directly from the ZIP’s extracted wheels. Later launches reuse that environment. The full source checkout also supports installation from the package index.
3. Click **Connect**. A cold Pi prepares itself and opens **Home**. Put the robot on its stand and confirm support. Confirm **Enable standing** when the Pi reports ready. The full Home flow connects the runner. Use **Connect** directly for an already standing robot. Connect the field camera. You can choose the simulated preview in Camera settings to inspect the field without a robot. A live match needs an enrolled Pi, trusted SSH key, active homed walker and live calibrated camera.

For terminal use, run `./pupper-fc --no-open` and open the local URL it prints. Host overrides and the port are listed by `./pupper-fc --help`. Run **Camera settings → Run offline check** or `./pupper-fc --self-test` for a repeatable check of the installed camera pipeline and kickoff on synthetic frames. The source checkout adds `./pupper-fc-check` for quick regressions and `./pupper-fc-check --ros` for the quick suite followed by all three match rosters in ROS Jazzy. The ROS rehearsal requires Docker and the local `ros:jazzy-ros-base` image.

## Pi

Extract the Pi archive into its own permanent directory on each prepared robot. Keep the previous extraction and platform. The Pi must already have the FC walking controller, Stop gateway, Hailo runtime and approved model, and trusted SSH from the laptop. With the owning native session shut down, run one installer as the robot operator from the new extraction.

```sh
./pupper-fc-install blue
```

Use `red` for the second robot. With multiple installed platform environments, pass the intended `environment.sh` as the second argument. The installer builds only the changed control-board package, enrolls this release, publishes its team over Avahi, then uses sudo for the next-boot appliance setting. It leaves the robot powered as it is. At the next boot the Pi comes up headless with networking and FC discovery. Motors first activate at supported Home.

Check the boot selection or restore the previous boot target from the same extracted directory. Restoration takes effect at the next normal reboot.

```sh
python3 foundation/hardware/fc_appliance.py status
sudo python3 foundation/hardware/fc_appliance.py normal
```

The installer chooses strict balance handling for floor play. The separate supported damping profile is a bench experiment. Keep the robot externally supported for its entire duration. Enrollment and motor behavior are described in `foundation/hardware/FC_OPERATOR.md`.

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
