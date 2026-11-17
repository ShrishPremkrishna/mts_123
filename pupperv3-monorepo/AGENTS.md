# pupperv3-monorepo

This directory is vendored into the team repository from [Nate711's Pupper V3 repository](https://github.com/Nate711/pupperv3-monorepo). Local edits therefore diverge from upstream. Shared principles are in the [root AGENTS.md](../AGENTS.md).

The ROS 2 robot platform lives in `ros2_ws/src`. The README documents the platform build and the robot image steps. `../pupper_gait_deploy/install.py` can copy controller source and policies into a selected platform workspace. Its default destination is `/home/pi/pupperv3-monorepo`.

The team's offline ROS runner is `python3 foundation/scripts/check_ros_offline.py` from the team root. It builds selected controllers with in-memory and fake-I2C interfaces inside the prepared Docker image.

For the team's neural controller, `../pupper_gait_deploy/src/` is the maintained source; its installer overlays this vendor package. Inspect the install mappings before editing a vendor copy. The cached Ubuntu ROS image verifies software but does not produce compatible deployment binaries for the currently recorded Debian Pi. The [shared robot build handoff](../final_project/ROBOT_BUILD_AND_TEST_HANDOFF.md) records the target and release-selection workflow.

The upstream [README](README.md) documents platform installation and simulation. Robot image and launch operations can replace software or initialize hardware; the team's [hardware procedure](../foundation/HARDWARE_CHECKLIST.md) describes the prepared runtime.
