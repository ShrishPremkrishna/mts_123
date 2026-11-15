#!/usr/bin/env python3
"""Write a copy of the course controller config with kick-session overrides.

Shared files stay unchanged; launch the course stack with the copy:

    python3 make_course_config.py --speed 1.0 --out /tmp/kick_course_config.yaml
    python3 make_course_config.py --kick-max-body-angle 0.65 --out /tmp/learned_kick_config.yaml
    ros2 launch neural_controller launch.py teleop:=False config_file:=/tmp/kick_course_config.yaml

Check first that the installed launch has the argument:
    ros2 launch neural_controller launch.py --show-args

--speed: the X walker (neural_controller_default) caps forward speed at 0.49 m/s;
  this raises speed_cap and max_vx_forward, as follow_me_pupper/robot/speed_cap.yaml
  does. Use it only for the run-through speed ladder, never in a learned-kick session.
--kick-max-body-angle: tilt [rad] at which the square slot (neural_controller_student)
  e-stops; the course value is 1.5 rad (86 deg), training ended episodes at 35 deg.
--kick-init / --kick-fade: shorter init and fade-in [s] for the square slot (course
  values 2.0 and 2.0); pass the sum to the bridge as -p fade_in_s:=<sum>.
"""

import argparse
from pathlib import Path

import yaml

WALKER = "neural_controller_default"
KICK = "neural_controller_student"
RUN_CAP = 1.5  # the default policy's run cap and trained command clip


class _NoAliases(yaml.SafeDumper):  # ROS parameter files reject YAML aliases
  def ignore_aliases(self, data):
    return True


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--speed", type=float, help="X walker forward cap [m/s], at most 1.5")
  ap.add_argument("--kick-max-body-angle", type=float, help="square slot tilt e-stop [rad]")
  ap.add_argument("--kick-init", type=float, help="square slot init_duration [s]")
  ap.add_argument("--kick-fade", type=float, help="square slot fade_in_duration [s]")
  ap.add_argument("--config", type=Path, help="course config.yaml (default: installed neural_controller)")
  ap.add_argument("--out", type=Path, required=True)
  args = ap.parse_args()
  changes = {}
  if args.speed is not None:
    if not 0.0 < args.speed <= RUN_CAP:
      raise SystemExit(f"--speed must be in (0, {RUN_CAP}]")
    changes[WALKER] = {"speed_cap": args.speed, "max_vx_forward": args.speed}
  kick = {}
  if args.kick_max_body_angle is not None:
    if not 0.3 <= args.kick_max_body_angle <= 1.0:
      raise SystemExit("--kick-max-body-angle must be in [0.3, 1.0] rad")
    kick["max_body_angle"] = args.kick_max_body_angle
  for key, value in (("init_duration", args.kick_init), ("fade_in_duration", args.kick_fade)):
    if value is not None:
      if not 0.2 <= value <= 2.0:
        raise SystemExit(f"{key} must be in [0.2, 2.0] s")
      kick[key] = value
  if kick:
    changes[KICK] = kick
  if not changes:
    raise SystemExit("nothing to change: pass --speed and/or a --kick-* option")
  config = args.config
  if config is None:
    from ament_index_python.packages import get_package_share_directory

    config = Path(get_package_share_directory("neural_controller")) / "launch" / "config.yaml"
  data = yaml.safe_load(config.read_text())
  for controller, values in changes.items():
    params = data[controller]["ros__parameters"]
    for key, value in values.items():
      print(f"{controller}.{key}: {params.get(key)} -> {value}")
      params[key] = float(value)
  args.out.write_text(yaml.dump(data, Dumper=_NoAliases, sort_keys=False))
  print(f"wrote {args.out}")


if __name__ == "__main__":
  main()
