"""Publish an enrolled team's SSH service on the Pi's existing Avahi service."""
import argparse
import json
import os
from pathlib import Path
import tempfile

SERVICE = Path("/etc/avahi/services/pupper-fc.service")
PROFILE = Path("/home/pi/.config/pupper-fc/session.json")


def service_xml(robot):
    if robot not in ("pupper-1", "pupper-2"):
        raise ValueError("Select the enrolled Blue or Red robot")
    return f"""<?xml version="1.0" standalone="no"?>
<!DOCTYPE service-group SYSTEM "avahi-service.dtd">
<service-group>
  <name replace-wildcards="yes">Pupper FC {robot} on %h</name>
  <service>
    <type>_pupper-fc._tcp</type>
    <port>22</port>
    <txt-record>protocol=1</txt-record>
    <txt-record>robot_id={robot}</txt-record>
  </service>
</service-group>
"""


def install(robot, profile=PROFILE, target=SERVICE):
    if json.loads(profile.read_text()).get("robot_id") != robot:
        raise ValueError("Team does not match the enrolled session profile")
    xml = service_xml(robot)
    if target.exists() and target.read_text() == xml:
        return
    with tempfile.NamedTemporaryFile(mode="w", dir=target.parent, delete=False) as fp:
        tmp = Path(fp.name)
        fp.write(xml)
    try:
        tmp.chmod(0o644)
        os.replace(tmp, target)
    finally:
        tmp.unlink(missing_ok=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", required=True, choices=("pupper-1", "pupper-2"))
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--install", action="store_true", help="Install the advertisement in the Pi's Avahi directory")
    action.add_argument("--remove", action="store_true", help="Remove this team's matching FC advertisement")
    args = parser.parse_args()
    if args.install:
        try:
            install(args.robot_id)
        except (OSError, ValueError) as exc:
            parser.exit(1, str(exc) + "\n")
    elif args.remove:
        if SERVICE.exists() and SERVICE.read_text() != service_xml(args.robot_id):
            parser.exit(1, "The installed advertisement belongs to a different configuration\n")
        SERVICE.unlink(missing_ok=True)
    else:
        print(service_xml(args.robot_id), end="")

