"""Find team advertisements on the connected local network."""
import queue
import re
import time

SERVICE = "_pupper-fc._tcp.local."
ROBOTS = ("pupper-1", "pupper-2")
LOCAL_HOST = re.compile(r"([a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?)\.local\.?", re.I)


def teams(records):
    found = {robot: set() for robot in ROBOTS}
    for record in records:
        name = LOCAL_HOST.fullmatch(record.server or "")
        if not name or record.port != 22 or record.properties.get(b"protocol") != b"1":
            continue
        robot = record.properties.get(b"robot_id", b"")
        if not isinstance(robot, bytes):
            continue
        robot = robot.decode("ascii", "replace")
        if robot in found:
            found[robot].add("pi@" + name.group(1).lower() + ".local")
    return {robot: sorted(hosts) for robot, hosts in found.items()}


def discover(duration=1.3):
    from zeroconf import ServiceBrowser, ServiceStateChange, Zeroconf
    until = time.monotonic() + duration
    changes = queue.SimpleQueue()

    def changed(zeroconf, service_type, name, state_change):
        changes.put((name, state_change))

    records = {}
    with Zeroconf() as zc:
        browser = ServiceBrowser(zc, SERVICE, handlers=[changed])
        try:
            while (remaining := until - time.monotonic()) > 0:
                try:
                    name, change = changes.get(timeout=remaining)
                except queue.Empty:
                    break
                if change == ServiceStateChange.Removed:
                    records.pop(name, None)
                else:
                    info = zc.get_service_info(SERVICE, name, timeout=max(1, int(1000*(until-time.monotonic()))))
                    if info:
                        records[name] = info
        finally:
            browser.cancel()
    return teams(records.values())

