"""SSH control channels for a prepared Pi. Camera datagrams use the actual SSH peer addresses."""
import asyncio
from contextlib import suppress
import ipaddress
import json
import re
import shlex
import time

from .match_director import number

SESSION = "/home/pi/pupper-fc/bin/fc_session.py"
PIPE = asyncio.subprocess.PIPE


def ssh_host(host):
    if not isinstance(host, str) or not re.fullmatch(r"(?:[A-Za-z0-9][A-Za-z0-9_.-]{0,31}@)?[A-Za-z0-9][A-Za-z0-9_.-]{0,252}", host):
        raise ValueError("Enter a configured SSH host name or pi@robot.local")
    return host


def remote_argv(host, *arguments, tty=False):
    return ["ssh", "-tt" if tty else "-T", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
            "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
            "--", ssh_host(host), shlex.join(["python3", SESSION, *arguments])]


async def finish_runner(proc):
    if proc.stdin:
        proc.stdin.close()
    for grace, signal in ((3, proc.terminate), (2, proc.kill)):
        try:
            await asyncio.wait_for(proc.wait(), grace)
            return
        except asyncio.TimeoutError:
            with suppress(ProcessLookupError):
                signal()
    await proc.wait()


def validate_profile(reply, robot):
    if not isinstance(reply, dict) or reply.get("robot_id") != robot:
        raise ValueError("The selected Pi has a different team identity")
    for key in ("source_host", "robot_host"):
        ip = ipaddress.IPv4Address(reply.get(key))
        if ip.is_multicast or ip.is_unspecified or ip.is_reserved:
            raise ValueError("The Pi must report the two IPv4 endpoints of its SSH connection")
    if type(reply.get("port")) is not int or not 1 <= reply["port"] <= 65535:
        raise ValueError("The Pi profile has an invalid camera port")
    cfg = reply.get("field_config", {})
    field = cfg.get("field") if isinstance(cfg, dict) else None
    if not isinstance(field, dict) or not all(number(field.get(k)) for k in ("length", "width", "ball_radius", "goal_width")):
        raise ValueError("The Pi has no measured field dimensions")
    return reply


async def describe(host, robot):
    proc = await asyncio.create_subprocess_exec(*remote_argv(host, "describe", "--robot-id", robot),
                                               stdout=PIPE, stderr=PIPE, stdin=asyncio.subprocess.DEVNULL)
    try:
        out, err = await asyncio.wait_for(proc.communicate(), 8)
        if proc.returncode:
            detail = err.decode(errors="replace").strip()[-350:]
            if SESSION in detail and "No such file" in detail:
                raise ValueError("This Pi has no Pupper FC session helper. Connect an enrolled Pi with the matching FC runtime.")
            raise ValueError(detail or "Pi profile is unavailable")
        return validate_profile(json.loads(out), robot)
    finally:
        if proc.returncode is None:
            with suppress(ProcessLookupError):
                proc.kill()
            await proc.wait()


async def require_session(host, robot):
    """Read the enrolled team and field protocol before opening a remote command."""
    profile = await describe(host, robot)
    if type(profile.get("match_pitch_version")) is not int or profile["match_pitch_version"] != 1:
        raise ValueError("This Pi's field protocol is incompatible. Connect a Pi enrolled for Pupper FC match pitch version 1.")


class FieldFeed:
    def __init__(self, proc, robot, epoch, session):
        self.proc, self.identity = proc, (robot, epoch, session)
        self.changed = asyncio.Event()
        self.opened = False
        self.paired_at = 0.
        self.problem = None
        self.status = {}
        self.reader = asyncio.create_task(self._read())

    async def _read(self):
        try:
            while line := await self.proc.stdout.readline():
                try:
                    event = json.loads(line)
                except (ValueError, UnicodeError):
                    continue
                if not isinstance(event, dict):
                    continue
                if event.get("event") == "source_open":
                    identity = (event.get("robot_id"), event.get("epoch"), event.get("source", {}).get("source_session"))
                    if identity != self.identity or event.get("mode") != "publish":
                        raise ValueError("Field source reported a different robot, camera or match")
                    self.opened = True
                if self.opened and event.get("mode") == "publish" and "frames" in event:
                    advanced = type(event["frames"]) is int and event["frames"] > self.status.get("frames", 0)
                    self.status = event
                    latest = event.get("latest") or {}
                    usable = latest.get("ready_to_arm") is True or latest.get("paired") is True
                    self.paired_at = time.monotonic() if advanced and usable else 0.
                self.changed.set()
            raise ValueError(f"Field source on {self.identity[0]} closed")
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            self.problem = str(exc)
            self.changed.set()

    async def wait_ready(self, timeout=8):
        try:
            async with asyncio.timeout(timeout):
                while not self.problem and time.monotonic() - self.paired_at > 1.3:
                    self.changed.clear()
                    await self.changed.wait()
        except TimeoutError:
            latest = self.status.get("latest") or {}
            raise ValueError(latest.get("reason") or self.status.get("reason") or "Waiting for paired camera measurements") from None
        if self.problem:
            raise ValueError(self.problem)

    async def close(self):
        try:
            await finish_runner(self.proc)
        finally:
            self.reader.cancel()
            await asyncio.gather(self.reader, return_exceptions=True)


async def open_field(host, robot, info, epoch, session, revision, tracks):
    options = ["field", "--epoch", epoch, "--source-session", session,
               "--source-host", info["source_host"], "--revision", str(revision),
               "--tracks", json.dumps(tracks, separators=(",", ":")), "--robot-id", robot,
               "--duration", "3600"]
    if info.get("pitch") is not None:
        options.extend(["--pitch", json.dumps(info["pitch"], separators=(",", ":"))])
    args = remote_argv(host, *options)
    proc = await asyncio.create_subprocess_exec(*args, stdin=PIPE, stdout=PIPE, stderr=asyncio.subprocess.DEVNULL)
    return FieldFeed(proc, robot, epoch, session)
