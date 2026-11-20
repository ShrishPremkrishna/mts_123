"""Run the enrolled Pi's Prepare and supported Home from one operator's browser."""
import asyncio
from contextlib import suppress
import json
import uuid

from anyio import CancelScope

from .field_transport import finish_runner, remote_argv


async def open_setup(host, robot, action, home_only):
    arguments = [action, "--robot-id", robot, "--dashboard"]
    if action == "home":
        arguments.append("--supported")
        if home_only:
            arguments.append("--home-only")
    # The Pi flushes the terminal, checks its physical Stop gateway and then
    # emits stand_prompt. Allocate a remote terminal only for that standing cue.
    return await asyncio.create_subprocess_exec(
        *remote_argv(host, *arguments, tty=action == "home" and not home_only),
        stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)


def setup_request(data):
    if data == {"action": "prepare"}:
        return "prepare", False
    if (not isinstance(data, dict) or data.get("action") != "home" or data.get("supported") is not True
            or type(data.get("home_only", False)) is not bool or data.keys() - {"action", "supported", "home_only"}):
        raise ValueError("Confirm the robot is supported on the stand before Home")
    return "home", data.get("home_only", False)


async def run_setup(ws, host, robot, launch):
    action, home_only = setup_request(await ws.receive_json())
    proc = await launch(host, robot, action, home_only)
    events = asyncio.Queue()
    cue = None
    sent_stand = abandoned = False

    async def monitor():
        nonlocal cue
        receipt, diagnostic = None, ""
        while line := await proc.stdout.readline():
            text = line.decode(errors="replace").strip()
            try:
                event = json.loads(text)
            except ValueError:
                if (text and not text.startswith("Connection to ")
                        and not (diagnostic and "returned non-zero exit status" in text)):
                    diagnostic = text[-350:]
                continue
            if not isinstance(event, dict):
                continue
            if event.get("event") == "stand_prompt" and action == "home" and not home_only:
                if abandoned:
                    proc.stdin.write(b"cancel\n")
                    await proc.stdin.drain()
                else:
                    cue = uuid.uuid4().hex
                    events.put_nowait(dict(stage="stand_prompt", cue=cue, message="Homed. Keep Pupper supported to enable standing."))
            if (event.get("stage") == action and event.get("robot_id") == robot
                    and event.get("match_armed") is False and event.get("home_only") is home_only
                    and isinstance(event.get("run"), str) and event["run"]):
                receipt = event
        code = await proc.wait()
        if code or receipt is None:
            raise ValueError(diagnostic or "The Pi closed setup without confirming this team's action")
        message = ("Session prepared." if action == "prepare" else
                   "Homed on stand." if home_only else "Walker active. Ready to connect for play.")
        events.put_nowait(dict(stage="complete", run=receipt["run"], message=message))

    async def send_events():
        await ws.send_json(dict(event="setup", stage="running", robot_id=robot, action=action,
                                message="Preparing session…" if action == "prepare" else "Homing on stand…"))
        while True:
            event = await events.get()
            await ws.send_json(dict(event, event="setup", action=action, robot_id=robot, home_only=home_only))
            if event["stage"] == "complete":
                return

    async def read_operator():
        nonlocal cue, sent_stand
        while True:
            message = await ws.receive_json()
            if not cue or message != {"action": "stand", "cue": cue}:
                raise ValueError("Wait for the Pi's current standing cue")
            cue, sent_stand = None, True
            proc.stdin.write(b"\n")
            await proc.stdin.drain()

    pi = asyncio.create_task(monitor())
    sender = asyncio.create_task(send_events())
    operator = asyncio.create_task(read_operator())
    try:
        done, _ = await asyncio.wait((pi, sender, operator), return_when=asyncio.FIRST_COMPLETED)
        for task in done:
            task.result()
        await sender  # deliver the receipt before releasing the connection
    finally:
        # A lost browser cannot submit Start. Let a supported Home already in
        # progress write its normal Pi readiness receipt, then cancel the cue.
        with CancelScope(shield=True):
            abandoned = True
            operator.cancel()
            sender.cancel()
            await asyncio.gather(operator, sender, return_exceptions=True)
            if not pi.done() and (cue or sent_stand):
                with suppress(BrokenPipeError, ConnectionResetError):
                    proc.stdin.write(b"cancel\n" if cue else b"\x03")
                    await proc.stdin.drain()
            await asyncio.gather(pi, return_exceptions=True)
            if proc.returncode is not None:
                proc.stdin.close()
            else:
                # A failed output reader (for example an overlong remote line)
                # may have left a live SSH child and a full pipe. Reap both.
                async def discard_output():
                    while await proc.stdout.read(65536):
                        pass
                drain = asyncio.create_task(discard_output())
                try:
                    await finish_runner(proc)
                finally:
                    drain.cancel()
                    await asyncio.gather(drain, return_exceptions=True)
