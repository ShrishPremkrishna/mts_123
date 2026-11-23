"""Carry one Pi Stop gateway's challenges to and from the owning browser.

The Pi's ``fc_session.py operator --owner`` process owns the watchdog. This relay
never renews authority itself: it forwards only a browser echo of a challenge the
Pi issued recently, at most once per action. Stand needs an explicit
``supported: true`` click on the displayed challenge. Browser Stop always passes.
A standing receipt reports controller state; it does not observe physical standing.
"""
import asyncio
import json
import time

from anyio import CancelScope

from .field_transport import finish_runner, remote_argv

# The selected gateway rejects an echo issued more than 0.30 s earlier. Relaying
# an older one could only be refused, so the laptop applies the same window.
CHALLENGE_WINDOW_S = .30
# Match play needs a recent browser echo. This is a liveness check on the
# relay, not a replacement for the Pi watchdog.
ECHO_LIVENESS_S = .75
# The selected fc_operator stream. Each event names its robot; Pi failures arrive as
# diagnostic text before EOF, never as an event, so no other row is relayed.
EVENTS = ("operator_challenge", "operator_status", "operator_standing", "operator_stop")


async def open_operator(host, robot, owner):
    return await asyncio.create_subprocess_exec(
        *remote_argv(host, "operator", "--robot-id", robot, "--owner", owner.replace("-", "")),
        stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)


def browser_input(row):
    """Return the validated browser action. Anything else ends the relay."""
    if row == {"action": "stop"}:
        return "stop"
    if isinstance(row, dict):
        challenge = row.get("challenge")
        if isinstance(challenge, str) and 0 < len(challenge) <= 256:
            if row == {"action": "heartbeat", "challenge": challenge}:
                return "heartbeat"
            if row == {"action": "stand", "challenge": challenge, "supported": True} and row["supported"] is True:
                return "stand"
    raise ValueError("Invalid operator input")


class OperatorLink:
    def __init__(self, owner, robot, *, clock=time.monotonic):
        self.owner, self.robot, self.clock = owner, robot, clock
        self.ready = self.standing = self.stopping = False
        self.last_echo = float("-inf")
        self.result = None

    def current(self):
        """True only while the Pi reported standing and the browser is still echoing."""
        return self.standing and not self.stopping and self.clock() - self.last_echo < ECHO_LIVENESS_S

    def summary(self):
        return dict(ready=self.ready, standing=self.standing, stopping=self.stopping,
                    current=self.current(), result=self.result)

    async def run(self, ws, proc):
        offers = {}

        async def from_pi():
            diagnostic = "The Pi operator connection closed."
            while line := await proc.stdout.readline():
                try:
                    row = json.loads(line)
                except (UnicodeError, ValueError):
                    diagnostic = line.decode(errors="replace").strip()[-300:] or diagnostic
                    continue
                if not isinstance(row, dict) or row.get("event") not in EVENTS:
                    continue
                event = row["event"]
                if row.get("robot_id") != self.robot:
                    raise ValueError("The operator connection did not identify this team"
                                     if "robot_id" not in row else "The operator connection belongs to a different team")
                if event == "operator_challenge":
                    challenge = row.get("challenge")
                    if not isinstance(challenge, str) or not 0 < len(challenge) <= 256:
                        raise ValueError("Invalid Pi operator challenge")
                    if self.stopping:
                        continue
                    now = self.clock()
                    for key, (issued, _) in list(offers.items()):
                        if now - issued >= CHALLENGE_WINDOW_S:
                            del offers[key]
                    offers.setdefault(challenge, (now, set()))
                elif event == "operator_status":
                    if type(row.get("ready")) is not bool or type(row.get("standing")) is not bool:
                        raise ValueError("Invalid Pi operator status")
                    self.ready = row["ready"] and not self.stopping
                    if not row["standing"]:
                        self.standing = False
                elif event == "operator_standing":
                    self.standing = not self.stopping
                else:
                    self.stopping, self.ready, self.standing = True, False, False
                    if type(row.get("confirmed")) is not bool or not isinstance(row.get("reason"), str):
                        raise ValueError("The Pi reported Stop without a valid acknowledgement")
                    self.result = row
                await ws.send_json(row)
                if event == "operator_stop":
                    return
            raise ValueError(diagnostic)

        async def from_browser():
            while True:
                row = await ws.receive_json()
                action = browser_input(row)
                if action == "stop":
                    self.stopping, self.ready = True, False
                    offers.clear()
                else:
                    if self.stopping:
                        continue
                    offer = offers.get(row["challenge"])
                    if offer is None:
                        continue  # never substitute a newer challenge for the one displayed
                    offered, used = offer
                    if action in used or self.clock() - offered >= CHALLENGE_WINDOW_S:
                        continue
                    used.add(action)
                    self.last_echo = max(offered, self.last_echo)
                proc.stdin.write((json.dumps(row, separators=(",", ":")) + "\n").encode())
                await asyncio.wait_for(proc.stdin.drain(), .5)

        tasks = [asyncio.create_task(fn()) for fn in (from_pi, from_browser)]
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        finally:
            self.stopping, self.ready, self.standing = True, False, False
            with CancelScope(shield=True):
                # EOF tells the Pi relay to request Stop if standing was ever requested.
                proc.stdin.close()
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)

                async def discard_output():
                    while await proc.stdout.read(65536):
                        pass
                drain = asyncio.create_task(discard_output())
                try:
                    await finish_runner(proc)
                finally:
                    drain.cancel()
                    await asyncio.gather(drain, return_exceptions=True)
