"""Carry one native Stop gateway's challenges to and from the owning browser."""
import asyncio
import json
import time

from anyio import CancelScope

from .field_transport import finish_runner, remote_argv


async def open_operator(host, robot, owner):
    return await asyncio.create_subprocess_exec(
        *remote_argv(host, "operator", "--robot-id", robot, "--owner", owner.replace("-", "")),
        stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)


class OperatorLink:
    def __init__(self, owner, robot):
        self.owner, self.robot = owner, robot
        self.standing = self.stopping = False
        self.last_echo = float("-inf")

    def current(self):
        return self.standing and not self.stopping and time.monotonic() - self.last_echo < .75

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
                if not isinstance(row, dict):
                    continue
                event = row.get("event")
                if event not in ("operator_challenge", "operator_status", "operator_standing", "operator_stop", "error"):
                    continue
                if row.get("robot_id", self.robot) != self.robot:
                    raise ValueError("The operator connection belongs to a different team")
                if event == "operator_challenge":
                    challenge = row.get("challenge")
                    if not isinstance(challenge, str) or not 0 < len(challenge) <= 256:
                        raise ValueError("Invalid Pi operator challenge")
                    if self.stopping:
                        continue
                    now = time.monotonic()
                    for key, (issued, _) in list(offers.items()):
                        if now - issued >= .75:
                            del offers[key]
                    offers.setdefault(challenge, (now, set()))
                elif event == "operator_standing":
                    self.standing = not self.stopping
                elif event == "operator_status":
                    if row.get("standing") is False:
                        self.standing = False
                elif event in ("operator_stop", "error"):
                    self.stopping = True
                await ws.send_json(row)
                if event in ("operator_stop", "error"):
                    # Earlier Pi builds printed the actual Stand exception to stderr
                    # immediately after a generic Stop receipt. Keep both on screen.
                    if row.get("reason") == "Standing or operator connection interrupted":
                        try:
                            async with asyncio.timeout(1):
                                while tail := await proc.stdout.readline():
                                    detail = tail.decode(errors="replace").strip()
                                    if detail.startswith("Pupper FC: "):
                                        await ws.send_json(dict(event="operator_diagnostic", robot_id=self.robot,
                                                                reason=detail.removeprefix("Pupper FC: ")[:500]))
                                        break
                        except TimeoutError:
                            pass
                    return
            raise ValueError(diagnostic)

        async def from_browser():
            while True:
                row = await ws.receive_json()
                if row == {"action": "stop"}:
                    self.stopping = True
                    offers.clear()
                else:
                    if self.stopping:
                        continue
                    if not isinstance(row, dict):
                        raise ValueError("Invalid operator input")
                    challenge = row.get("challenge")
                    if not isinstance(challenge, str) or not (
                            row == {"action": "heartbeat", "challenge": challenge} or
                            (row == {"action": "stand", "challenge": challenge, "supported": True}
                             and row["supported"] is True)):
                        raise ValueError("Invalid operator input")
                    offer = offers.get(challenge)
                    if offer is None:
                        continue
                    offered, used = offer
                    if row["action"] in used or time.monotonic() - offered >= .75:
                        continue
                    # The gateway separately verifies its own issuance time. Never
                    # substitute a newer challenge, even if SSH/JS was suspended.
                    used.add(row["action"])
                    self.last_echo = max(offered, self.last_echo)
                proc.stdin.write((json.dumps(row, separators=(",", ":")) + "\n").encode())
                await asyncio.wait_for(proc.stdin.drain(), .5)

        tasks = [asyncio.create_task(fn()) for fn in (from_pi, from_browser)]
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        finally:
            self.stopping = True
            with CancelScope(shield=True):
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
