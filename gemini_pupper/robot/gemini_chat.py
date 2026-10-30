#!/usr/bin/env python3
"""Talk to Gemini in text, and watch every tool call it makes.

Gemini Live in about a hundred lines: no browser, no microphone. You type, Gemini
answers and calls tools, and this script runs each call and sends the reply
back, the same loop the web app runs when you talk to Pupper.

    python3 lab/chat.py            # YOUR tools, on a fake robot (a laptop is enough)
    python3 lab/chat.py --realtime # fake robot with real queue timing/cancellation
    python3 lab/chat.py --robot    # your tools on the real Pupper (it moves!)

(default)  Your lab/prompt.md, and your tools from lab/, exactly as the web app
           gets them, run on the fake robot from robot/test_tools.py. Time is fake, so
           a queued job finishes at once: you see the reply Gemini hears
           (accepted or refused), then what the job did (the outcome) and what
           the fake robot was told. Nothing moves; no Pupper needed.
--realtime Fake robot only, using real elapsed time. Calls return acceptance
           immediately; queued work continues while Gemini answers. Stop can
           cancel pending/running work. This checks queue timing, not physics,
           ROS transport latency or real-controller completion.
--robot    The same, but every call goes to the robot server
           (http://localhost:8000, on the Pupper or through your ssh tunnel),
           the exact path a voice command takes. Pupper on the ground, your hand
           near the e-stop.

The Gemini key comes from GEMINI_API_KEY, or ~/.config/gemini-pupper/api_key
(robot/setup.sh writes it). Needs the google-genai package:
python3 -m pip install --user --break-system-packages google-genai

Based on Peng Xu's (Google DeepMind) Gemini Live robot-tool example. Thank you, Peng!
"""

from __future__ import annotations

import argparse
import asyncio
from contextlib import suppress
import json
import os
import re
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
KEY_FILE = Path.home() / ".config" / "gemini-pupper" / "api_key"
MODEL = "gemini-3.1-flash-live-preview"  # the web app's default (constants.ts)


# ============================================================ your tools, on the fake robot
def prompt_text(path: Path = HERE.parent / "lab" / "prompt.md") -> str:
    """prompt.md without its <!-- comments -->, as the robot server sends it."""
    return re.sub(r"<!--.*?-->", "", path.read_text(), flags=re.S).strip()


class _RealtimeClock:
    """The existing fake rig's clock interface, paced by actual elapsed time."""

    def __init__(self):
        self.on_advance = []

    @property
    def now(self):
        return time.monotonic()

    def monotonic(self):
        return self.now

    async def sleep(self, seconds, result=None):
        await asyncio.sleep(seconds)
        return result


class DeskBackend:
    """Your lab/ tools and behaviors on the fake robot from test_tools.py."""

    name = "your tools on a fake robot (accelerated per-call completion; nothing moves)"

    def __init__(self, *, realtime: bool = False):
        self.realtime = realtime
        self._ticker = None
        self._closed = False
        if realtime:
            self.name = "your tools on a fake robot (real queue timing; no physics or hardware)"

    def setup(self) -> tuple[str, list[dict]]:
        import test_tools  # installs a fake ROS before robot_api is imported
        import tricks
        import tool_registry

        self.tt, self.registry = test_tools, tool_registry
        self._previous_bindings = {name: getattr(test_tools.robot_api, name)
                                   for name in ("time", "asyncio", "load_tricks")}
        self._previous_robot = test_tools.ROBOT
        original_clock = test_tools.VirtualClock
        try:
            if self.realtime:
                test_tools.VirtualClock = _RealtimeClock
            # The real tricks/ folder; only the fake clock construction changes.
            self.rig = test_tools.Rig(tricks=tricks.load_tricks)
        finally:
            test_tools.VirtualClock = original_clock
        self.seen_pub = self.seen_switch = self.seen_log = self.seen_follow = 0
        self.what_happened()  # forget the start-up
        if self.realtime:
            self._ticker = asyncio.create_task(self._tick_feedback())
        return prompt_text(), tool_registry.declarations(self.rig.ts)

    async def call(self, name: str, args: dict) -> dict:
        res = await self.registry.call(self.rig.ts, name, args)
        if not self.realtime:
            await self.rig.settle()  # accelerated demonstration mode only
        return res.as_dict()

    async def _tick_feedback(self) -> None:
        # Reuse fake gateway/clip feedback and watchdogs while jobs or the model
        # are waiting. This never simulates contact or actuator behavior.
        previous_outcome = None
        while True:
            for callback in tuple(self.rig.clock.on_advance):
                callback()
            outcome = self.rig.ts.last_outcome
            if outcome is not previous_outcome:
                previous_outcome = outcome
                self.after()
            await asyncio.sleep(0.02)

    async def aclose(self) -> None:
        if self._closed or not hasattr(self, "rig"):
            return
        self._closed = True
        # All of these objects are fake. Cancel pending work rather than silently
        # completing a requested sequence when the diagnostic connection closes.
        self.rig.close()
        with suppress(asyncio.CancelledError):
            await self.rig.ts.worker
        if self.rig.ts._stop_cleanup_task is not None:
            with suppress(asyncio.CancelledError):
                await self.rig.ts._stop_cleanup_task
        if self._ticker is not None:
            self._ticker.cancel()
            with suppress(asyncio.CancelledError):
                await self._ticker
        for name, value in self._previous_bindings.items():
            setattr(self.tt.robot_api, name, value)
        self.tt.ROBOT = self._previous_robot

    def after(self) -> None:
        """Print observed fake activity; these logs are not extra Gemini replies."""
        lines = self.what_happened()
        if lines:
            print("     ...later, when the queue gets to it:")
        for line in lines:
            print(f"     [fake robot] {line}")

    def what_happened(self) -> list[str]:
        robot = self.rig.robot
        lines = []
        for t, off, on in robot.switches[self.seen_switch:]:
            lines.append(f"legs: {' + '.join(on) or 'nobody (motors relaxed)'} took over" if on else
                         f"legs: relaxed {', '.join(off)}")
        twists = [(t, m) for t, topic, m in robot.published[self.seen_pub:] if topic == "/llm_cmd_vel"]
        moving = [(t, m) for t, m in twists if abs(m.linear.x) + abs(m.linear.y) + abs(m.angular.z) > 1e-9]
        if moving:
            speeds = sorted({(round(m.linear.x, 2), round(m.linear.y, 2), round(m.angular.z * 57.2958))
                             for _, m in moving})
            span = moving[-1][0] - moving[0][0]
            lines.append(f"speed: {len(moving)} commands over {span:.1f} s, (vx, vy, wz_deg) = "
                         + ", ".join(map(str, speeds)) + ("; then a zero command (physical stopping unmeasured)"
                         if all(v == 0 for v in (twists[-1][1].linear.x, twists[-1][1].linear.y,
                                                twists[-1][1].angular.z)) else ""))
        for _, topic, m in robot.published[self.seen_pub:]:
            if topic.endswith("/clip"):
                lines.append(f"clip: {topic.split('/')[1]} plays {m.data}")
            elif topic.endswith("/reference_trick"):
                lines.append("reference: sent to the locomotion policy")
        for call in robot.follow_calls[self.seen_follow:]:
            lines.append(f"person follower: {call}")
        for rec in self.rig.records[self.seen_log:]:
            lines.append(f"outcome: {rec.getMessage()}")
        self.seen_pub, self.seen_switch = len(robot.published), len(robot.switches)
        self.seen_log, self.seen_follow = len(self.rig.records), len(robot.follow_calls)
        return lines


# ============================================================ your tools, on the real robot
class RobotBackend:
    """The robot server, exactly as the web app uses it: GET /tools, POST /tools/<name>."""

    def __init__(self, url: str):
        self.url = url.rstrip("/")
        self.name = f"the real robot at {self.url} (Pupper moves!)"

    def _get(self, path: str):
        with urllib.request.urlopen(self.url + path, timeout=10) as r:
            return json.loads(r.read())

    def setup(self) -> tuple[str, list[dict]]:
        spec = self._get("/tools")
        return spec["prompt"], spec["tools"]

    async def call(self, name: str, args: dict) -> dict:
        req = urllib.request.Request(f"{self.url}/tools/{name}", data=json.dumps(args).encode(),
                                     headers={"content-type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=30) as r:
            return json.loads(r.read())


# ============================================================ the loop
def api_key() -> str:
    key = os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
    if not key and KEY_FILE.exists():
        key = KEY_FILE.read_text().strip()
    if not key:
        sys.exit(f"No Gemini key: set GEMINI_API_KEY, or put it in {KEY_FILE} (robot/setup.sh does).")
    return key


def show(prompt: str, tools: list[dict], full: bool) -> None:
    if full:
        print("=== What Gemini reads: the prompt ===\n" + prompt + "\n")
        print("=== ... and the tools ===\n" + json.dumps(tools, indent=2) + "\n")
    else:
        print("Gemini's tools: " + ", ".join(t["name"] for t in tools) + "   (--show prints everything Gemini reads)")


async def chat(backend, model: str, full: bool) -> None:
    try:
        await _chat(backend, model, full)
    finally:
        if isinstance(backend, DeskBackend):
            await backend.aclose()


async def _chat(backend, model: str, full: bool) -> None:
    try:
        from google import genai
        from google.genai import types
    except ImportError:
        sys.exit("Needs the google-genai package: python3 -m pip install --user --break-system-packages google-genai")

    prompt, tools = backend.setup()
    show(prompt, tools, full)
    config = types.LiveConnectConfig(
        response_modalities=["AUDIO"],  # Live models answer in audio...
        output_audio_transcription=types.AudioTranscriptionConfig(),  # ...and we print what they say
        tools=[types.Tool(function_declarations=[types.FunctionDeclaration(**t) for t in tools])],
        system_instruction=types.Content(parts=[types.Part(text=prompt)]),
    )
    client = genai.Client(api_key=api_key())
    async with client.aio.live.connect(model=model, config=config) as session:
        print(f"Connected to {model}, with {backend.name}. Type 'quit' to exit.\n")
        while True:
            user = (await asyncio.to_thread(input, "You > ")).strip()
            if not user:
                continue
            if user.lower() in ("quit", "exit"):
                break
            await session.send_client_content(
                turns=[types.Content(role="user", parts=[types.Part(text=user)])], turn_complete=True
            )
            said = False
            async for msg in session.receive():
                content = msg.server_content
                if content and content.output_transcription and content.output_transcription.text:
                    if not said:
                        print("Pupper > ", end="")
                        said = True
                    print(content.output_transcription.text, end="", flush=True)
                if msg.tool_call:  # Gemini wants a tool: run it, and send back what happened
                    replies = []
                    for fc in msg.tool_call.function_calls:
                        args = dict(fc.args or {})
                        shown_args = "[redacted]" if fc.name == "unlock" else json.dumps(args)
                        print(("\n" if said else "") + f"  -> Gemini calls {fc.name}({shown_args})")
                        said = False
                        try:
                            result = await backend.call(fc.name, args)
                        except Exception as e:  # e.g. the robot server is not running
                            result = {"success": False, "message": f"{type(e).__name__}: {e}"}
                        print(f"  <- reply Gemini hears now: {json.dumps(result)}")
                        if hasattr(backend, "after"):
                            backend.after()
                        replies.append(types.FunctionResponse(id=fc.id, name=fc.name, response={"result": result}))
                    await session.send_tool_response(function_responses=replies)
                if content and content.turn_complete:
                    print("\n" if said else "")
                    break


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--realtime", action="store_true",
                      help="fake robot only: real elapsed queue timing, immediate acceptance and Stop cancellation")
    mode.add_argument("--robot", nargs="?", const="http://localhost:8000", metavar="URL",
                      help="your tools on the real robot server (default http://localhost:8000)")
    ap.add_argument("--show", action="store_true", help="print the whole prompt and every tool declaration first")
    ap.add_argument("--model", default=MODEL, help=f"Gemini Live model (default {MODEL})")
    args = ap.parse_args()
    sys.path.insert(0, str(HERE))
    backend = RobotBackend(args.robot) if args.robot else DeskBackend(realtime=args.realtime)
    try:
        asyncio.run(chat(backend, args.model, args.show))
    except (KeyboardInterrupt, EOFError):
        print()


if __name__ == "__main__":
    main()
