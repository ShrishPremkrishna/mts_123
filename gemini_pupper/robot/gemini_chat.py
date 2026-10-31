#!/usr/bin/env python3
"""Talk to Gemini in text, and watch every tool call it makes.

A text diagnostic for Gemini Live: no browser or microphone. Conversation and
all robot tools share one session. Tool-bearing turns use only server receipts;
completed tool-free conversation passes a conservative English claim screen
that is not a semantic truth guarantee. --receipts-only mutes all model prose.
You type, Gemini
answers and calls tools, and this script runs each call and sends the reply
back through the same tool API used by the web app. A received model Stop can
bypass a slow tool call. Input is still one user utterance per model turn; this
is not an independent keyboard Stop control. Exit/disconnect cancels pending
dispatch and requests a best-effort Stop for submitted work, not physical proof
that the robot stopped.

    python3 lab/chat.py            # YOUR tools, on a fake robot (a laptop is enough)
    python3 lab/chat.py --realtime # fake robot with real queue timing/cancellation
    python3 lab/chat.py --realtime --thinking-level MINIMAL # explicit comparison
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
import urllib.error
from pathlib import Path

from operator_feedback import CONVERSATION_BLOCKED, conversation_claim_detected, receipt_feedback, sending_feedback

HERE = Path(__file__).resolve().parent
KEY_FILE = Path.home() / ".config" / "gemini-pupper" / "api_key"
MODEL = "gemini-3.1-flash-live-preview"  # the web app's default (constants.ts)
THINKING_LEVELS = ("AUTO", "MINIMAL", "LOW", "MEDIUM", "HIGH")


def resolved_thinking_level(model: str, level: str = "AUTO") -> str | None:
    """AUTO matches the browser's 3.1 Live setting; other models keep defaults."""
    level = level.upper()
    if level not in THINKING_LEVELS:
        raise ValueError(f"unknown thinking level {level!r}; choose {', '.join(THINKING_LEVELS)}")
    if level != "AUTO":
        return level
    # https://ai.google.dev/gemini-api/docs/models/gemini-3.1-flash-live-preview
    return "HIGH" if model.lower().removeprefix("models/") == MODEL else None


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
        self.diagnostic_output = False
        if realtime:
            self.name = "your tools on a fake robot (real queue timing; no physics or hardware)"

    def setup(self) -> tuple[str, list[dict]]:
        import test_tools  # installs a fake ROS before robot_api is imported
        import tricks
        import tool_registry
        from runtime_config import follower_mode

        follow_available = follower_mode() != "disabled"

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
            self.rig.ts.follow_available = follow_available
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
        if not self.diagnostic_output:
            return
        lines = self.what_happened()
        if lines:
            print("DIAGNOSTIC fake activity (not physical evidence):")
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
        self._command_epoch: str | None = None
        self._stop_serial = 0
        self._stops_pending = 0
        self._stop_unknown = False
        self._epoch_invalidated = False

    def _get(self, path: str):
        with urllib.request.urlopen(self.url + path, timeout=10) as r:
            return json.loads(r.read())

    def setup(self) -> tuple[str, list[dict]]:
        spec = self._get("/tools")
        health = self._get("/health")
        self._command_epoch = self._epoch(health)
        self._stop_unknown = False
        self._epoch_invalidated = False
        return spec["prompt"], spec["tools"]

    @staticmethod
    def _epoch(reply) -> str | None:
        if not isinstance(reply, dict) or reply.get("command_epoch_required") is not True:
            return None
        value = reply.get("command_epoch") if isinstance(reply, dict) else None
        return value if isinstance(value, str) and value else None

    def _post(self, name: str, args: dict, epoch: str | None) -> dict:
        headers = {"content-type": "application/json"}
        if epoch is not None:
            headers["X-Pupper-Command-Epoch"] = epoch
        req = urllib.request.Request(f"{self.url}/tools/{name}", data=json.dumps(args).encode(),
                                     headers=headers, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as exc:
            # The server explicitly refused this request before dispatch.
            if exc.code in (409, 428):
                reply = json.loads(exc.read())
                if reply.get("success") is False and reply.get("command_epoch_required") is True:
                    return reply
            raise

    async def call(self, name: str, args: dict) -> dict:
        stopping = name == "stop"
        protected = name not in ("stop", "robot_status")
        if stopping:
            self._stop_serial += 1
            self._stops_pending += 1
            self._command_epoch = None
        serial = self._stop_serial
        dispatched = False
        try:
            if protected:
                if self._stops_pending or self._stop_unknown or self._epoch_invalidated:
                    return {"success": False, "not_dispatched": True, "message": "No command submitted: Stop is pending/unconfirmed or the command epoch changed; use Stop or reconnect after checking status."}
                epoch = self._command_epoch
                if epoch is None:
                    health = await asyncio.to_thread(self._get, "/health")
                    epoch = self._epoch(health)
                # A Stop while obtaining metadata invalidates this original intent.
                if serial != self._stop_serial or self._stops_pending:
                    return {"success": False, "not_dispatched": True, "message": "No command submitted: Stop superseded this request."}
                if epoch is None:
                    return {"success": False, "not_dispatched": True, "message": "No command submitted: server command-epoch support is unavailable."}
                self._command_epoch = epoch
            else:
                epoch = None
            dispatched = True
            reply = await asyncio.to_thread(self._post, name, args, epoch)
            if protected and serial != self._stop_serial:
                rejected = reply.get("success") is False and reply.get("command_epoch_required") is True
                return {"success": False, "superseded": True, "outcome_unknown": not rejected,
                        "message": "Stop superseded this request. " + ("The server refused it before dispatch."
                            if rejected else "Its late reply is not a new acceptance; check status for any earlier execution.")}
            if serial == self._stop_serial:
                # Refresh only for a later request; never retry the rejected action.
                if protected and reply.get("command_epoch_required") is True and reply.get("success") is False:
                    self._epoch_invalidated = True
                    self._command_epoch = None
                if stopping:
                    if new_epoch := self._epoch(reply):
                        self._command_epoch = new_epoch
                    self._stop_unknown = reply.get("success") is not True or not self._epoch(reply)
            return reply
        except asyncio.CancelledError:
            if stopping and serial == self._stop_serial:
                self._stop_unknown = True
            raise
        except Exception as exc:
            if stopping and serial == self._stop_serial:
                self._stop_unknown = True
            return {"success": False, "outcome_unknown": dispatched, "not_dispatched": not dispatched,
                    "message": ("Request outcome unknown" if dispatched else "No command submitted")
                    + f": {type(exc).__name__}: {exc}"}
        finally:
            if stopping:
                self._stops_pending -= 1


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
        print("=== DIAGNOSTIC prompt (not operator evidence) ===\n" + prompt + "\n")
        print("=== DIAGNOSTIC tool declarations ===\n" + json.dumps(tools, indent=2) + "\n=== END DIAGNOSTIC CONFIGURATION ===")
    else:
        print("Tool definitions loaded. --show displays diagnostic configuration, not execution evidence.")


async def chat(backend, model: str, full: bool, *, thinking_level: str = "AUTO", conversation: bool = True) -> None:
    try:
        await _chat(backend, model, full, thinking_level=thinking_level, conversation=conversation)
    finally:
        if isinstance(backend, DeskBackend):
            await backend.aclose()


class _ToolDispatcher:
    """Keep reading Live packets while a tool waits; Stop bypasses normal work.

    Normal batches stay ordered. Observing Stop invalidates earlier intents
    synchronously, and no more motion is accepted in that model turn. A later
    user turn can request new work after a confirmed Stop; unknown outcomes
    require reconnecting. Cancellation cannot retract an HTTP thread, so the
    RobotBackend/server command epoch remains the transport fence.
    """

    def __init__(self, backend, session, response_type, *, conversation=True, diagnostics=False):
        self.backend, self.session, self.response_type = backend, session, response_type
        self.conversation, self.diagnostics = bool(conversation), bool(diagnostics)
        self.turn_calls = 0
        self.normal_lock = asyncio.Lock()
        self.tasks = []
        self.failure = asyncio.get_running_loop().create_future()
        self.generation = self.pending_stops = 0
        self.stopped_in_turn = self.paused = self.closed = self.needs_stop = False

    def begin_turn(self):
        self.stopped_in_turn = False
        self.turn_calls = 0

    def submit(self, calls):
        if self.closed:
            return
        calls = list(calls)
        self.turn_calls += len(calls)
        has_stop = any(fc.name == "stop" for fc in calls)
        if has_stop:
            self.generation += 1
            self.pending_stops += 1
            self.stopped_in_turn = True
        task = asyncio.create_task(self._run(calls, self.generation, has_stop))
        self.tasks.append(task)
        task.add_done_callback(self._finished)

    def _finished(self, task):
        # A failed response send must also interrupt a stalled receive(), rather
        # than leaving an unobserved background exception and an active queue.
        if not task.cancelled() and (error := task.exception()) is not None and not self.failure.done():
            self.failure.set_result(error)

    async def _run(self, calls, generation, has_stop):
        async def respond():
            replies = []
            ordered = sorted(calls, key=lambda fc: fc.name != "stop") if has_stop else calls
            for fc in ordered:
                if self.closed:
                    return
                args = dict(fc.args or {})
                shown_args = "[redacted]" if fc.name == "unlock" else json.dumps(args)
                if self.diagnostics:
                    print(f"DIAGNOSTIC model tool request: {fc.name}({shown_args})")
                dispatched = False
                protected = fc.name not in ("stop", "robot_status")
                if protected and has_stop:
                    result = {"success": False, "message": "Not dispatched: Stop takes priority over other actions in this batch. No delayed Stop was scheduled."}
                elif protected and (generation != self.generation or self.stopped_in_turn
                                    or self.pending_stops or self.paused):
                    result = {"success": False, "message": "Not dispatched: Stop superseded this turn, or an outcome is unknown. Ask again in a later turn after a confirmed Stop; reconnect if an outcome remains unknown."}
                else:
                    dispatched = True
                    print(sending_feedback(fc.name))
                    if fc.name != "robot_status":
                        self.needs_stop = True
                    try:
                        result = await self.backend.call(fc.name, args)
                    except Exception as exc:
                        result = {"success": False, "outcome_unknown": True,
                                  "message": f"{type(exc).__name__}: {exc}"}
                    if self.closed:
                        return
                    if protected and generation != self.generation:
                        result = {"success": False, "superseded": True,
                                  "outcome_unknown": result.get("outcome_unknown", result.get("success") is not False),
                                  "message": "Stop superseded this request. Its late reply is not a new acceptance; check status for any earlier execution.",
                                  "original_result": result}
                    if result.get("outcome_unknown") or (result.get("command_epoch_required") is True
                                                         and result.get("success") is False):
                        self.paused = True
                    if fc.name == "stop":
                        if result.get("success") is not True:
                            self.paused = True
                        elif generation == self.generation:
                            self.needs_stop = False
                print(receipt_feedback(fc.name, result, current=generation == self.generation,
                                       dispatched=dispatched))
                if self.diagnostics:
                    print(f"DIAGNOSTIC raw tool reply (not operator evidence): {json.dumps(result)}")
                    if hasattr(self.backend, "after"):
                        self.backend.after()
                replies.append(self.response_type(id=fc.id, name=fc.name, response={"result": result}))
            if not self.closed:
                await self.session.send_tool_response(function_responses=replies)

        try:
            if has_stop:
                await respond()
            else:
                async with self.normal_lock:
                    await respond()
        finally:
            if has_stop:
                self.pending_stops -= 1

    async def drain(self):
        # gather raises on one failed response without cancelling siblings.
        # Retain unfinished tasks so close() still owns and cancels them.
        tasks = list(self.tasks)
        try:
            if tasks:
                await asyncio.gather(*tasks)
        finally:
            self.tasks = [task for task in self.tasks if not task.done()]

    async def close(self):
        self.closed = True
        self.generation += 1
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self.tasks.clear()
        if self.needs_stop:
            # Best effort, bounded shutdown. A timeout is explicitly unknown;
            # it never establishes that the robot physically stopped.
            try:
                result = await asyncio.wait_for(self.backend.call("stop", {}), timeout=5)
                print("Connection cleanup: " + receipt_feedback("stop", result))
                if self.diagnostics:
                    print(f"DIAGNOSTIC raw cleanup reply: {json.dumps(result)}")
            except Exception:
                print("Connection cleanup: " + receipt_feedback("stop", None))


async def _receive_turn(session, dispatcher):
    # Buffer output synchronously before any tool awaits. A later tool in the
    # same response invalidates all narration, including an earlier "queued".
    transcript, text_parts = [], []
    characters = chunks = 0
    complete = interrupted = overflow = False
    # Expiry is checked on incoming content; this is not a receive timeout.
    deadline = asyncio.get_running_loop().time() + 20
    async for msg in session.receive():
        content = msg.server_content
        if content:
            interrupted = interrupted or bool(getattr(content, "interrupted", False))
            if asyncio.get_running_loop().time() > deadline:
                overflow = True
            transcription = getattr(getattr(content, "output_transcription", None), "text", None)
            model_turn = getattr(content, "model_turn", None)
            incoming = [(transcript, transcription)] + [(text_parts, getattr(part, "text", None))
                for part in (getattr(model_turn, "parts", None) or []) if not getattr(part, "thought", False)]
            for target, text in incoming:
                if not isinstance(text, str) or not text:
                    continue
                characters += len(text)
                chunks += 1
                if characters > 16000 or chunks > 256:
                    overflow = True
                if dispatcher.conversation and not (overflow or interrupted):
                    target.append(text)
            if overflow or interrupted:
                transcript.clear()
                text_parts.clear()
        if msg.tool_call:
            dispatcher.submit(msg.tool_call.function_calls)
            transcript.clear()
            text_parts.clear()
        if content and getattr(content, "turn_complete", False):
            complete = True
            break
    # Pending requests still get their own receipts. Completing a model turn
    # cannot cause narration to escape while a backend request remains pending.
    await dispatcher.drain()
    if not dispatcher.conversation or dispatcher.turn_calls:
        return
    if not complete or interrupted or overflow:
        print("Conversation output withheld: response incomplete, interrupted or over its buffer limit.")
        return
    text = "".join(transcript or text_parts).strip()
    if not text:
        return
    if conversation_claim_detected(text):
        print(CONVERSATION_BLOCKED)
    else:
        print("Pupper > " + text.replace("\n", "\nPupper > "))


async def _chat(backend, model: str, full: bool, *, thinking_level: str = "AUTO", conversation: bool = True) -> None:
    try:
        from google import genai
        from google.genai import types
    except ImportError:
        sys.exit("Needs the google-genai package: python3 -m pip install --user --break-system-packages google-genai")

    selected_thinking = resolved_thinking_level(model, thinking_level)
    prompt, tools = backend.setup()
    if isinstance(backend, DeskBackend):
        backend.diagnostic_output = full
    print("CONVERSATION + CONTROL: all tools remain available. Tool replies use server receipts; other conversation is screened, not verified." if conversation
          else "RECEIPT-ONLY CONTROL: model narration is suppressed; confirmations are API evidence, not physical proof.")
    show(prompt, tools, full)
    config = types.LiveConnectConfig(
        response_modalities=["AUDIO"],  # Live models answer in audio...
        output_audio_transcription=types.AudioTranscriptionConfig(),  # ...and we print what they say
        tools=[types.Tool(function_declarations=[types.FunctionDeclaration(**t) for t in tools])],
        system_instruction=types.Content(parts=[types.Part(text=prompt)]),
    )
    if selected_thinking is not None:
        config.thinking_config = types.ThinkingConfig(thinking_level=selected_thinking)
    print(f"Thinking level: {selected_thinking or 'model default (not overridden)'}")
    client = genai.Client(api_key=api_key())
    async with client.aio.live.connect(model=model, config=config) as session:
        print(f"Connected to {model}, with {backend.name}. Type 'quit' to exit.\n")
        dispatcher = _ToolDispatcher(backend, session, types.FunctionResponse, conversation=conversation, diagnostics=full)
        try:
            while True:
                user = (await asyncio.to_thread(input, "You > ")).strip()
                if not user:
                    continue
                if user.lower() in ("quit", "exit"):
                    break
                dispatcher.begin_turn()
                await session.send_client_content(
                    turns=[types.Content(role="user", parts=[types.Part(text=user)])], turn_complete=True
                )
                receiver = asyncio.create_task(_receive_turn(session, dispatcher))
                try:
                    done, _ = await asyncio.wait((receiver, dispatcher.failure),
                                                 return_when=asyncio.FIRST_COMPLETED)
                    if dispatcher.failure in done:
                        raise dispatcher.failure.result()
                    await receiver
                finally:
                    receiver.cancel()
                    await asyncio.gather(receiver, return_exceptions=True)
        finally:
            await dispatcher.close()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--realtime", action="store_true",
                      help="fake robot only: real elapsed queue timing, immediate acceptance and Stop cancellation")
    mode.add_argument("--robot", nargs="?", const="http://localhost:8000", metavar="URL",
                      help="your tools on the real robot server (default http://localhost:8000)")
    ap.add_argument("--show", action="store_true", help="show clearly labeled raw diagnostic configuration and tool replies")
    output = ap.add_mutually_exclusive_group()
    output.add_argument("--conversation", action="store_true", help="conversation and full robot tools in one session (the default)")
    output.add_argument("--receipts-only", action="store_true", help="diagnostic strict output: suppress all model prose, retain all robot tools")
    ap.add_argument("--model", default=MODEL, help=f"Gemini Live model (default {MODEL})")
    ap.add_argument("--thinking-level", type=str.upper, choices=THINKING_LEVELS, default="AUTO",
                    help="AUTO uses HIGH for Gemini 3.1 Flash Live and leaves other models unchanged; "
                         "an explicit level overrides the model default for reproducible comparisons")
    args = ap.parse_args()
    sys.path.insert(0, str(HERE))
    backend = RobotBackend(args.robot) if args.robot else DeskBackend(realtime=args.realtime)
    try:
        options = {"thinking_level": args.thinking_level}
        if args.receipts_only:
            options["conversation"] = False
        elif args.conversation:
            options["conversation"] = True
        asyncio.run(chat(backend, args.model, args.show, **options))
    except (KeyboardInterrupt, EOFError):
        print()


if __name__ == "__main__":
    main()
