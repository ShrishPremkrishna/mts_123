"""Laptop setup and match controls for enrolled Pupper FC robots over their SSH identities."""
import argparse
import asyncio
from contextlib import asynccontextmanager, suppress
import json
import math
import os
from pathlib import Path
import re
import subprocess
import uuid
from urllib.parse import urlsplit

from anyio import CancelScope
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from starlette.middleware.trustedhost import TrustedHostMiddleware
from .field_transport import finish_runner, remote_argv, require_session, ssh_host
from .lobby import Lobby, Peer
from .operator_link import OperatorLink, open_operator
from .setup_session import open_setup, run_setup

STATIC = Path(__file__).with_name("static")
ROOT = Path(__file__).resolve().parents[2]
ROBOTS = {"blue": "pupper-1", "red": "pupper-2"}
MODES = ("live", "replay")
REPLAY_NOTE = ("Replay · synthetic transport, field and robot I/O. The selected field play, runner and executor "
               "decide; robot motion, kick clips, board health, guard and Stop gateway are simulated and never physical.")
REPLAY_EXECUTOR = dict(
    planner="behaviors.live_field_attack.LiveMatchPlay leases; combined_fc_runner.RunnerSession with OperatorDrive, "
            "fc_walk_kick.CombinedRuntime and integration.Executor through fc_executor_bridge.BridgeCore",
    ingress="behaviors.overhead_ingress.IngressCore admits fieldcam StatePublisher encodings in memory",
    substituted="ROS transport, controller manager, kick clip playback, guard and native authority reports, board "
                "record, Stop gateway latch, walker plant, onboard camera",
    limits="No gait, contact, latency or braking model. Home and standing are not rehearsed. "
           "Executed decisions are software behavior only.")


def source_identity(root=ROOT):
    """Name the code this app runs: the ZIP's release stamp, else the local Git head."""
    stamp = root / "release-source.json"
    if stamp.is_file():
        try:
            commit = json.loads(stamp.read_text()).get("source_commit")
        except (OSError, ValueError, AttributeError):
            commit = None
        valid = isinstance(commit, str) and re.fullmatch(r"[0-9a-f]{7,64}", commit)
        return dict(kind="release", commit=commit if valid else None, dirty=None,
                    detail="release-source.json" if valid else "release-source.json is unreadable")
    try:
        head = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True,
                              timeout=2, stdin=subprocess.DEVNULL)
        commit = head.stdout.strip()
        if head.returncode or not re.fullmatch(r"[0-9a-f]{40,64}", commit):
            raise ValueError
        status = subprocess.run(["git", "-C", str(root), "status", "--porcelain", "--untracked-files=no"],
                                capture_output=True, text=True, timeout=2, stdin=subprocess.DEVNULL)
        dirty = bool(status.stdout.strip()) if status.returncode == 0 else None
        return dict(kind="git", commit=commit, dirty=dirty, detail="local Git checkout")
    except (OSError, ValueError, subprocess.SubprocessError):
        return dict(kind="unknown", commit=None, dirty=None, detail="No release stamp or readable Git head")


def app_identity(mode, source):
    try:
        selection = json.loads(Path(__file__).with_name("release_selection.json").read_text())
    except (OSError, ValueError):
        selection = {"qualification": {"physical": "unqualified", "target_build": "pending"},
                     "limits": ["Source selection manifest unavailable."]}
    return dict(selection=selection, mode=mode, synthetic=mode == "replay", hardware_qualified=False, source=source,
                # The lead integrates the canonical release manifest later. Never guess one.
                native_candidate=dict(status="pending", detail="No verified native candidate is recorded for this app"),
                executor=REPLAY_EXECUTOR if mode == "replay" else None,
                onboard_feed=dict(status="unavailable", detail="No onboard preview stream contract is selected"),
                limitation=REPLAY_NOTE if mode == "replay" else
                "Live · commands reach enrolled robots. Software checks here do not qualify hardware.")


async def open_runner(host, robot_id, operator_owner=None):
    """Match controls. A standing operator attaches its lease by owner; the Pi rechecks it."""
    await require_session(host, robot_id)
    owner_args = ["--operator-owner", operator_owner.replace("-", "")] if operator_owner else []
    return await asyncio.create_subprocess_exec(
        *remote_argv(host, "match", "--runner-only", "--connect", "--robot-id", robot_id, *owner_args),
        stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)


def operator_message(data, robot_id):
    if not isinstance(data, dict) or data.get("robot_id") != robot_id:
        raise ValueError("Wrong robot")
    action = data.get("command")
    if action == "stop":
        return {"command": "stop"}
    ticket = data.get("ticket")
    if type(ticket) is not int or not 0 < ticket <= 2**53 - 1:
        raise ValueError("Missing current robot ticket")
    message = dict(command=action, ticket=ticket, robot_id=robot_id)
    if action == "mode" and data.get("mode") in ("teleop", "autonomous"):
        return dict(message, mode=data["mode"])
    if action == "onboard_ball_mode" and data.get("mode") in ("pink", "confirmed", "off"):
        return dict(message, mode=data["mode"])
    if action == "arm" and isinstance(data.get("epoch"), str) and 0 < len(data["epoch"]) <= 128:
        return dict(message, epoch=data["epoch"])
    if action == "drive":
        epoch = data.get("epoch")
        if not isinstance(epoch, str) or not 0 < len(epoch) <= 96:
            raise ValueError("Drive needs a current match")
        message["epoch"] = epoch
        for key in ("forward", "left", "turn"):
            value = data.get(key)
            if type(value) not in (float, int) or not -1 <= value <= 1:
                raise ValueError("Drive axes must be finite and within bounds")
            message[key] = value
        if math.hypot(message["forward"], message["left"]) <= 1.00000001:
            return message
    raise ValueError("Invalid operator control")


def create_app(hosts=None, connect=None, *, lobby=None, scan=None, setup=None, operator=None, mode="live",
               identity=None):
    if mode not in MODES:
        raise ValueError("Choose live or replay mode")
    replay = mode == "replay"
    hosts = hosts or {}
    hosts = {slot: ssh_host(host) for slot, host in hosts.items() if slot in ROBOTS and host}
    if len(set(hosts.values())) != len(hosts):
        raise ValueError("Blue and Red must have different SSH hosts")
    if replay:
        # Server-side boundary: replay owns every endpoint. Nothing reaches SSH, discovery or Home.
        if hosts or connect is not None or scan is not None or setup is not None or operator is not None:
            raise ValueError("Replay mode uses its own synthetic transport and never accepts SSH hosts")
        from .replay_transport import REPLAY_HOSTS, replay_lobby
        lobby = lobby or replay_lobby()
        network = getattr(lobby, "replay_network", None)
        connect = network.open_runner if network is not None else None
    else:
        connect, setup, lobby = connect or open_runner, setup or open_setup, lobby or Lobby()
        operator = operator or open_operator
    if lobby.mode != mode:
        raise ValueError(f"The {mode} app requires a {mode} lobby")
    if replay and connect is None:
        raise ValueError("Replay requires the replay network's own endpoints")
    identity = app_identity(mode, identity or source_identity())
    configured = dict(hosts)
    selected_by = {}

    @asynccontextmanager
    async def lifespan(app):
        yield
        await lobby.end(lobby.owner, "App closed")
        if lobby.camera:
            await asyncio.to_thread(lobby.camera.close)

    app = FastAPI(title="Pupper FC", docs_url=None, redoc_url=None, lifespan=lifespan)
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=["localhost", "127.0.0.1"])
    owners = set()
    setting_up = set()
    operators = {}

    @app.get("/")
    async def home():
        return FileResponse(STATIC / "controls.html")

    @app.get("/health")
    async def health(response: Response):
        response.headers["X-Pupper-FC-Installation"] = os.environ.get("PUPPER_FC_INSTALLATION", "")
        return {"app":"pupper-fc-lobby", "mode":mode}

    @app.get("/api/app")
    @app.get("/api/release")
    async def app_info():
        return identity

    @app.get("/api/state")
    async def arena_state():
        state = dict(lobby.state(), app=identity,
                     operators={slot: link.summary() for slot, link in operators.items()})
        if replay:
            state["replay"] = lobby.replay_network.summary(lobby)
        return state

    @app.get("/api/frame.jpg")
    async def frame():
        data = lobby.camera.jpeg() if lobby.camera else None
        if not data:
            raise HTTPException(404, "Waiting for the camera")
        return Response(data, media_type="image/jpeg", headers={"Cache-Control":"no-store"})

    @app.post("/api/{action:path}")
    async def control(action: str, request: Request):
        try:
            token = str(uuid.UUID(request.headers.get("x-pupper-fc","")))
            origin = request.headers.get("origin")
            if origin and origin != "http://" + request.headers.get("host", ""):
                raise ValueError("Open the controls on this laptop")
        except ValueError:
            raise HTTPException(403, "Open the controls on this laptop") from None
        try:
            if action == "match/start":
                if setting_up:
                    raise ValueError("Finish robot setup before kickoff")
                return await lobby.start(token)
            if action == "match/end":
                return await lobby.end(token)
            if action == "match/resume":
                return await lobby.resume(token)
            if action == "replay/place-ball":
                # The operator's manual ball replacement, on the synthetic plant only. Never automatic.
                if not replay:
                    raise ValueError("Place the real ball by hand in live mode")
                lobby.check_owner(token)
                lobby.replay_network.place_ball(lobby)
                return lobby.state()
            data = await request.json()
            if not isinstance(data, dict):
                raise ValueError("Invalid settings")
            if action == "match/balance-resume":
                return await lobby.resume_balance(token, data.get("id"))
            if action == "self-test":
                if lobby.phase != "idle":
                    raise ValueError("End the match before running the offline check")
                from .self_test import check
                report = await asyncio.to_thread(check)
                return dict(report, scope="software-only", hardware_qualified=False)
            if action in ("camera/snapshot", "camera/confirm", "camera/far-corner"):
                return await lobby.setup_snapshot(action, data, token)
            if action == "camera":
                return await lobby.attach_camera(data, token)
            if action == "goals/configured":
                return await lobby.use_configured_goals(token)
            if action == "goals":
                return await lobby.survey(data, token)
            if action == "discover":
                slot = data.get("slot")
                if slot not in ROBOTS:
                    raise ValueError("Choose Blue or Red")
                def can_select():
                    lobby.check_owner(token)
                    if lobby.phase != "idle" or slot in owners or slot in operators:
                        raise ValueError("Disconnect this team and end the match before changing its connection")
                can_select()
                if replay:
                    if data.get("host") or data.get("save_address") is True:
                        raise ValueError("Replay mode never contacts robot addresses. Clear the SSH address.")
                    hosts[slot], selected_by[slot] = REPLAY_HOSTS[ROBOTS[slot]], token
                    return {"robot_id":ROBOTS[slot], "host":hosts[slot], "synthetic":True}
                explicit = data.get("host", "")
                if not isinstance(explicit, str):
                    raise ValueError("Enter an SSH alias or leave the address blank")
                host = ssh_host(explicit) if explicit else configured.get(slot)
                saving = data.get("save_address") is True
                if not host and not saving:
                    from .discovery import discover
                    found = await asyncio.to_thread(scan or discover)
                    choices = found.get(ROBOTS[slot], [])
                    if len(choices) != 1:
                        raise ValueError("Two robots advertise the same team" if choices else
                                         "No " + slot + " robot found. Check Wi-Fi or enter an SSH address in team setup.")
                    host = ssh_host(choices[0])
                    can_select()  # A connection or kickoff may have started during discovery.
                if host:
                    key = host.rsplit("@", 1)[-1].lower().rstrip(".")
                    if any(v.rsplit("@", 1)[-1].lower().rstrip(".") == key for k,v in hosts.items() if k != slot):
                        raise ValueError("Two teams resolved to the same robot")
                    hosts[slot] = host
                    selected_by[slot] = token
                else:
                    hosts.pop(slot, None)
                    selected_by.pop(slot, None)
                return {"robot_id":ROBOTS[slot], "host":host}
            raise HTTPException(404, "Unknown action")
        except (ValueError, OSError, RuntimeError, TimeoutError) as exc:
            raise HTTPException(409, str(exc) or "The operation timed out") from None

    @app.get("/slots")
    async def slots():
        return {slot: {"configured": slot in hosts, "robot_id": robot} for slot, robot in ROBOTS.items()}

    def available(ws, slot):
        origin = urlsplit(ws.headers.get("origin", ""))
        return (slot in hosts and slot not in owners
                and (slot not in selected_by or selected_by[slot] == ws.query_params.get("owner"))
                and (slot not in operators or operators[slot].owner == ws.query_params.get("owner")) and origin.scheme == "http"
                and origin.netloc == ws.headers.get("host") and origin.hostname in ("localhost", "127.0.0.1"))

    @app.websocket("/operator/{slot}")
    async def supervise(ws: WebSocket, slot: str):
        """One window's laptop Stop link and deliberate Stand, relayed to the Pi gateway."""
        if not available(ws, slot) or slot in operators or lobby.phase != "idle":
            await ws.close(code=1008)
            return
        if replay:
            await ws.accept()
            await ws.send_json(dict(event="error", robot_id=ROBOTS[slot], synthetic=True,
                                    reason="Replay has no Pi Stop gateway. Standing is not rehearsed here."))
            await ws.close()
            return
        try:
            owner = str(uuid.UUID(ws.query_params.get("owner", "")))
            lobby.check_owner(owner)
        except ValueError:
            await ws.close(code=1008)
            return
        link = operators[slot] = OperatorLink(owner, ROBOTS[slot])
        try:
            await ws.accept()
            proc = await operator(hosts[slot], link.robot, owner)
            await link.run(ws, proc)
        except (WebSocketDisconnect, BrokenPipeError, ConnectionResetError):
            pass
        except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
            with suppress(RuntimeError, WebSocketDisconnect):
                await ws.send_json(dict(event="error", robot_id=link.robot, reason=str(exc) or "Pi operator connection lost"))
        finally:
            with CancelScope(shield=True):
                operators.pop(slot, None)
                # Losing laptop Stop authority ends this window's match controls too.
                peer = lobby.peers.get(link.robot)
                if peer and peer.owner == owner and peer.operator_link is link:
                    lobby.stop_requested(peer)
                    with suppress(OSError, ValueError, RuntimeError, asyncio.TimeoutError):
                        await peer.send({"command": "stop"})
                with suppress(RuntimeError, WebSocketDisconnect):
                    await ws.close()

    @app.websocket("/setup/{slot}")
    async def setup_robot(ws: WebSocket, slot: str):
        if not available(ws, slot) or slot in operators:
            await ws.close(code=1008)
            return
        if replay:
            await ws.accept()
            await ws.send_json(dict(event="setup", stage="error", robot_id=ROBOTS[slot], synthetic=True,
                                    message="Replay mode skips Prepare and Home. It never contacts a robot."))
            await ws.close()
            return
        owners.add(slot)
        setting_up.add(slot)
        try:
            await ws.accept()
            owner = str(uuid.UUID(ws.query_params.get("owner", "")))
            lobby.check_owner(owner)
            if lobby.phase != "idle":
                raise ValueError("End the current match before robot setup")
            await run_setup(ws, hosts[slot], ROBOTS[slot], setup)
        except WebSocketDisconnect:
            pass
        except (ValueError, OSError, RuntimeError) as exc:
            with suppress(RuntimeError, WebSocketDisconnect):
                await ws.send_json(dict(event="setup", stage="error", message=str(exc) or "Robot setup failed"))
        finally:
            owners.discard(slot)
            setting_up.discard(slot)
            with suppress(RuntimeError, WebSocketDisconnect):
                await ws.close()

    @app.websocket("/play/{slot}")
    async def play(ws: WebSocket, slot: str):
        if not available(ws, slot):
            await ws.close(code=1008)
            return
        owners.add(slot)
        proc, peer, tasks = None, None, []
        robot_id = ROBOTS[slot]
        try:
            await ws.accept()
            if lobby.phase != "idle":
                raise ValueError("End the current match before adding a team")
            owner = ws.query_params.get("owner")
            owner = str(uuid.UUID(owner)) if owner else uuid.uuid4().hex
            link = operators.get(slot)
            if ws.query_params.get("lease") == "1":
                # Match controls attach to this window's standing operator. The Pi checks again.
                if replay or not link or link.owner != owner or not link.current():
                    raise ValueError("Reconnect and confirm supported standing in this window")
                proc = await connect(hosts[slot], robot_id, owner)
            else:
                if link:
                    raise ValueError("Finish supported standing before opening match controls")
                proc = await connect(hosts[slot], robot_id)
            peer = Peer(robot_id, hosts[slot], owner, proc, operator_link=link)
            lobby.peers[robot_id] = peer

            async def from_robot():
                has_controls = preparing = False
                diagnostic = ""
                first_ticket_by = asyncio.get_running_loop().time() + 15
                while True:
                    timeout = 2 if has_controls else max(.01, first_ticket_by - asyncio.get_running_loop().time())
                    line = await asyncio.wait_for(proc.stdout.readline(), timeout)
                    if not line:
                        if not has_controls and diagnostic:
                            raise ValueError(diagnostic)
                        return
                    try:
                        event = json.loads(line)
                    except (ValueError, UnicodeError):
                        diagnostic = line.decode(errors="replace").strip()[-350:] or diagnostic
                        continue  # native ROS diagnostics also use stdout
                    if not isinstance(event, dict):
                        continue
                    if event.get("event") == "preparing" and not has_controls and not preparing:
                        if event.get("robot_id") != robot_id:
                            raise ValueError("Prepared Pi belongs to the other team")
                        preparing = True
                        first_ticket_by = asyncio.get_running_loop().time() + 95
                        await ws.send_json(event)
                        continue
                    if event.get("event") == "home_required" and not has_controls:
                        if event.get("robot_id") != robot_id:
                            raise ValueError("Prepared Pi belongs to the other team")
                        await ws.send_json(event)
                        return
                    if event.get("event") == "kick" and event.get("robot_id") != robot_id:
                        # Every runner kick receipt names its robot; the referee never sees another's.
                        raise ValueError("Connected runner returned the wrong robot identity")
                    peer.observe(event)
                    if event.get("event") in ("imu", "controls"):
                        lobby.observe_controls(peer)
                    if event.get("event") == "stop_result":
                        lobby.stop_requested(peer)
                    if event.get("event") == "kick" and lobby.camera and peer in lobby.players:
                        lobby.camera.observe_kick(event)
                    if event.get("event") == "controls":
                        if event.get("robot_id") != robot_id:
                            raise ValueError("Connected runner returned the wrong robot identity")
                        has_controls = True
                    if has_controls or "reason" in event or event.get("event") in ("stop_result", "profile"):
                        await ws.send_json(event)

            async def to_robot():
                while True:
                    raw = await ws.receive_text()
                    if len(raw) > 2048:
                        raise ValueError("Operator input exceeds the size limit")
                    message = operator_message(json.loads(raw), robot_id)
                    if message["command"] == "stop":
                        lobby.stop_requested(peer)
                    if message["command"] == "arm" or (lobby.phase != "idle" and message["command"] == "mode"):
                        await ws.send_json({"event":"error","reason":"Use match controls for this action"})
                        continue
                    if message["command"] == "drive" and lobby.phase in ("balance_hold", "balance_resuming"):
                        # Held teams already have zero motion. Reserve their tickets for roster recovery.
                        continue
                    if message["command"] == "drive" and not lobby.drive_allowed(peer, message["epoch"]):
                        # Stale browser or overhead state can only brake. The Pi checks the epoch again.
                        message.update(forward=0, left=0, turn=0)
                    await peer.send(message)

            tasks = [asyncio.create_task(coro()) for coro in (from_robot, to_robot)]
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        except (WebSocketDisconnect, BrokenPipeError, ConnectionResetError):
            pass
        except (OSError, ValueError, asyncio.TimeoutError) as exc:
            with suppress(RuntimeError, WebSocketDisconnect):
                await ws.send_json({"event": "error", "reason": str(exc) or "Runner stopped reporting controls"})
        finally:
            # Disconnect cancels the ASGI request. Finish the robot's EOF/Stop
            # handoff before releasing this slot to the next browser.
            with CancelScope(shield=True):
                try:
                    if proc is not None:
                        proc.stdin.close()
                    if peer is not None:
                        await lobby.remove(peer)
                    for task in tasks:
                        task.cancel()
                    await asyncio.gather(*tasks, return_exceptions=True)
                    if proc is not None:
                        await finish_runner(proc)
                finally:
                    owners.discard(slot)
                    with suppress(RuntimeError, WebSocketDisconnect):
                        await ws.close()

    app.mount("/assets", StaticFiles(directory=STATIC), name="assets")
    return app


def main():
    import uvicorn
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--blue", metavar="SSH_HOST")
    parser.add_argument("--red", metavar="SSH_HOST")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--mode", choices=MODES, default="live",
                        help="live controls enrolled robots; replay uses only synthetic endpoints")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("Select a port between 1 and 65535")
    if args.mode == "replay" and (args.blue or args.red):
        parser.error("Replay mode never uses SSH hosts")
    try:
        app = create_app({"blue": args.blue, "red": args.red}, mode=args.mode)
    except ValueError as exc:
        parser.error(str(exc))
    uvicorn.run(app, host="127.0.0.1", port=args.port, access_log=False, ws_max_size=2048)


if __name__ == "__main__":
    main()
