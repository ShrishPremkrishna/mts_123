"""Laptop setup and match controls for enrolled Pupper FC robots over their SSH identities."""
import argparse
import asyncio
from contextlib import asynccontextmanager, suppress
import json
import math
import os
from pathlib import Path
import uuid
from urllib.parse import urlsplit

from anyio import CancelScope
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from starlette.middleware.trustedhost import TrustedHostMiddleware
from .field_transport import finish_runner, remote_argv, ssh_host
from .lobby import Lobby, Peer
from .setup_session import open_setup, run_setup

STATIC = Path(__file__).with_name("static")
ROBOTS = {"blue": "pupper-1", "red": "pupper-2"}


async def open_runner(host, robot_id):
    return await asyncio.create_subprocess_exec(
        *remote_argv(host, "match", "--runner-only", "--connect", "--robot-id", robot_id),
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


def create_app(hosts=None, connect=open_runner, *, lobby=None, scan=None, setup=open_setup):
    hosts = hosts or {}
    hosts = {slot: ssh_host(host) for slot, host in hosts.items() if slot in ROBOTS and host}
    if len(set(hosts.values())) != len(hosts):
        raise ValueError("Blue and Red must have different SSH hosts")
    configured = dict(hosts)
    lobby = lobby or Lobby()

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

    @app.get("/")
    async def home():
        return FileResponse(STATIC / "controls.html")

    @app.get("/health")
    async def health(response: Response):
        response.headers["X-Pupper-FC-Installation"] = os.environ.get("PUPPER_FC_INSTALLATION", "")
        return {"app":"pupper-fc-lobby"}

    @app.get("/api/state")
    async def arena_state():
        return lobby.state()

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
            data = await request.json()
            if not isinstance(data, dict):
                raise ValueError("Invalid settings")
            if action == "self-test":
                if lobby.phase != "idle":
                    raise ValueError("End the match before running the offline check")
                from .self_test import check
                return await asyncio.to_thread(check)
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
                if slot not in configured:
                    from .discovery import discover
                    found = await asyncio.to_thread(scan or discover)
                    choices = found.get(ROBOTS[slot], [])
                    if len(choices) != 1:
                        raise ValueError("Two robots advertise the same team" if choices else
                                         "No " + slot + " robot found. Check power, Wi-Fi and team pairing.")
                    host = ssh_host(choices[0])
                    if host in (v for k,v in hosts.items() if k != slot):
                        raise ValueError("Two teams resolved to the same robot")
                    hosts[slot] = host
                return {"robot_id":ROBOTS[slot], "host":hosts[slot]}
            raise HTTPException(404, "Unknown action")
        except (ValueError, OSError, RuntimeError, TimeoutError) as exc:
            raise HTTPException(409, str(exc) or "The operation timed out") from None

    @app.get("/slots")
    async def slots():
        return {slot: {"configured": slot in hosts, "robot_id": robot} for slot, robot in ROBOTS.items()}

    def available(ws, slot):
        origin = urlsplit(ws.headers.get("origin", ""))
        return (slot in hosts and slot not in owners and origin.scheme == "http"
                and origin.netloc == ws.headers.get("host") and origin.hostname in ("localhost", "127.0.0.1"))

    @app.websocket("/setup/{slot}")
    async def setup_robot(ws: WebSocket, slot: str):
        if not available(ws, slot):
            await ws.close(code=1008)
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
            proc = await connect(hosts[slot], robot_id)
            peer = Peer(robot_id, hosts[slot], owner, proc)
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
                    peer.observe(event)
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
                    if message["command"] == "arm" or (lobby.phase != "idle" and message["command"] == "mode"):
                        await ws.send_json({"event":"error","reason":"Use match controls for this action"})
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
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("Select a port between 1 and 65535")
    try:
        app = create_app({"blue": args.blue, "red": args.red})
    except ValueError as exc:
        parser.error(str(exc))
    uvicorn.run(app, host="127.0.0.1", port=args.port, access_log=False, ws_max_size=2048)


if __name__ == "__main__":
    main()
