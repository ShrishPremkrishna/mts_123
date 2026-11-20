"""Exercise the public match endpoints with solo, mixed-mode and paired Pi offers."""
import asyncio
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import json
import time

import pytest

from final_project.pupper_fc.match_director import MatchDirector, directive_ok
from final_project.pupper_fc.tests import test_match_lobby as lab


class Runner(lab.LabRunner):
    def __init__(self, robot, reject=False):
        self.imu = dict(state="healthy", board_session=robot+"-boot", generation=4, incident=0, hold_id=None, epoch=None)
        self.reject_imu = self.defer_imu = self.start_paused = False
        self.pending_imu = None
        self.silent = self.omit_imu = False
        super().__init__(robot, reject)

    async def heartbeat(self):
        while not self.closed:
            if self.silent:
                await asyncio.sleep(.02)
                continue
            self.ticket += 1
            self.emit(dict(event="controls",mode=self.mode,ticket=self.ticket,ready=True,resume_ready=True,
                           **({} if self.omit_imu else {"imu":self.imu})))
            if self.pending_imu and not self.defer_imu:
                self.resume_reply(self.pending_imu)
                self.pending_imu = None
            await asyncio.sleep(.02)

    def pause(self, epoch):
        self.imu = dict(self.imu, state="ready_to_resume", epoch=epoch, hold_id=self.imu["hold_id"] or self.robot+epoch)

    def resume_reply(self, msg):
        accepted = not self.reject_imu
        old = self.imu
        if accepted:
            self.imu = dict(old, state="healthy", epoch=msg["epoch"], hold_id=None)
        self.emit(dict(event="operator",command="imu_resume",ticket=msg["ticket"],accepted=accepted,
                       state="armed",epoch=msg["epoch"],hold_id=old["hold_id"],imu=self.imu,reason="injected rejection"))

    def write(self, data):
        msg = json.loads(data)
        action = msg["command"]
        if action == "arm":
            self.imu = dict(self.imu, epoch=msg["epoch"])
            if self.start_paused:
                self.commands.append(msg)
                self.pause(msg["epoch"])
                self.emit(dict(event="operator",command=action,accepted=True,ticket=msg["ticket"],state="paused"))
                return
        if action == "imu_pause":
            self.commands.append(msg)
            self.pause(msg["epoch"])
            self.emit(dict(event="operator",command=action,ticket=msg["ticket"],accepted=True,
                           state="paused",epoch=msg["epoch"],hold_id=self.imu["hold_id"],imu=self.imu))
        elif action == "imu_resume":
            self.commands.append(msg)
            self.pending_imu = msg
        else:
            super().write(data)


class Camera(lab.Camera):
    def __init__(self, profile, source, *, appearance):
        super().__init__(profile, source, appearance=appearance)
        self.field = profile["field"]
        self.ball_x = 0.

    def snapshot(self):
        s=super().snapshot()
        s["frame"].update(field=self.field,ball={"x":self.ball_x,"y":0.},
                          robots={name:{"x":0.,"y":0.} for name in ("striker","keeper")})
        if s["match"] and self.director:
            s["match"]["score"] = dict(self.director.score)
        return s

    def pause_balance(self):
        self.director.pause_balance()
        self.playing = False

    def after_balance(self, epoch):
        return self.director.after_balance(epoch)


def until(fn):
    end = time.monotonic() + 4
    while time.monotonic() < end:
        result = fn()
        if result:
            return result
        time.sleep(.01)
    pytest.fail("Timed out waiting for " + str(fn))


@pytest.fixture
def rig(monkeypatch, tmp_path):
    monkeypatch.setattr(lab, "LabRunner", Runner)
    monkeypatch.setattr(lab, "Camera", Camera)
    result = lab.rig(tmp_path)
    client = result[0]
    assert client.post("/api/camera",json={"source":"0"}).status_code == 200
    return result


@pytest.mark.parametrize("slots", [("blue",),("red",),("blue","red")])
def test_roster_resume_keeps_pending_until_all_new_epoch_receipts(rig, slots):
    client,lobby,robots,_,_ = rig
    with client, ExitStack() as stack:
        for slot in slots:
            sock = stack.enter_context(lab.enter(client,slot))
            sock.receive_json()
            if slot == "red":
                robot = robots[lab.ROBOT[slot]]
                robot.mode="teleop"
                until(lambda: lobby.peers[robot.robot].offer["mode"] == "teleop")
        result=client.post("/api/match/start").json()
        old=result["camera"]["match"]["epoch"]
        lobby.referee.score["pupper-1"]=2
        first=robots[lab.ROBOT[slots[0]]]
        first.imu=dict(first.imu,state="paused",hold_id="incident",incident=1)
        get=lambda:client.get("/api/state").json()
        until(lambda: (get()["balance"] or {}).get("ready"))
        paused=get()
        assert paused["phase"]=="balance_hold" and paused["score"]["pupper-1"]==2
        assert all(any(c["command"]=="imu_pause" and c["epoch"]==old for c in r.commands) for r in robots.values())
        assert client.post("/api/match/resume").status_code==409
        assert client.post("/api/match/balance-resume",json={"id":"old"}).status_code==409
        last=robots[lab.ROBOT[slots[-1]]]
        last.defer_imu=True
        with ThreadPoolExecutor(1) as worker:
            result=worker.submit(client.post,"/api/match/balance-resume",json={"id":paused["balance"]["id"]})
            until(lambda:last.pending_imu)
            assert get()["phase"]=="balance_resuming" and lobby.referee.pending_start
            assert all(not lobby.drive_allowed(p,lobby.referee.epoch) for p in lobby.players)
            last.defer_imu=False
            response=result.result(timeout=4)
        assert response.status_code==200,response.text
        resumed=response.json()
        assert resumed["phase"]=="playing" and resumed["score"]["pupper-1"]==2
        new=resumed["camera"]["match"]["epoch"]
        assert new!=old and all(r.imu["epoch"]==new for r in robots.values())
        assert all(not lobby.drive_allowed(p,old) for p in lobby.players)
        assert all(not p.closed for p in lobby.players)
        assert client.post("/api/match/end").json()["score"]["pupper-1"]==2


def test_partial_release_stops_everyone_and_preserves_score(rig):
    client,lobby,robots,_,_ = rig
    with client, lab.enter(client,"blue") as blue, lab.enter(client,"red") as red:
        blue.receive_json();red.receive_json()
        assert client.post("/api/match/start").status_code==200
        lobby.referee.score["pupper-2"]=3
        robots["pupper-1"].imu=dict(robots["pupper-1"].imu,state="paused",incident=1)
        robots["pupper-2"].reject_imu=True
        get=lambda:client.get("/api/state").json()
        until(lambda:(get()["balance"] or {}).get("ready"))
        incident=get()["balance"]["id"]
        assert client.post("/api/match/balance-resume",json={"id":incident}).status_code==409
        failed=get()
        assert failed["balance"]["blocked"] and not failed["balance"]["ready"] and failed["score"]["pupper-2"]==3
        assert all(any(c["command"]=="stop" for c in r.commands) for r in robots.values())
        assert lobby.referee.balance_paused


def test_supported_home_latch_and_pre_kickoff_pause(rig):
    client,lobby,robots,_,_ = rig
    with client, lab.enter(client,"blue") as blue:
        blue.receive_json()
        r=robots["pupper-1"];r.start_paused=True
        started=client.post("/api/match/start").json()
        assert started["phase"]=="balance_hold" and not lobby.camera.playing
        get=lambda:client.get("/api/state").json()["balance"]
        until(lambda:get()["ready"])
        r.imu=dict(r.imu,state="home_required",generation=5,reason="board restarted")
        until(lambda:get()["blocked"])
        r.imu=dict(r.imu,state="ready_to_resume",generation=4)
        assert client.post("/api/match/balance-resume",json={"id":get()["id"]}).status_code==409
        assert not any(c["command"]=="imu_resume" for c in r.commands)


def test_fenced_goal_remains_scored_without_authorizing_home_motion():
    d=MatchDirector("old",{"striker":"pupper-1"},{"pupper-1":-1,"pupper-2":1})
    d.score["pupper-2"]=2;d.rally=3;d.goal_sign=1;d.reset_ready=True
    d.pause_balance()
    assert directive_ok(d.state,"old",["pupper-1"],-1)
    assert d.state["phase"]=="holding" and d.state["goal_sign"]==0
    new=d.after_balance("new")
    assert new.state["phase"]=="pending" and new.state["goal_sign"]==0
    assert new.rally==3 and new.goal_sign==1 and new.score==d.score
    assert not new.reset_ready and new.previous is None


def test_new_incident_requires_new_operator_confirmation(rig):
    client, lobby, robots, _, _ = rig
    with client, lab.enter(client, "blue") as blue:
        blue.receive_json()
        assert client.post("/api/match/start").is_success
        runner = robots["pupper-1"]
        runner.imu = dict(runner.imu, state="paused", hold_id="first", incident=1)
        def hold():
            return client.get("/api/state").json()["balance"] or {}
        until(lambda: hold().get("ready"))
        first = hold()["id"]
        runner.imu = dict(runner.imu, state="paused", hold_id="second", incident=2)
        until(lambda: hold().get("id") != first and hold().get("ready"))
        second = hold()["id"]
        assert client.post("/api/match/balance-resume", json={"id": first}).status_code == 409
        assert not any(msg["command"] == "imu_resume" for msg in runner.commands)
        assert client.post("/api/match/balance-resume", json={"id": second}).is_success


def test_stale_peer_holds_both_and_disconnect_preserves_score(rig):
    client, lobby, robots, _, _ = rig
    with client, lab.enter(client, "blue") as blue:
        blue.receive_json()
        with lab.enter(client, "red") as red:
            red.receive_json()
            assert client.post("/api/match/start").is_success
            lobby.referee.score["pupper-1"] = 3
            robots["pupper-2"].silent = True
            def state():
                return client.get("/api/state").json()
            until(lambda: state()["phase"] == "balance_hold")
            hold = state()["balance"]
            assert not hold["ready"] and "heartbeat" in hold["teams"]["pupper-2"]
            assert client.post("/api/match/balance-resume", json={"id": hold["id"]}).status_code == 409
            until(lambda: any(c["command"] == "imu_pause" for c in robots["pupper-1"].commands))
        until(lambda: state()["phase"] == "idle")
        assert state()["score"]["pupper-1"] == 3 and "disconnected" in state()["message"]
        assert not any(c["command"] == "imu_resume" for r in robots.values() for c in r.commands)


def test_stop_preempts_partial_balance_release(rig):
    client, lobby, robots, _, _ = rig
    with client, lab.enter(client, "blue") as blue, lab.enter(client, "red") as red:
        blue.receive_json()
        red.receive_json()
        get = lambda: client.get("/api/state").json()
        assert client.post("/api/match/start").is_success
        first, last = robots["pupper-1"], robots["pupper-2"]
        first.imu = dict(first.imu, state="paused", incident=1)
        until(lambda: (get()["balance"] or {}).get("ready"))
        incident = get()["balance"]["id"]
        last.defer_imu = True
        with ThreadPoolExecutor(1) as worker:
            attempt = worker.submit(client.post, "/api/match/balance-resume", json={"id": incident})
            until(lambda: last.pending_imu)
            blue.send_json({"command": "stop", "robot_id": "pupper-1"})
            assert attempt.result(timeout=4).status_code == 409
        until(lambda: get()["phase"] == "idle")
        assert all(any(c["command"] == "stop" for c in r.commands) for r in robots.values())
        assert not lobby.camera.playing


@pytest.mark.parametrize("board", [None, {}, {"state":"unknown"}, {"state":"healthy","board_session":"boot","generation":1,"incident":True}])
def test_missing_or_malformed_board_blocks_before_any_robot_arm(rig, board):
    client, lobby, robots, feeds, _ = rig
    with client, lab.enter(client, "blue") as blue, lab.enter(client, "red") as red:
        blue.receive_json()
        red.receive_json()
        red_runner = robots["pupper-2"]
        red_runner.imu = board
        red_runner.omit_imu = board is None
        until(lambda: lobby.peers["pupper-2"].offer.get("imu") == board)
        response = client.post("/api/match/start")
        assert response.status_code == 409 and "pupper-2" in response.json()["detail"]
        if board is None:
            assert "imu" not in lobby.peers["pupper-2"].offer
        assert lobby.phase == "idle" and not lobby.camera.playing and not feeds
        assert all(not r.armed and not r.commands for r in robots.values())


def test_out_of_bounds_ball_holds_recovery_until_returned(rig):
    client, lobby, robots, _, _ = rig
    with client, lab.enter(client, "blue") as blue:
        blue.receive_json()
        assert client.post("/api/match/start").is_success
        runner = robots["pupper-1"]
        lobby.camera.ball_x = 25.
        runner.imu = dict(runner.imu, state="paused", incident=1)
        def hold():
            return client.get("/api/state").json()["balance"] or {}
        until(lambda: hold() and lobby.balance.paused)
        incident = hold()["id"]
        assert not hold()["camera_ready"] and not hold()["ready"]
        assert client.post("/api/match/balance-resume", json={"id":incident}).status_code == 409
        lobby.camera.ball_x = 0.
        until(lambda: hold()["ready"])
        assert client.post("/api/match/balance-resume", json={"id":incident}).is_success
