"""Physical goal fronts are independent of the marker rectangle."""
from copy import deepcopy

import pytest

from fieldcam.camera_model import default_intrinsics
from fieldcam.config import SIM_CONFIG, load_config, set_field_size
from fieldcam.pipeline import Perception
from fieldcam.pitch import Pitch
from fieldcam.sim import SimArena
from final_project.pupper_fc.match_director import MatchDirector


def selected_pitch():
    field = dict(length=3., width=2., goal_width=.6096, ball_radius=.11)
    goals = {"west": dict(x=-1.8, y_min=-.3048, y_max=.3048),
             "east": dict(x=1.8, y_min=-.3048, y_max=.3048)}
    return Pitch.from_field(field, goals)


def test_pitch_keeps_marker_coordinates_and_playable_goal_strip_distinct():
    pitch = selected_pitch()
    assert Pitch.read(pitch.record()) == pitch
    assert not pitch.marker_bounds.contains(1.65, 0.)
    assert pitch.playable_bounds.contains(1.65, 0.)
    assert pitch.ball_search_bounds.contains(1.8 + 2 * .11, 0.)
    bad = pitch.record()
    bad["playable_bounds"]["x_max"] = 1.5
    with pytest.raises(ValueError, match="end lines"):
        Pitch.read(bad)


def test_rendered_ball_beyond_markers_crosses_surveyed_front_and_scores_once():
    pitch = selected_pitch()
    cfg = load_config(SIM_CONFIG)
    cfg["camera"]["intrinsics"] = "config/_none_.yaml"
    cfg["ball"]["radius"] = pitch.ball_radius
    cfg["field"]["goal_width"] = pitch.goal_width
    set_field_size(cfg, 3., 2.)
    cfg["pitch"] = pitch.record()
    size = (1920, 1080)
    K, _ = default_intrinsics(*size, cfg["camera"]["hfov_deg"])
    arena = SimArena(cfg, K, cam_pos=(0., -.2, 3.5), size=size, noise_sigma=0.)
    perception = Perception(cfg, *size)
    assert perception.calibrate(arena.render({}, None))
    director = MatchDirector("geometry-v2", {"striker": "pupper-1"},
                             {"pupper-1": 1, "pupper-2": -1}, pitch=pitch.record())
    robot = {"striker": (-.6, -.3, .2)}
    saw_strip = False
    for k in range(12):
        capture = 100. + .05 * k
        x = min(1.95, 1.40 + .05 * k)
        frame = perception.process(arena.render(robot, (x, 0.)), capture, capture + .08)
        assert frame["ball"]["measured_this_frame"], (k, x, frame["ball"])
        result = director.update(frame, now=frame["stamp"])
        if 1.55 <= x <= 1.65:
            assert result["phase"] == "playing"
            saw_strip = True
    assert saw_strip and result["phase"] == "goal"
    assert result["score"] == {"pupper-1": 1, "pupper-2": 0}
    for k in range(3):
        capture += .05
        frame = perception.process(arena.render(robot, (1.95, 0.)), capture, capture + .08)
        assert director.update(frame, now=frame["stamp"])["score"]["pupper-1"] == 1
    changed = deepcopy(frame)
    changed["seq"] += 1
    changed["captured_at"] += .05
    changed["pitch"]["goals"]["east"]["x"] += .1
    director.update(changed, now=changed["captured_at"] + .08)
    assert director.calibration_changed


# ---- rendered perception -> publisher bytes -> ingress -> shared executor ----
SURVEYED = {"west": dict(x=-1.80, y_min=-.20, y_max=.41), "east": dict(x=1.80, y_min=-.40, y_max=.21)}


class CapturedSocket:
    """Receives StatePublisher's exact datagrams in memory instead of the network."""

    def __init__(self):
        self.sent = []

    def sendto(self, data, address):
        self.sent.append(data)

    def close(self):
        pass


def rendered_chain(surveyed=SURVEYED):
    import json
    from dataclasses import replace
    from fieldcam.net import REPLY_PROTOCOL, StatePublisher
    from final_project.behaviors import fc_executor_bridge as bridge
    from final_project.behaviors import live_field_attack as attack
    from final_project.behaviors import overhead_ingress as ingress
    from final_project.pupper_fc.kick_handoff import KickHandoff
    from final_project.pupper_fc.model import Field
    from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M

    markers = dict(length=3., width=2., goal_width=.6096, ball_radius=.11)
    record = Pitch.from_field(markers, surveyed).record()
    cfg = load_config(SIM_CONFIG)
    cfg["camera"]["intrinsics"] = "config/_none_.yaml"
    cfg["ball"]["radius"], cfg["field"]["goal_width"] = .11, .6096
    set_field_size(cfg, 3., 2.)
    cfg["goals"] = surveyed
    cfg["pitch"] = record
    size = (1920, 1080)
    K, _ = default_intrinsics(*size, cfg["camera"]["hfov_deg"])
    arena = SimArena(cfg, K, cam_pos=(0., -.2, 3.5), size=size, noise_sigma=0.)
    perception = Perception(cfg, *size)
    clock = {"v": 0.}
    publisher = StatePublisher("127.0.0.1", 9, source_id="fieldcam", source_session="render", live=True,
                               monotonic=lambda: 10. + clock["v"], wall=lambda: 1000. + clock["v"],
                               clock_worker=False)
    publisher.sock.close()
    publisher.sock = CapturedSocket()
    assert perception.calibrate(arena.render({}, None))
    epoch, tracks, signs = "rendered-v2", {"striker": "pupper-1", "keeper": "pupper-2"}, {"pupper-1": 1, "pupper-2": -1}
    enrolled = Field(**markers)
    chains = {}
    for robot in tracks.values():
        source = ingress.IngressCore(source_id="fieldcam", source_session="render", producer_id="pi-" + robot,
                                     epoch=epoch, boot_id="boot-" + robot, robot_map=tracks,
                                     calibration_revision=perception.calibration_revision, field=markers,
                                     pitch=record)
        legacy = ingress.IngressCore(source_id="fieldcam", source_session="render", producer_id="pi-" + robot,
                                     epoch=epoch, boot_id="boot-" + robot, robot_map=tracks,
                                     calibration_revision=perception.calibration_revision, field=markers)
        selected = replace(enrolled, attack_sign=signs[robot]).with_pitch(record)
        play = attack.LiveMatchPlay(robot, selected, .30, epoch, tracks.values())
        runner = bridge.BridgeCore(robot, "pi-" + robot, "boot-" + robot,
                                   {r: replace(enrolled, attack_sign=s) for r, s in signs.items()},
                                   clock=lambda: clock["v"], kick_side="right",
                                   paw_forward_m=COMBINED_PAW_FORWARD_M["right"])
        assert runner.operator({"command": "arm", "epoch": epoch, "pitch": record})["accepted"]
        chains[robot] = dict(source=source, legacy=legacy, play=play, runner=runner, handoff=KickHandoff(robot))
    director = MatchDirector(epoch, tracks, signs, pitch=record)
    legacy_director = MatchDirector(epoch, tracks, signs)

    def sync():
        for chain in chains.values():
            for core in (chain["source"], chain["legacy"]):
                if core.poisoned:
                    continue
                probe = core.make_probe(clock["v"] - .002)
                core.accept_probe({**core.identity(), "nonce": probe["nonce"], "protocol": REPLY_PROTOCOL,
                                   "received_mono": 10 + clock["v"] - .001, "sent_mono": 10 + clock["v"] - .001},
                                  clock["v"])

    def frame(robots, ball, k):
        clock["v"] = .05 * k
        capture = 1000. + clock["v"] - .03
        state = perception.process(arena.render(robots, ball), capture, capture + .02)
        legacy_state = legacy_director.update(state, now=state["stamp"])
        state["fc_match"] = director.update(state, now=state["stamp"])
        publisher.send(state)
        datagram = json.loads(publisher.sock.sent[-1])
        sync()
        out = {"state": state, "legacy_referee": legacy_state, "robots": {}}
        for robot, chain in chains.items():
            row = {}
            try:
                chain["legacy"].normalize(json.loads(json.dumps(datagram)), clock["v"])
                row["legacy_ingress"] = "admitted"
            except ValueError as exc:
                row["legacy_ingress"] = str(exc)
            try:
                normalized = chain["source"].normalize(datagram, clock["v"])
            except ValueError as exc:
                normalized = chain["source"].revoke(exc)
            if normalized is not None:
                outgoing, intent, reason = chain["play"].events(normalized, clock["v"])
                chain["runner"].ingest(outgoing)
                if intent is not None:
                    assert chain["runner"].ingest(intent)["accepted"]
                row.update(normalized=normalized, intent=intent, reason=reason)
            row["step"] = chain["runner"].executor.kick_step(chain["handoff"], clock["v"])
            out["robots"][robot] = row
        return out

    return record, perception, frame


def test_rendered_strip_ball_reaches_the_shared_executor_and_scores_at_the_offset_front_once():
    import math
    record, perception, frame = rendered_chain()
    east = Pitch.read(record).east
    robots = {"striker": (.95, -.12, 0.), "keeper": (1.15, .62, math.pi)}
    for k in range(1, 7):
        out = frame(robots, (1.63, -.10), k)
    # Legacy chain reproduces the original mismatch on identical rendered frames.
    assert "goal geometry differs" in out["robots"]["pupper-1"]["legacy_ingress"]
    assert out["legacy_referee"]["phase"] == "holding"
    # Selected chain: measured ball in the strip, physical goal centres, independent roles.
    blue = out["robots"]["pupper-1"]
    assert out["state"]["ball"]["measured_this_frame"] and out["state"]["fc_match"]["phase"] == "playing"
    assert blue["normalized"]["goals"]["right"] == {"x": 1.80, "y": pytest.approx(-.095)}
    assert blue["normalized"]["pitch"] == record
    assert blue["intent"]["kind"] == "auto" and blue["step"].decision.role == "offense"
    assert blue["step"].decision.phase in ("approach", "align")
    from final_project.pupper_fc.model import Ball
    from final_project.pupper_fc.paw_approach import COMBINED_PAW_FORWARD_M, paw_pose
    ball = blue["normalized"]["ball"]
    stance = paw_pose(Ball(ball["x"], ball["y"]), east.center, "right", forward_m=COMBINED_PAW_FORWARD_M["right"])
    assert blue["step"].decision.target == pytest.approx((stance.x, stance.y))
    # Ball crosses the surveyed front inside the off-centre mouth: one goal, then return home.
    for k, x in enumerate((1.72, 1.80, 1.88, 1.95, 1.97, 1.97), 7):
        out = frame(robots, (x, -.10), k)
    assert out["state"]["fc_match"]["phase"] == "goal"
    assert out["state"]["fc_match"]["score"] == {"pupper-1": 1, "pupper-2": 0}
    assert out["robots"]["pupper-1"]["step"].decision.phase == "return_home"


def test_rendered_pitch_reselection_revokes_bound_consumers():
    import math
    record, perception, frame = rendered_chain()
    robots = {"striker": (.4, -.1, 0.), "keeper": (-.4, .3, math.pi)}
    for k in range(1, 4):
        out = frame(robots, (0., 0.), k)
    assert out["state"]["fc_match"]["phase"] == "playing"
    changed = Pitch.read(record).record()
    changed["goals"]["east"].update(y_min=-.30, y_max=.31)
    perception.select_pitch(changed)
    out = frame(robots, (0., 0.), 4)
    blue = out["robots"]["pupper-1"]
    assert out["state"]["fc_match"]["phase"] == "holding"
    assert blue["normalized"]["captured_at"] is None and "calibration revision changed" in blue["reason"]
    step = blue["step"].decision
    assert (step.command.vx, step.command.vy, step.command.wz) == (0., 0., 0.)
