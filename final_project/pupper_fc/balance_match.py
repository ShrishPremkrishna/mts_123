"""Proof from the native board, bound to one match and one operator recovery."""
import time
import uuid

OFFER_LIFETIME = .6


def proof(peer):
    value = peer.offer.get("imu")
    return value if isinstance(value, dict) else {}


def identity(value):
    session, generation = value.get("board_session"), value.get("generation")
    if isinstance(session, str) and session and type(generation) is int and generation > 0:
        return session, generation
    return None


def valid_incident(value):
    return type(value.get("incident")) is int and value["incident"] >= 0


def fresh(peer):
    return not peer.closed and 0 <= time.monotonic() - peer.received <= OFFER_LIFETIME


def play_problem(peer, anchors, epoch=None):
    row, original = proof(peer), anchors.get(peer.robot)
    if not fresh(peer):
        return "Control heartbeat expired"
    if not row:
        return "Board IMU feedback missing. Check the Pi release and team setup."
    if row.get("state") != "healthy":
        return str(row.get("reason") or "Balance feedback is unavailable")[:160]
    if (not original or not identity(row) or identity(row) != identity(original)
            or not valid_incident(row) or row["incident"] != original.get("incident")):
        return "Board session, Home generation or balance incident changed"
    if epoch is not None and row.get("epoch") != epoch:
        return "Waiting for current balance feedback for this match"
    return ""


class BalanceHold:
    def __init__(self, epoch, anchors, reason):
        self.id, self.epoch = uuid.uuid4().hex, epoch
        self.anchors, self.reason = anchors, reason
        self.paused = {}
        self.blocked = ""
        self.preparing = True

    def record_pause(self, peer, reply):
        detail = reply.get("imu") or {}
        original = self.anchors.get(peer.robot) or {}
        hold_id = reply.get("hold_id")
        if (reply.get("state") != "paused" or reply.get("epoch") != self.epoch
                or detail.get("state") not in ("paused", "ready_to_resume") or not valid_incident(detail)
                or not isinstance(hold_id, str) or not hold_id or detail.get("hold_id") != hold_id
                or detail.get("epoch") != self.epoch
                or not identity(original) or identity(detail) != identity(original)):
            raise ValueError(f"{peer.robot} did not confirm a pause on the original board and Home generation")
        self.paused[peer.robot] = hold_id

    def review(self, peer):
        status = proof(peer)
        original = self.anchors.get(peer.robot)
        reason = str(status.get("reason") or "")[:160]
        if not fresh(peer):
            return "Waiting for the Pi heartbeat", False
        if status.get("state") == "home_required":
            return "Supported Home required. " + reason, False
        if not original or not identity(original) or identity(status) != identity(original):
            return "Board continuity unavailable. End the match and check setup.", False
        if status.get("state") == "unknown" or not valid_incident(status):
            return "Board health unverified. " + reason, False
        if (self.paused.get(peer.robot) and self.paused[peer.robot] == status.get("hold_id")
                and status.get("epoch") == self.epoch and status.get("state") == "ready_to_resume"):
            return "Board continuity verified. Ready to resume.", True
        return reason or "Waiting for the Pi to confirm balance recovery", False

    def report(self, players, camera_ready):
        reviews = {p.robot:self.review(p) for p in players}
        return dict(id=self.id, reason=self.blocked or self.reason,
                    teams={k:message for k,(message, _) in reviews.items()},
                    ready=bool(players) and camera_ready and not self.preparing and not self.blocked
                    and all(ready for _, ready in reviews.values()),
                    blocked=bool(self.blocked), camera_ready=camera_ready)
