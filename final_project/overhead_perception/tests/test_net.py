"""Publisher protocol compatibility and measured-clock lifecycle, no camera."""
import json
import select
import socket

from final_project.overhead_perception.fieldcam.net import StatePublisher, StateSubscriber, PROBE_PROTOCOL, REPLY_PROTOCOL


def test_preserves_state_adds_live_session_and_latches_wall_step():
    receiver = StateSubscriber(0, "127.0.0.1")
    mono, wall = [10.], [1000.]
    publisher = StatePublisher("127.0.0.1", receiver.sock.getsockname()[1], live=True,
        source_id="camera", source_session="fixed", monotonic=lambda: mono[0], wall=lambda: wall[0], clock_worker=False)
    original = {"stamp": 1000., "captured_at": 999.9, "robots": {}, "seq": 1}
    try:
        publisher.send(original)
        assert select.select([receiver.sock], [], [], .2)[0]
        first = receiver.latest()
        assert "_transport" not in original and first["captured_at"] == original["captured_at"]
        assert first["_transport"]["clock_valid"] is True
        assert first["_transport"]["live"] is True
        assert first["_transport"]["source_session"] == "fixed"
        mono[0] += .1
        wall[0] += 1.
        publisher.send(original)
        assert select.select([receiver.sock], [], [], .2)[0]
        assert receiver.latest()["_transport"]["clock_valid"] is False
        mono[0] += .1
        wall[0] += .1
        publisher.send(original)
        assert select.select([receiver.sock], [], [], .2)[0]
        assert receiver.latest()["_transport"]["clock_valid"] is False
    finally:
        publisher.close()
        receiver.close()


def test_nonce_probe_returns_original_remote_receive_send_times_and_pinned_identity():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.settimeout(.2)
    publisher = StatePublisher("127.0.0.1", receiver.getsockname()[1], source_session="fixed",
                               monotonic=lambda: 20., wall=lambda: 1000., clock_worker=False)
    try:
        publisher.send({"stamp": 1000.})
        first, address = receiver.recvfrom(65535)
        assert json.loads(first)["_transport"]["live"] is False
        identity = {"source_id": publisher.source_id, "source_session": "fixed", "clock_id": "monotonic:fixed"}
        probe = {**identity, "protocol": PROBE_PROTOCOL, "nonce": "measurement-1"}
        receiver.sendto(json.dumps(probe).encode(), address)
        assert select.select([publisher.sock], [], [], .2)[0]
        publisher.service_clock()
        reply = json.loads(receiver.recvfrom(2048)[0])
        assert reply == {**identity, "protocol": REPLY_PROTOCOL, "nonce": "measurement-1",
                         "received_mono": 20., "sent_mono": 20.}
        receiver.sendto(json.dumps({**probe, "source_session": "wrong"}).encode(), address)
        assert select.select([publisher.sock], [], [], .2)[0]
        publisher.service_clock()
        receiver.setblocking(False)
        try:
            receiver.recvfrom(2048)
        except BlockingIOError:
            pass
        else:
            raise AssertionError("wrong process identity must not get a clock reply")
    finally:
        publisher.close()
        receiver.close()


def test_new_publisher_gets_distinct_process_session_and_closes_worker():
    first = StatePublisher("127.0.0.1", 5005)
    second = StatePublisher("127.0.0.1", 5005)
    assert first.source_session != second.source_session
    first.close()
    second.close()
    assert not first._worker.is_alive() and not second._worker.is_alive()
