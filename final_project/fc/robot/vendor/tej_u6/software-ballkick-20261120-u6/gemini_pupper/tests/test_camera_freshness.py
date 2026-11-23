"""Read-only camera HTTP freshness with fake ROS and a controlled receipt clock."""
from __future__ import annotations

import asyncio
from contextlib import contextmanager
import queue
from types import SimpleNamespace

import httpx
import pytest

from test_server_transport import server  # reuse its no-network/no-hardware fixture
import robot_api


@contextmanager
def camera_state(server, monkeypatch, *, received=100.0):
    now = [100.0]
    monkeypatch.setattr(server, "monotonic", lambda: now[0], raising=False)
    msg = SimpleNamespace(data=b"raw-camera-bytes")
    compatibility = queue.Queue(maxsize=1)
    compatibility.put_nowait(msg)
    owner = SimpleNamespace(latest_camera_frame=(msg, received), latest_image=compatibility)
    server.tools = owner
    monkeypatch.setattr(server, "undistort_jpeg", lambda data: b"decoded-jpeg")
    try:
        yield owner, now
    finally:
        server.tools = None


async def get_image(server):
    async with httpx.AsyncClient(transport=httpx.ASGITransport(app=server.app), base_url="http://in-process.invalid") as client:
        return await client.get("/camera/image")


def test_repeated_gets_do_not_refresh_frame_receipt_time(server, monkeypatch):
    with camera_state(server, monkeypatch) as (owner, now):
        async def check():
            first = await get_image(server)
            assert first.status_code == 200
            now[0] = 101.9
            assert (await get_image(server)).status_code == 200
            now[0] = 102.01
            expired = await get_image(server)
            assert expired.status_code == 503
            assert "stale" in expired.text.lower()
            assert owner.latest_camera_frame[1] == 100.0
            assert first.headers["cache-control"] == expired.headers["cache-control"] == "no-store"
        asyncio.run(check())


def test_a_new_producer_receipt_restores_availability(server, monkeypatch):
    with camera_state(server, monkeypatch, received=90.0) as (owner, now):
        async def check():
            assert (await get_image(server)).status_code == 503
            owner.latest_camera_frame = (SimpleNamespace(data=b"new-frame"), now[0])
            fresh = await get_image(server)
            assert fresh.status_code == 200 and fresh.content == b"decoded-jpeg"
            assert float(fresh.headers["x-pupper-camera-age-seconds"]) == 0
        asyncio.run(check())


@pytest.mark.parametrize("received", [None, True, "100.0", float("nan"), float("inf"), 100.01])
def test_missing_invalid_or_future_receipt_is_unavailable(server, monkeypatch, received):
    with camera_state(server, monkeypatch, received=received):
        assert asyncio.run(get_image(server)).status_code == 503


def test_missing_snapshot_does_not_serve_untimestamped_compatibility_queue(server, monkeypatch):
    with camera_state(server, monkeypatch) as (owner, _):
        owner.latest_camera_frame = None
        assert asyncio.run(get_image(server)).status_code == 503


def test_frame_expiring_during_jpeg_processing_is_not_sent(server, monkeypatch):
    with camera_state(server, monkeypatch) as (_, now):
        def slow_decode(_data):
            now[0] += 2.01
            return b"old-jpeg"
        monkeypatch.setattr(server, "undistort_jpeg", slow_decode)
        assert asyncio.run(get_image(server)).status_code == 503


def test_serving_does_not_reinsert_an_old_frame_over_a_new_callback(server, monkeypatch):
    with camera_state(server, monkeypatch) as (owner, now):
        new = SimpleNamespace(data=b"new-frame")
        def receive_while_decoding(_data):
            now[0] += .1
            owner.latest_camera_frame = (new, now[0])
            owner.latest_image.get_nowait()
            owner.latest_image.put_nowait(new)
            return b"decoded-old-frame"
        monkeypatch.setattr(server, "undistort_jpeg", receive_while_decoding)
        assert asyncio.run(get_image(server)).status_code == 200
        assert owner.latest_camera_frame == (new, 100.1)
        assert owner.latest_image.get_nowait() is new


@pytest.mark.parametrize("mode", ["none", "empty", "raises"])
def test_invalid_image_processing_is_an_explicit_unavailable_response(server, monkeypatch, mode):
    with camera_state(server, monkeypatch):
        def decode(_data):
            if mode == "raises":
                raise ValueError("synthetic invalid frame")
            return None if mode == "none" else b""
        monkeypatch.setattr(server, "undistort_jpeg", decode)
        result = asyncio.run(get_image(server))
        assert result.status_code == 503
        assert result.headers["cache-control"] == "no-store"


def test_fake_runtime_never_decodes_a_camera_frame(server, monkeypatch):
    with camera_state(server, monkeypatch):
        monkeypatch.setattr(server, "FAKE_MODE", True)
        monkeypatch.setattr(server, "undistort_jpeg", lambda _data: pytest.fail("fake mode must not decode"))
        assert asyncio.run(get_image(server)).status_code == 503


def test_ros_callback_binds_message_to_its_monotonic_receipt(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(robot_api, "time", SimpleNamespace(monotonic=lambda: now[0]))
    robot = robot_api.Robot.__new__(robot_api.Robot)
    robot.latest_image = queue.Queue(maxsize=1)
    robot.latest_camera_frame = None
    first, second = SimpleNamespace(data=b"a"), SimpleNamespace(data=b"b")
    robot._on_image(first)
    assert robot.latest_camera_frame == (first, 10.0)
    now[0] = 11.0
    robot._on_image(second)
    assert robot.latest_camera_frame == (second, 11.0)
    assert robot.latest_image.get_nowait() is second
