import os
from queue import Queue
import select
from threading import Event, Thread
import time
from types import SimpleNamespace

import pytest

from pupper_bci import source
from pupper_bci.source import CytonSource, Sample, read_recording


@pytest.fixture
def fake_cyton():
    pty = pytest.importorskip("pty")
    pytest.importorskip("serial")
    master, slave = pty.openpty()
    halt = Event()
    firmware = SimpleNamespace(
        port=os.ttyname(slave),
        commands=Queue(),
        welcome=b"OpenBCI V3 8-16 channel\r\nOn Board ADS1299 Device ID: 0x3E\r\n"
                b"LIS3DH Device ID: 0x33\r\nFirmware: v3.1.2\r\n$$$",
    )

    def simulate():
        while not halt.is_set():
            if select.select([master], [], [], 0.02)[0]:
                for command in os.read(master, 256):
                    command = bytes([command])
                    firmware.commands.put(command)
                    if command == b"v" and firmware.welcome:
                        os.write(master, firmware.welcome)

    simulation = Thread(target=simulate, daemon=True)
    simulation.start()

    def disconnect():
        nonlocal master
        halt.set()
        simulation.join(1.0)
        if master is not None:
            os.close(master)
            master = None

    firmware.send = lambda data: os.write(master, data)
    firmware.disconnect = disconnect
    try:
        yield firmware
    finally:
        disconnect()
        os.close(slave)


def _frame(sequence, *, fp1=0, fp2=0, accel=(0, 0, 0), footer=0xC0):
    eeg = (123456, -654321, fp1, 0, 0, 0, fp2, 0)
    return (
        bytes([0xA0, sequence])
        + b"".join(value.to_bytes(3, "big", signed=True) for value in eeg)
        + b"".join(value.to_bytes(2, "big", signed=True) for value in accel)
        + bytes([footer])
    )


def test_live_serial_preserves_signed_values_packet_age_and_resync(fake_cyton):
    from serial import Serial, SerialException

    firmware = fake_cyton
    live = CytonSource(firmware.port, (3, 7))
    with pytest.raises(RuntimeError, match="before reading"):
        live.read()
    with live as stream:
        assert firmware.commands.get(timeout=1.0) == b"v"
        assert firmware.commands.get(timeout=1.0) == b"b"
        assert stream.read() == []
        with pytest.raises(RuntimeError, match="already open"):
            live.__enter__()
        with pytest.raises(SerialException, match="exclusive"):
            Serial(firmware.port, exclusive=True)

        first = _frame(254, fp1=-8192, fp2=3500, accel=(-2000, 0, 7840))
        firmware.send(first[:11])
        time.sleep(0.015)
        assert stream.read() == []
        firmware.send(
            first[11:]
            + _frame(255)
            + _frame(0, fp1=0xA012, footer=0x00)
            + b"noise"
            + _frame(1, accel=(0, 16, 8000))
            + _frame(2, accel=(0, 16, 8000), footer=0xC1)
            + _frame(3, accel=(0, 16, 8000), footer=0xC4)
        )
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            with stream._lock:
                if len(stream._samples) == 5:
                    break
            time.sleep(0.005)
        settled_at = time.monotonic()
        time.sleep(0.035)
        samples = stream.read()

        assert [sample.sequence for sample in samples] == [254, 255, 1, 2, 3]
        assert all(sample.t <= settled_at for sample in samples)
        assert [sample.t for sample in samples] == sorted(sample.t for sample in samples)
        uv_per_count = 4_500_000 / 24 / (2**23 - 1)
        assert samples[0].eeg_uv == pytest.approx((-8192 * uv_per_count, 3500 * uv_per_count))
        assert samples[0].accel_g == pytest.approx((-0.25, 0, 0.98))
        assert samples[1].accel_g is None
        assert samples[2].accel_g == pytest.approx((0, 0.002, 1.0))
        assert samples[3].accel_g is None
        assert samples[4].accel_g is None
        assert stream.read() == []
    assert firmware.commands.get(timeout=1.0) == b"s"
    with pytest.raises(RuntimeError, match="before reading"):
        live.read()


@pytest.mark.parametrize("welcome,match", [
    (b"", "no Cyton startup response"),
    (b"unrecognized board$$$", "unexpected Cyton startup response"),
    (b"OpenBCI V3\r\nOn Daisy ADS1299 Device ID: 0x3E\r\nLIS3DH\r\n$$$", "Daisy is attached"),
])
def test_invalid_startup_closes_port(fake_cyton, monkeypatch, welcome, match):
    from serial import Serial

    fake_cyton.welcome = welcome
    monkeypatch.setattr(source, "_RESET_TIMEOUT", 0.1)
    with pytest.raises(RuntimeError, match=match):
        with CytonSource(fake_cyton.port):
            pytest.fail("stream should not start")
    assert fake_cyton.commands.get(timeout=1.0) == b"v"
    assert fake_cyton.commands.empty()
    with Serial(fake_cyton.port, exclusive=True) as port:
        assert port.is_open


def test_serial_disconnect_surfaces_reader_failure(fake_cyton):
    with pytest.raises(RuntimeError, match="could not read Cyton"):
        with CytonSource(fake_cyton.port) as stream:
            assert fake_cyton.commands.get(timeout=1.0) == b"v"
            assert fake_cyton.commands.get(timeout=1.0) == b"b"
            fake_cyton.disconnect()
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                stream.read()
                time.sleep(0.005)


@pytest.mark.parametrize("channels", [(1,), (0, 2), (2, 9), (1, 1), (1.5, 2)])
def test_invalid_channel_selection_fails_without_loading_board(channels):
    with pytest.raises(ValueError, match="Cyton EEG channels"):
        CytonSource("/dev/cu.test", channels)


def test_recording_format_preserves_gaps_and_missing_acceleration(tmp_path):
    path = tmp_path / "samples.csv"
    path.write_text(
        "t,sequence,eeg1_uv,eeg2_uv,ax_g,ay_g,az_g\n"
        "0,255,-1.25,2.5,0,0,1\n"
        "0.008,1,1.5,3,,,\n"
        "0.012,2,-2.25,3.5,0.125,0,0.99\n",
        encoding="utf-8",
    )
    assert read_recording(path) == [
        Sample(0.0, 255, (-1.25, 2.5), (0.0, 0.0, 1.0)),
        Sample(0.008, 1, (1.5, 3.0), None),
        Sample(0.012, 2, (-2.25, 3.5), (0.125, 0.0, 0.99)),
    ]


@pytest.mark.parametrize("content,match", [
    ("time,sequence,eeg1_uv,eeg2_uv,ax_g,ay_g,az_g\n", "CSV header"),
    ("", "contains no samples"),
    ("0,256,1,2,,,\n", "sequence must be a byte"),
    ("0,0,NaN,2,,,\n", "EEG values must be finite"),
    ("inf,0,1,2,,,\n", "time and EEG values must be finite"),
    ("0,0,1,2,0,0,-inf\n", "acceleration must be finite"),
    ("0,0,1,2,0,,1\n", "could not convert"),
    ("0,0,1,2,,,x,extra\n", "seven columns"),
    ("-1,0,1,2,,,\n", "nonnegative and nondecreasing"),
    ("1,0,1,2,,,\n0,1,1,2,,,\n", "nonnegative and nondecreasing"),
])
def test_invalid_recordings_are_rejected(tmp_path, content, match):
    path = tmp_path / "samples.csv"
    header = "t,sequence,eeg1_uv,eeg2_uv,ax_g,ay_g,az_g\n"
    path.write_text(content if content.startswith("time,") else header + content, encoding="utf-8")
    with pytest.raises(ValueError, match=match):
        read_recording(path)
