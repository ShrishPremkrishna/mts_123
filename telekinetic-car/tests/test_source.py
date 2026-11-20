import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from pupper_bci import source
from pupper_bci.source import CytonSource, Sample, read_recording


@pytest.fixture
def fake_brainflow(monkeypatch):
    boards = []

    class FakeBoardError(Exception):
        pass

    class FakeBoard:
        rate = 250
        fail_start = False
        fail_read = False

        def __init__(self, board_id, params):
            self.board_id = board_id
            self.port = params.serial_port
            self.actions = []
            self.data = np.empty((24, 0))
            boards.append(self)

        @staticmethod
        def get_eeg_channels(board_id):
            return list(range(1, 9))

        @staticmethod
        def get_other_channels(board_id):
            return list(range(12, 19))

        @staticmethod
        def get_package_num_channel(board_id):
            return 0

        @staticmethod
        def get_timestamp_channel(board_id):
            return 22

        def get_board_sampling_rate(self):
            return self.rate

        def prepare_session(self):
            self.actions.append("prepare")

        def start_stream(self, capacity):
            self.actions.append("start")
            assert capacity >= 250
            if self.fail_start:
                raise RuntimeError("start failed")

        def get_board_data(self):
            if self.fail_read:
                raise FakeBoardError("stream is gone")
            data, self.data = self.data, np.empty((24, 0))
            return data

        def release_session(self):
            self.actions.append("release")

    brainflow = ModuleType("brainflow")
    board_shim = ModuleType("brainflow.board_shim")
    board_shim.BoardShim = FakeBoard
    board_shim.BoardIds = SimpleNamespace(CYTON_BOARD=SimpleNamespace(value=0))
    board_shim.BrainFlowError = FakeBoardError
    board_shim.BrainFlowInputParams = SimpleNamespace
    monkeypatch.setitem(sys.modules, "brainflow", brainflow)
    monkeypatch.setitem(sys.modules, "brainflow.board_shim", board_shim)
    return FakeBoard, boards


def test_live_read_preserves_capture_time_sequence_and_raw_acceleration(fake_brainflow, monkeypatch):
    _, boards = fake_brainflow
    clock = SimpleNamespace(time=1000.0, monotonic=10.0)
    monkeypatch.setattr(source, "time", SimpleNamespace(
        time=lambda: clock.time,
        monotonic=lambda: clock.monotonic,
    ))
    live = CytonSource("/dev/cu.test", (3, 7))
    with pytest.raises(RuntimeError, match="before reading"):
        live.read()
    with live as stream:
        board = boards[0]
        assert board.board_id == 0
        assert board.port == "/dev/cu.test"
        assert board.actions == ["prepare", "start"]
        assert stream.read() == []

        packets = np.zeros((24, 5))
        packets[0] = [254, 255, 0, 1, 2]
        packets[3] = [-4.25, 4.5, 5.0, 5.25, 1.5]
        packets[7] = [6.125, -3.0, 0.0, 5.5, -3.25]
        packets[9:12] = 2.0  # Processed accelerometer rows may contain held values.
        packets[12] = [0xC0, 0xC0, 0xC0, 0xC1, 0xC4]
        packets[13:19, 0] = [0xF8, 0x30, 0, 0, 0x1E, 0xA0]  # -2000, 0, 7840 counts.
        packets[13:19, 2:] = np.array([[0, 0, 0, 0x10, 0x1F, 0x40]]).T
        packets[22] = [1000.004, 1000.008, 1000.012, 1000.016, 1000.020]
        board.data = packets
        clock.monotonic = 13.0
        clock.time = 1003.0
        samples = stream.read()

        assert [sample.sequence for sample in samples] == [254, 255, 0, 1, 2]
        assert [sample.t for sample in samples] == pytest.approx(
            [10.004, 10.008, 10.012, 10.016, 10.020],
        )
        assert samples[0].eeg_uv == (-4.25, 6.125)
        assert samples[1].eeg_uv == (4.5, -3.0)
        assert samples[0].accel_g == pytest.approx((-0.25, 0, 0.98))
        assert samples[1].accel_g is None  # Empty AUX on a standard packet.
        assert samples[2].accel_g == pytest.approx((0, 0.002, 1.0))
        assert samples[3].accel_g is None  # Non-accelerometer packet mode.
        assert samples[4].accel_g is None  # Timestamped AUX is not the default full triple.
        assert stream.read() == []
    assert board.actions == ["prepare", "start", "release"]
    with pytest.raises(RuntimeError, match="before reading"):
        live.read()


@pytest.mark.parametrize("rate,fail_start,match,actions", [
    (125, False, "250 Hz", ["prepare", "release"]),
    (250, True, "start failed", ["prepare", "start", "release"]),
])
def test_open_failures_release_prepared_session(fake_brainflow, rate, fail_start, match, actions):
    fake_board, boards = fake_brainflow
    fake_board.rate = rate
    fake_board.fail_start = fail_start
    with pytest.raises(RuntimeError, match=match):
        with CytonSource("/dev/cu.test"):
            pytest.fail("session should not open")
    assert boards[0].actions == actions


def test_driver_read_error_is_reported_and_cleans_up(fake_brainflow):
    fake_board, boards = fake_brainflow
    fake_board.fail_read = True
    with pytest.raises(RuntimeError, match="could not read Cyton.*stream is gone"):
        with CytonSource("/dev/cu.test") as device:
            device.read()
    assert boards[0].actions == ["prepare", "start", "release"]


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
