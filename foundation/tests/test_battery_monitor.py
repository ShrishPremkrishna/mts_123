"""Battery monitor behavior with fake I2C, time, wall and shutdown commands only."""
import builtins
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

SOURCE = Path(__file__).resolve().parents[2] / 'pupperv3-monorepo/robot/utils/check_batt_voltage.py'


@pytest.fixture
def battery(monkeypatch):
    original_import = builtins.__import__

    def refuse_hardware(name, *args, **kwargs):
        if name in {'board', 'busio'}:
            raise AssertionError('Importing the utility must not load hardware bindings')
        return original_import(name, *args, **kwargs)

    spec = importlib.util.spec_from_file_location('battery_under_test', SOURCE)
    module = importlib.util.module_from_spec(spec)
    with monkeypatch.context() as guarded:
        guarded.setattr(builtins, '__import__', refuse_hardware)
        spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'broadcast_message', lambda message: None)
    # Any unexpected direct shell path must fail instead of touching this host.
    monkeypatch.setattr(module.os, 'system', lambda *args: pytest.fail('Unexpected shell command'))
    monkeypatch.setattr(module, 'subprocess', SimpleNamespace(run=lambda *a, **k: pytest.fail('Unexpected command')))
    return module


@pytest.mark.parametrize('volts,percent', [
    (21., 100.), (20.25, 100.), (19.125, 83.), (18., 66.),
    (17.5, 49.5), (17., 33.), (16.75, 16.5), (16.5, 0.), (15.5, 0.),
])
def test_calibration_anchors_and_intervals(battery, volts, percent):
    assert battery.battery_percentage(volts) == pytest.approx(percent, abs=1e-12)


@pytest.mark.parametrize('anchor', [16.5, 17., 18., 20.25])
def test_percentage_is_monotone_across_every_boundary(battery, anchor):
    values = [battery.battery_percentage(anchor + d) for d in [-1e-6, 0., 1e-6]]
    assert values == sorted(values)
    assert 0 <= values[0] <= values[-1] <= 100
    assert values[-1] - values[0] < .001


class EndPolls(Exception):
    pass


def run_service(battery, monkeypatch, readings, shutdown_results=()):
    """Exercise the real CLI branch; stop before the next bounded fake poll."""
    samples = iter(readings)
    outcomes = iter(shutdown_results)
    trace = []
    warnings = []
    commands = []
    polls = 0
    bus = object()
    monkeypatch.setitem(sys.modules, 'board', SimpleNamespace(SCL='scl', SDA='sda'))
    monkeypatch.setitem(sys.modules, 'busio', SimpleNamespace(I2C=lambda *pins: bus))
    monkeypatch.setattr(sys, 'argv', [str(SOURCE), '--service_mode'])

    def sleep(seconds):
        nonlocal polls
        assert seconds == 10
        if polls == len(readings):
            raise EndPolls
        polls += 1
        trace.append('sleep')

    def sample(actual_bus):
        assert actual_bus is bus
        trace.append('read')
        return next(samples)

    def shutdown(args, *, check):
        assert args == ['sudo', 'shutdown', '-h', '+1'] and check is False
        commands.append(args)
        outcome = next(outcomes)
        if isinstance(outcome, Exception):
            raise outcome
        return SimpleNamespace(returncode=outcome)

    monkeypatch.setattr(battery, 'time', SimpleNamespace(sleep=sleep))
    monkeypatch.setattr(battery, 'get_battery_voltage', sample)
    monkeypatch.setattr(battery, 'broadcast_message', warnings.append)
    monkeypatch.setattr(battery, 'subprocess', SimpleNamespace(run=shutdown))
    with pytest.raises(EndPolls):
        battery.main()
    assert trace == ['sleep', 'read'] * len(readings)
    return warnings, commands


def test_service_refreshes_each_poll_and_only_schedules_once(battery, monkeypatch):
    warnings, commands = run_service(battery, monkeypatch, [19., 16., 15., 14.5, 19., 15.], [0])
    assert len(commands) == 1
    assert not any('19.00' in message for message in warnings)
    assert any('16.00' in message for message in warnings)
    assert any('14.50' in message for message in warnings)
    assert sum('Shutdown scheduled' in message for message in warnings) == 1


def test_wall_power_to_battery_and_exact_thresholds(battery, monkeypatch):
    warnings, commands = run_service(battery, monkeypatch, [0., 4.99, 16.5, 15.5, 15.49], [0])
    assert len(commands) == 1
    assert not any('0.00' in message or '4.99' in message or '16.50' in message for message in warnings)
    assert any('15.50' in message for message in warnings)
    assert all('15.49' in message for message in warnings if 'Shutdown scheduled' in message)


@pytest.mark.parametrize('failed', [1, 127, OSError('fake missing shutdown')])
def test_failed_schedule_retries_only_on_later_low_reading(battery, monkeypatch, capsys, failed):
    warnings, commands = run_service(battery, monkeypatch, [15., 19., 15., 15.], [failed, 0])
    assert len(commands) == 2
    assert sum('Shutdown scheduled' in message for message in warnings) == 1
    assert 'Failed to schedule battery shutdown:' in capsys.readouterr().err


@pytest.mark.parametrize('flags,expected', [
    ([], 'Battery:\t66%\t18.00V\n'), (['--percentage_only'], '66\n'),
])
def test_single_read_cli_keeps_output_contract(battery, monkeypatch, capsys, flags, expected):
    bus = object()
    reads = []
    monkeypatch.setitem(sys.modules, 'board', SimpleNamespace(SCL='scl', SDA='sda'))
    monkeypatch.setitem(sys.modules, 'busio', SimpleNamespace(I2C=lambda *pins: bus))
    monkeypatch.setattr(sys, 'argv', [str(SOURCE), *flags])
    monkeypatch.setattr(battery, 'get_battery_voltage', lambda actual: reads.append(actual) or 18.)
    battery.main()
    assert reads == [bus]
    assert capsys.readouterr().out == expected


def test_i2c_register_scale_and_unlock_unchanged(battery):
    trace = []

    def read(address, result):
        trace.append(('read', address, len(result)))
        result[:] = b'\x38\x94'

    bus = SimpleNamespace(
        try_lock=lambda: True,
        writeto=lambda address, data: trace.append(('write', address, data)),
        readfrom_into=read,
        unlock=lambda: trace.append(('unlock',)),
    )
    assert battery.get_battery_voltage(bus) == pytest.approx(0x3894 * .0013125)
    assert trace == [('write', 0x48, b'\x8c'), ('read', 0x48, 2), ('unlock',)]
