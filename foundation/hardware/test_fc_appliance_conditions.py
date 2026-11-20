"""Boot-file semantics on a temporary image. No services or devices are opened."""
import json
from pathlib import Path

import pytest

from foundation.hardware import fc_appliance as fc


@pytest.fixture
def tree(tmp_path):
    def put(path, text):
        f = tmp_path / path.lstrip("/")
        f.parent.mkdir(parents=True, exist_ok=True)
        f.write_text(text)
        return f
    vendor = "/usr/lib/systemd/system/"
    for target in ("graphical.target", "multi-user.target", "rescue.target"):
        put(vendor + target, "[Unit]\nDescription=Fixture target\n")
    for unit in ("robot.service", "ssh.service"):
        put(vendor + unit, "[Service]\nExecStart=/bin/true\n")
    recorded = Path(__file__).resolve().parents[2] / "pupper_rebase/manifests/pupper-after/conf/units"
    put(vendor + "lightdm.service", (recorded / "display-manager.service").read_text())
    put("/home/pi/pupperv3-monorepo/pupper-rs/pupper-gui.service", (recorded / "pupper-gui.service").read_text())
    etc = tmp_path / fc.ETC_UNITS.lstrip("/")
    etc.mkdir(parents=True)
    (etc/"default.target").symlink_to(vendor + "graphical.target")
    (etc/"display-manager.service").symlink_to(vendor + "lightdm.service")
    (etc/"pupper-gui.service").symlink_to("/home/pi/pupperv3-monorepo/pupper-rs/pupper-gui.service")
    (etc/"graphical.target.wants").mkdir()
    (etc/"graphical.target.wants"/"pupper-gui.service").symlink_to("../pupper-gui.service")
    return fc.FcAppliance(str(tmp_path), systemctl=None), put, etc


@pytest.mark.parametrize("name", ["robot.service", "robot-agent.service", "service"])
def test_condition_reset_of_another_kind_in_all_effective_scopes(tree, name):
    app, put, etc = tree
    (etc/"robot-agent.service").symlink_to("/usr/lib/systemd/system/robot.service")
    app.enable()
    path = fc.ETC_UNITS + "/" + name + ".d/99-reset.conf"
    put(path, "[Unit]\nConditionKernelCommandLine=\nConditionKernelCommandLine=quiet\n")
    report = app.inspect(audit=False)
    assert report["mode"] == "degraded"
    assert any(path in reason for reason in report["problems"])


def test_ignore_shadowed_vendor_condition_reset(tree):
    app, put, _ = tree
    path = "/robot.service.d/99-condition.conf"
    put("/usr/lib/systemd/system" + path, "[Unit]\nConditionArchitecture=\n")
    put(fc.ETC_UNITS + path, "[Service]\nNice=1\n")
    app.enable()
    assert app.inspect(audit=False)["mode"] == "appliance"


@pytest.mark.parametrize("through_default", [False, True])
def test_alias_to_protected_ssh_is_refused_before_writing(tree, through_default):
    app, _, etc = tree
    name = "robot.service" if through_default else "console-backend.service"
    (etc/name).symlink_to("/usr/lib/systemd/system/ssh.service")
    with pytest.raises(fc.ApplianceError, match="protected ssh.service"):
        app.enable(() if through_default else (name,))
    assert app.fs.kind(fc.STATE) == "absent" and app.fs.kind(fc.MARKER) == "absent"
    assert app.fs.readlink(fc.DEFAULT_LINK).endswith("/graphical.target")


def test_replacement_marker_and_preexisting_scratch_file_survive(tree):
    app, put, _ = tree
    marker = put(fc.MARKER, "owner data\n")
    temp = put(fc.ETC_UNITS + "/" + fc.TMP_PREFIX + "default.target", "owner data\n")
    with pytest.raises(fc.ApplianceError, match="marker"):
        app.enable()
    assert app.normal()["errors"] and marker.read_text() == "owner data\n"
    marker.unlink()
    app.enable()
    marker.write_text("new owner data\n")
    assert app.inspect(audit=False)["mode"] == "degraded"
    assert app.normal()["errors"]
    assert marker.read_text() == "new owner data\n" and temp.read_text() == "owner data\n"


def test_repeated_enable_preserves_later_admin_target(tree):
    app, _, etc = tree
    app.enable()
    link = etc/"default.target"
    link.unlink()
    link.symlink_to("/usr/lib/systemd/system/rescue.target")
    with pytest.raises(fc.ApplianceError, match="changed since enable"):
        app.enable()
    app.normal()
    assert str(link.readlink()).endswith("/rescue.target")


def test_lingering_user_service_with_late_custom_executable_is_visible(tree):
    app, put, _ = tree
    put("/var/lib/systemd/linger/pi", "")
    put("/usr/lib/systemd/user/maintenance.service", "[Service]\nExecStart=/bin/true\n")
    link = put("/usr/lib/systemd/user/default.target.wants/maintenance.service", "")
    link.unlink()
    link.symlink_to("../maintenance.service")
    path = "/home/pi/.config/systemd/user/maintenance.service.d/99-launch.conf"
    put(path, "[Service]\nExecStart=\nExecStart=/home/pi/pupper/bin/start\n")
    items = app.inspect()["audit"]["items"]
    assert any(i["surface"] == "user-linger" for i in items)
    assert any(i["where"] == path and i["level"] == "review" for i in items)


def legacy_headless(tree, original):
    app, put, etc = tree
    old = "/usr/lib/systemd/system/multi-user.target"
    (etc / "default.target").unlink()
    (etc / "default.target").symlink_to(old)
    entries = []
    for unit in (*fc.CONFLICTING_UNITS, "lightdm.service", "pupper-gui.service", "glamor-test.service",
                 "rpi-display-backlight.service", "local-extra.service"):
        put(app.dropin_path(unit), fc.DROPIN_TEXT.decode())
        entries.append({"unit": unit, "dir_created": True, "adopted": False})
    put(fc.MARKER, fc.MARKER_TEXT.decode())
    put(fc.STATE, json.dumps({"tool": fc.TOOL, "schema": 1, "phase": "enabled", "dropins": entries,
                              "default_target": {"original": original, "applied": old}, "own_dir_created": True}))
    return app, put, etc


@pytest.mark.parametrize("original", [
    {"kind":"absent"}, {"kind":"symlink","target":"/usr/lib/systemd/system/graphical.target"},
    {"kind":"symlink","target":"/usr/lib/systemd/system/rescue.target"}])
def test_migrate_enrolled_headless_preserves_gui_motion_fences_and_original_rollback(tree, original):
    app, put, etc = legacy_headless(tree, original)
    local = put(fc.ETC_UNITS + "/pupper-gui.service.d/45-site.conf", "[Service]\nNice=1\n")
    assert app.enable()["result"] == "enabled"
    assert app.inspect()["mode"] == "appliance"
    assert app._boot_target()["effective"] == "graphical.target"
    assert all(app._dropin_status(unit) == "missing" for unit in fc.DISPLAY_UNITS)
    assert all(app._dropin_status(unit) == "ok" for unit in (*fc.CONFLICTING_UNITS, "local-extra.service"))
    assert app._own_marker() and local.exists()
    assert app.enable()["result"] == "already-enabled"
    assert app.normal()["errors"] == []
    assert app.fs.readlink(fc.DEFAULT_LINK) == original.get("target")
    assert local.exists() and (etc / "graphical.target.wants/pupper-gui.service").is_symlink()
    assert app.normal()["result"] == "already-normal"


@pytest.mark.parametrize("retry", [True, False])
def test_interrupted_migration_keeps_motion_fences_and_can_retry_or_restore(tree, monkeypatch, retry):
    original = {"kind":"symlink","target":"/usr/lib/systemd/system/rescue.target"}
    app, _, _ = legacy_headless(tree, original)
    def interrupted(*args):
        raise OSError("injected link failure")
    with monkeypatch.context() as m:
        m.setattr(app, "_symlink_atomic", interrupted)
        with pytest.raises(fc.ApplianceError, match="course fences retained"):
            app.enable()
    assert app._boot_target()["effective"] == "multi-user.target"
    assert app._own_marker() and app._dropin_status("robot.service") == "ok"
    if retry:
        assert app.enable()["result"] == "enabled"
        assert app._boot_target()["effective"] == "graphical.target"
    assert app.normal()["errors"] == [] and app.fs.readlink(fc.DEFAULT_LINK) == original["target"]


def test_reject_modified_display_file_without_withdrawing_motion_fence(tree):
    app, put, _ = legacy_headless(tree, {"kind":"absent"})
    custom = put(app.dropin_path("pupper-gui.service"), "# administrator changed this file\n")
    with pytest.raises(fc.ApplianceError, match="touchscreen drop-in was changed"):
        app.enable()
    assert app._own_marker() and app._boot_target()["effective"] == "multi-user.target"
    assert custom.read_text() == "# administrator changed this file\n"


@pytest.mark.parametrize("where,content", [
    ("/home/pi/.config/labwc/autostart", "/home/pi/start-lab\n"),
    ("/etc/xdg/autostart/lab.desktop", "[Desktop Entry]\nExec=/bin/sh -c 'ros2 launch old motor.launch.py'\n"),
    ("/home/pi/.config/wayfire.ini", "[autostart]\nlab=/home/pi/start-lab\n")])
def test_refuse_unfenced_graphical_launch_before_changing_enrolled_state(tree, where, content):
    app, put, _ = legacy_headless(tree, {"kind":"absent"})
    put("/home/pi/start-lab", "#!/bin/sh\nexec ros2 launch lab motor.launch.py\n")
    put(where, content)
    with pytest.raises(fc.ApplianceError, match="unfenced automatic robot launch"):
        app.enable()
    assert app._boot_target()["effective"] == "multi-user.target" and app._own_marker()


def test_missing_gui_activation_and_attempt_to_suppress_wifi_or_display_are_refused(tree):
    app, _, etc = tree
    for unit in ("pupper-gui.service", "lightdm.service", "comitup.service", "NetworkManager.service"):
        with pytest.raises(fc.ApplianceError, match="preserved"):
            app.enable((unit,))
    (etc / "graphical.target.wants/pupper-gui.service").unlink()
    with pytest.raises(fc.ApplianceError, match="not enabled for graphical.target"):
        app.enable()
    assert not app._own_marker()


def test_unfencing_the_installed_gui_checks_its_actual_service_wrapper(tree):
    app, put, _ = legacy_headless(tree, {"kind":"absent"})
    put("/home/pi/pupperv3-monorepo/pupper-rs/run_gui.sh", "#!/bin/sh\n/home/pi/start_all_services.sh &\n")
    with pytest.raises(fc.ApplianceError, match="unfenced automatic robot launch"):
        app.enable()
    assert app._boot_target()["effective"] == "multi-user.target"
