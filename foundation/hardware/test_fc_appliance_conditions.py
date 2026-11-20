"""Boot-file semantics on a temporary image. No services or devices are opened."""
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
    etc = tmp_path / fc.ETC_UNITS.lstrip("/")
    etc.mkdir(parents=True)
    (etc/"default.target").symlink_to(vendor + "graphical.target")
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
