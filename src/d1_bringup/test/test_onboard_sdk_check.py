from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from d1_bringup import onboard_sdk_check
from d1_bringup.local_protocol import PACKET_VERSION


PACKAGE_PATH = Path(__file__).resolve().parents[1]
WORKSPACE_PATH = PACKAGE_PATH.parents[1]


def _configuration(**overrides):
    values = {
        "host": "192.168.123.100",
        "ssh_user": "ubuntu",
        "ssh_password": "123",
        "service_name": "d1-control.service",
        "executable_path": "/home/ubuntu/marm_code/build/d1_control_node",
        "version_manifest_path": "/home/ubuntu/marm_code/build/d1-control-release.env",
        "expected_version": "0.1.0",
        "connect_timeout_s": 3,
    }
    values.update(overrides)
    return onboard_sdk_check.OnboardCheckConfig(**values)


def test_load_check_config_requires_deployment_identity(tmp_path):
    path = tmp_path / "real.yaml"
    path.write_text(yaml.safe_dump({"arm": {"onboard_control": {"host": "board"}}}))

    with pytest.raises(ValueError, match="expected_version"):
        onboard_sdk_check.load_check_config(path)


def test_checked_in_release_identity_matches_host_and_real_config():
    release_path = (
        WORKSPACE_PATH
        / "third_party/unitree-d1-control/deploy/d1-control-release.env"
    )
    release = dict(
        line.split("=", 1)
        for line in release_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    )
    real_config = onboard_sdk_check.load_check_config(
        PACKAGE_PATH / "config/real_machine.yaml"
    )

    assert release["D1_CONTROL_VERSION"] == real_config.expected_version
    assert int(release["D1_CONTROL_PROTOCOL"]) == PACKET_VERSION


def test_active_matching_executor_passes(monkeypatch):
    monkeypatch.setattr(
        onboard_sdk_check.shutil, "which", lambda name: f"/usr/bin/{name}"
    )
    monkeypatch.setattr(
        onboard_sdk_check.subprocess,
        "run",
        lambda *args, **kwargs: SimpleNamespace(
            returncode=0,
            stdout=(
                "SERVICE_STATE=active\n"
                "D1_CONTROL_VERSION=0.1.0\n"
                "D1_CONTROL_PROTOCOL=2\n"
            ),
            stderr="",
        ),
    )

    result = onboard_sdk_check.check_onboard_sdk(_configuration())

    assert "service=d1-control.service active" in result
    assert "protocol=2" in result


def test_inactive_service_fails_with_specific_reason(monkeypatch):
    monkeypatch.setattr(
        onboard_sdk_check.shutil, "which", lambda name: f"/usr/bin/{name}"
    )
    monkeypatch.setattr(
        onboard_sdk_check.subprocess,
        "run",
        lambda *args, **kwargs: SimpleNamespace(
            returncode=20,
            stdout="SERVICE_STATE=inactive\n",
            stderr="",
        ),
    )

    with pytest.raises(RuntimeError, match="is 'inactive'"):
        onboard_sdk_check.check_onboard_sdk(_configuration())


@pytest.mark.parametrize(
    "identity",
    [
        "D1_CONTROL_VERSION=0.0.9\nD1_CONTROL_PROTOCOL=2",
        "D1_CONTROL_VERSION=0.1.0\nD1_CONTROL_PROTOCOL=1",
    ],
)
def test_version_or_protocol_mismatch_fails(monkeypatch, identity):
    monkeypatch.setattr(
        onboard_sdk_check.shutil, "which", lambda name: f"/usr/bin/{name}"
    )
    monkeypatch.setattr(
        onboard_sdk_check.subprocess,
        "run",
        lambda *args, **kwargs: SimpleNamespace(
            returncode=0,
            stdout=f"SERVICE_STATE=active\n{identity}\n",
            stderr="",
        ),
    )

    with pytest.raises(RuntimeError, match="version mismatch"):
        onboard_sdk_check.check_onboard_sdk(_configuration())


def test_configured_password_requires_sshpass(monkeypatch):
    monkeypatch.setattr(
        onboard_sdk_check.shutil,
        "which",
        lambda name: "/usr/bin/ssh" if name == "ssh" else None,
    )

    with pytest.raises(RuntimeError, match="sshpass is required"):
        onboard_sdk_check.check_onboard_sdk(_configuration())
