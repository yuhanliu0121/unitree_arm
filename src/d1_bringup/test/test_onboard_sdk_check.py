from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from d1_bringup import onboard_sdk_check


def _configuration(**overrides):
    values = {
        "host": "192.168.123.100",
        "ssh_user": "ubuntu",
        "ssh_password": "123",
        "service_name": "d1-control.service",
        "executable_path": "/home/ubuntu/marm_code/build/d1_control_node",
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
                "d1_control_node version=0.1.0 protocol=2\n"
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
        "d1_control_node version=0.0.9 protocol=2",
        "d1_control_node version=0.1.0 protocol=1",
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
