"""One-shot identity and systemd-state check for the D1 onboard executor."""

import argparse
import os
import re
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

from d1_bringup.local_protocol import PACKET_VERSION


VERSION_PATTERN = re.compile(
    r"^D1_CONTROL_VERSION=(?P<version>[^\s]+)$", re.MULTILINE
)
PROTOCOL_PATTERN = re.compile(
    r"^D1_CONTROL_PROTOCOL=(?P<protocol>\d+)$", re.MULTILINE
)


@dataclass(frozen=True)
class OnboardCheckConfig:
    host: str
    ssh_user: str
    ssh_password: str
    service_name: str
    executable_path: str
    version_manifest_path: str
    expected_version: str
    connect_timeout_s: int


def load_check_config(config_path: Path) -> OnboardCheckConfig:
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    arm = config.get("arm", {})
    onboard = arm.get("onboard_control", {})
    required = {
        "host",
        "ssh_user",
        "service_name",
        "executable_path",
        "version_manifest_path",
        "expected_version",
    }
    missing = sorted(
        name for name in required if not str(onboard.get(name, "")).strip()
    )
    if missing:
        raise ValueError(
            "arm.onboard_control is missing required field(s): " + ", ".join(missing)
        )
    timeout = onboard.get("connect_timeout_s", 3)
    if isinstance(timeout, bool) or not isinstance(timeout, int) or timeout < 1:
        raise ValueError(
            "arm.onboard_control.connect_timeout_s must be a positive integer"
        )
    return OnboardCheckConfig(
        host=str(onboard["host"]).strip(),
        ssh_user=str(onboard["ssh_user"]).strip(),
        ssh_password=str(onboard.get("ssh_password", "")),
        service_name=str(onboard["service_name"]).strip(),
        executable_path=str(onboard["executable_path"]).strip(),
        version_manifest_path=str(onboard["version_manifest_path"]).strip(),
        expected_version=str(onboard["expected_version"]).strip(),
        connect_timeout_s=timeout,
    )


def _remote_command(config: OnboardCheckConfig) -> str:
    service = shlex.quote(config.service_name)
    executable = shlex.quote(config.executable_path)
    manifest = shlex.quote(config.version_manifest_path)
    return (
        f"state=$(systemctl is-active {service} 2>&1 || true); "
        'printf "SERVICE_STATE=%s\\n" "$state"; '
        f'[ "$state" = active ] || exit 20; '
        f'[ -x {executable} ] || '
        f'{{ printf "EXECUTOR_MISSING=%s\\n" {executable}; exit 21; }}; '
        f'[ -r {manifest} ] || '
        f'{{ printf "MANIFEST_MISSING=%s\\n" {manifest}; exit 22; }}; '
        f"cat {manifest}"
    )


def _ssh_command(config: OnboardCheckConfig, use_password=False):
    ssh = shutil.which("ssh")
    if not ssh:
        raise RuntimeError("ssh is not installed")
    command = [
        ssh,
        "-o", "ConnectTimeout=" + str(config.connect_timeout_s),
        "-o", "ConnectionAttempts=1",
        "-o", "StrictHostKeyChecking=accept-new",
    ]
    environment = os.environ.copy()
    if use_password:
        sshpass = shutil.which("sshpass")
        if not sshpass:
            raise RuntimeError(
                "SSH key authentication failed and sshpass is not installed for the "
                "configured arm.onboard_control.ssh_password fallback"
            )
        command = [sshpass, "-e", *command, "-o", "BatchMode=no"]
        environment["SSHPASS"] = config.ssh_password
    else:
        command.extend(["-o", "BatchMode=yes"])
    command.extend([
        f"{config.ssh_user}@{config.host}",
        _remote_command(config),
    ])
    return command, environment


def _run_remote_check(config: OnboardCheckConfig, use_password=False):
    command, environment = _ssh_command(config, use_password=use_password)
    return subprocess.run(
        command,
        env=environment,
        capture_output=True,
        text=True,
        timeout=config.connect_timeout_s + 3,
        check=False,
    )


def check_onboard_sdk(config: OnboardCheckConfig) -> str:
    result = _run_remote_check(config)
    if result.returncode == 255 and config.ssh_password:
        result = _run_remote_check(config, use_password=True)
    output = "\n".join(
        part.strip() for part in (result.stdout, result.stderr) if part.strip()
    )
    service_match = re.search(r"^SERVICE_STATE=(.*)$", output, re.MULTILINE)
    if result.returncode == 255:
        raise RuntimeError(
            f"cannot reach/authenticate D1 board {config.ssh_user}@{config.host}: {output}"
        )
    if service_match and service_match.group(1).strip() != "active":
        raise RuntimeError(
            f"onboard service {config.service_name} is "
            f"{service_match.group(1).strip()!r}, expected 'active'"
        )
    if result.returncode != 0:
        raise RuntimeError(
            f"onboard SDK check command failed (exit {result.returncode}): {output}"
        )
    version_match = VERSION_PATTERN.search(result.stdout)
    protocol_match = PROTOCOL_PATTERN.search(result.stdout)
    if not version_match or not protocol_match:
        raise RuntimeError(
            "onboard release manifest is missing or invalid; the board may contain "
            f"an old or incomplete deployment: {output}"
        )
    actual_version = version_match.group("version")
    actual_protocol = int(protocol_match.group("protocol"))
    if actual_version != config.expected_version or actual_protocol != PACKET_VERSION:
        raise RuntimeError(
            "onboard SDK version mismatch: "
            f"expected version={config.expected_version} protocol={PACKET_VERSION}, "
            f"got version={actual_version} protocol={actual_protocol}"
        )
    return (
        f"service={config.service_name} active; version={actual_version}; "
        f"protocol={actual_protocol}"
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        config = load_check_config(args.config.resolve())
        detail = check_onboard_sdk(config)
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f"D1 ONBOARD SDK CHECK FAILED: {error}", file=sys.stderr)
        return 2
    print(f"D1 ONBOARD SDK CHECK PASSED: {detail}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
