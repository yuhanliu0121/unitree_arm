#!/usr/bin/env python3
"""Generate, validate, and materialize host-specific D1 deployment config."""

from __future__ import annotations

import argparse
import ipaddress
import json
import math
import os
import platform
import re
import shutil
import subprocess
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


D1_HOST = "192.168.123.100"
D1_NETWORK = ipaddress.ip_network("192.168.123.0/24")
SUPPORTED_FRAME_RATES = {6, 15, 30}


def deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read YAML {path}: {error}") from error
    if not isinstance(document, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return document


def materialize(defaults_path: Path, local_path: Path) -> dict[str, Any]:
    return deep_merge(load_yaml(defaults_path), load_yaml(local_path))


def detect_platform(requested: str) -> tuple[str, str]:
    architecture = platform.machine().lower()
    normalized_arch = {
        "x86_64": "amd64",
        "amd64": "amd64",
        "aarch64": "arm64",
        "arm64": "arm64",
    }.get(architecture, architecture)

    if requested != "auto":
        expected = "amd64" if requested == "amd64" else "arm64"
        if normalized_arch != expected:
            raise ValueError(
                f"--platform {requested} requires {expected}, but uname reports {architecture}"
            )
        return requested, normalized_arch

    if normalized_arch == "amd64":
        return "amd64", normalized_arch
    if normalized_arch == "arm64":
        model = ""
        model_path = Path("/proc/device-tree/model")
        if model_path.is_file():
            model = model_path.read_bytes().replace(b"\x00", b"").decode(
                "utf-8", errors="replace"
            )
        if (
            Path("/etc/nv_tegra_release").exists()
            or "nvidia" in model.lower()
            or "jetson" in model.lower()
        ):
            return "go2", normalized_arch
        raise ValueError(
            "ARM64 host is not identifiable as an NVIDIA Jetson/Go2 platform; "
            "rerun with --platform go2 only after confirming the hardware"
        )
    raise ValueError(f"unsupported host architecture: {architecture}")


def _run(command: list[str], timeout: float = 3.0) -> str:
    try:
        completed = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired):
        return ""
    return completed.stdout if completed.returncode == 0 else ""


def detect_d1_interfaces() -> list[str]:
    candidates: list[str] = []
    route = _run(["ip", "-o", "route", "get", D1_HOST])
    route_dev = re.search(r"\bdev\s+(\S+)", route)
    route_src = re.search(r"\bsrc\s+(\S+)", route)
    if route_dev and route_src:
        try:
            if ipaddress.ip_address(route_src.group(1)) in D1_NETWORK:
                candidates.append(route_dev.group(1))
        except ValueError:
            pass

    addresses = _run(["ip", "-o", "-4", "address", "show"])
    for line in addresses.splitlines():
        fields = line.split()
        if len(fields) < 4:
            continue
        interface = fields[1]
        try:
            inet_index = fields.index("inet")
            address = ipaddress.ip_interface(fields[inet_index + 1]).ip
        except (ValueError, IndexError):
            continue
        if address in D1_NETWORK and interface not in candidates:
            candidates.append(interface)
    return candidates


def parse_realsense_device_list(output: str) -> list[tuple[str, str]]:
    devices: list[tuple[str, str]] = []
    for line in output.splitlines():
        columns = re.split(r"\s{2,}", line.strip())
        if len(columns) < 3 or columns[0] == "Device Name":
            continue
        product, serial = columns[0], columns[1]
        if "realsense" not in product.lower() or not serial:
            continue
        if all(existing_serial != serial for existing_serial, _ in devices):
            devices.append((serial, product))
    return devices


def detect_realsense_devices() -> list[tuple[str, str]]:
    # The USB descriptor serial exposed by sysfs is not necessarily the ASIC
    # serial accepted by librealsense's serial_no selector. Always obtain the
    # identity through librealsense itself.
    supplied_output = os.environ.get("D1_REALSENSE_DEVICE_LIST", "")
    if supplied_output:
        return parse_realsense_device_list(supplied_output)

    candidates = [
        shutil.which("rs-enumerate-devices"),
        "/opt/ros/humble/bin/rs-enumerate-devices",
    ]
    for executable in candidates:
        if not executable or not Path(executable).is_file():
            continue
        devices = parse_realsense_device_list(
            _run([executable, "-s"], timeout=15.0)
        )
        if devices:
            return devices
    return []


def _yaml_string(value: str) -> str:
    # A JSON string is also a valid YAML scalar and never emits YAML's `...`
    # document terminator for plain values such as interface names.
    return json.dumps(value)


def render_local_config(
    deployment_platform: str,
    network_interface: str,
    camera_serial: str,
) -> str:
    ros_domain = "31" if deployment_platform == "amd64" else "null"
    go2_translation = "[0.0, 0.0, 0.0]" if deployment_platform == "amd64" else "[]"
    go2_quaternion = "[0.0, 0.0, 0.0, 1.0]" if deployment_platform == "amd64" else "[]"
    return f"""# Generated by deploy/configure_d1_manipulation.sh.
# Edit only the fields marked REQUIRED, then run the script with --validate.
# This file is machine-local and intentionally ignored by Git.
deployment:
  platform: {deployment_platform}
  # REQUIRED on Go2: use the same ROS Domain as navigation and task nodes.
  ros_domain_id: {ros_domain}

perception:
  # Frozen model for the real wood/plush/plastic objects. For paper-object
  # development, use paper_objects_dev_best.pt in the same directory.
  model_path: ../../../runtime/perception_runtime_v1/weights/mcislab_trash_collect.pt

arm:
  # Optional hardware-profile selector. Empty uses the repository's base URDF.
  # A non-empty serial loads matching per-arm overrides from
  # d1_description/config/hardware_profiles.yaml; add a profile there first.
  serial_no: ""
  # Auto-detected when exactly one interface is on 192.168.123.0/24.
  # REQUIRED if automatic detection leaves this empty.
  network_interface: {_yaml_string(network_interface)}

wrist_camera:
  driver_location: local
  # Auto-detected only when exactly one RealSense is attached.
  # REQUIRED if automatic detection leaves this empty.
  serial_no: {_yaml_string(camera_serial)}
  frame_rate_hz: 15
  # Default measured Link6 -> wrist_camera_link hand-eye transform. Recalibrate
  # only if the camera, bracket, or their relative installation changes.
  link6_to_camera_link:
    translation_xyz_m: [-0.113314288587, 0.018109307512, -0.051022947448]
    quaternion_xyzw: [-0.015349552858, -0.613749035851, 0.010264332423, 0.789285218219]

site:
  # REQUIRED on Go2; standalone amd64 development uses identity.
  go2_base_to_arm_base:
    translation_xyz_m: {go2_translation}
    quaternion_xyzw: {go2_quaternion}
"""


def _finite_vector(value: Any, length: int, field: str, errors: list[str]) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        errors.append(f"{field} must contain {length} numbers")
        return []
    try:
        vector = [float(item) for item in value]
    except (TypeError, ValueError):
        errors.append(f"{field} must contain only numbers")
        return []
    if not all(math.isfinite(item) for item in vector):
        errors.append(f"{field} must contain only finite numbers")
        return []
    return vector


def validate_config(config: dict[str, Any], local_path: Path) -> list[str]:
    errors: list[str] = []
    deployment = config.get("deployment", {})
    deployment_platform = str(deployment.get("platform", "")).strip()
    if deployment_platform not in {"amd64", "go2"}:
        errors.append("deployment.platform must be amd64 or go2")
    domain = deployment.get("ros_domain_id")
    if isinstance(domain, bool) or not isinstance(domain, int) or not 0 <= domain <= 232:
        errors.append("deployment.ros_domain_id must be an integer in [0, 232]")

    configured_model = str(config.get("perception", {}).get("model_path", "")).strip()
    if not configured_model:
        errors.append("perception.model_path is required")
    else:
        model_path = Path(configured_model).expanduser()
        if not model_path.is_absolute():
            model_path = local_path.resolve().parent / model_path
        if not model_path.is_file():
            errors.append(f"perception.model_path is not a file: {model_path.resolve()}")

    arm = config.get("arm", {})
    arm_serial = str(arm.get("serial_no", "")).strip()
    if arm_serial:
        profile_path = (
            Path(__file__).resolve().parents[2]
            / "src"
            / "d1_description"
            / "config"
            / "hardware_profiles.yaml"
        )
        try:
            profiles = load_yaml(profile_path).get("hardware_profiles", {})
        except ValueError as error:
            errors.append(str(error))
            profiles = {}
        if arm_serial not in profiles:
            errors.append(
                "arm.serial_no has no matching hardware profile; leave it empty "
                f"to use the base URDF or add a validated profile for {arm_serial}"
            )
    interface = str(arm.get("network_interface", "")).strip()
    if not interface:
        errors.append("arm.network_interface is required; connect/configure the D1 network first")
    elif not Path("/sys/class/net", interface).exists():
        errors.append(f"arm.network_interface does not exist on this host: {interface}")

    camera = config.get("wrist_camera", {})
    if str(camera.get("driver_location", "")).strip() not in {"local", "remote"}:
        errors.append("wrist_camera.driver_location must be local or remote")
    if not str(camera.get("serial_no", "")).strip():
        errors.append("wrist_camera.serial_no is required")
    if camera.get("frame_rate_hz") not in SUPPORTED_FRAME_RATES:
        errors.append("wrist_camera.frame_rate_hz must be one of [6, 15, 30]")
    hand_eye = camera.get("link6_to_camera_link", {})
    _finite_vector(
        hand_eye.get("translation_xyz_m"),
        3,
        "wrist_camera.link6_to_camera_link.translation_xyz_m",
        errors,
    )
    hand_eye_q = _finite_vector(
        hand_eye.get("quaternion_xyzw"),
        4,
        "wrist_camera.link6_to_camera_link.quaternion_xyzw",
        errors,
    )
    if hand_eye_q and math.sqrt(sum(item * item for item in hand_eye_q)) < 1e-6:
        errors.append("wrist_camera.link6_to_camera_link quaternion cannot be zero")

    mount = config.get("site", {}).get("go2_base_to_arm_base", {})
    mount_t = _finite_vector(
        mount.get("translation_xyz_m"),
        3,
        "site.go2_base_to_arm_base.translation_xyz_m",
        errors,
    )
    mount_q = _finite_vector(
        mount.get("quaternion_xyzw"),
        4,
        "site.go2_base_to_arm_base.quaternion_xyzw",
        errors,
    )
    if mount_q and math.sqrt(sum(item * item for item in mount_q)) < 1e-6:
        errors.append("site.go2_base_to_arm_base quaternion cannot be zero")
    if deployment_platform == "go2" and mount_t and mount_q:
        identity_translation = max(abs(item) for item in mount_t) < 1e-9
        identity_quaternion = (
            max(abs(mount_q[index]) for index in range(3)) < 1e-9
            and abs(mount_q[3] - 1.0) < 1e-9
        )
        if identity_translation and identity_quaternion:
            errors.append(
                "site.go2_base_to_arm_base must be measured; identity is not accepted on Go2"
            )
    return errors


def validate_attached_hardware(config: dict[str, Any]) -> list[str]:
    camera = config.get("wrist_camera", {})
    if str(camera.get("driver_location", "")).strip() != "local":
        return []

    configured_serial = str(camera.get("serial_no", "")).strip()
    devices = detect_realsense_devices()
    if not devices:
        return [
            "no RealSense device was detected by librealsense; connect the wrist "
            "camera before validating a local-camera deployment"
        ]
    detected_serials = [serial for serial, _ in devices]
    if configured_serial and configured_serial not in detected_serials:
        return [
            "wrist_camera.serial_no does not match an attached RealSense: "
            f"configured={configured_serial}, detected={', '.join(detected_serials)}"
        ]
    return []


def default_paths() -> tuple[Path, Path]:
    workspace = Path(__file__).resolve().parents[2]
    config_dir = workspace / "src" / "d1_bringup" / "config"
    return config_dir / "real_machine.yaml", config_dir / "real_machine.local.yaml"


def parse_arguments() -> argparse.Namespace:
    defaults_path, local_path = default_paths()
    parser = argparse.ArgumentParser(
        prog="./deploy/configure_d1_manipulation.sh",
        description="Detect deployment parameters and create the local D1 configuration overlay."
    )
    parser.add_argument("--platform", choices=("auto", "amd64", "go2"), default="auto")
    parser.add_argument("--output", type=Path, default=local_path)
    parser.add_argument("--config", type=Path, default=local_path)
    parser.add_argument("--defaults", type=Path, default=defaults_path)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--validate", action="store_true")
    parser.add_argument("--materialize", type=Path, metavar="OUTPUT")
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if args.validate or args.materialize:
        if not args.defaults.is_file():
            print(f"ERROR: default config is missing: {args.defaults}", file=sys.stderr)
            return 2
        if not args.config.is_file():
            print(f"ERROR: local config is missing: {args.config}", file=sys.stderr)
            return 2
        try:
            config = materialize(args.defaults, args.config)
            errors = validate_config(config, args.config)
            errors.extend(validate_attached_hardware(config))
            detected_platform, _ = detect_platform(args.platform)
            configured_platform = str(
                config.get("deployment", {}).get("platform", "")
            ).strip()
            if configured_platform != detected_platform:
                errors.append(
                    "deployment.platform does not match this host: "
                    f"configured={configured_platform}, detected={detected_platform}"
                )
        except ValueError as error:
            print(f"ERROR: {error}", file=sys.stderr)
            return 2
        if errors:
            print("Configuration is incomplete:", file=sys.stderr)
            for error in errors:
                print(f"  - {error}", file=sys.stderr)
            return 2
        if args.materialize:
            args.materialize.parent.mkdir(parents=True, exist_ok=True)
            args.materialize.write_text(
                yaml.safe_dump(config, sort_keys=False), encoding="utf-8"
            )
            print(f"Effective configuration: {args.materialize}")
        else:
            print("Configuration: OK")
        return 0

    if args.output.exists() and not args.force:
        print(f"ERROR: local config already exists: {args.output}", file=sys.stderr)
        print("Use --validate to check it, or --force to regenerate it.", file=sys.stderr)
        return 2
    try:
        deployment_platform, architecture = detect_platform(args.platform)
    except ValueError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    interfaces = detect_d1_interfaces()
    cameras = detect_realsense_devices()
    network_interface = interfaces[0] if len(interfaces) == 1 else ""
    camera_serial = cameras[0][0] if len(cameras) == 1 else ""

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        render_local_config(deployment_platform, network_interface, camera_serial),
        encoding="utf-8",
    )
    print(f"[OK] architecture: {architecture}")
    print(f"[OK] deployment platform: {deployment_platform}")
    if network_interface:
        print(f"[OK] D1 network interface: {network_interface}")
    elif interfaces:
        print(f"[NEED INPUT] multiple D1-network interfaces detected: {', '.join(interfaces)}")
    else:
        print(f"[NEED INPUT] no interface on {D1_NETWORK} was detected")
    if camera_serial:
        print(f"[OK] wrist RealSense candidate: {camera_serial} ({cameras[0][1]})")
    elif cameras:
        print("[NEED INPUT] multiple RealSense devices detected:")
        for serial, product in cameras:
            print(f"  - {serial}: {product}")
    else:
        print("[NEED INPUT] no attached RealSense device was detected")
    print(f"[CREATED] {args.output}")
    print("Fill the REQUIRED fields, then run:")
    print("  ./deploy/configure_d1_manipulation.sh --validate")
    return 0


if __name__ == "__main__":
    sys.exit(main())
