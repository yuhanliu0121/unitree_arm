from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def load_joint_limit_overrides(profile_path, arm_serial):
    """Return explicit per-unit joint-limit overrides, or an empty mapping."""
    serial = str(arm_serial).strip()
    if not serial:
        return {}
    document = yaml.safe_load(Path(profile_path).read_text(encoding="utf-8")) or {}
    profile = document.get("hardware_profiles", {}).get(serial)
    if profile is None:
        return {}
    overrides = profile.get("joint_limits", {})
    result = {}
    for joint_name, limits in overrides.items():
        parsed = {}
        for bound in ("lower", "upper"):
            if bound in limits:
                parsed[bound] = float(limits[bound])
        if parsed:
            result[str(joint_name)] = parsed
    return result


def apply_joint_limit_overrides(robot_description, overrides):
    """Apply validated limit overrides to an in-memory URDF string."""
    if not overrides:
        return robot_description
    root = ET.fromstring(robot_description)
    joints = {joint.attrib.get("name"): joint for joint in root.findall("joint")}
    for joint_name, bounds in overrides.items():
        joint = joints.get(joint_name)
        if joint is None:
            raise ValueError(f"hardware profile references unknown joint: {joint_name}")
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"joint has no limit element: {joint_name}")
        lower = float(bounds.get("lower", limit.attrib["lower"]))
        upper = float(bounds.get("upper", limit.attrib["upper"]))
        if lower >= upper:
            raise ValueError(f"invalid limits for {joint_name}: {lower} >= {upper}")
        limit.attrib["lower"] = f"{lower:.15g}"
        limit.attrib["upper"] = f"{upper:.15g}"
    return ET.tostring(root, encoding="unicode")


def joint_limits_from_urdf(robot_description, joint_names):
    """Extract ordered lower/upper pairs from an effective URDF string."""
    root = ET.fromstring(robot_description)
    joints = {joint.attrib.get("name"): joint for joint in root.findall("joint")}
    result = []
    for joint_name in joint_names:
        joint = joints.get(joint_name)
        if joint is None or joint.find("limit") is None:
            raise ValueError(f"missing joint limit: {joint_name}")
        limit = joint.find("limit")
        result.append((float(limit.attrib["lower"]), float(limit.attrib["upper"])))
    return result
