from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


_COLLISIONS = {
    "base_link": (
        "cylinder",
        "0 0 0.0289",
        "0 0 0",
        {"radius": "0.059", "length": "0.0578"},
    ),
    "Link1": (
        "cylinder",
        "-0.00175 0 0.03465",
        "0 0 0",
        {"radius": "0.052", "length": "0.0693"},
    ),
    "Link2": (
        "box",
        "0.0009 0.13965 -0.0271",
        "0 0 0",
        {"size": "0.0422 0.2853 0.061"},
    ),
    "Link3": (
        "box",
        "0.02295 0.0345 -0.026",
        "0 0 0",
        {"size": "0.0775 0.075 0.064"},
    ),
    "Link4": (
        "box",
        "0.00045 0.0006 0.075",
        "0 0 0",
        {"size": "0.0251 0.0526 0.151"},
    ),
    "Link5": (
        "box",
        "0.04175 0.0086 -0.02509",
        "0 0 0",
        {"size": "0.0895 0.058 0.05418"},
    ),
    "Link6": (
        "cylinder",
        "-0.0093 0 0.0378",
        "1.5708 0 0",
        {"radius": "0.0383", "length": "0.132"},
    ),
    "left_finger": (
        "box",
        "0.024 0.006 0.0105",
        "0 0 0",
        {"size": "0.062 0.026 0.021"},
    ),
    "right_finger": (
        "box",
        "0.024 -0.006 0.0105",
        "0 0 0",
        {"size": "0.062 0.026 0.021"},
    ),
}


def load_planning_description(urdf_path: Path) -> str:
    """Return shared kinematics with planning-only primitive collisions."""
    root = ET.fromstring(Path(urdf_path).read_text(encoding="utf-8"))
    links = {link.attrib["name"]: link for link in root.findall("link")}
    for link_name, (kind, xyz, rpy, attributes) in _COLLISIONS.items():
        collision = links[link_name].find("collision")
        if collision is None:
            raise ValueError(
                f"URDF link has no collision element: {link_name}"
            )
        origin = collision.find("origin")
        if origin is None:
            origin = ET.SubElement(collision, "origin")
        origin.attrib.update({"xyz": xyz, "rpy": rpy})
        geometry = collision.find("geometry")
        if geometry is None:
            geometry = ET.SubElement(collision, "geometry")
        geometry.clear()
        ET.SubElement(geometry, kind, attributes)
    return ET.tostring(root, encoding="unicode")
