#!/usr/bin/env python3
"""Publish a calibrated cube safe-descend prism for RViz inspection."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def _point(value) -> Point:
    result = Point()
    result.x, result.y, result.z = map(float, value)
    return result


def _color(marker: Marker, red: float, green: float, blue: float, alpha: float) -> None:
    marker.color.r = red
    marker.color.g = green
    marker.color.b = blue
    marker.color.a = alpha


def _line_marker(frame: str, namespace: str, marker_id: int, width: float,
                 color: tuple[float, float, float, float]) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = width
    _color(marker, *color)
    return marker


def _corners(closing, finger, gravity, closing_bounds, finger_bounds,
             gravity_bounds):
    return {
        (ci, fi, gi): (
            closing_bounds[ci] * closing
            + finger_bounds[fi] * finger
            + gravity_bounds[gi] * gravity
        )
        for ci in (0, 1) for fi in (0, 1) for gi in (0, 1)
    }


def build_markers(region: dict, length_m: float) -> MarkerArray:
    frame = region["frame_id"]
    basis = region["basis_camera"]
    closing = np.asarray(basis["closing_axis"], dtype=np.float64)
    finger = np.asarray(basis["finger_length_axis"], dtype=np.float64)
    gravity = np.asarray(basis["extrusion_axis_gravity_up"], dtype=np.float64)
    closing_bounds = region["safe_bounds_m"]["closing"]
    finger_bounds = region["safe_bounds_m"]["finger_length"]
    coordinates = region["diagnostics"]["all_point_coordinates"]
    gravity_mid = float(np.mean([value["gravity_m"] for value in coordinates.values()]))
    gravity_bounds = [gravity_mid - 0.5 * length_m, gravity_mid + 0.5 * length_m]
    corners = _corners(
        closing, finger, gravity, closing_bounds, finger_bounds, gravity_bounds
    )

    array = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    array.markers.append(clear)

    edges = _line_marker(frame, "safe_prism_edges", 0, 0.0025, (0.0, 0.9, 1.0, 0.95))
    edge_indices = []
    for ci in (0, 1):
        for fi in (0, 1):
            edge_indices.append(((ci, fi, 0), (ci, fi, 1)))
    for gi in (0, 1):
        for ci in (0, 1):
            edge_indices.append(((ci, 0, gi), (ci, 1, gi)))
        for fi in (0, 1):
            edge_indices.append(((0, fi, gi), (1, fi, gi)))
    for first, second in edge_indices:
        edges.points.extend((_point(corners[first]), _point(corners[second])))
    array.markers.append(edges)

    boundary_lines = _line_marker(
        frame, "validated_boundary_lines", 1, 0.004, (1.0, 0.8, 0.0, 1.0)
    )
    for ci in (0, 1):
        for fi in (0, 1):
            boundary_lines.points.extend((
                _point(corners[(ci, fi, 0)]), _point(corners[(ci, fi, 1)])
            ))
    array.markers.append(boundary_lines)

    centre = _line_marker(frame, "preferred_centre_line", 2, 0.006, (1.0, 0.0, 1.0, 1.0))
    closing_mid = 0.5 * sum(closing_bounds)
    finger_mid = 0.5 * sum(finger_bounds)
    centre.points.extend((
        _point(closing_mid * closing + finger_mid * finger + gravity_bounds[0] * gravity),
        _point(closing_mid * closing + finger_mid * finger + gravity_bounds[1] * gravity),
    ))
    array.markers.append(centre)

    samples = Marker()
    samples.header.frame_id = frame
    samples.ns = "measured_boundary_centres"
    samples.id = 3
    samples.type = Marker.SPHERE_LIST
    samples.action = Marker.ADD
    samples.pose.orientation.w = 1.0
    samples.scale.x = samples.scale.y = samples.scale.z = 0.009
    _color(samples, 1.0, 0.25, 0.05, 1.0)
    for value in coordinates.values():
        location = (
            value["closing_m"] * closing
            + value["finger_length_m"] * finger
            + value["gravity_m"] * gravity
        )
        samples.points.append(_point(location))
    array.markers.append(samples)
    return array


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("region", type=Path)
    parser.add_argument("--topic", default="/arm/debug/cube_grasp_markers")
    parser.add_argument("--length-m", type=float, default=0.20)
    args = parser.parse_args()
    region = json.loads(args.region.expanduser().read_text(encoding="utf-8"))

    rclpy.init()
    node = Node("visualize_cube_safe_region")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(MarkerArray, args.topic, qos)
    markers = build_markers(region, args.length_m)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        publisher.publish(markers)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    print(f"Published cube safe region from {args.region} on {args.topic}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
