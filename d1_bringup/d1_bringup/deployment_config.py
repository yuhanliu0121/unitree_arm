"""Validation helpers shared by physical D1 launch files."""

import re


CAMERA_DRIVER_LOCATIONS = ("local", "remote")
REALSENSE_FRAME_RATES_HZ = (6, 15, 30)
_RESOLUTION_PATTERN = re.compile(r"^[1-9][0-9]*x[1-9][0-9]*$")


def resolve_camera_driver_location(camera_config, requested=""):
    """Return the validated process location for the wrist camera driver.

    ``local`` means this bringup owns ``realsense2_camera``. ``remote`` means
    another computer publishes the canonical wrist-camera ROS topics on the
    same ROS domain, while this stack remains their consumer.
    """

    value = str(requested or camera_config.get("driver_location", "local")).strip().lower()
    if value not in CAMERA_DRIVER_LOCATIONS:
        choices = ", ".join(CAMERA_DRIVER_LOCATIONS)
        raise ValueError(
            f"wrist camera driver_location must be one of [{choices}], got {value!r}"
        )
    return value


def resolve_camera_stream_profiles(camera_config):
    """Build validated RealSense profiles from resolution and frame rate."""

    color_resolution = str(camera_config.get("color_resolution", "")).strip()
    depth_resolution = str(camera_config.get("depth_resolution", "")).strip()
    for field, value in (
        ("color_resolution", color_resolution),
        ("depth_resolution", depth_resolution),
    ):
        if not _RESOLUTION_PATTERN.fullmatch(value):
            raise ValueError(
                f"wrist camera {field} must use WIDTHxHEIGHT, got {value!r}"
            )

    requested_rate = camera_config.get("frame_rate_hz")
    if isinstance(requested_rate, bool):
        frame_rate_hz = -1
    else:
        try:
            frame_rate_hz = int(requested_rate)
        except (TypeError, ValueError):
            frame_rate_hz = -1
    if frame_rate_hz not in REALSENSE_FRAME_RATES_HZ or str(
        requested_rate
    ).strip() not in {str(rate) for rate in REALSENSE_FRAME_RATES_HZ}:
        choices = ", ".join(str(rate) for rate in REALSENSE_FRAME_RATES_HZ)
        raise ValueError(
            f"wrist camera frame_rate_hz must be one of [{choices}], "
            f"got {requested_rate!r}"
        )

    return (
        f"{color_resolution}x{frame_rate_hz}",
        f"{depth_resolution}x{frame_rate_hz}",
    )
