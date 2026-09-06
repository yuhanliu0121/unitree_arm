from pathlib import Path

import pytest
import yaml

from d1_bringup.deployment_config import (
    resolve_camera_driver_location,
    resolve_camera_stream_profiles,
    resolve_perception_model_path,
)


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "real_machine.yaml"


def test_camera_driver_location_defaults_to_local():
    assert resolve_camera_driver_location({}) == "local"


def test_camera_driver_location_uses_yaml_and_normalizes_case():
    assert resolve_camera_driver_location({"driver_location": " Remote "}) == "remote"


def test_camera_driver_location_cli_override_wins():
    assert (
        resolve_camera_driver_location({"driver_location": "local"}, "remote")
        == "remote"
    )


def test_camera_driver_location_rejects_unknown_value():
    with pytest.raises(ValueError, match="driver_location"):
        resolve_camera_driver_location({"driver_location": "orin"})


@pytest.mark.parametrize("frame_rate_hz", [6, 15, 30])
def test_camera_stream_profiles_accept_native_frame_rates(frame_rate_hz):
    assert resolve_camera_stream_profiles({
        "color_resolution": "1280x720",
        "depth_resolution": "848x480",
        "frame_rate_hz": frame_rate_hz,
    }) == (f"1280x720x{frame_rate_hz}", f"848x480x{frame_rate_hz}")


@pytest.mark.parametrize("frame_rate_hz", [None, True, 10, 15.5, "fast"])
def test_camera_stream_profiles_reject_unsupported_frame_rates(frame_rate_hz):
    with pytest.raises(ValueError, match="frame_rate_hz"):
        resolve_camera_stream_profiles({
            "color_resolution": "1280x720",
            "depth_resolution": "848x480",
            "frame_rate_hz": frame_rate_hz,
        })


def test_camera_stream_profiles_reject_malformed_resolution():
    with pytest.raises(ValueError, match="color_resolution"):
        resolve_camera_stream_profiles({
            "color_resolution": "1280*720",
            "depth_resolution": "848x480",
            "frame_rate_hz": 6,
        })


def test_perception_model_path_is_required(tmp_path):
    with pytest.raises(ValueError, match="perception.model_path is required"):
        resolve_perception_model_path({}, tmp_path / "real_machine.yaml")


def test_perception_model_path_resolves_relative_to_config(tmp_path):
    model = tmp_path / "weights" / "mcislab_trash_collect.pt"
    model.parent.mkdir()
    model.write_bytes(b"weights")
    assert resolve_perception_model_path(
        {"perception": {"model_path": "weights/mcislab_trash_collect.pt"}},
        tmp_path / "real_machine.yaml",
    ) == model


def test_perception_model_path_rejects_missing_file(tmp_path):
    with pytest.raises(ValueError, match="is not a file"):
        resolve_perception_model_path(
            {"perception": {"model_path": "weights/missing.pt"}},
            tmp_path / "real_machine.yaml",
        )


def test_checked_in_development_config_is_immediately_testable():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    assert config["wrist_camera"]["frame_rate_hz"] == 15
    assert resolve_perception_model_path(config, CONFIG_PATH).is_file()
