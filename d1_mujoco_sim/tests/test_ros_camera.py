from pathlib import Path

import numpy as np
import yaml

from d1_mujoco_sim.ros_camera import (
    CAMERA_LINK_FROM_OPTICAL,
    RosCameraPublisher,
    align_depth_to_color,
    apply_distortion,
    camera_static_transforms,
    colorize_depth,
    depth_to_uint16,
    distortion_source_indices,
)


ROOT = Path(__file__).resolve().parents[2]
CONFIG = yaml.safe_load(
    (ROOT / "d1_mujoco_sim" / "config" / "sim.yaml").read_text()
)


def test_depth_encoding_matches_realsense_z16_millimetres() -> None:
    depth_m = np.asarray([[0.05, 0.1, 0.4564, 10.0, 10.1, np.inf]])
    encoded = depth_to_uint16(depth_m, 0.001, 0.1, 10.0)
    np.testing.assert_array_equal(encoded, [[0, 100, 456, 10000, 0, 0]])


def test_config_uses_fixed_d435i_depth_and_display_ranges() -> None:
    camera = CONFIG["camera"]
    assert camera["topic_root"] == "/wrist_camera"
    assert camera["link_frame"] == "wrist_camera_link"
    assert camera["color"]["frame"] == "wrist_camera_color_optical_frame"
    assert camera["depth"]["frame"] == "wrist_camera_depth_optical_frame"
    assert camera["min_depth_m"] == 0.2
    assert camera["max_depth_m"] == 10.0
    assert camera["display_min_depth_m"] == 0.2
    assert camera["display_max_depth_m"] == 2.0
    assert "display_auto_range" not in camera
    assert "display_percentiles" not in camera


def test_wrist_camera_topic_contract_separates_data_and_debug() -> None:
    topics = RosCameraPublisher._publisher_topics("/wrist_camera/")
    assert topics == {
        "color_image": "/wrist_camera/color/image_raw",
        "color_info": "/wrist_camera/color/camera_info",
        "depth_image": "/wrist_camera/depth/image_rect_raw",
        "depth_info": "/wrist_camera/depth/camera_info",
        "aligned_depth_image": (
            "/wrist_camera/aligned_depth_to_color/image_raw"
        ),
        "aligned_depth_info": (
            "/wrist_camera/aligned_depth_to_color/camera_info"
        ),
        "raw_depth_debug": "/wrist_camera/debug/depth_plasma",
        "aligned_depth_debug": (
            "/wrist_camera/debug/aligned_depth_plasma"
        ),
    }


def test_wrist_camera_tf_tree_reconstructs_calibrated_optical_poses() -> None:
    camera = CONFIG["camera"]
    transforms = camera_static_transforms(camera)
    assert [(parent, child) for parent, child, _ in transforms] == [
        ("Link6", "wrist_camera_link"),
        ("wrist_camera_link", "wrist_camera_color_optical_frame"),
        ("wrist_camera_link", "wrist_camera_depth_optical_frame"),
    ]
    link_from_camera = transforms[0][2]
    camera_from_color = transforms[1][2]
    camera_from_depth = transforms[2][2]
    np.testing.assert_array_equal(
        camera_from_color,
        CAMERA_LINK_FROM_OPTICAL,
    )
    np.testing.assert_allclose(
        link_from_camera @ camera_from_color,
        camera["T_link6_color_optical"],
        atol=1e-12,
    )
    np.testing.assert_allclose(
        link_from_camera @ camera_from_depth,
        np.asarray(camera["T_link6_color_optical"])
        @ np.asarray(camera["T_color_depth_optical"]),
        atol=1e-12,
    )


def test_depth_alignment_fills_projected_pixels_and_keeps_nearest_z16() -> None:
    depth_config = {
        "width": 2,
        "height": 1,
        "fx": 1.0,
        "fy": 1.0,
        "cx": 0.0,
        "cy": 0.0,
        "depth_scale_m_per_unit": 0.001,
    }
    color_config = {
        "width": 3,
        "height": 2,
        "fx": 1.0,
        "fy": 1.0,
        "cx": 0.0,
        "cy": 0.0,
        "distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
    }
    aligned = align_depth_to_color(
        np.asarray([[1000, 500]], dtype=np.uint16),
        depth_config,
        color_config,
        np.eye(4),
    )
    np.testing.assert_array_equal(
        aligned,
        [[1000, 500, 500], [1000, 500, 500]],
    )


def test_depth_alignment_preserves_empty_pixels_as_zero() -> None:
    depth = CONFIG["camera"]["depth"]
    color = CONFIG["camera"]["color"]
    aligned = align_depth_to_color(
        np.zeros((depth["height"], depth["width"]), dtype=np.uint16),
        depth,
        color,
        np.asarray(CONFIG["camera"]["T_color_depth_optical"]),
    )
    assert aligned.shape == (color["height"], color["width"])
    assert aligned.dtype == np.uint16
    assert not np.any(aligned)


def test_colorized_depth_is_rgb_and_marks_invalid_pixels_black() -> None:
    depth_m = np.asarray([[0.0, 0.1, 1.05, 2.0, 3.0]], dtype=np.float32)
    image = colorize_depth(depth_m, 0.1, 2.0)
    assert image.shape == (1, 5, 3)
    assert image.dtype == np.uint8
    np.testing.assert_array_equal(image[0, 0], [0, 0, 0])
    np.testing.assert_array_equal(image[0, 4], [0, 0, 0])
    assert not np.array_equal(image[0, 1], image[0, 3])


def test_rgb_distortion_map_uses_measured_calibration() -> None:
    color = CONFIG["camera"]["color"]
    indices = distortion_source_indices(color)
    assert indices is not None
    assert indices.shape == (color["height"], color["width"])
    centre_x = round(color["cx"])
    centre_y = round(color["cy"])
    centre_source = indices[centre_y, centre_x]
    source_y, source_x = divmod(int(centre_source), color["width"])
    assert abs(source_x - centre_x) <= 1
    assert abs(source_y - centre_y) <= 1

    image = np.zeros((color["height"], color["width"], 3), dtype=np.uint8)
    image[centre_y, centre_x] = [1, 2, 3]
    distorted = apply_distortion(image, indices)
    np.testing.assert_array_equal(
        distorted[centre_y, centre_x],
        [1, 2, 3],
    )


def test_camera_info_matrices_keep_calibrated_absolute_principal_point() -> None:
    color = CONFIG["camera"]["color"]
    intrinsic, projection = RosCameraPublisher._camera_info_values(color)
    np.testing.assert_allclose(
        intrinsic,
        [
            color["fx"],
            0.0,
            color["cx"],
            0.0,
            color["fy"],
            color["cy"],
            0.0,
            0.0,
            1.0,
        ],
    )
    np.testing.assert_allclose(
        projection,
        [
            color["fx"],
            0.0,
            color["cx"],
            0.0,
            0.0,
            color["fy"],
            color["cy"],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ],
    )
