import numpy as np
from sensor_msgs.msg import Image

from d1_camera_visualization.depth_debug_visualizer import (
    colorize_depth,
    image_depth_array,
)


def test_colorize_depth_uses_fixed_range_and_black_invalid_pixels() -> None:
    depth = np.asarray([[0, 200, 1100, 2000, 2001, 10001]], dtype=np.uint16)
    colored = colorize_depth(depth, 0.001, 0.2, 10.0, 0.2, 2.0)
    assert colored.shape == (1, 6, 3)
    assert colored.dtype == np.uint8
    np.testing.assert_array_equal(colored[0, 0], [0, 0, 0])
    np.testing.assert_array_equal(colored[0, 4], [0, 0, 0])
    np.testing.assert_array_equal(colored[0, 5], [0, 0, 0])
    assert not np.array_equal(colored[0, 1], colored[0, 3])


def test_decode_16uc1_honours_row_padding() -> None:
    message = Image()
    message.height = 2
    message.width = 2
    message.encoding = "16UC1"
    message.is_bigendian = 0
    message.step = 6
    message.data = bytes([1, 0, 2, 0, 99, 99, 3, 0, 4, 0, 88, 88])
    np.testing.assert_array_equal(image_depth_array(message), [[1, 2], [3, 4]])


def test_display_range_can_be_narrower_than_sensor_range() -> None:
    depth = np.asarray([[150, 200]], dtype=np.uint16)
    colored = colorize_depth(depth, 0.001, 0.1, 10.0, 0.2, 2.0)
    np.testing.assert_array_equal(colored[0, 0], [0, 0, 0])
    assert np.any(colored[0, 1])
