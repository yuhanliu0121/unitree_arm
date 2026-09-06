from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml


MODULE_PATH = Path(__file__).resolve().parents[1] / "lib" / "deployment_config_tool.py"
SPEC = importlib.util.spec_from_file_location("deployment_config_tool", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
TOOL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TOOL)


class DeploymentConfigToolTest(unittest.TestCase):
    def test_realsense_table_uses_librealsense_asic_serial(self) -> None:
        output = """Device Name                   Serial Number       Firmware Version
RealSense D435I               250122077729        5.17.3.10
"""
        self.assertEqual(
            TOOL.parse_realsense_device_list(output),
            [("250122077729", "RealSense D435I")],
        )

    def test_generated_config_uses_release_defaults(self) -> None:
        config = yaml.safe_load(
            TOOL.render_local_config("amd64", "enp3s0", "250122077729")
        )
        self.assertEqual(
            config["perception"]["model_path"],
            "../../../runtime/perception_runtime_v1/weights/mcislab_trash_collect.pt",
        )
        self.assertEqual(config["arm"]["serial_no"], "")
        self.assertEqual(
            config["wrist_camera"]["link6_to_camera_link"]["translation_xyz_m"],
            [-0.113314288587, 0.018109307512, -0.051022947448],
        )

    def test_deep_merge_preserves_implementation_defaults(self) -> None:
        merged = TOOL.deep_merge(
            {"arm": {"topic": "rt/arm_Command", "serial_no": "old"}},
            {"arm": {"serial_no": "new"}},
        )
        self.assertEqual(merged["arm"]["topic"], "rt/arm_Command")
        self.assertEqual(merged["arm"]["serial_no"], "new")

    def test_complete_amd64_override_validates(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            model = directory / "model.pt"
            model.touch()
            config = {
                "deployment": {"platform": "amd64", "ros_domain_id": 31},
                "perception": {"model_path": str(model)},
                "arm": {"serial_no": "", "network_interface": "lo"},
                "wrist_camera": {
                    "driver_location": "local",
                    "serial_no": "123",
                    "frame_rate_hz": 15,
                    "link6_to_camera_link": {
                        "translation_xyz_m": [0.1, 0.0, 0.0],
                        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                },
                "site": {
                    "go2_base_to_arm_base": {
                        "translation_xyz_m": [0.0, 0.0, 0.0],
                        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                    }
                },
            }
            self.assertEqual(TOOL.validate_config(config, directory / "local.yaml"), [])

    def test_unknown_arm_serial_requires_a_hardware_profile(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            model = directory / "model.pt"
            model.touch()
            config = yaml.safe_load(
                TOOL.render_local_config("amd64", "lo", "123")
            )
            config["perception"]["model_path"] = str(model)
            config["arm"]["serial_no"] = "UNKNOWN"
            errors = TOOL.validate_config(config, directory / "local.yaml")
            self.assertTrue(
                any("has no matching hardware profile" in error for error in errors)
            )

    def test_go2_identity_mount_is_rejected(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            model = directory / "model.pt"
            model.touch()
            config = yaml.safe_load(
                TOOL.render_local_config("amd64", "lo", "123")
            )
            config["deployment"]["platform"] = "go2"
            config["deployment"]["ros_domain_id"] = 31
            config["perception"]["model_path"] = str(model)
            config["arm"]["serial_no"] = "D1095"
            config["wrist_camera"]["link6_to_camera_link"] = {
                "translation_xyz_m": [0.1, 0.0, 0.0],
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            }
            errors = TOOL.validate_config(config, directory / "local.yaml")
            self.assertIn(
                "site.go2_base_to_arm_base must be measured; identity is not accepted on Go2",
                errors,
            )

    def test_local_camera_serial_must_match_librealsense(self) -> None:
        config = {
            "wrist_camera": {
                "driver_location": "local",
                "serial_no": "usb-descriptor-serial",
            }
        }
        original_detector = TOOL.detect_realsense_devices
        try:
            TOOL.detect_realsense_devices = lambda: [
                ("asic-serial", "RealSense D435I")
            ]
            errors = TOOL.validate_attached_hardware(config)
        finally:
            TOOL.detect_realsense_devices = original_detector
        self.assertEqual(len(errors), 1)
        self.assertIn("configured=usb-descriptor-serial", errors[0])
        self.assertIn("detected=asic-serial", errors[0])


if __name__ == "__main__":
    unittest.main()
