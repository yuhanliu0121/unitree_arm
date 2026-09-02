# Real deployment configuration ownership

`real_machine.yaml` is the single runtime input so startup remains atomic and
easy to audit. Its fields have three owners:

| Owner | Fields |
| --- | --- |
| Go2 integration | `deployment.ros_domain_id`, deployment override of `perception.model_path`, `arm.network_interface`, `arm.expected_ipv4_subnet`, `wrist_camera.driver_location`, camera stream rate/topics, and `site.go2_base_to_arm_base` |
| Arm/camera calibration | `arm.serial_no`, `wrist_camera.serial_no`, `wrist_camera.link6_to_camera_link`, hardware joint-limit profile, and object gripper profiles |
| Arm stack implementation | Native D1 DDS topics/domain, loopback ports, segment-controller timing, canonical frame names, and preflight thresholds |

The Go2 repository should carry its own copy of this file (or generate it during
deployment) and must not edit implementation defaults in launch files. Real
bringup rejects a missing perception model, unsupported camera profile, wrong
ROS domain, absent network interface, invalid TF/calibration, stale camera data,
or stale/out-of-limit arm feedback before activating controllers.

The checked-in model path intentionally targets the development paper-object
weights; it keeps local real-arm acceptance runnable and is not the final Go2
deployment choice. Do not commit site secrets or machine-local temporary paths.
Absolute model paths are recommended on the deployed computer; relative model
paths are resolved from the YAML location.
