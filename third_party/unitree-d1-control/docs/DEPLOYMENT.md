# D1-computer deployment and rollback

Deployment changes the process that owns the physical arm serial bus. Perform
it only in a maintenance window with the arm supported, workspace clear and a
tested unload method available.

## Back up before installation

- `/etc/systemd/system/marm_controller.service`
- `/home/ubuntu/autoStartController.sh`
- `/home/ubuntu/marm_code/build/marm_controller_node`

Install `d1_control_node` under a distinct filename. Do not
overwrite the vendor binary. Stop `marm_controller.service` and verify the old
PID has exited before starting the new executable.

Install `deploy/systemd/d1-control.service` as
`/etc/systemd/system/d1-control.service`. The enhanced and vendor
services conflict explicitly, so only one process can own `/dev/ttyS4`.
The installed executable supports a motionless identity query:

```bash
/home/ubuntu/marm_code/build/d1_control_node --version
```

The host bringup requires its semantic version and local protocol version to
match the checked-in deployment configuration before activating controllers.

## Validation order

1. Start without a host command publisher and verify passive seven-joint feedback.
2. Send one native gripper segment while preserving Joint0–5.
3. Send one native seven-joint segment with every joint change below 5 degrees;
   confirm all joints finish together.
4. Cancel a standard trajectory and verify no replacement or stale command is
   sent.
5. Validate STOWED to OBSERVE.
6. Run the existing guarded pick test.

Record serial dispatch time, requested versus measured arrival time, feedback
age and endpoint error.

## Host maintenance commands

With `d1-control.service` active on the D1 computer, the host-side
maintenance tools use the same DDS topics and therefore keep `/dev/ttyS4` under
one owner. They do not require `real_bringup` or a ROS environment:

`D1_MIN_JOINT_INTERVAL_MS` configures the minimum per-joint interval used by
`uniform_joint_speed`. It defaults to 10 ms and keeps complete seven-joint
commands from issuing pathological 1 ms hold targets for nearly stationary
joints. Change it through the systemd service environment and restart the
service; accepted values are 1--1000 ms. `common_arrival` is unaffected.

```bash
d1_all_joints_unload --interface enp3s0 --confirm UNLOAD_ALL
d1_move_stowed --interface enp3s0 --confirm STOWED_MOVE
d1_move_zero --interface enp3s0 --confirm ZERO_MOVE
```

Do not run a pose tool concurrently with a task/control command publisher.
The unload command is intentionally available as a direct emergency
maintenance operation, but its effect must be checked physically.

## Rollback

Stop and disable `d1-control.service`, then start the untouched
`marm_controller.service`. Confirm passive feedback before attempting motion.
The vendor executable, wrapper and unit must remain unmodified. A rollback that
has not been exercised does not count as a rollback plan.
