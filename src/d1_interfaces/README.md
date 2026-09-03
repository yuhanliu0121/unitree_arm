# D1 public interfaces

This package is the only compile-time dependency required by the Go2 task
manager. Its supported external contract is:

- `d1_interfaces/action/PickObject` on `/arm/tasks/pick_object`;
- `d1_interfaces/action/DropObject` on `/arm/tasks/drop_object`;
- `d1_interfaces/msg/ArmTaskStatus` on `/arm/task_status`.

`DropObject` distinguishes an unreachable current arrangement from an invalid
drop reference. `OUTCOME_NEW_TARGET_REQUIRED` with
`FAILURE_TARGET_IN_KEEP_OUT` means that the requested point lies inside the
Go2-platform keep-out. The arm remains/returns to the stable state from which
DROP began (`READY_STOWED/EMPTY` or `READY_CARRY/HELD`), and the caller should
submit another drop target.

Observation, perception, state-transition and commissioning interfaces remain
private to the arm stack and deliberately stay in `d1_manipulation`.
