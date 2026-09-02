# D1 public interfaces

This package is the only compile-time dependency required by the Go2 task
manager. Its supported external contract is:

- `d1_interfaces/action/PickObject` on `/arm/tasks/pick_object`;
- `d1_interfaces/action/DropObject` on `/arm/tasks/drop_object`;
- `d1_interfaces/msg/ArmTaskStatus` on `/arm/task_status`.

Observation, perception, state-transition and commissioning interfaces remain
private to the arm stack and deliberately stay in `d1_manipulation`.
