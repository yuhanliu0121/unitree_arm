builtin_interfaces/Time stamp

uint8 INITIALIZING=0
uint8 READY_STOWED=1
uint8 READY_CARRY=2
uint8 PICKING=3
uint8 DROPPING=4
uint8 RECOVERING_TO_STOWED=5
uint8 RECOVERING_TO_CARRY=6
uint8 FAULTED=7
uint8 state

uint8 PAYLOAD_EMPTY=0
uint8 PAYLOAD_HELD=1
uint8 PAYLOAD_UNKNOWN=2
uint8 payload_state

uint8 POSE_STOWED=0
uint8 POSE_CARRY=1
uint8 POSE_OTHER=2
uint8 POSE_UNKNOWN=3
uint8 canonical_pose

string active_operation
string active_phase
string failure_code
string detail
