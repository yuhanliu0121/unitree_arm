from dataclasses import dataclass

from cyclonedds.idl import IdlStruct
from cyclonedds.idl.annotations import final
from cyclonedds.idl.types import float32


@final
@dataclass
class ArmString_(
    IdlStruct,
    typename="unitree_arm::msg::dds_::ArmString_",
):
    data_: str


@final
@dataclass
class PubServoInfo_(
    IdlStruct,
    typename="unitree_arm::msg::dds_::PubServoInfo_",
):
    servo0_data_: float32
    servo1_data_: float32
    servo2_data_: float32
    servo3_data_: float32
    servo4_data_: float32
    servo5_data_: float32
    servo6_data_: float32

    @classmethod
    def from_angles(cls, angles: list[float]) -> "PubServoInfo_":
        if len(angles) != 7:
            raise ValueError("expected seven servo angles")
        return cls(*(float(value) for value in angles))
