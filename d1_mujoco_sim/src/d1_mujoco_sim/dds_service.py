from __future__ import annotations

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic

from .dds_types import ArmString_, PubServoInfo_


class D1DdsService:
    def __init__(
        self,
        domain_id: int,
        command_topic: str,
        feedback_topic: str,
        joint_feedback_topic: str,
    ) -> None:
        self.participant = DomainParticipant(domain_id)
        command = Topic(self.participant, command_topic, ArmString_)
        feedback = Topic(self.participant, feedback_topic, ArmString_)
        joint_feedback = Topic(
            self.participant,
            joint_feedback_topic,
            PubServoInfo_,
        )
        self.command_reader = DataReader(self.participant, command)
        self.feedback_writer = DataWriter(self.participant, feedback)
        self.joint_feedback_writer = DataWriter(
            self.participant,
            joint_feedback,
        )

    def take_commands(self) -> list[str]:
        # Cyclone DDS can emit InvalidSample objects when a writer disappears.
        return [
            sample.data_
            for sample in self.command_reader.take()
            if isinstance(sample, ArmString_)
        ]

    def write_feedback(self, payload: str) -> None:
        self.feedback_writer.write(ArmString_(payload))

    def write_joint_feedback(self, angles_deg: list[float]) -> None:
        self.joint_feedback_writer.write(
            PubServoInfo_.from_angles(angles_deg)
        )
