'''
WORK IN PROGRESS - NOT YET PROPERLY IMPLEMENTED!!

Provides fluent builder pattern for activities to be sent to GBC.

'''

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional

from gbc import ACTIVITYTYPE, TriggerParams


@dataclass
class ActivityBuilder(ABC):
    tag: int = 0
    triggers: List[TriggerParams] = field(default_factory=list)

    def with_tag(self, tag: int) -> 'ActivityBuilder':
        self.tag = tag
        return self

    @property
    @abstractmethod
    def command_name(self) -> str:
        pass

    @property
    @abstractmethod
    def activity_type(self) -> 'ACTIVITYTYPE':
        pass

    @abstractmethod
    def build(self) -> Optional[dict]:
        pass

    @property
    def command(self) -> dict:
        command = {
            'activityType': self.activity_type,
            'tag': self.tag
        }
        params = self.build()
        if params:
            command[self.command_name] = params
        return {**command, 'triggers': self.triggers} if self.triggers else command

    def add_trigger(self, trigger: 'TriggerParams') -> 'ActivityBuilder':
        self.triggers.append(trigger)
        return self


@dataclass
class CancelActivityBuilder(ActivityBuilder):
    @property
    def command_name(self) -> str:
        return ""

    @property
    def activity_type(self) -> 'ACTIVITYTYPE':
        return ACTIVITYTYPE.ACTIVITYTYPE_NONE

    def build(self) -> Optional[dict]:
        return None


@dataclass
class EndProgramBuilder(ActivityBuilder):
    @property
    def command_name(self) -> str:
        return "endProgram"

    @property
    def activity_type(self) -> 'ACTIVITYTYPE':
        return ACTIVITYTYPE.ACTIVITYTYPE_ENDPROGRAM

    def build(self) -> Optional[dict]:
        return None


@dataclass
class ModbusDoutBuilder(ActivityBuilder):
    dout_to_set: int = 0
    value_to_set_array: List[bool] = field(default_factory=list)

    @property
    def command_name(self) -> str:
        return "setModbusDout"

    @property
    def activity_type(self) -> 'ACTIVITYTYPE':
        return ACTIVITYTYPE.ACTIVITYTYPE_SETMODBUSDOUT

    def dout(self, index: int) -> 'ModbusDoutBuilder':
        self.dout_to_set = index
        return self

    def value(self, value: List[bool]) -> 'ModbusDoutBuilder':
        self.value_to_set_array = value
        return self

    def build(self) -> dict:
        return {
            'doutToSet': self.dout_to_set,
            'valueToSetArray': self.value_to_set_array
        }


@dataclass
class DwellActivityBuilder(ActivityBuilder):
    ms_to_dwell: Optional[int] = None

    @property
    def command_name(self) -> str:
        return "dwell"

    @property
    def activity_type(self) -> 'ACTIVITYTYPE':
        return ACTIVITYTYPE.ACTIVITYTYPE_DWELL

    def ms_to_dwell(self, ms: int) -> 'DwellActivityBuilder':
        self.ms_to_dwell = ms
        return self

    def build(self) -> dict:
        return {
            'msToDwell': self.ms_to_dwell
        }


@dataclass
class ModbusUioutBuilder(ActivityBuilder):
    uiout_to_set: int = 0
    value_to_set_array: List[int] = field(default_factory=list)

    @property
    def command_name(self) -> str:
        return "setModbusUiout"

    @property
    def activity_type(self) -> 'ACTIVITYTYPE':
        return ACTIVITYTYPE.ACTIVITYTYPE_SETMODBUSUIOUT

    def uiout(self, index: int) -> 'ModbusUioutBuilder':
        self.uiout_to_set = index
        return self

    def value(self, value: List[int]) -> 'ModbusUioutBuilder':
        self.value_to_set_array = value
        return self

    def build(self) -> dict:
        return {
            'uioutToSet': self.uiout_to_set,
            'valueToSetArray': self.value_to_set_array
        }


@dataclass
class SpindleActivityBuilder(ActivityBuilder):
    spindle_index: int = 0
    speed: int = 0
    direction: Optional['ACTIVITYTYPE'] = None
    enable: bool = False

    @property
    def command_name(self) -> str:
        return "spindle"

    @property
    def activity_type(self) -> 'ACTIVITYTYPE':
        return ACTIVITYTYPE.ACTIVITYTYPE_SPINDLE

    def spindle_index(self, index: int) -> 'SpindleActivityBuilder':
        self.spindle_index = index
        return self

    def speed(self, speed: int) -> 'SpindleActivityBuilder':
        self.speed = speed
        return self

    def direction(self, direction) -> 'SpindleActivityBuilder':
        self.direction = direction
        return self

    def enable(self, enable: bool) -> 'SpindleActivityBuilder':
        self.enable = enable
        return self

    def build(self) -> dict:
        return {
            'spindleIndex': self.spindle_index,
            'speed': self.speed,
            'direction': self.direction,
            'enable': self.enable
        }


@dataclass
class MoveToPositionBuilder(ActivityBuilder):
    translation: Optional[dict] = None
    rotation: Optional[dict] = None
    configuration: int = 255  # Magic for "null" / not specified

    @property
    def command_name(self) -> str:
        return "moveToPosition"

    @property
    def activity_type(self) -> 'ACTIVITYTYPE':
        return ACTIVITYTYPE.ACTIVITYTYPE_MOVETOPOSITION

    def translation(self, x: Optional[float], y: Optional[float], z: Optional[float]) -> 'MoveToPositionBuilder':
        self.translation = {'x': x, 'y': y, 'z': z}
        return self

    def rotation(self, x: float, y: float, z: float, w: float) -> 'MoveToPositionBuilder':
        self.rotation = {'x': x, 'y': y, 'z': z, 'w': w}
        return self

    def configuration(self, config: int) -> 'MoveToPositionBuilder':
        self.configuration = config
        return self

    def build(self) -> dict:
        return {
            'translation': self.translation,
            'rotation': self.rotation,
            'configuration': self.configuration
        }


@dataclass
class MoveJointsBuilder(ActivityBuilder):
    joint_positions: List[float] = field(default_factory=list)
    position_reference: Optional['ACTIVITYTYPE'] = None

    @property
    def command_name(self) -> str:
        return "moveJoints"

    @property
    def activity_type(self) -> 'ACTIVITYTYPE':
        return ACTIVITYTYPE.ACTIVITYTYPE_MOVEJOINTS

    def joints(self, positions: List[float]) -> 'MoveJointsBuilder':
        self.joint_positions = positions
        return self

    def relative(self, is_relative: bool = True) -> 'MoveJointsBuilder':
        self.position_reference = ACTIVITYTYPE.RELATIVE if is_relative else ACTIVITYTYPE.ABSOLUTE
        return self

    def build(self) -> dict:
        return {
            'jointPositionArray': self.joint_positions,
            'positionReference': self.position_reference
        }


@dataclass
class MoveJointsInterpolatedBuilder(ActivityBuilder):
    joint_positions: List[float] = field(default_factory=list)
    joint_velocities: List[float] = field(default_factory=list)
    duration: int = 0

    @property
    def command_name(self) -> str:
        return "moveJointsInterpolated"

    @property
    def activity_type(self) -> 'ACTIVITYTYPE':
        return ACTIVITYTYPE.ACTIVITYTYPE_MOVEJOINTSINTERPOLATED

    def positions(self, positions: List[float]) -> 'MoveJointsInterpolatedBuilder':
        self.joint_positions = positions
        return self
