import logging
from typing import TypeAlias, Tuple

import gbp.effects
from gbp.client import ConnectionCommandMethods
from gbp.effects import RegisteredGbcMessageEffect
from gbp.gbc import OPERATION_ERROR
from gbp.gbc_extra import GlowbuzzerInboundMessage
from gbp.machine_state import MachineState, determine_machine_state


class MachineStateLogger(RegisteredGbcMessageEffect):
    """
    Simple effect to print when the machine state changes
    """
    state: MachineState = MachineState.UNKNOWN

    def select(self, msg: GlowbuzzerInboundMessage):
        if msg.status and msg.status.machine:
            return determine_machine_state(msg.status.machine.statusWord)

    async def on_change(self, new_value: MachineState, send: ConnectionCommandMethods):
        self.state = new_value
        logging.info("Change in CIA402 state: %s", self.state)


OperationErrorLoggingSelectType: TypeAlias = Tuple[OPERATION_ERROR, str]


class OperationErrorLogger(gbp.effects.RegisteredGbcMessageEffect):
    """
    Simple effect to print when the operation error state changes
    """

    def select(self, msg: GlowbuzzerInboundMessage) -> OperationErrorLoggingSelectType:
        if msg.status:
            return msg.status.machine.operationError, msg.status.machine.operationErrorMessage

    async def on_change(self, args: OperationErrorLoggingSelectType, send: ConnectionCommandMethods):
        logging.info("Operating error state changed: %s", args)
