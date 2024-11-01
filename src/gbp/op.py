import asyncio

from gbp.client import ConnectionCommandMethods
from gbp.effects import RegisteredGbcMessageEffect
from gbp.gbc import MachineCommand
from gbp.gbc_extra import GlowbuzzerInboundMessage
from gbp.machine_state import determine_machine_state, MachineState, DesiredState, handle_machine_state


# TODO: H: This needs some error handling and perhaps timeout.
# For example, if the state is FAULT_REACTION_ACTIVE, it can never reach OPERATION_ENABLED.
# Or GBC might fault immediately after reaching OPERATION_ENABLED if outside machine envelope.


class OpEnabledEffect(RegisteredGbcMessageEffect):
    """
    Tracks status word and attempts to enable operation by moving throught the CIA402 state machine
    """

    def __init__(self):
        self.state: MachineState = MachineState.UNKNOWN
        self._desired_state_future = asyncio.get_event_loop().create_future()

    def select(self, msg: GlowbuzzerInboundMessage):
        if msg.status and msg.status.machine:
            return msg.status.machine.statusWord

    async def on_change(self, new_state: int, send: ConnectionCommandMethods):
        new_state = new_state
        self.state = determine_machine_state(new_state)
        next_control_word = handle_machine_state(self.state, new_state, DesiredState.OPERATIONAL)
        if next_control_word:
            command: MachineCommand = MachineCommand(
                controlWord=next_control_word
            )
            await send.machine_command(command)

        if self.state == MachineState.OPERATION_ENABLED:
            self._desired_state_future.set_result(None)

    async def enable_operation(self):
        if self.state == MachineState.OPERATION_ENABLED:
            return

        self._desired_state_future = asyncio.get_event_loop().create_future()
        return await self._desired_state_future
