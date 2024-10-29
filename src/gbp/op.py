import asyncio
import json

from gbp.client import ConnectionController
from gbp.effects import RegisteredGbcMessageEffect
from gbp.gbc import MachineCommand
from gbp.gbc_extra import GlowbuzzerInboundMessage
from gbp.machine_state import determine_machine_state, MachineState, DesiredState, handle_machine_state

'''
Tracks status word and desired state and can trigger a state change to operational
'''

class OpEnabledEffect(RegisteredGbcMessageEffect):
    def __init__(self, controller: ConnectionController):
        self.controller = controller
        self.state: MachineState = MachineState.UNKNOWN
        self.desiredState: DesiredState = DesiredState.NONE
        self._desired_state_future = asyncio.get_event_loop().create_future()
        controller.register_effect(self)

    def map(self, msg: GlowbuzzerInboundMessage):
        if msg.status and msg.status.machine:
            return msg.status.machine.statusWord, self.desiredState

    async def act(self, new_state: tuple[int, DesiredState]):
        new_state, desired_state = new_state
        self.state = determine_machine_state(new_state)
        next_control_word = handle_machine_state(self.state, new_state, desired_state)
        if next_control_word:
            command: MachineCommand = MachineCommand(
                controlWord=next_control_word
            )
            msg = {"command": {"machine": {"0": {"command": command.model_dump(exclude_none=True)}}}}
            print("Sending new control word", json.dumps(msg))
            await self.controller.send(json.dumps(msg))

        if self.state == MachineState.OPERATION_ENABLED:
            self._desired_state_future.set_result(None)

    async def enable_operation(self):
        self.desiredState = DesiredState.OPERATIONAL
        self._desired_state_future = asyncio.get_event_loop().create_future()
        return await self._desired_state_future

