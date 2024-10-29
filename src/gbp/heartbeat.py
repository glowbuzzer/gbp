import json

from gbp.client import ConnectionController
from gbp.effects import RegisteredGbcMessageEffect
from gbp.gbc import MachineCommand
from gbp.gbc_extra import GlowbuzzerInboundMessage


class HeatbeatEffect(RegisteredGbcMessageEffect):
    def __init__(self, controller: ConnectionController):
        self.controller = controller
        self.previous_heartbeat = 0
        controller.register_effect(self)

    def map(self, msg: GlowbuzzerInboundMessage):
        if msg.status and msg.status.machine:
            return msg.status.machine.heartbeat

    async def act(self, new_heartbeat: int):
        if new_heartbeat - self.previous_heartbeat > 100:
            # print("Heartbeat missed", new_heartbeat)
            command: MachineCommand = MachineCommand(
                heartbeat=new_heartbeat
            )
            self.previous_heartbeat=new_heartbeat
            msg = {"command": {"machine": {"0": {"command": command.model_dump(exclude_none=True)}}}}
            await self.controller.send(json.dumps(msg))
            # print("Sending command", json.dumps(msg))
