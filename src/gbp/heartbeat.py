from gbp.client import ConnectionCommandMethods
from gbp.effects import RegisteredGbcMessageEffect
from gbp.gbc import MachineCommand
from gbp.gbc_extra import GlowbuzzerInboundMessage


class HeatbeatEcho(RegisteredGbcMessageEffect):
    def __init__(self):
        self.previous_heartbeat = 0

    def __repr__(self):
        return f"HeatbeatEffect({self.previous_heartbeat})"

    def select(self, msg: GlowbuzzerInboundMessage):
        if msg.status and msg.status.machine:
            return msg.status.machine.heartbeat

    async def on_change(self, new_heartbeat: int, send: ConnectionCommandMethods):
        if new_heartbeat - self.previous_heartbeat > 100:
            command: MachineCommand = MachineCommand(
                heartbeat=new_heartbeat
            )
            self.previous_heartbeat = new_heartbeat
            await send.machine_command(command)
