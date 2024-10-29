from gbp.effects import RegisteredGbcMessageEffect
from gbp.gbc_extra import GlowbuzzerInboundMessage
from gbp.machine_state import MachineState, determine_machine_state

'''
Simple effect to print when the machine state changes
'''

class MachineStateTracker(RegisteredGbcMessageEffect):
    state: MachineState = MachineState.UNKNOWN

    def map(self, msg: GlowbuzzerInboundMessage):
        if msg.status and hasattr(msg.status, "machine") and msg.status.machine:
            return determine_machine_state(msg.status.machine.statusWord)

    async def act(self, new_value: MachineState):
        self.state=new_value
        print(f"Operation enabled: {self.state}")
