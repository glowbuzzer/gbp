from glowbuzzer.gbp import GbcClient
from glowbuzzer.gbp.effects import HeartbeatEcho, Stream, OpEnabledEffect
from glowbuzzer.gbp.effects.input.digital import DigitalInputTriggerEffect
from glowbuzzer.gbp.effects.solo_activity import SoloActivity
from glowbuzzer.gbp.gbc import ActivityStreamItem, TRIGGERTYPE, DoutCommand


class SimpleApi:
    def __init__(self, gbc: GbcClient):
        self.gbc = gbc
        self.solo_activity = SoloActivity(0)
        self.stream = Stream(0)

        gbc.register(HeartbeatEcho(), self.solo_activity, self.stream)

    async def enable(self):
        """
        Enable operation
        """
        await self.gbc.run_once(OpEnabledEffect(), lambda op: op.enable_operation())

    async def disable(self):
        """
        Disable operation
        """
        await self.gbc.run_once(OpEnabledEffect(), lambda op: op.disable_operation())

    async def digital_input_trigger(self, input: int, trigger: TRIGGERTYPE, timeout: int):
        """
        Wait for digital input to trigger according to trigger type
        :param input: Which digital input
        :param trigger: Trigger type
        :param timeout: How long to wait for trigger
        :raises TimeoutError: If timeout is reached
        """
        await self.gbc.run_once(DigitalInputTriggerEffect(input, trigger), lambda din: din.wait(timeout))

    async def set_digital_output(self, output: int, value: bool):
        """
        Set digital output
        :param output: Which digital output
        :param value: Value to set
        """
        await self.gbc.command(DoutCommand(setValue=value, override=True), output)

    async def stream(self, activities: ActivityStreamItem):
        """
        Stream list of activities
        :param activities: Activities to stream
        """
        await self.stream.exec(*activities)
