import asyncio

import gbp.effects
from gbp import client
from gbp.gbc import ActivityStreamItem, ACTIVITYTYPE, DwellActivityParams
from gbp.gbc_extra import GlowbuzzerCombinedStatus, GlowbuzzerInboundMessage
from gbp.heartbeat import HeatbeatEffect
from gbp.stream import Stream
from gbp.op import OpEnabledEffect


class ErrorStateEffect(gbp.effects.RegisteredGbcMessageEffect):
    def map(self, msg: GlowbuzzerInboundMessage):
        return msg.status.machine.operationError, msg.status.machine.operationErrorMessage

    async def act(self, args):
        print(f"Operating error state changed: {args[0]} ({args[1]})")


async def main():
    # Create a WebSocket client
    controller = client.ConnectionController("ws://10.10.0.2:9001/ws")
    controller.register_effect(ErrorStateEffect())

    try:
        # Connect to the WebSocket server
        await controller.connect()

        HeatbeatEffect(controller)

        op=OpEnabledEffect(controller)
        await op.enable_operation()

        stream=Stream(controller, 0)
        await stream.exec([
            ActivityStreamItem(
                activityType=ACTIVITYTYPE.ACTIVITYTYPE_DWELL,
                dwell=DwellActivityParams(msToDwell=1000)
            )
        ])

    except asyncio.CancelledError:
        print("Main loop cancelled")
        exit(0)

    # Close the connection
    await controller.close()

# Run the main function
try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Program interrupted!")
