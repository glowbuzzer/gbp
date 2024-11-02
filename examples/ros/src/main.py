import asyncio
import threading

import rclpy

from gbp.connection import GbcClient
from gbp.effects.debug import OperationErrorLogger, MachineStateLogger
from gbp.effects.heartbeat import HeatbeatEcho
from node import SimpleNode
from spinner import RosSpinner


async def main():
    print("Starting main, thread id=", threading.get_ident())

    # Create a WebSocket client
    controller = GbcClient("ws://localhost:9001/ws")

    # Initialize ROS
    rclpy.init()

    # Create the ROS node
    node = SimpleNode(controller, asyncio.get_running_loop())

    # Register effects that will be invoked when a GBC websocket message is received
    controller.register(
        RosSpinner(node),
        HeatbeatEcho(),
        OperationErrorLogger(),
        MachineStateLogger()
    )

    # Run the controller and receive messages
    await controller.connect()

# Run the main function
try:
    asyncio.run(main())

except KeyboardInterrupt:
    print("Program interrupted!")
