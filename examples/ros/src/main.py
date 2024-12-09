import asyncio
import signal
import threading

import rclpy

from glowbuzzer.gbp.connection import GbcClient
from glowbuzzer.gbp.effects.debug import OperationErrorLogger, MachineStateLogger
from glowbuzzer.gbp.effects.heartbeat import HeartbeatEcho
from glowbuzzer.gbp.ros import Ros2Spinner
from node import SimpleNode


async def main():
    print("Starting main, thread id=", threading.get_ident())

    # Create a WebSocket client
    gbc = GbcClient("ws://localhost:9001/ws")

    # Initialize ROS
    rclpy.init()

    # Create the ROS node
    node = SimpleNode(gbc, asyncio.get_running_loop())

    # Register effects that will be invoked when a GBC websocket message is received
    gbc.register(
        Ros2Spinner(node),
        HeartbeatEcho(),
        OperationErrorLogger(),
        MachineStateLogger()
    )

    # Run the controller and receive messages
    await gbc.connect(blocking=True)

def handle_sigterm(*args):
    print("Received SIGTERM, shutting down...")
    raise KeyboardInterrupt

# Run the main function
try:
    signal.signal(signal.SIGTERM, handle_sigterm)
    asyncio.run(main())

except KeyboardInterrupt:
    print("Program interrupted!")
