import asyncio
import logging

from gbp.connection import GbcConnectionProvider
from gbp.debug import OperationErrorLogger, MachineStateLogger
from gbp.heartbeat import HeatbeatEcho
from gbp.op import OpEnabledEffect


async def main():
    logging.getLogger().setLevel(logging.INFO)
    logging.info("Starting")

    controller = GbcConnectionProvider("ws://localhost:9001/ws")
    controller.register(
        HeatbeatEcho(), # maintain heartbeat with gbc
        OperationErrorLogger(), # log when an operation error is received
        MachineStateLogger() # log machine state changes
    )

    await controller.run(blocking=False)
    await controller.run_effect(OpEnabledEffect, lambda op: op.enable_operation())
    logging.info("Operation enabled!")

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Program interrupted!")
