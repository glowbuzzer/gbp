import asyncio
import logging
from typing import cast

from gbp import log, GbcClient
from gbp.effects import HeatbeatEcho, OperationErrorLogger, MachineStateLogger, OpEnabledEffect

from stream_test import test as stream

async def main():
    log.setLevel(logging.DEBUG)

    gbc = GbcClient("ws://localhost:9001/ws")
    gbc.register(
        HeatbeatEcho(),  # maintain heartbeat with gbc
        OperationErrorLogger(),  # log when an operation error is received
        MachineStateLogger(),  # log machine state changes
    )

    await gbc.connect(blocking=False)

    await gbc.run_once(OpEnabledEffect(), lambda op: cast(OpEnabledEffect, op).enable_operation())

    await stream(gbc)

    await gbc.close()

    log.info("All tests done")

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("Program interrupted!")
