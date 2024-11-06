import asyncio
import logging
import signal
from typing import cast

from gbp.connection import GbcClient
from gbp.effects import HeatbeatEcho
from gbp.effects import OpEnabledEffect
from gbp.effects import OperationErrorLogger, MachineStateLogger
from gbp.effects import Stream
from gbp.gbc import ActivityStreamItem, ACTIVITYTYPE, DwellActivityParams
from gbp.logger import log


async def main():
    log.setLevel(logging.INFO)

    log.info("Starting")

    controller = GbcClient("ws://localhost:9001/ws")
    controller.register(
        HeatbeatEcho(),  # maintain heartbeat with gbc
        OperationErrorLogger(),  # log when an operation error is received
        MachineStateLogger(),  # log machine state changes
    )

    await controller.connect(blocking=False)

    async def stream_callback(stream: Stream):
        log.debug("Stream callback: %s", stream)
        await stream.exec(
            ActivityStreamItem(activityType=ACTIVITYTYPE.ACTIVITYTYPE_DWELL, dwell=DwellActivityParams(msToDwell=2000))
        )

    await controller.run_once(OpEnabledEffect(), lambda op: cast(OpEnabledEffect, op).enable_operation())
    log.info("Operation enabled!")
    log.info("Executing dwell activity")

    await controller.run_once(Stream(0), stream_callback)

    log.info("Dwell done!")
    await controller.close()


def handle_sigterm(*args):
    log.info("Received SIGTERM, shutting down...")
    raise KeyboardInterrupt


try:
    signal.signal(signal.SIGTERM, handle_sigterm)

    asyncio.run(main())
except KeyboardInterrupt:
    log.info("Program interrupted")
