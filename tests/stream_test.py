import asyncio
import logging
from typing import Any, cast

import pytest
import pytest_asyncio

from gbp import log, GbcClient
from gbp.client import GbcWebsocketInterface
from gbp.effects import (
    Stream,
    GbcMessageEffect,
    HeatbeatEcho,
    OperationErrorLogger,
    MachineStateLogger,
    OpEnabledEffect,
)
from gbp.gbc import ActivityStreamItem, ACTIVITYTYPE, DwellActivityParams, STREAMCOMMAND, STREAMSTATE
from gbp.gbc_extra import GlowbuzzerInboundMessage


class StreamTracker(GbcMessageEffect):
    def __init__(self):
        self.state = None
        self.read_count = None

    def select(self, status: GlowbuzzerInboundMessage) -> Any:
        if status.stream and status.stream[0]:
            s = status.stream[0]
            return s.state, s.readCount

    async def on_change(self, update, send: GbcWebsocketInterface) -> None:
        self.state, self.read_count = update


@pytest.fixture(scope='session', autouse=True)
def set_up_logging():
    # Clear existing handlers
    for handler in logging.root.handlers[:]:
        logging.root.removeHandler(handler)

    # Configure new logging
    print("LOGGING")
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
    logging.root.addHandler(handler)
    logging.root.setLevel(logging.DEBUG)
    logging.info("Logging set up")

    yield

@pytest_asyncio.fixture
async def testX():
    print("test fixture start")
    yield "a"
    print("test fixture done")
    # return "a"


@pytest.mark.asyncio
async def test_fixture(testX):
    print("test_fixture", testX)


@pytest_asyncio.fixture
async def gbc():
    log.setLevel(logging.DEBUG)

    gbc = GbcClient("ws://localhost:9001/ws")
    try:
        gbc.register(
            HeatbeatEcho(),  # maintain heartbeat with gbc
            OperationErrorLogger(),  # log when an operation error is received
            MachineStateLogger(),  # log machine state changes
        )

        await gbc.connect(blocking=False)

        await gbc.run_once(OpEnabledEffect(), lambda op: cast(OpEnabledEffect, op).enable_operation())
    except Exception as e:
        await gbc.close()
        logging.error("Failed to connect to GBC: %s", e)
        pytest.skip("Failed to connect to GBC")

    yield gbc

    await gbc.close()


@pytest.mark.asyncio
async def test_test(gbc: GbcClient):
    logging.debug("test_test %s", gbc)

@pytest.mark.asyncio
async def test_buffer_overflow(gbc: GbcClient):
    """
    Test that we can stream more than the buffer size of 100 items
    """

    def stream_callback(stream: Stream):
        activity_stream_items = [
            ActivityStreamItem(activityType=ACTIVITYTYPE.ACTIVITYTYPE_DWELL, dwell=DwellActivityParams(msToDwell=2))
            for _ in range(300)
        ]
        return stream.exec(*activity_stream_items)

    await gbc.run_once(Stream(0), stream_callback)


async def test_cancel_stream(gbc: GbcClient):
    """
    Test that we can cancel a stream. We are going to send two dwell activities and then cancel the stream before the
    first one has a chance to complete. We should see that the second activity is not executed.
    """

    def stream_callback(stream: Stream):
        activity_stream_items = [
            ActivityStreamItem(activityType=ACTIVITYTYPE.ACTIVITYTYPE_DWELL, dwell=DwellActivityParams(msToDwell=3000))
            for _ in range(2)
        ]
        return stream.exec(*activity_stream_items)

    tracker = StreamTracker()
    try:
        gbc.register(tracker)

        stream = Stream(0)
        task = asyncio.create_task(gbc.run_once(stream, stream_callback))  # run in background so we can cancel

        await asyncio.sleep(1)  # this is not long enough for the first dwell to complete
        await gbc.stream_command(0, STREAMCOMMAND.STREAMCOMMAND_STOP)

        await task

        assert tracker.state == STREAMSTATE.STREAMSTATE_STOPPED
        # we don't currently have a way to assert that the second dwell was not executed
    finally:
        gbc.unregister(tracker)


async def test(gbc: GbcClient):
    await test_cancel_stream(gbc)
    # await test_buffer_overflow(gbc)
