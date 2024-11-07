import asyncio
from typing import Any

import pytest

from gbp import GbcClient
from gbp.client import GbcWebsocketInterface
from gbp.effects import (
    Stream,
    RegisteredGbcMessageEffect,
)
from gbp.gbc import ActivityStreamItem, ACTIVITYTYPE, DwellActivityParams, STREAMCOMMAND, STREAMSTATE
from gbp.gbc_extra import GlowbuzzerInboundMessage


class StreamTracker(RegisteredGbcMessageEffect):
    """
    Simple effect to track the state of a stream used in tests
    """

    def __init__(self):
        self.state = None
        self.read_count = None

    def select(self, status: GlowbuzzerInboundMessage) -> Any:
        if status.stream and status.stream[0]:
            s = status.stream[0]
            return s.state, s.readCount

    async def on_change(self, update, send: GbcWebsocketInterface) -> None:
        self.state, self.read_count = update


@pytest.fixture
def stream():
    """
    Stream instance fixture, used to send activities to GBC
    :return: Stream instance
    """
    yield Stream(0)


@pytest.fixture
def gbc(gbc: GbcClient, stream: Stream):
    """
    Fixture to register the stream effect with the GBC client
    :param gbc: Provided by the global fixture
    :param stream: Provided by the stream fixture
    :return: The modified gbc instance
    """
    gbc.register(stream)

    yield gbc


@pytest.mark.asyncio
async def test_buffer_more_than_capacity(gbc: GbcClient, stream: Stream):
    """
    Test that we can stream more than the buffer size of 100 items and it doesn't block or fail
    """

    activity_stream_items = [
        ActivityStreamItem(activityType=ACTIVITYTYPE.ACTIVITYTYPE_DWELL, dwell=DwellActivityParams(msToDwell=2))
        for _ in range(300)
    ]
    await stream.exec(*activity_stream_items)


@pytest.mark.asyncio
async def test_cancel_stream(gbc: GbcClient, stream: Stream):
    """
    Test that we can cancel a stream. We are going to send two dwell activities and then cancel the stream before the
    first one has a chance to complete. We should see that the second activity is not executed.
    """

    async def stream_callback():
        await stream.exec(
            *[
                ActivityStreamItem(
                    activityType=ACTIVITYTYPE.ACTIVITYTYPE_DWELL, dwell=DwellActivityParams(msToDwell=3000)
                )
                for _ in range(2)
            ]
        )

    tracker = StreamTracker()
    try:
        gbc.register(tracker)

        task = asyncio.create_task(stream_callback())  # run in background so we can cancel

        await asyncio.sleep(1)  # this is not long enough for the first dwell to complete
        await gbc.stream_command(0, STREAMCOMMAND.STREAMCOMMAND_STOP)

        await task

        assert tracker.state == STREAMSTATE.STREAMSTATE_STOPPED
        # we don't currently have a way to assert that the second dwell was not executed
    finally:
        gbc.unregister(tracker)
