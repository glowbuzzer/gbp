import asyncio
import logging
from typing import List

'''
Tracks stream state and will push new activities to a stream when capacity is available.
Maintains futures that can be awaited when submitted activities are complete.
'''

from .client import ConnectionCommandMethods
from .effects import RegisteredGbcMessageEffect
from .gbc import ActivityStreamItem
from .gbc_extra import StreamStatus, GlowbuzzerInboundMessage


class Stream(RegisteredGbcMessageEffect):
    def __init__(self, index: int):
        self.tag = 0
        self.queue = []
        self.streamIndex = index
        self._status = StreamStatus()

    def select(self, msg: GlowbuzzerInboundMessage):
        return msg.stream

    async def on_change(self, state: GlowbuzzerInboundMessage, send: ConnectionCommandMethods):
        logging.info("Ready to stream! Queue size: %d", len(self.queue))
        await send.stream_items([activity[0] for activity in self.queue])

    def exec(self, activities: List[ActivityStreamItem]):
        def create_future(activity):
            self.tag += 1
            activity_dict = vars(activity)
            activity_dict["tag"] = self.tag
            return ActivityStreamItem(**activity_dict), asyncio.get_event_loop().create_future()

        with_futures = [create_future(activity) for activity in activities]
        self.queue.extend(with_futures)

        return asyncio.gather(*[activity[1] for activity in with_futures])
