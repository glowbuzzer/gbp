import asyncio
from typing import List

from .debug import MachineStateTracker

'''
Tracks stream state and will push new activities to a stream when capacity is available.
Maintains futures that can be awaited when submitted activities are complete.
'''

from .client import ConnectionController
from .effects import RegisteredGbcMessageEffect
from .gbc import ActivityStreamItem
from .gbc_extra import StreamStatus, GlowbuzzerInboundMessage


class Stream(RegisteredGbcMessageEffect):
    def __init__(self, controller: ConnectionController, index: int):
        self.controller = controller
        self.tag = 0
        self.queue = []
        self.streamIndex = index
        self._status = StreamStatus()
        self.opEnabledEffect = MachineStateTracker()
        controller.register_effect(self)
        controller.register_effect(self.opEnabledEffect)

    # def __enter__(self):
    #     print("stream entered")
    #     self.controller.register_stream(self)

    def __exit__(self):
        print("Exit stream")
        self.controller.unregister_effect(self)
        self.controller.unregister_effect(self.opEnabledEffect)

    def map(self, msg: GlowbuzzerInboundMessage):
        return msg.stream

    async def act(self, status: GlowbuzzerInboundMessage):
        print("Ready to stream!", status, "queue size:", len(self.queue))
        await self.controller.send_stream_items([activity[0] for activity in self.queue])

    def exec(self, activities: List[ActivityStreamItem]):
        def create_future(activity):
            self.tag += 1
            activity_dict = vars(activity)
            activity_dict["tag"] = self.tag
            return ActivityStreamItem(**activity_dict), asyncio.get_event_loop().create_future()

        with_futures = [create_future(activity) for activity in activities]
        self.queue.extend(with_futures)

        return asyncio.gather(*[activity[1] for activity in with_futures])
