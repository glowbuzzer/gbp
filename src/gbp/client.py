import asyncio
from typing import Dict, Any, List

import websockets

from gbp.gbc_extra import GlowbuzzerInboundMessage
from .effects import RegisteredGbcMessageEffect
from .gbc import ActivityStreamItem


class ConnectionController:
    def __init__(self, uri):
        self.uri = uri
        self.websocket = None
        self.registered_message_effects: Dict[RegisteredGbcMessageEffect, Any] = {}

    async def connect(self, task_creator=asyncio.create_task):
        # Establish connection and start the message receiver
        self.websocket = await websockets.connect(self.uri)
        task_creator(self.receive_messages())

    def register_effect(self, effect: RegisteredGbcMessageEffect):
        self.registered_message_effects[effect] = None

    def unregister_effect(self, effect: RegisteredGbcMessageEffect):
        self.registered_message_effects.pop(effect)

    async def send_stream_items(self, items: List[ActivityStreamItem]):
        json_string = '[' + ','.join([model.model_dump_json(exclude_none=True) for model in items]) + ']'
        print("Sending stream items", json_string)
        # Send the request via WebSocket
        # await self.websocket.send(json_string)

    async def send(self, message:str):
        await self.websocket.send(message)

    async def receive_messages(self):
        print("Receiving")
        while True:
            message = await self.websocket.recv()
            try:
                msg = GlowbuzzerInboundMessage.model_validate_json(message)
                # iterate over all registered effects and call their callback
                if msg.status:
                    for effect, previous_state in self.registered_message_effects.items():
                        current_state=effect.map(msg)
                        if current_state and previous_state!=current_state:
                            await effect.act(current_state)
                            self.registered_message_effects[effect]=current_state


            except Exception as e:
                print(f"Error handling inbound websocket message: {e}")


    async def close(self):
        # Close the WebSocket connection
        await self.websocket.close()
