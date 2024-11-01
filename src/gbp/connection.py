import asyncio
import logging
from typing import Dict, Any, Callable, Coroutine, Type

import websockets

from .client import ConnectionCommandMethods
from .effects import RegisteredGbcMessageEffect
from .gbc_extra import GlowbuzzerInboundMessage


class GbcConnectionProvider(ConnectionCommandMethods):
    """
    Provides a connection to the GBC websocket server and handles all incoming messages
    """

    def __init__(self, uri):
        self.uri = uri
        self.websocket = None
        self.registered_message_effects: Dict[RegisteredGbcMessageEffect, Any] = {}

    async def run(self, blocking=True):
        """
        Connect to the websocket and start receiving messages. If blocking is True, this method will block until the
        connection is closed. If blocking is False, this method will return immediately and messages will be received
        in the background.
        :param blocking: Whether to block waiting for messages
        """
        self.websocket = await websockets.connect(self.uri)
        if blocking:
            await self.receive_messages()
        else:
            asyncio.get_event_loop().create_task(self.receive_messages())

    async def run_effect(self, effect_cls: Type[RegisteredGbcMessageEffect],
                         fn: Callable[[RegisteredGbcMessageEffect], Coroutine]):
        """
        Run a temporary effect in the context of the connection. The effect will be registered, the function will be called with
        the effect, and then the effect will be unregistered when complete.
        :param effect_cls: The effect class to instantiate
        :param fn: The function to call with the effect
        :return:
        """
        op = effect_cls()
        self.register(op)
        try:
            return await fn(op)
        finally:
            self.unregister(op)

    def register(self, *effects: RegisteredGbcMessageEffect):
        """
        Register effects that will be invoked when a GBC websocket message is received.
        :param effects: The effects to register
        """
        for effect in effects:
            self.registered_message_effects[effect] = None

    def unregister(self, effect: RegisteredGbcMessageEffect):
        """
        Unregister an effect. The effect will no longer be invoked when a GBC websocket message is received.
        :param effect: The effect to unregister
        """
        self.registered_message_effects.pop(effect)

    async def send(self, message: str):
        """
        Low level method to send a message to the GBC websocket server. Not intended to be called directly.
        :param message: The raw JSON message to send
        """
        await self.websocket.send(message)

    async def receive_messages(self):
        """
        Receive messages from the GBC websocket server and invoke the registered effects.
        :return:
        """
        logging.info("Starting to receive messages")
        n = 0
        while True:
            message = await self.websocket.recv()
            # TODO: remove debug logging, but this is helpful to know that the connection is still alive
            n += 1
            if n % 25 == 0:
                logging.info("Got message: %d", n)
            try:
                # Convert to a GlowbuzzerInboundMessage object
                msg = GlowbuzzerInboundMessage.model_validate_json(message)
                # Iterate over all registered effects and call their callback
                for effect, previous_state in self.registered_message_effects.items():
                    try:
                        current_state = effect.select(msg)
                        # TODO: ?: do we need to make sure states are deeply compared?
                        if current_state and previous_state != current_state:
                            await effect.on_change(current_state, self)
                            self.registered_message_effects[effect] = current_state
                    except Exception as e:
                        logging.error(f"Error handling effect {effect}: {e}")

            except Exception as e:
                logging.error(f"Error handling inbound websocket message: {e}")

    async def close(self):
        # Close the WebSocket connection
        await self.websocket.close()
