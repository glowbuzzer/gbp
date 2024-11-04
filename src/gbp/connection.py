import asyncio
import logging
from typing import Dict, Any, Callable, Coroutine, TypeVar

import websockets
from websockets import ConnectionClosedError

from .client import GbcWebsocketInterface
from .effects import RegisteredGbcMessageEffect
from .gbc_extra import GlowbuzzerInboundMessage

T = TypeVar("T", bound=RegisteredGbcMessageEffect)


class GbcClient(GbcWebsocketInterface):
    """
    Provides a connection to the GBC websocket server and handles all incoming messages
    """

    def __init__(self, uri):
        self.uri = uri
        self.websocket = None
        self.registered_message_effects: Dict[RegisteredGbcMessageEffect, Any] = {}
        self.shutdown = False

    async def connect(self, blocking=True):
        """
        Connect to the websocket and start receiving messages. If blocking is True, this method will block until the
        connection is closed. If blocking is False, this method will return immediately and messages will be received
        in the background.
        :param blocking: Whether to block waiting for messages
        """
        self.shutdown = False
        self.websocket = await websockets.connect(self.uri)
        if blocking:
            await self.receive_messages()
        else:
            asyncio.get_event_loop().create_task(self.receive_messages())

    async def run_once(self, effect: T, callback: Callable[[T], Coroutine]):
        """
        Run a temporary effect in the context of the connection. The effect will be registered, the callback function will be invoked with
        the effect as a parameter, and the effect will be unregistered when complete.
        :param effect: The effect
        :param callback: The function to call with the effect
        :return:
        """
        self.register(effect)
        try:
            return await callback(effect)
        finally:
            self.unregister(effect)

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
            try:
                message = await self.websocket.recv()
            except ConnectionClosedError:
                break

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

                if self.shutdown:
                    self.websocket.close()
                    break

            except Exception as e:
                logging.error(f"Error handling inbound websocket message: {e}")

    async def close(self):
        self.shutdown = True
        if self.websocket:
            await self.websocket.close()