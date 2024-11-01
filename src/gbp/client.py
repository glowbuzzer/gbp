import json
import logging
from abc import ABC, abstractmethod
from typing import List

from gbp.gbc import MachineCommand, ActivityStreamItem


class ConnectionCommandMethods(ABC):
    @abstractmethod
    def send(self, message:str):
        pass

    def machine_command(self, command: MachineCommand):
        msg = {"command": {"machine": {"0": {"command": command.model_dump(exclude_none=True)}}}}
        return self.send(json.dumps(msg))

    # noinspection PyMethodMayBeStatic
    def stream_items(self, items: List[ActivityStreamItem]):
        # TODO
        json_string = '[' + ','.join([model.model_dump_json(exclude_none=True) for model in items]) + ']'
        logging.info("Sending stream items: %s", json_string)
        # Send the request via WebSocket
        # await self.websocket.send(json_string)
