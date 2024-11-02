import json
from abc import ABC, abstractmethod

from gbp.gbc import MachineCommand
from gbp.gbc_extra import GlowbuzzerStreamRequest
from gbp.logger import log


class GbcWebsocketInterface(ABC):
    @abstractmethod
    async def send(self, message: str):
        """
        Send raw message to GBC.
        :param message: The message to send
        """
        pass

    async def command(self, command: MachineCommand):
        """
        Send a machine command to GBC.
        :param command: The command to send
        """
        msg = {"command": {"machine": {"0": {"command": command.model_dump(exclude_none=True)}}}}
        return await self.send(json.dumps(msg))

    async def stream(self, request: GlowbuzzerStreamRequest):
        """
        Send activities to GBC.
        :param request: The stream index and activities to send.
        """
        msg = {"stream": request.model_dump(exclude_none=True, mode="json")}
        log.debug("Sending stream items: %s", msg)
        return await self.send(json.dumps(msg))
