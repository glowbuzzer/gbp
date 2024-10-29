from abc import ABC, abstractmethod
from gbp.gbc_extra import GlowbuzzerCombinedStatus, StreamStatus, GlowbuzzerInboundMessage


class RegisteredGbcMessageEffect(ABC):
    @abstractmethod
    def map(self, status: GlowbuzzerInboundMessage):
        pass

    @abstractmethod
    def act(self, status: GlowbuzzerInboundMessage):
        pass

