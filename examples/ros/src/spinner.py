from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from gbp.client import ConnectionCommandMethods
from gbp.effects import RegisteredGbcMessageEffect
from gbp.gbc_extra import GlowbuzzerInboundMessage

'''
Simple GBC message effect to spin ROS2 node when a message is received
'''
class RosSpinner(RegisteredGbcMessageEffect):
    def __init__(self, node: Node):
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(node)

    def select(self, msg: GlowbuzzerInboundMessage) -> int:
        if msg.status and msg.status.machine:
            return msg.status.machine.heartbeat

    async def on_change(self, state: int, controller: ConnectionCommandMethods):
        self.executor.spin_once(0)
