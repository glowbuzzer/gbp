import logging
import threading
from asyncio import AbstractEventLoop

from action_simple.action import ActionSimple
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.lifecycle.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

from gbp import GbcClient
from gbp.client import GbcWebsocketInterface
from gbp.effects import Stream, OpEnabledEffect, RegisteredGbcMessageEffect
from gbp.gbc import ActivityStreamItem, ACTIVITYTYPE, DwellActivityParams
from gbp.gbc_extra import GlowbuzzerInboundMessage
from gbp.ros import Ros2LoggingHandler
from gbp.ros import with_asyncio, AsyncIoSupport
from util import feedback_msg


class HeartbeatPublisher(RegisteredGbcMessageEffect):
    def __init__(self, publisher: Publisher):
        self.publisher = publisher

    def select(self, msg: GlowbuzzerInboundMessage):
        if msg.status and msg.status.machine:
            return msg.status.machine.heartbeat

    async def on_change(self, new_state: int, send: GbcWebsocketInterface):
        self.publisher.publish(String(data=str(new_state)))


class SimpleNode(Node, AsyncIoSupport):
    def __init__(self, gbc: GbcClient, loop: AbstractEventLoop):
        Node.__init__(self, "simple_node")
        AsyncIoSupport.__init__(self, loop)

        ros_handler = Ros2LoggingHandler(self.get_logger())
        logging.getLogger().addHandler(ros_handler)

        self.gbc = gbc
        gbc.register(HeartbeatPublisher(self.create_publisher(String, "heartbeat", 10)))

        self._action_server = ActionServer(self, ActionSimple, "simple", self.action_server_callback)

        logging.info("Node created")

    @with_asyncio(timeout=20)
    async def action_server_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f"Executing goal, thread id={threading.get_ident()}")

        goal_handle.publish_feedback(feedback_msg("Starting"))

        logging.info("Enabling operation")
        await self.gbc.run_once(OpEnabledEffect(), lambda op: op.enable_operation())

        goal_handle.publish_feedback(feedback_msg("Operation enabled!"))

        async def stream_callback(stream: Stream):
            logging.info("Stream callback: %s", stream)
            await stream.exec(
                ActivityStreamItem(
                    activityType=ACTIVITYTYPE.ACTIVITYTYPE_DWELL, dwell=DwellActivityParams(msToDwell=2000)
                )
            )

        await self.gbc.run_once(Stream(0), stream_callback)

        result = ActionSimple.Result()
        result.output = 42
        goal_handle.succeed()

        return result
