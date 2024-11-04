import asyncio
import logging
import threading
from asyncio import AbstractEventLoop

from action_simple.action import ActionSimple
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.lifecycle.node import Node
from std_srvs.srv import Empty

from decorator import with_asyncio, AsyncIoSupport
from gbp.connection import GbcClient
from gbp.effects import Stream, OpEnabledEffect
from gbp.gbc import ActivityStreamItem, ACTIVITYTYPE, DwellActivityParams
from logger import Ros2LoggingHandler
from util import enable_operation, feedback_msg


class SimpleNode(Node, AsyncIoSupport):
    def __init__(self, gbc: GbcClient, loop: AbstractEventLoop):
        Node.__init__(self, "simple_node")
        AsyncIoSupport.__init__(self, loop)

        ros_handler = Ros2LoggingHandler(self.get_logger())
        logging.getLogger().addHandler(ros_handler)

        self.gbc = gbc

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
