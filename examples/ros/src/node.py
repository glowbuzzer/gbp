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
from gbp.connection import GbcConnectionProvider
from logger import Ros2LoggingHandler
from util import enable_operation, feedback_msg


class SimpleNode(Node, AsyncIoSupport):
    def __init__(self, controller: GbcConnectionProvider, loop: AbstractEventLoop):
        Node.__init__(self, "simple_node")
        AsyncIoSupport.__init__(self, loop)

        ros_handler = Ros2LoggingHandler(self.get_logger())
        logging.getLogger().addHandler(ros_handler)

        self.controller = controller

        self.task = self.create_service(Empty, "test_service", self.service_callback)

        self._action_server = ActionServer(
            self,
            ActionSimple,
            'simple',
            self.action_server_callback)

        logging.info("Node created")

    @with_asyncio(timeout=20)
    async def action_server_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info(f'Executing goal, thread id={threading.get_ident()}')

        goal_handle.publish_feedback(feedback_msg("Starting"))

        self.get_logger().info("Enabling operation")
        await enable_operation(self.controller)

        goal_handle.publish_feedback(feedback_msg("Operation enabled!"))

        # TODO: execute do a stream command (not implemented yet in gbp)
        # stream = Stream(0)
        # controller.register(stream)
        # try:
        #     await stream.exec([
        #         ActivityStreamItem(
        #             activityType=ACTIVITYTYPE.ACTIVITYTYPE_DWELL,
        #             dwell=DwellActivityParams(msToDwell=1000)
        #         )
        #     ])
        # finally:
        #     controller.unregister(stream)

        # simulate an activity
        self.get_logger().info('Sleeping')
        await asyncio.sleep(5)
        self.get_logger().info('Sleep done')

        result = ActionSimple.Result()
        result.output = 42
        goal_handle.succeed()

        return result

    @with_asyncio(timeout=20)
    async def service_callback(self, _request, response) -> None:
        self.get_logger().info("Task started")
        await asyncio.sleep(5)
        self.get_logger().info("Task finished")

        return response
