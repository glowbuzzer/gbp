import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Empty
import asyncio

from gbp import client
from gbp.effects.op import OpEnabledEffect
from gbp.effects.heartbeat import HeatbeatEcho
from gbp.effects.stream import Stream
from gbp.gbc import ActivityStreamItem, ACTIVITYTYPE, DwellActivityParams

# This class combines a ROS2 node and a GBC WebSocket client
class WebSocketNode(Node):
    count = 0
    def __init__(self, websocket_url : str = "ws://localhost:9001/ws") -> None:
        super().__init__('websocket_node')
        
        self.publisher_ = self.create_publisher(String, 'test_topic', 10)
        self.subscriber = self.create_subscription(String, 'test_topic', self.command_callback, 10)
        self.timer = self.create_timer(1.5, self.publish_and_send_command)
        self.srv = self.create_service(Empty, 'test_service', self.service_callback)

        self.websocket_url = websocket_url
        self.loop = asyncio.get_event_loop()
        
    @classmethod
    async def create(cls, websocket_url):
        instance = cls(websocket_url)
        await instance.connect_to_websocket()
        return instance

    def command_callback(self, msg) -> None:
        self.get_logger().info(f"ROS Subscriber: {msg.data}")

    async def publish_and_send_command(self) -> None:
        msg = String()
        msg.data = f"Counter: {self.count}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"ROS Publisher: {msg.data}")
        self.count = self.count + 1
    
    def service_callback(self, request, response) -> Empty:
        self.get_logger().info(f"ROS service")
        asyncio.ensure_future(self.send_commands_to_gbc())
        # --- call to self.send_commands_to_gbc() ---
        return response

    async def connect_to_websocket(self) -> None:
        try:
            self.controller = client.ConnectionController(self.websocket_url)
            await self.controller.connect()
            self.get_logger().info(f"Connected to GBC WebSocket")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to GBC WebSocket: {e}")
            return None

    async def send_commands_to_gbc(self) -> None:
        print("Sending commands to GBC")
        try:
            op=OpEnabledEffect(self.controller)
            await op.enable_operation()
            HeatbeatEcho(self.controller)
            stream=Stream(self.controller, 0)
            await stream.exec([
                ActivityStreamItem(
                    activityType=ACTIVITYTYPE.ACTIVITYTYPE_DWELL,
                    dwell=DwellActivityParams(msToDwell=1000)
                )
            ])           
            self.get_logger().info(f"Sent command over Websocket")
        except Exception as e:
            self.get_logger().error(f"Failed to send command over WebSocket: {e}")

async def main(args=None):
    rclpy.init(args=args)
    node = await WebSocketNode.create(websocket_url="ws://localhost:9001/ws")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        exit(0)

if __name__ == '__main__':
    asyncio.run(main())