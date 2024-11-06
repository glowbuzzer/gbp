import rclpy
from action_simple.action import ActionSimple
from rclpy.action import ActionClient
from rclpy.node import Node

class SimpleActionClient(Node):
    def __init__(self):
        super().__init__('simple_action_client')
        self._client = ActionClient(self, ActionSimple, 'simple')

    def send_goal(self, goal_request):
        self._client.wait_for_server()
        goal_future = self._client.send_goal_async(goal_request, feedback_callback=self.feedback_callback)
        goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Final result: {result.output}')
        self._client.destroy()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = SimpleActionClient()
    goal_msg = ActionSimple.Goal(input=5)
    action_client.send_goal(goal_msg)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
