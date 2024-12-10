import rclpy
from rclpy.node import Node
from commander_interfaces.srv import SetGoal

class ROSClient(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.client = self.create_client(SetGoal, '/set_goal')

    def send_goal(self, x, y, yaw):
        if not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Service /set_goal unavailable!")
            return False

        request = SetGoal.Request()
        request.x = x
        request.y = y
        request.yaw = yaw

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error("Failed to call service!")
            return False
