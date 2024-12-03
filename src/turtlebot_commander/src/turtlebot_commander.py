import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy

from commander_interfaces.srv import SetGoal


class TurtlebotCommander(Node):
    """
    Node responsável por expor um serviço para receber um objetivo (x, y, yaw) e enviar
    o comando para o robô utilizando a Simple Commander API do Nav2.
    """

    def __init__(self):
        super().__init__("turtlebot_commander")

        self.navigator = BasicNavigator()
        self.get_logger().info("BasicNavigator initialized")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.service = self.create_service(
            SetGoal, "set_goal", self.handle_set_goal_request, qos_profile=qos
        )
        self.get_logger().info("Goal service ready on topic: /set_goal")

    def handle_set_goal_request(self, request, response):
        """
        Callback para lidar com requisições do serviço.
        """
        x, y, yaw = request.x, request.y, request.yaw
        self.get_logger().info(f"Received goal: x={x}, y={y}, yaw={yaw}")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation = self.quaternion_from_yaw(yaw)

        self.navigator.goToPose(goal_pose)

        response.success = True
        self.get_logger().info("Goal sent successfully!")
        return response

    @staticmethod
    def quaternion_from_yaw(yaw):
        """
        Utilitário para converter yaw em uma orientação quaternion.
        """
        from math import sin, cos
        from geometry_msgs.msg import Quaternion

        quaternion = Quaternion()
        quaternion.z = sin(yaw / 2.0)
        quaternion.w = cos(yaw / 2.0)
        return quaternion


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotCommander()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
