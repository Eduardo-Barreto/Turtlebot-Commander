import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select


class TeleopTurtlebot3(Node):
    def __init__(self):
        super().__init__("teleop_turtlebot3")

        # Declare parameters with default values
        self.declare_parameter("key_forward", "w")
        self.declare_parameter("key_backward", "s")
        self.declare_parameter("key_left", "a")
        self.declare_parameter("key_right", "d")
        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("speed_modifier_factor", 2.0)

        # Load parameters
        self.key_forward = (
            self.get_parameter("key_forward").get_parameter_value().string_value
        )
        self.key_backward = (
            self.get_parameter("key_backward").get_parameter_value().string_value
        )
        self.key_left = (
            self.get_parameter("key_left").get_parameter_value().string_value
        )
        self.key_right = (
            self.get_parameter("key_right").get_parameter_value().string_value
        )
        self.linear_speed = (
            self.get_parameter("linear_speed").get_parameter_value().double_value
        )
        self.angular_speed = (
            self.get_parameter("angular_speed").get_parameter_value().double_value
        )
        self.speed_modifier_factor = (
            self.get_parameter("speed_modifier_factor")
            .get_parameter_value()
            .double_value
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # State variables
        self.speed_modifier_active = False

        self.get_logger().info(
            "TeleopTurtlebot3 Node Initialized. Use keyboard to control the TurtleBot3."
        )

    def get_key(self):
        """Reads a single key press from stdin."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def stop_robot(self):
        """Publishes a zero velocity command to stop the robot."""
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)

    def run(self):
        """Main loop to capture keyboard input and control the robot."""
        self.get_logger().info("Press keys to control the robot. 'q' to quit.")
        twist = Twist()
        while rclpy.ok():
            try:
                key = self.get_key()
                self.get_logger().info(f"Key pressed: {key}")
            except KeyboardInterrupt:
                break

            linear = 0.0
            angular = 0.0
            self.speed_modifier_active = False

            # Check for quit
            if key == "q":
                self.get_logger().info("Quitting teleop.")
                break

            # Speed modifier (if key is UPPERCASE)
            if key.isupper():
                self.speed_modifier_active = True

            key = key.lower()

            # Movement logic
            if key == self.key_forward:
                linear = self.linear_speed
            elif key == self.key_backward:
                linear = -self.linear_speed
            elif key == self.key_left:
                angular = self.angular_speed
            elif key == self.key_right:
                angular = -self.angular_speed

            # Apply speed modifier
            if self.speed_modifier_active:
                linear *= self.speed_modifier_factor
                angular *= self.speed_modifier_factor

            # Update twist and publish
            twist.linear.x = linear
            twist.angular.z = angular
            self.cmd_vel_pub.publish(twist)

        # Stop robot when exiting
        self.stop_robot()

        self.get_logger().info("Teleop stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopTurtlebot3()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TeleopTurtlebot3.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
