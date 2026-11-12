import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class DriveForward(Node):
    def __init__(self):
        super().__init__("drive_forward")
        self.declare_parameter("target_distance", 2.0)
        self.declare_parameter("linear_speed", 0.1)

        self.target_distance = self.get_parameter("target_distance").value
        self.linear_speed = self.get_parameter("linear_speed").value

        self.start_pos = None
        self.distance_travelled = 0.0
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        if self.start_pos is None:
            self.start_pos = (pos.x, pos.y)
            return
        dx = pos.x - self.start_pos[0]
        dy = pos.y - self.start_pos[1]
        self.distance_travelled = math.hypot(dx, dy)

    def timer_callback(self):
        cmd = Twist()

        if self.start_pos is None:
            # waiting for the first odom reading before moving
            self.cmd_pub.publish(cmd)
            return

        if self.distance_travelled >= self.target_distance:
            if not self.done:
                self.get_logger().info(
                    f"Reached {self.distance_travelled:.2f} m, stopping."
                )
                self.done = True
            self.cmd_pub.publish(cmd)  # zero Twist
            return

        cmd.linear.x = self.linear_speed
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = DriveForward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
