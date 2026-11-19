import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.interpolate import RegularGridInterpolator

from my_robot_controller_msgs.msg import ObstacleArray

DEFAULT_VALUE_FUNCTION_PATH = str(
    Path.home() / "repos" / "optimized_dp" / "turtlebot_4d_avoid_value_function.npz"
)

# Must match the control bounds used to generate the value function in
# optimized_dp/examples/turtlebot_4d_avoid_test.py. Speed is now a state
# (dynamics state index 2), bounded by the grid itself (data["minBounds"]/
# data["maxBounds"]); ACCEL_MIN/MAX below is the control that drives it.
# Measured from a step response (0 -> 0.22 m/s) logged via /odom: ramp to
# 90% of target took ~0.156s, i.e. ~1.27 m/s^2. Using 1.0 to stay under the
# measured value -- the reachability guarantee only holds if the solved
# avoid set never assumes more acceleration than the robot actually has.
# Deceleration (ACCEL_MIN) was not separately measured; assumed symmetric.
ACCEL_MIN, ACCEL_MAX = -1.0, 1.0
W_MIN, W_MAX = -2.84, 2.84


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SafetyFilter(Node):
    def __init__(self):
        super().__init__("safety_filter")
        self.declare_parameter("value_function_path", DEFAULT_VALUE_FUNCTION_PATH)
        self.declare_parameter("safety_margin", 0.15)
        self.declare_parameter("release_margin", 0.25)
        self.declare_parameter("input_topic", "cmd_vel_raw")
        self.declare_parameter("output_topic", "cmd_vel")
        # One-step Euler horizon used to turn the optimal acceleration into a
        # commanded speed for turtlebot3's velocity-controlled cmd_vel.
        self.declare_parameter("accel_dt", 0.1)
        self.declare_parameter("obstacle_topic", "detected_obstacles")
        # How stale detected_obstacles can be before we no longer trust it.
        # Fail-safe: no recent obstacle data means we stop, not pass commands
        # through -- the opposite of latest_state below, where no odom yet
        # just means we have nothing to filter against.
        self.declare_parameter("obstacle_timeout", 0.5)

        value_function_path = self.get_parameter("value_function_path").value
        self.safety_margin = self.get_parameter("safety_margin").value
        self.release_margin = self.get_parameter("release_margin").value
        self.accel_dt = self.get_parameter("accel_dt").value
        self.obstacle_timeout = self.get_parameter("obstacle_timeout").value

        self._load_value_function(value_function_path)

        self.latest_state = None  # (x, y, v, theta)
        self.obstacles = []  # [(x, y, radius), ...] in the same frame as latest_state
        self.last_obstacle_msg_time = None
        self.overriding = False
        self.stopped = False

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter("output_topic").value, 10
        )
        self.cmd_sub = self.create_subscription(
            Twist, self.get_parameter("input_topic").value, self.cmd_callback, 10
        )
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.obstacle_sub = self.create_subscription(
            ObstacleArray,
            self.get_parameter("obstacle_topic").value,
            self.obstacle_callback,
            10,
        )

        self.get_logger().info(
            f"Safety filter active: solved obstacle radius {self.obstacle_radius}, "
            f"safety margin {self.safety_margin}"
        )

    def _load_value_function(self, path):
        data = np.load(path)
        V = data["V"]
        dVdx = data["dVdx"]
        dVdy = data["dVdy"]
        dVdv = data["dVdv"]
        dVdtheta = data["dVdtheta"]
        min_bounds = data["minBounds"]
        max_bounds = data["maxBounds"]
        pts_each_dim = data["pts_each_dim"]
        self.obstacle_center = data["obstacle_center"]
        self.obstacle_radius = float(data["obstacle_radius"])
        self.min_bounds = min_bounds
        self.max_bounds = max_bounds

        axes = [
            np.linspace(min_bounds[i], max_bounds[i], int(pts_each_dim[i]))
            for i in range(4)
        ]

        self.v_interp = RegularGridInterpolator(axes, V, bounds_error=False, fill_value=None)
        self.dvdx_interp = RegularGridInterpolator(
            axes, dVdx, bounds_error=False, fill_value=None
        )
        self.dvdy_interp = RegularGridInterpolator(
            axes, dVdy, bounds_error=False, fill_value=None
        )
        self.dvdv_interp = RegularGridInterpolator(
            axes, dVdv, bounds_error=False, fill_value=None
        )
        self.dvdtheta_interp = RegularGridInterpolator(
            axes, dVdtheta, bounds_error=False, fill_value=None
        )

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        speed = msg.twist.twist.linear.x
        self.latest_state = (pos.x, pos.y, speed, yaw)

    def obstacle_callback(self, msg):
        self.obstacles = [(o.x, o.y, o.radius) for o in msg.obstacles]
        self.last_obstacle_msg_time = self.get_clock().now()

    def _worst_case_value(self, x, y, v, theta):
        """Transforms (x, y) into each detected obstacle's frame and queries
        the (obstacle-relative) value function there in one batched call,
        returning the value and gradients for whichever obstacle is most
        dangerous (lowest V). With no detected obstacles this returns +inf --
        clear by definition."""
        if not self.obstacles:
            return math.inf, 0.0, 0.0

        obstacle_xy = np.array([(ox, oy) for ox, oy, _radius in self.obstacles])
        dx = np.clip(x - obstacle_xy[:, 0], self.min_bounds[0], self.max_bounds[0])
        dy = np.clip(y - obstacle_xy[:, 1], self.min_bounds[1], self.max_bounds[1])
        points = np.column_stack([dx, dy, np.full_like(dx, v), np.full_like(dx, theta)])

        values = self.v_interp(points)
        worst_idx = int(np.argmin(values))
        worst_point = points[worst_idx : worst_idx + 1]

        worst_value = float(values[worst_idx])
        worst_dvdv = float(self.dvdv_interp(worst_point)[0])
        worst_dvdtheta = float(self.dvdtheta_interp(worst_point)[0])
        return worst_value, worst_dvdv, worst_dvdtheta

    def cmd_callback(self, msg):
        if self.stopped:
            self.cmd_pub.publish(Twist())
            return

        if self.latest_state is None:
            self.cmd_pub.publish(msg)
            return

        if self.last_obstacle_msg_time is None:
            # Never heard from the obstacle detector -- unlike latest_state
            # above, we can't treat "no data" as "nothing to filter against":
            # the detector might just be down. Fail safe, not open.
            self.get_logger().warn(
                "No obstacle data received yet -- stopping.", throttle_duration_sec=2.0
            )
            self.cmd_pub.publish(Twist())
            return

        age = (self.get_clock().now() - self.last_obstacle_msg_time).nanoseconds / 1e9
        if age > self.obstacle_timeout:
            self.get_logger().warn(
                f"Obstacle data stale ({age:.2f}s old) -- stopping.",
                throttle_duration_sec=2.0,
            )
            self.cmd_pub.publish(Twist())
            return

        x, y, v, theta = self.latest_state
        # v is a grid axis regardless of obstacle position; x, y are clamped
        # per-obstacle inside _worst_case_value, since that's relative to each
        # obstacle's own frame.
        v = min(max(v, self.min_bounds[2]), self.max_bounds[2])

        value, dvdv, dvdtheta = self._worst_case_value(x, y, v, theta)

        # Hysteresis: trigger the override at safety_margin, but only hand
        # control back once clearly past release_margin. Without this gap,
        # releasing control right at the trigger point lets whatever is
        # upstream immediately drive back into the margin, overriding again
        # next tick forever.
        if not self.overriding:
            if value >= self.safety_margin:
                self.cmd_pub.publish(msg)
                return
            self.overriding = True
        else:
            if value >= self.release_margin:
                self.overriding = False
                self.stopped = True
                self.get_logger().info(
                    "Safety maneuver complete, state is clear -- stopping."
                )
                self.cmd_pub.publish(Twist())
                return

        # Optimal "max" control: pushes V (distance from the unsafe/avoid
        # set) up as fast as possible, matching DubinsCar4.opt_ctrl with
        # uMode="max" from the offline solve. Unlike the 3D filter, speed is
        # now a state driven by acceleration, so the optimal control depends
        # on dV/dv and dV/dtheta only -- x_dot/y_dot carry no direct control
        # term in this dynamics model.
        safe_accel = ACCEL_MAX if dvdv >= 0 else ACCEL_MIN
        safe_w = W_MIN if dvdtheta < 0 else W_MAX

        commanded_speed = min(
            max(v + safe_accel * self.accel_dt, self.min_bounds[2]), self.max_bounds[2]
        )

        safe_cmd = Twist()
        safe_cmd.linear.x = commanded_speed
        safe_cmd.angular.z = safe_w

        self.get_logger().warn(
            f"Safety filter override: V={value:.3f} < margin={self.safety_margin:.2f} "
            f"at (x={x:.2f}, y={y:.2f}, v={v:.2f}, theta={theta:.2f}); "
            f"commanding speed={commanded_speed:.2f}, w={safe_w:.2f}"
        )
        self.cmd_pub.publish(safe_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Most diff-drive controllers have no command watchdog -- they keep
        # executing the last Twist they received forever, not just until the
        # publisher goes away. Without this, Ctrl-C leaves the robot coasting
        # on whatever was last commanded.
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
