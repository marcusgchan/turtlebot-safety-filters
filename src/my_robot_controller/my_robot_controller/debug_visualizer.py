import math
import threading

import matplotlib.pyplot as plt
import rclpy
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from my_robot_controller_msgs.msg import ObstacleArray

# Lightweight stand-in for RViz2: an ego-centric (robot always at the origin,
# facing +x) top-down view of /scan and /detected_obstacles. Skips RViz2's 3D
# rendering entirely -- useful when RViz2 is unstable (e.g. freezing over
# forwarded X11).

# Same palette/clustering logic as obstacle_detector.py, duplicated here so
# this node can show clusters without obstacle_detector having to publish
# RViz2 markers.
CLUSTER_COLORS = [
    (0.9, 0.1, 0.1),
    (0.1, 0.6, 0.9),
    (0.1, 0.8, 0.2),
    (0.9, 0.7, 0.1),
    (0.7, 0.1, 0.9),
    (0.1, 0.9, 0.8),
]


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DebugVisualizer(Node):
    def __init__(self):
        super().__init__("debug_visualizer")

        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("obstacle_topic", "detected_obstacles")
        self.declare_parameter("view_range", 4.0)
        # Must match obstacle_detector's cluster_distance_threshold/
        # min_cluster_points to show the same clusters it's acting on.
        self.declare_parameter("cluster_distance_threshold", 0.15)
        self.declare_parameter("min_cluster_points", 3)

        self.view_range = self.get_parameter("view_range").value
        self.cluster_distance_threshold = self.get_parameter(
            "cluster_distance_threshold"
        ).value
        self.min_cluster_points = self.get_parameter("min_cluster_points").value

        self.latest_pose = None  # (x, y, theta)
        self.clusters = []  # [[(x, y), ...], ...] in sensor/ego frame
        self.obstacles = []  # [(x, y, radius), ...] in the odom/target frame

        self.create_subscription(Odometry, "odom", self._odom_cb, 10)
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self._scan_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ObstacleArray,
            self.get_parameter("obstacle_topic").value,
            self._obstacle_cb,
            10,
        )

    def _odom_cb(self, msg):
        pos = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.latest_pose = (pos.x, pos.y, yaw)

    def _scan_cb(self, msg):
        # Clustering only cares about Euclidean distance between consecutive
        # points, so it's done directly in the sensor frame -- no TF lookup
        # needed, and it matches the ego-centric frame this view plots in.
        points = []  # (original_scan_index, x, y)
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            points.append((i, r * math.cos(angle), r * math.sin(angle)))

        is_full_circle = (msg.angle_max - msg.angle_min + msg.angle_increment) >= (
            2 * math.pi - 1e-3
        )
        self.clusters = self._cluster(points, len(msg.ranges), is_full_circle)

    def _cluster(self, points, num_ranges, is_full_circle):
        if not points:
            return []

        clusters = [[points[0]]]
        for prev, curr in zip(points, points[1:]):
            index_gap = curr[0] - prev[0]
            distance = math.hypot(curr[1] - prev[1], curr[2] - prev[2])
            if index_gap == 1 and distance <= self.cluster_distance_threshold:
                clusters[-1].append(curr)
            else:
                clusters.append([curr])

        if is_full_circle and len(clusters) > 1:
            first, last = clusters[0], clusters[-1]
            index_gap = (first[0][0] + num_ranges) - last[-1][0]
            distance = math.hypot(first[0][1] - last[-1][1], first[0][2] - last[-1][2])
            if index_gap == 1 and distance <= self.cluster_distance_threshold:
                clusters[0] = last + first
                clusters.pop()

        return clusters

    def _obstacle_cb(self, msg):
        self.obstacles = [(o.x, o.y, o.radius) for o in msg.obstacles]

    def robot_relative_obstacles(self):
        """Transforms self.obstacles (odom/target frame) into the robot's
        current frame so they overlay on the (sensor-frame) scan points."""
        if self.latest_pose is None:
            return []
        x, y, theta = self.latest_pose
        cos_t, sin_t = math.cos(-theta), math.sin(-theta)
        result = []
        for ox, oy, radius in self.obstacles:
            dx, dy = ox - x, oy - y
            rx = cos_t * dx - sin_t * dy
            ry = sin_t * dx + cos_t * dy
            result.append((rx, ry, radius))
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DebugVisualizer()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, ax = plt.subplots(figsize=(7, 7))
    r = node.view_range

    def update(_frame):
        ax.clear()
        ax.set_xlim(-r, r)
        ax.set_ylim(-r, r)
        ax.set_aspect("equal")
        ax.set_title("Ego-centric view (robot at origin, facing +x)")
        ax.grid(True, alpha=0.3)

        for idx, cluster in enumerate(node.clusters):
            xs = [p[1] for p in cluster]
            ys = [p[2] for p in cluster]
            if len(cluster) >= node.min_cluster_points:
                color = CLUSTER_COLORS[idx % len(CLUSTER_COLORS)]
                ax.scatter(xs, ys, s=10, c=[color])
            else:
                ax.scatter(xs, ys, s=4, c="gray")

        for rx, ry, radius in node.robot_relative_obstacles():
            ax.add_patch(Circle((rx, ry), radius, fill=False, color="red", linewidth=2))

        ax.arrow(0, 0, 0.3, 0, head_width=0.1, color="blue")
        ax.plot(0, 0, "bo", markersize=8, label="robot")
        ax.legend(loc="upper right")

    ani = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
