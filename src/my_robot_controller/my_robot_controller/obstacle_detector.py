import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

from my_robot_controller_msgs.msg import Obstacle, ObstacleArray

# Distinct colors cycled per cluster index so adjacent obstacles in RViz2 are
# visually separable.
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


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__("obstacle_detector")

        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("cluster_distance_threshold", 0.15)
        self.declare_parameter("min_cluster_points", 3)
        # Must match OBSTACLE_RADIUS in turtlebot_4d_avoid_test.py. We no
        # longer fit each cluster's true shape/radius -- real obstacles
        # (walls, furniture) rarely look circular, so fitting and then
        # rejecting non-circular clusters threw away almost everything.
        # Instead every surviving cluster becomes a conservative circle of
        # this fixed, solved-for radius at its centroid; safety_filter.py
        # takes the worst case over all of them, same as before.
        self.declare_parameter("obstacle_radius", 0.3)
        self.declare_parameter("publish_debug_markers", True)

        self.target_frame = self.get_parameter("target_frame").value
        self.cluster_distance_threshold = self.get_parameter(
            "cluster_distance_threshold"
        ).value
        self.min_cluster_points = self.get_parameter("min_cluster_points").value
        self.obstacle_radius = self.get_parameter("obstacle_radius").value
        self.publish_debug_markers = self.get_parameter("publish_debug_markers").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.obstacle_pub = self.create_publisher(ObstacleArray, "detected_obstacles", 10)
        self.marker_pub = (
            self.create_publisher(MarkerArray, "obstacle_markers", 10)
            if self.publish_debug_markers
            else None
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Obstacle detector active: target_frame={self.target_frame}, "
            f"debug_markers={'on' if self.publish_debug_markers else 'off'}"
        )

    def scan_callback(self, msg):
        try:
            # lookup_transform defaults to a zero-duration timeout, which
            # fails instantly if the buffer hasn't yet processed a /tf
            # sample at this exact scan timestamp -- routine, since /tf is
            # published periodically while the scan stamp is set at capture
            # time. Give it a little room to catch up instead of bailing
            # immediately.
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=0.1),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform {msg.header.frame_id} to {self.target_frame}: {ex}",
                throttle_duration_sec=2.0,
            )
            return

        translation = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

        # --- Step 1 (filter) + Step 2 (Cartesian + TF transform) ---
        points = []  # (original_scan_index, x, y) in target_frame
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x_sensor = r * math.cos(angle)
            y_sensor = r * math.sin(angle)
            x = translation.x + cos_yaw * x_sensor - sin_yaw * y_sensor
            y = translation.y + sin_yaw * x_sensor + cos_yaw * y_sensor
            points.append((i, x, y))

        is_full_circle = (msg.angle_max - msg.angle_min + msg.angle_increment) >= (
            2 * math.pi - 1e-3
        )

        # --- Step 3 (cluster) ---
        clusters = self._cluster(points, len(msg.ranges), is_full_circle)

        # --- Step 4: a fixed-radius circle at each surviving cluster's
        # centroid, no shape/size rejection. Long clusters (e.g. a wall) are
        # first split into arc-length-bounded segments so they become a
        # string of overlapping circles instead of one undersized circle at
        # the whole cluster's centroid. ---
        obstacles = []
        for cluster in clusters:
            if len(cluster) < self.min_cluster_points:
                continue
            for segment in self._split_cluster(cluster):
                xs = np.array([p[1] for p in segment])
                ys = np.array([p[2] for p in segment])
                obstacles.append((float(xs.mean()), float(ys.mean()), self.obstacle_radius))

        obstacle_array = ObstacleArray()
        obstacle_array.header = msg.header
        obstacle_array.header.frame_id = self.target_frame
        for center_x, center_y, radius in obstacles:
            obstacle = Obstacle()
            obstacle.x = center_x
            obstacle.y = center_y
            obstacle.radius = radius
            obstacle_array.obstacles.append(obstacle)
        self.obstacle_pub.publish(obstacle_array)

        if self.marker_pub is not None:
            self._publish_debug_markers(clusters, obstacles)

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

        # A 360-degree scan wraps: the first and last clusters may actually be
        # one object split across the angle_min/angle_max seam.
        if is_full_circle and len(clusters) > 1:
            first, last = clusters[0], clusters[-1]
            index_gap = (first[0][0] + num_ranges) - last[-1][0]
            distance = math.hypot(first[0][1] - last[-1][1], first[0][2] - last[-1][2])
            if index_gap == 1 and distance <= self.cluster_distance_threshold:
                clusters[0] = last + first
                clusters.pop()

        return clusters

    def _split_cluster(self, cluster):
        """Splits a cluster into segments spanning at most 1.5x
        obstacle_radius of arc length each. 1.5x (not 2x) means neighboring
        circles' centers are close enough that their circles overlap,
        rather than just touching -- so a long, slightly curved surface
        (e.g. a wall) doesn't end up with gaps between consecutive circles.
        """
        segment_span = 1.5 * self.obstacle_radius
        segments = [[cluster[0]]]
        accumulated = 0.0
        for prev, curr in zip(cluster, cluster[1:]):
            step = math.hypot(curr[1] - prev[1], curr[2] - prev[2])
            if accumulated + step > segment_span:
                segments.append([curr])
                accumulated = 0.0
            else:
                segments[-1].append(curr)
                accumulated += step
        return segments

    def _publish_debug_markers(self, clusters, obstacles):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        raw_points = Marker()
        raw_points.header.frame_id = self.target_frame
        raw_points.header.stamp = stamp
        raw_points.ns = "filtered_points"
        raw_points.id = 0
        raw_points.type = Marker.POINTS
        raw_points.action = Marker.ADD
        raw_points.scale.x = 0.02
        raw_points.scale.y = 0.02
        raw_points.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.6)
        raw_points.points = [
            Point(x=p[1], y=p[2], z=0.0) for cluster in clusters for p in cluster
        ]
        markers.markers.append(raw_points)

        for idx, cluster in enumerate(clusters):
            color = CLUSTER_COLORS[idx % len(CLUSTER_COLORS)]
            cluster_marker = Marker()
            cluster_marker.header.frame_id = self.target_frame
            cluster_marker.header.stamp = stamp
            cluster_marker.ns = "clusters"
            cluster_marker.id = idx
            cluster_marker.type = Marker.SPHERE_LIST
            cluster_marker.action = Marker.ADD
            cluster_marker.scale.x = 0.04
            cluster_marker.scale.y = 0.04
            cluster_marker.scale.z = 0.04
            cluster_marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.9)
            cluster_marker.points = [Point(x=p[1], y=p[2], z=0.0) for p in cluster]
            markers.markers.append(cluster_marker)

        for idx, (center_x, center_y, radius) in enumerate(obstacles):
            circle_marker = Marker()
            circle_marker.header.frame_id = self.target_frame
            circle_marker.header.stamp = stamp
            circle_marker.ns = "obstacle_circles"
            circle_marker.id = idx
            circle_marker.type = Marker.LINE_STRIP
            circle_marker.action = Marker.ADD
            circle_marker.scale.x = 0.02
            circle_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            steps = 32
            circle_marker.points = [
                Point(
                    x=center_x + radius * math.cos(2 * math.pi * s / steps),
                    y=center_y + radius * math.sin(2 * math.pi * s / steps),
                    z=0.0,
                )
                for s in range(steps + 1)
            ]
            markers.markers.append(circle_marker)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    # scan_callback blocks (briefly) inside lookup_transform's timeout --
    # a single-threaded executor can't service the TF listener's own /tf
    # subscription while that wait is in progress, so the transform it's
    # waiting for could never arrive. A multi-threaded executor lets the
    # two run concurrently.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
