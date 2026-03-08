#!/usr/bin/env python3
"""
color_search_node.py  — DYNAMIC VERSION
========================================
Detects red ball → computes its REAL world position using:
  1. Camera centroid  → horizontal bearing angle to ball
  2. LiDAR scan       → distance in that direction  
  3. AMCL pose        → robot position in map frame
Then sends Nav2 goal directly to ball's location.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import cv2
import numpy as np
import math

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge

# ── HSV thresholds for RED ────────────────────────────────────────────────────
RED_LOWER1 = np.array([0,   120,  70])
RED_UPPER1 = np.array([10,  255, 255])
RED_LOWER2 = np.array([170, 120,  70])
RED_UPPER2 = np.array([180, 255, 255])
MIN_CONTOUR_AREA = 300

# Camera horizontal FOV (matches SDF: 1.3962634 rad = 80°)
HFOV = 1.3962634

# Stop this far in front of ball
STOP_DISTANCE = 0.35

SEARCHING  = "SEARCHING"
DETECTED   = "DETECTED"
NAVIGATING = "NAVIGATING"
DONE       = "DONE"


class ColorSearchNode(Node):

    def __init__(self):
        super().__init__("color_search_node")

        self.declare_parameter("rotate_speed",     0.4)
        self.declare_parameter("min_contour_area", MIN_CONTOUR_AREA)
        self.declare_parameter("stop_distance",    STOP_DISTANCE)

        self.rotate_speed = self.get_parameter("rotate_speed").value
        self.min_area     = self.get_parameter("min_contour_area").value
        self.stop_dist    = self.get_parameter("stop_distance").value

        self.state       = SEARCHING
        self.bridge      = CvBridge()
        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.robot_yaw   = 0.0
        self.latest_scan = None
        self.image_width = 640

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_pub  = self.create_publisher(Twist, "/cmd_vel", 10)
        self.img_sub  = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, sensor_qos)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, sensor_qos)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)

        self.nav_client   = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.rotate_timer = self.create_timer(0.1, self.rotate_callback)

        self.get_logger().info(
            "color_search_node (DYNAMIC) started → STATE: SEARCHING"
        )

    # ── Sensor callbacks ──────────────────────────────────────────────────────
    def scan_callback(self, msg):
        self.latest_scan = msg

    def pose_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    # ── Camera callback ───────────────────────────────────────────────────────
    def image_callback(self, msg):
        if self.state != SEARCHING:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge: {e}")
            return

        h, w = frame.shape[:2]
        self.image_width = w
        cx, area = self._detect_red(frame)

        if cx is not None:
            self.get_logger().info(
                f"🔴 Red ball DETECTED! centroid_x={cx}/{w}, area={area:.0f}"
            )
            self.state = DETECTED
            self._stop_robot()
            self._compute_and_send_goal(cx, w)

    # ── Red detection ─────────────────────────────────────────────────────────
    def _detect_red(self, bgr):
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m1   = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
        m2   = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
        mask = cv2.bitwise_or(m1, m2)
        k    = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,   k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, k)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, 0

        largest = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)
        if area < self.min_area:
            return None, 0

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None, 0
        return int(M["m10"] / M["m00"]), area

    # ── Compute real ball position and navigate ───────────────────────────────
    def _compute_and_send_goal(self, centroid_x, image_width):
        # 1. Bearing angle from camera centroid (left=positive, right=negative)
        norm_x      = (centroid_x - image_width / 2.0) / (image_width / 2.0)
        bearing_cam = -norm_x * (HFOV / 2.0)   # radians relative to robot forward

        # 2. Distance from LiDAR
        distance = self._lidar_distance_at_bearing(bearing_cam)
        if distance is None:
            self.get_logger().warn("LiDAR unavailable — using fallback 1.0 m")
            distance = 1.0

        self.get_logger().info(
            f"Ball → bearing: {math.degrees(bearing_cam):.1f}°, "
            f"distance: {distance:.2f} m"
        )

        # 3. World coordinates
        abs_angle      = self.robot_yaw + bearing_cam
        effective_dist = max(distance - self.stop_dist, 0.15)

        goal_x = self.robot_x + effective_dist * math.cos(abs_angle)
        goal_y = self.robot_y + effective_dist * math.sin(abs_angle)

        self.get_logger().info(
            f"Robot @ ({self.robot_x:.2f}, {self.robot_y:.2f}, "
            f"yaw={math.degrees(self.robot_yaw):.1f}°) → "
            f"Goal @ ({goal_x:.2f}, {goal_y:.2f})"
        )
        self._send_nav_goal(goal_x, goal_y, abs_angle)

    # ── LiDAR distance at a bearing ───────────────────────────────────────────
    def _lidar_distance_at_bearing(self, bearing_rad):
        if self.latest_scan is None:
            return None
        scan  = self.latest_scan
        angle = bearing_rad

        # Normalise to scan range
        while angle < scan.angle_min:
            angle += 2 * math.pi
        while angle > scan.angle_max:
            angle -= 2 * math.pi

        if not (scan.angle_min <= angle <= scan.angle_max):
            return None

        idx    = int(round((angle - scan.angle_min) / scan.angle_increment))
        idx    = max(0, min(idx, len(scan.ranges) - 1))
        window = 5
        valid  = []
        for i in range(idx - window, idx + window + 1):
            if 0 <= i < len(scan.ranges):
                r = scan.ranges[i]
                if scan.range_min < r < scan.range_max:
                    valid.append(r)

        return float(np.median(valid)) if valid else None

    # ── Send Nav2 goal ────────────────────────────────────────────────────────
    def _send_nav_goal(self, x, y, yaw):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 server unavailable — restarting search")
            self.state = SEARCHING
            return

        goal      = NavigateToPose.Goal()
        pose      = PoseStamped()
        pose.header.frame_id      = "map"
        pose.header.stamp         = self.get_clock().now().to_msg()
        pose.pose.position.x      = x
        pose.pose.position.y      = y
        pose.pose.orientation.z   = math.sin(yaw / 2.0)
        pose.pose.orientation.w   = math.cos(yaw / 2.0)
        goal.pose = pose

        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._response_cb)
        self.state = NAVIGATING

    def _response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal REJECTED — restarting search")
            self.state = SEARCHING
            return
        self.get_logger().info("Nav2 goal ACCEPTED — navigating to ball!")
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        self.get_logger().info(
            f"[Nav2] Distance remaining: "
            f"{feedback_msg.feedback.distance_remaining:.2f} m",
            throttle_duration_sec=2.0,
        )

    def _result_cb(self, future):
        if future.result().status == 4:
            self.get_logger().info(
                "✅ Navigation SUCCEEDED! Robot reached the red ball.")
            self.state = DONE
        else:
            self.get_logger().warn(
                f"Navigation status {future.result().status} — restarting search")
            self.state = SEARCHING

    # ── Rotation ──────────────────────────────────────────────────────────────
    def rotate_callback(self):
        if self.state == SEARCHING:
            twist = Twist()
            twist.angular.z = self.rotate_speed
            self.cmd_pub.publish(twist)

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())
        self.get_logger().info("Robot stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = ColorSearchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
