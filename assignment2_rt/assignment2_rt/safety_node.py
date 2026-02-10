#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from assignment2_custom_msg.msg import ObstacleInfo
from assignment2_custom_msg.srv import SetThreshold
import math


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        self.threshold = 0.8
        self.last_safe_pose = None

        # Recovery flag to avoid infinite backward motion
        self.recovering = False

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obs_pub = self.create_publisher(ObstacleInfo, '/closest_obstacle', 10)

        self.create_service(SetThreshold, '/set_threshold', self.set_threshold_cb)

        self.get_logger().info("Safety Node started")

    def odom_callback(self, msg):
        self.last_safe_pose = msg.pose.pose

    def scan_callback(self, msg):
        # Filter invalid laser readings
        valid_ranges = [
            r for r in msg.ranges
            if not math.isinf(r) and not math.isnan(r)
        ]

        if not valid_ranges:
            return

        min_dist = min(valid_ranges)
        min_index = msg.ranges.index(min_dist)

        direction = self.get_direction(min_index, len(msg.ranges))

        # Publish obstacle info
        info = ObstacleInfo()
        info.distance = min_dist
        info.direction = direction
        info.threshold = self.threshold
        self.obs_pub.publish(info)

        # Safety logic
        if min_dist < self.threshold:
            if not self.recovering:
                self.get_logger().warn("Obstacle too close! Moving back.")
                self.recovering = True
            self.move_back()
        else:
            if self.recovering:
                self.stop_robot()
                self.recovering = False

    def get_direction(self, index, total):
        third = total // 3
        if index < third:
            return "right"
        elif index < 2 * third:
            return "front"
        else:
            return "left"

    def move_back(self):
        cmd = Twist()
        cmd.linear.x = -0.3
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()  # all zeros
        self.cmd_pub.publish(cmd)

    def set_threshold_cb(self, request, response):
        self.threshold = request.threshold
        response.success = True
        self.get_logger().info(f"Threshold set to {self.threshold}")
        return response


def main():
    rclpy.init()
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
