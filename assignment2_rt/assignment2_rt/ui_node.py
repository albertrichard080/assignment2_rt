#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from assignment2_custom_msg.srv import GetAverageVelocity
import time


class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')

        # Publisher for robot velocity
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Store last 5 (linear, angular) commands
        self.commands = []

        # Service to get average velocity
        self.create_service(
            GetAverageVelocity,
            '/get_average_velocity',
            self.avg_velocity_cb
        )

        self.get_logger().info("UI Node started")

    def send_command(self, lin, ang):
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.cmd_pub.publish(cmd)

        # Store command
        self.commands.append((lin, ang))
        if len(self.commands) > 5:
            self.commands.pop(0)

        # Move for 1 second
        time.sleep(1)

        # Stop robot
        self.cmd_pub.publish(Twist())

    def avg_velocity_cb(self, request, response):
        if not self.commands:
            response.avg_linear = 0.0
            response.avg_angular = 0.0
        else:
            response.avg_linear = sum(c[0] for c in self.commands) / len(self.commands)
            response.avg_angular = sum(c[1] for c in self.commands) / len(self.commands)

        return response


def main():
    rclpy.init()
    node = UINode()

    try:
        while rclpy.ok():
            # Allow ROS to process service callbacks
            rclpy.spin_once(node, timeout_sec=0.1)

            # User input
            lin = float(input("Linear velocity: "))
            ang = float(input("Angular velocity: "))

            node.send_command(lin, ang)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
