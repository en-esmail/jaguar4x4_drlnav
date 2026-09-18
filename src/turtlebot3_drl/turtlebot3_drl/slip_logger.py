#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import csv
import os

def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )

class SlipLogger(Node):
    def __init__(self):
        super().__init__('slip_logger')

        # Subscribers
        self.create_subscription(Twist, '/ros_drrobot_motor_cmd', self.cmd_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # State
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.prev_t = None

        # CSV setup
        self.csv_file = open('slip_metrics.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'time',
            'x', 'y',
            'v_cmd', 'w_cmd',
            'v_actual', 'w_actual',
            'slip_ratio',
            'heading_error'
        ])

        self.get_logger().info("Slip logger started")

    def cmd_cb(self, msg):
        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z

    def odom_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)

        if self.prev_t is None:
            self.prev_x, self.prev_y = x, y
            self.prev_yaw = yaw
            self.prev_t = t
            return

        dt = t - self.prev_t
        if dt < 0.02:
            return

        # Actual velocities
        dx = x - self.prev_x
        dy = y - self.prev_y
        v_actual = math.sqrt(dx*dx + dy*dy) / dt

        dyaw = yaw - self.prev_yaw
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        w_actual = dyaw / dt

        # Slip ratio
        if abs(self.v_cmd) > 0.05:
            slip = 1.0 - (v_actual / abs(self.v_cmd))
            slip = max(0.0, min(slip, 1.0))
        else:
            slip = 0.0

        # Heading error (yaw slip)
        heading_error = abs(self.w_cmd - w_actual)

        # Write to CSV
        self.writer.writerow([
            t,
            x, y,
            self.v_cmd, self.w_cmd,
            v_actual, w_actual,
            slip,
            heading_error
        ])
        self.csv_file.flush()

        # Update state
        self.prev_x, self.prev_y = x, y
        self.prev_yaw = yaw
        self.prev_t = t

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SlipLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
