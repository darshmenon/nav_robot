#!/usr/bin/env python3
"""Warn when the lidar goes blind (no finite beams) or silent. A blind lidar makes Nav2 report goals as succeeded while the robot never moves."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanWatchdog(Node):
    def __init__(self):
        super().__init__('scan_watchdog')
        self.last = None
        self.finite = 0
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(5.0, self.check)

    def on_scan(self, msg):
        self.last = self.get_clock().now()
        self.finite = sum(1 for r in msg.ranges if math.isfinite(r))

    def check(self):
        if self.last is None:
            self.get_logger().warn('no /scan received yet')
        elif (self.get_clock().now() - self.last).nanoseconds > 3e9:
            self.get_logger().error('/scan has gone silent for more than 3 s')
        elif self.finite == 0:
            self.get_logger().error('LIDAR BLIND: every beam is inf. Restart the sim; Nav2 results are not trustworthy.')


def main():
    rclpy.init()
    rclpy.spin(ScanWatchdog())


if __name__ == '__main__':
    main()
