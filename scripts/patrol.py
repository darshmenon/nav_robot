#!/usr/bin/env python3
"""Loop through waypoints with Nav2, then return to the dock. Usage: patrol.py [--loops N]"""
import argparse
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry

DOCK = (-4.0, -4.0, 0.0)
WAYPOINTS = [(-4.0, 0.5, 1.57), (-3.0, 4.0, 0.0), (0.5, 0.2, -1.57), (2.0, 3.8, 0.0), (4.0, 4.0, -1.57)]


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--loops', type=int, default=1)
    args = ap.parse_args()
    rclpy.init()
    nav = BasicNavigator()
    dist = {'d': 0.0, 'last': None}

    def on_odom(m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        if dist['last'] is not None:
            dist['d'] += math.dist(p, dist['last'])
        dist['last'] = p

    nav.create_subscription(Odometry, '/odom', on_odom, 10)
    nav.setInitialPose(pose(nav, *DOCK))
    nav.waitUntilNav2Active()
    for i in range(args.loops):
        print(f'loop {i + 1}/{args.loops}')
        nav.followWaypoints([pose(nav, *w) for w in WAYPOINTS])
        last = -1
        while not nav.isTaskComplete():
            rclpy.spin_once(nav, timeout_sec=0.05)
            fb = nav.getFeedback()
            if fb and fb.current_waypoint != last:
                last = fb.current_waypoint
                print(f'  waypoint {last + 1}/{len(WAYPOINTS)}')
        print('\n ', nav.getResult())
    print('returning to dock')
    nav.goToPose(pose(nav, *DOCK))
    while not nav.isTaskComplete():
        rclpy.spin_once(nav, timeout_sec=0.05)
    print('docked' if nav.getResult() == TaskResult.SUCCEEDED else f'dock failed: {nav.getResult()}')
    print(f'driven {dist["d"]:.1f} m')
    if dist['d'] < 1.0:
        print('WARNING: the robot barely moved, so the "success" above is not trustworthy (blind lidar or stalled sim?)')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
