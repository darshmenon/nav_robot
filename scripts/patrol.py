#!/usr/bin/env python3
"""Loop through waypoints with Nav2, then return to the dock. Usage: patrol.py [--loops N]"""
import argparse
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

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
    nav.setInitialPose(pose(nav, *DOCK))
    nav.waitUntilNav2Active()
    for i in range(args.loops):
        print(f'loop {i + 1}/{args.loops}')
        nav.followWaypoints([pose(nav, *w) for w in WAYPOINTS])
        last = -1
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb and fb.current_waypoint != last:
                last = fb.current_waypoint
                print(f'  waypoint {last + 1}/{len(WAYPOINTS)}')
        print('\n ', nav.getResult())
    print('returning to dock')
    nav.goToPose(pose(nav, *DOCK))
    while not nav.isTaskComplete():
        pass
    print('docked' if nav.getResult() == TaskResult.SUCCEEDED else f'dock failed: {nav.getResult()}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
