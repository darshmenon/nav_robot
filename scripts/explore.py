#!/usr/bin/env python3
"""Minimal frontier exploration for slam:=true. Drives to the nearest frontier until none are left, then saves the map.
Usage: explore.py [--save PATH_PREFIX] [--max-goals N]"""
import argparse
import math
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from tf2_ros import Buffer, TransformListener


def frontiers(grid, robot=None, min_cells=4, min_dist=0.9):
    pad = 20     # the map array only spans what has been seen, so frontiers at its edge need unknown cells beyond them
    a = np.pad(np.array(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width), pad, constant_values=-1)
    ox = grid.info.origin.position.x - pad * grid.info.resolution
    oy = grid.info.origin.position.y - pad * grid.info.resolution
    free, unknown = a == 0, a == -1
    near_unknown = ndimage.binary_dilation(unknown, iterations=1)
    occ = a >= 65
    clear = ndimage.distance_transform_edt(~occ) * grid.info.resolution >= 0.45   # keep goals out of the inflated margin
    edge = free & near_unknown & clear
    if robot is not None:      # ray-gap speckle around the robot would merge into blobs centred on it, so cut it out first
        h, w = a.shape
        yy, xx = np.mgrid[0:h, 0:w]
        wx = ox + (xx + .5) * grid.info.resolution
        wy = oy + (yy + .5) * grid.info.resolution
        edge &= np.hypot(wx - robot[0], wy - robot[1]) >= min_dist
    lab, n = ndimage.label(edge, structure=np.ones((3, 3)))
    out = []
    for i in range(1, n + 1):
        ys, xs = np.nonzero(lab == i)
        if len(ys) < min_cells:
            continue
        cy, cx = int(ys.mean()), int(xs.mean())
        k = np.argmin((ys - cy) ** 2 + (xs - cx) ** 2)       # a real frontier cell near the centroid
        out.append((ox + (xs[k] + .5) * grid.info.resolution, oy + (ys[k] + .5) * grid.info.resolution))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--save', default='')
    ap.add_argument('--max-goals', type=int, default=40)
    args = ap.parse_args()
    rclpy.init()
    nav = BasicNavigator()
    buf = Buffer()
    TransformListener(buf, nav)
    latest = {}
    nav.create_subscription(OccupancyGrid, '/map', lambda m: latest.__setitem__('m', m),
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                       reliability=ReliabilityPolicy.RELIABLE))
    bad, empty = [], 0
    for i in range(args.max_goals):
        for _ in range(20):
            rclpy.spin_once(nav, timeout_sec=0.2)
        if 'm' not in latest:
            continue
        try:
            t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform.translation
        except Exception:
            continue
        cands = [f for f in frontiers(latest['m'], (t.x, t.y))
                 if all(math.dist(f, b) > 0.8 for b in bad)]
        if not cands:
            empty += 1
            if empty >= 4:
                print('no frontiers left')
                break
            nav.spin(spin_dist=6.28)    # SLAM may not have seen enough yet: turn on the spot and look again
            t0 = time.time()
            while not nav.isTaskComplete() and time.time() - t0 < 25:
                rclpy.spin_once(nav, timeout_sec=0.2)
            continue
        empty = 0
        goal = min(cands, key=lambda f: math.dist(f, (t.x, t.y)))
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = nav.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y, p.pose.orientation.w = float(goal[0]), float(goal[1]), 1.0
        nav.goToPose(p)
        t0 = time.time()
        while not nav.isTaskComplete() and time.time() - t0 < 45:
            rclpy.spin_once(nav, timeout_sec=0.2)
        if not nav.isTaskComplete():
            nav.cancelTask()
        res = nav.getResult()
        bad.append(goal)          # visited either way: a frontier that stays after we reached it is just sensor speckle
        print(f'goal {i + 1}: ({goal[0]:.1f}, {goal[1]:.1f}) {str(res).split(".")[-1]}', flush=True)
    if args.save:
        subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', args.save, '--ros-args', '-p', 'save_map_timeout:=10000.0'])
    rclpy.shutdown()


if __name__ == '__main__':
    main()
