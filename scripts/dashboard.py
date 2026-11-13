#!/usr/bin/env python3
"""Web dashboard: click the map to send Nav2 goals. Open http://localhost:8080"""
import io
import math
import os
import threading
import rclpy
from ament_index_python.packages import get_package_share_directory
from flask import Flask, Response, jsonify, request, send_from_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

WEB = os.path.join(get_package_share_directory('nav_robot'), 'web')
MAPS = os.path.join(get_package_share_directory('nav_robot'), 'maps')
app = Flask(__name__, static_folder=WEB)
rclpy.init()
nav = Node('dashboard')
client = ActionClient(nav, NavigateToPose, 'navigate_to_pose')
handle = {'h': None}
tfbuf = Buffer()
TransformListener(tfbuf, nav)
scan = {'pts': []}
state = {'status': 'idle'}


def on_scan(msg):
    pts, a = [], msg.angle_min
    for r in msg.ranges[::6]:
        if msg.range_min < r < msg.range_max:
            pts.append((r * math.cos(a), r * math.sin(a)))
        a += msg.angle_increment * 6
    scan['pts'] = pts


nav.create_subscription(LaserScan, '/scan', on_scan, 1)
threading.Thread(target=lambda: rclpy.spin(nav), daemon=True).start()


@app.get('/')
def index():
    return send_from_directory(WEB, 'index.html')


@app.get('/map.png')
def map_png():
    from PIL import Image
    buf = io.BytesIO()
    Image.open(os.path.join(MAPS, 'office.pgm')).save(buf, 'PNG')
    return Response(buf.getvalue(), mimetype='image/png')


@app.get('/api/state')
def api_state():
    try:
        t = tfbuf.lookup_transform('map', 'base_footprint', Time(), Duration(seconds=0.2))
        q = t.transform.rotation
        pose = {'x': t.transform.translation.x, 'y': t.transform.translation.y,
                'yaw': math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)}
    except Exception:
        pose = None
    return jsonify(pose=pose, status=state['status'], scan=scan['pts'])


@app.post('/api/goal')
def api_goal():
    d = request.get_json()
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(d['x']), float(d['y'])
    yaw = float(d.get('yaw', 0))
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    if not client.server_is_ready():
        return jsonify(ok=False, error='nav2 not ready'), 503
    goal = NavigateToPose.Goal()
    goal.pose = p
    state['status'] = 'navigating'

    def on_accept(fut):
        h = fut.result()
        if not h.accepted:
            state['status'] = 'rejected'
            return
        handle['h'] = h
        h.get_result_async().add_done_callback(on_result)

    def on_result(fut):
        names = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled',
                 GoalStatus.STATUS_ABORTED: 'failed'}
        state['status'] = names.get(fut.result().status, 'idle')

    client.send_goal_async(goal).add_done_callback(on_accept)
    return jsonify(ok=True)


@app.post('/api/cancel')
def api_cancel():
    if handle['h'] is not None:
        handle['h'].cancel_goal_async()
    return jsonify(ok=True)


if __name__ == '__main__':
    app.run(host='127.0.0.1', port=8080, threaded=True)
