import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = share('nav_robot')
    headless = LaunchConfiguration('headless')
    x, y, yaw = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('yaw')
    robot_desc = Command(['xacro ', os.path.join(pkg, 'urdf', 'robot.urdf.xacro')])

    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', os.path.join(pkg, 'worlds', 'office.sdf')],
        output='screen', condition=IfCondition(headless))
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', os.path.join(pkg, 'worlds', 'office.sdf')],
        output='screen', condition=UnlessCondition(headless))

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('x', default_value='-4.0'),
        DeclareLaunchArgument('y', default_value='-4.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gz_headless, gz_gui,
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]),
        TimerAction(period=4.0, actions=[ExecuteProcess(output='screen', shell=True, cmd=[
            'xacro ', os.path.join(pkg, 'urdf', 'robot.urdf.xacro'), ' > /tmp/nav_robot.urdf && ',
            'gz service -s /world/office/create --reqtype gz.msgs.EntityFactory ',
            '--reptype gz.msgs.Boolean --timeout 60000 --req "sdf_filename: \'/tmp/nav_robot.urdf\' name: \'nav_robot\' ',
            'pose: {position: {x: ', x, ' y: ', y, ' z: 0.05} orientation: {z: $(awk \'BEGIN{print sin(', yaw, '/2)}\') ',
            'w: $(awk \'BEGIN{print cos(', yaw, '/2)}\')}}"'])]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml')}]),
        Node(package='robot_localization', executable='ekf_node', output='screen',
             parameters=[os.path.join(pkg, 'config', 'ekf.yaml')]),
    ])
