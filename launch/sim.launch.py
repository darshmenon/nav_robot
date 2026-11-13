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
    world, world_name = LaunchConfiguration('world'), LaunchConfiguration('world_name')
    x, y, yaw = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('yaw')
    robot_desc = Command(['xacro ', os.path.join(pkg, 'urdf', 'robot.urdf.xacro')])

    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', world],
        output='screen', condition=IfCondition(headless))
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen', condition=UnlessCondition(headless))

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'office.sdf')),
        DeclareLaunchArgument('world_name', default_value='office'),
        DeclareLaunchArgument('x', default_value='-4.0'),
        DeclareLaunchArgument('y', default_value='-4.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gz_headless, gz_gui,
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]),
        TimerAction(period=4.0, actions=[ExecuteProcess(output='screen', shell=True, cmd=[
            'xacro ', os.path.join(pkg, 'urdf', 'robot.urdf.xacro'), ' > /tmp/nav_robot.urdf && ',
            ['gz service -s /world/', world_name, '/create --reqtype gz.msgs.EntityFactory '],
            '--reptype gz.msgs.Boolean --timeout 60000 --req "sdf_filename: \'/tmp/nav_robot.urdf\' name: \'nav_robot\' ',
            'pose: {position: {x: ', x, ' y: ', y, ' z: 0.05} orientation: {z: $(awk \'BEGIN{print sin(', yaw, '/2)}\') ',
            'w: $(awk \'BEGIN{print cos(', yaw, '/2)}\')}}"'])]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                        ['/world/', world_name, '/model/nav_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model']],
             remappings=[(['/world/', world_name, '/model/nav_robot/joint_state'], '/joint_states')]),
        Node(package='robot_localization', executable='ekf_node', output='screen',
             parameters=[os.path.join(pkg, 'config', 'ekf.yaml')]),
    ])
