import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

pkg = share('nav_robot')
PARAMS = os.path.join(pkg, 'config', 'nav2_params.yaml')
BT = os.path.join(pkg, 'behavior_trees', 'nav_recovery.xml')


def lifecycle(name, nodes):
    return Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name=name, output='screen',
                parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': nodes,
                             'bond_timeout': 0.0}])


def nav2_node(pkg_name, exe, name, extra=None, remap=None):
    return Node(package=pkg_name, executable=exe, name=name, output='screen',
                parameters=[PARAMS] + ([extra] if extra else []), remappings=remap or [])


def setup(context):
    slam = LaunchConfiguration('slam').perform(context) == 'true'
    nodes = []
    if slam:
        nodes.append(Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                          output='screen', parameters=[os.path.join(pkg, 'config', 'slam.yaml')]))
        # keepout filter needs a static map; skip it while mapping
        loc = []
    else:
        mp = os.path.join(pkg, 'maps', 'office.yaml')
        nodes += [nav2_node('nav2_map_server', 'map_server', 'map_server', {'yaml_filename': mp}),
                  nav2_node('nav2_amcl', 'amcl', 'amcl')]
        loc = ['map_server', 'amcl']

    nav_nodes = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
                 'bt_navigator', 'waypoint_follower', 'velocity_smoother', 'collision_monitor']
    nodes += [
        nav2_node('nav2_controller', 'controller_server', 'controller_server', remap=[('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_smoother', 'smoother_server', 'smoother_server'),
        nav2_node('nav2_planner', 'planner_server', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server', 'behavior_server', remap=[('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', {'default_nav_to_pose_bt_xml': BT}),
        nav2_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
                  remap=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')]),
        nav2_node('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
    ]
    if not slam:
        m = lambda n: os.path.join(pkg, 'maps', n)
        nodes += [
            nav2_node('nav2_map_server', 'map_server', 'keepout_mask_server', {'yaml_filename': m('keepout_mask.yaml')}),
            nav2_node('nav2_map_server', 'costmap_filter_info_server', 'keepout_info_server'),
            nav2_node('nav2_map_server', 'map_server', 'speed_mask_server', {'yaml_filename': m('speed_mask.yaml')}),
            nav2_node('nav2_map_server', 'costmap_filter_info_server', 'speed_info_server'),
        ]
    filt = [] if slam else ['keepout_mask_server', 'keepout_info_server', 'speed_mask_server', 'speed_info_server']
    nodes.append(lifecycle('lifecycle_navigation', filt + loc + nav_nodes))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=setup),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
