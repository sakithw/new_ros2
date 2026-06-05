from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('museum_robot')
    nav2_cfg = os.path.join(pkg, 'config', 'nav2_params.yaml')
    reset_script = os.path.join(pkg, 'scripts', 'reset_lidar.py')
    map_yaml = '/home/pi/maps2/museum_map.yaml'
    return LaunchDescription([
        ExecuteProcess(cmd=['python3', reset_script], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_laser_tf', output='screen',
             arguments=['--x','0.10','--y','0.0','--z','0.15',
                        '--roll','0.0','--pitch','0.0','--yaw','0.0',
                        '--frame-id','base_link','--child-frame-id','laser']),
        Node(package='museum_robot', executable='arduino_bridge',
             name='arduino_bridge', output='screen',
             respawn=True, respawn_delay=2.0),
        Node(package='museum_robot', executable='scan_filter',
             name='scan_filter', output='screen',
             respawn=True, respawn_delay=2.0),
        TimerAction(period=5.0, actions=[
            Node(package='nav2_map_server', executable='map_server',
                 name='map_server', output='screen',
                 parameters=[nav2_cfg, {'yaml_filename': map_yaml}]),
            Node(package='nav2_amcl', executable='amcl',
                 name='amcl', output='screen', parameters=[nav2_cfg]),
            Node(package='nav2_controller', executable='controller_server',
                 name='controller_server', output='screen', parameters=[nav2_cfg]),
            Node(package='nav2_smoother', executable='smoother_server',
                 name='smoother_server', output='screen', parameters=[nav2_cfg]),
            Node(package='nav2_planner', executable='planner_server',
                 name='planner_server', output='screen', parameters=[nav2_cfg]),
            Node(package='nav2_behaviors', executable='behavior_server',
                 name='behavior_server', output='screen', parameters=[nav2_cfg]),
            Node(package='nav2_bt_navigator', executable='bt_navigator',
                 name='bt_navigator', output='screen', parameters=[nav2_cfg]),
            Node(package='nav2_waypoint_follower', executable='waypoint_follower',
                 name='waypoint_follower', output='screen', parameters=[nav2_cfg]),
            Node(package='nav2_velocity_smoother', executable='velocity_smoother',
                 name='velocity_smoother', output='screen', parameters=[nav2_cfg]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'autostart':True,
                               'node_names':['map_server','amcl']}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[{'autostart':True,
                               'node_names':['controller_server','smoother_server',
                                             'planner_server','behavior_server',
                                             'bt_navigator','waypoint_follower',
                                             'velocity_smoother']}]),
        ]),
        TimerAction(period=22.0, actions=[
            Node(package='sllidar_ros2', executable='sllidar_node',
                 name='sllidar_node', output='screen',
                 respawn=True, respawn_delay=25.0,
                 parameters=[{'channel_type':'serial',
                               'serial_port':'/dev/lidar',
                               'serial_baudrate':1000000,
                               'frame_id':'laser',
                               'angle_compensate':True,
                               'scan_mode':'DenseBoost'}])
        ]),
    ])
