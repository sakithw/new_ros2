from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('museum_robot')
    slam_cfg = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    reset_script = os.path.join(pkg, 'scripts', 'reset_lidar.py')
    return LaunchDescription([
        ExecuteProcess(cmd=['bash', '-c', 'fuser -k /dev/ttyAMA0 2>/dev/null; sleep 0.5'],
                       output='screen'),
        ExecuteProcess(cmd=['python3', reset_script], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_laser_tf', output='screen',
             arguments=['--x','0.10','--y','0.0','--z','0.15',
                        '--roll','0.0','--pitch','0.0','--yaw','0.0',
                        '--frame-id','base_link','--child-frame-id','laser']),
        Node(package='museum_robot', executable='arduino_bridge',
             name='arduino_bridge', output='screen'),
        Node(package='museum_robot', executable='lidar_watchdog',
             name='lidar_watchdog', output='screen',
             respawn=True, respawn_delay=2.0),
        TimerAction(period=10.0, actions=[
            Node(package='slam_toolbox',
                 executable='async_slam_toolbox_node',
                 name='slam_toolbox', output='screen',
                 parameters=[slam_cfg])
        ]),
        TimerAction(period=24.0, actions=[
            Node(package='rplidar_ros', executable='rplidar_composition',
                 name='sllidar_node', output='screen',
                 respawn=True, respawn_delay=25.0,
                 parameters=[{'channel_type':'serial',
                               'serial_port':'/dev/lidar',
                               'serial_baudrate':1000000,
                               'frame_id':'laser',
                               'angle_compensate':True,
                               'scan_mode':'Standard'}])
        ]),
    ])
