from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('museum_robot')
    slam_cfg   = os.path.join(pkg, 'config', 'slam_toolbox.yaml')
    tcp_relay  = os.path.join(pkg, 'scripts', 'lidar_tcp_relay.py')
    return LaunchDescription([
        # Kill anything holding ttyAMA0 (Arduino port)
        ExecuteProcess(cmd=['bash', '-c', 'fuser -k /dev/ttyAMA0 2>/dev/null; sleep 0.5'],
                       output='screen'),

        # TCP relay: opens /dev/lidar, sends RESET, waits 1.2s for health=0 window,
        # then listens on TCP 127.0.0.1:10660 and bridges to the real serial port.
        # This is the only process that opens /dev/lidar — no DTR glitch possible.
        ExecuteProcess(cmd=['python3', tcp_relay], output='screen',
                       respawn=True, respawn_delay=2.0),

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

        # rplidar_composition via TCP relay — relay ready in ~1.5s, start at t=3s.
        # Uses channel_type:=tcp to avoid any serial port DTR glitch.
        # Short respawn_delay: relay stays up, so re-open = new TCP connect = new RESET.
        TimerAction(period=3.0, actions=[
            Node(package='rplidar_ros', executable='rplidar_composition',
                 name='sllidar_node', output='screen',
                 respawn=True, respawn_delay=3.0,
                 parameters=[{'channel_type':'tcp',
                               'tcp_ip':'127.0.0.1',
                               'tcp_port':10660,
                               'frame_id':'laser',
                               'angle_compensate':True,
                               'scan_mode':'Standard'}])
        ]),
    ])
