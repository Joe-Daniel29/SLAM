"""
slam_bot.launch.py — Launch the full SLAM differential drive robot stack

Launches:
  1. robot_state_publisher     (URDF → static TF tree)
  2. custom_lidar_node         (LiDAR → /scan)
  3. diff_drive_controller     (cmd_vel → motors, IMU → /imu/data_raw, /odom/cmd_vel)
  4. rf2o_laser_odometry       (scan-matching → /odom/rf2o)
  5. robot_localization EKF    (fuses rf2o + IMU + cmd_vel → /odom + odom→base_link TF)
  6. slam_toolbox              (online async SLAM → /map)

Usage:
  ros2 launch slam_bot slam_bot.launch.py

Optional overrides:
  ros2 launch slam_bot slam_bot.launch.py serial_port:=/dev/ttyUSB1
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_bot')

    # ── URDF ──────────────────────────────────────────────────────────────────
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_description.urdf')
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # ── Launch arguments ──────────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Serial port for Arduino motor/IMU bridge'
    )

    # ── Nodes ─────────────────────────────────────────────────────────────────

    # 1. Robot State Publisher — broadcasts URDF static transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False,
        }],
        output='screen',
    )

    # 2. Custom LiDAR Node — publishes /scan
    custom_lidar_node = Node(
        package='slam_bot',
        executable='custom_lidar_node',
        name='custom_lidar_node',
        output='screen',
    )

    # 3. Diff Drive Controller — cmd_vel → serial, IMU → /imu/data_raw, dead-reckoning → /odom/cmd_vel
    diff_drive_controller = Node(
        package='slam_bot',
        executable='diff_drive_controller',
        name='diff_drive_controller',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200,
            'wheel_radius': 0.09,
            'wheel_separation': 0.2,
            'max_rpm': 600.0,
            'cmd_vel_timeout_ms': 500,
        }],
        output='screen',
    )

    # 4. RF2O Laser Odometry — scan-matching → /odom/rf2o
    rf2o_laser_odometry = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom/rf2o',
            'publish_tf': False,           # EKF handles TF
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'freq': 20.0,                  # Match our odom rate
        }],
        output='screen',
    )

    # 5. Robot Localization EKF — fuses rf2o + IMU + cmd_vel → /odom + odom→base_link TF
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf_params.yaml')
        ],
        output='screen',
    )

    # 6. SLAM Toolbox — online async SLAM → /map
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
        ],
        output='screen',
    )

    return LaunchDescription([
        serial_port_arg,
        robot_state_publisher,
        custom_lidar_node,
        diff_drive_controller,
        rf2o_laser_odometry,
        ekf_filter_node,
        slam_toolbox,
    ])
