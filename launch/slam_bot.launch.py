"""
slam_bot.launch.py — Launch the full SLAM differential drive robot stack

Launches:
  1. robot_state_publisher     (URDF → static TF tree)
  2. custom_lidar_node         (LiDAR → /scan)
  3. diff_drive_controller     (cmd_vel → motors, IMU → /imu/data_raw, /odom/cmd_vel)
  4. rf2o_laser_odometry       (scan-matching → /odom/rf2o)
  5. robot_localization EKF    (fuses rf2o + IMU + cmd_vel → /odom + odom→base_link TF)
  6. slam_toolbox              (lifecycle node, auto-activated → /map)

Usage:
  ros2 launch slam_bot slam_bot.launch.py

Optional overrides:
  ros2 launch slam_bot slam_bot.launch.py serial_port:=/dev/ttyAMA0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_bot')

    # ── URDF ──────────────────────────────────────────────────────────────────
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_description.urdf')
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # ── Launch arguments ──────────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyAMA0',
        description='Serial port for Arduino motor/IMU bridge'
    )

    # ── Nodes ─────────────────────────────────────────────────────────────────

    # 1. Robot State Publisher
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

    # 2. Custom LiDAR Node
    custom_lidar_node = Node(
        package='slam_bot',
        executable='custom_lidar_node',
        name='custom_lidar_node',
        output='screen',
    )

    # 3. Diff Drive Controller
    diff_drive_controller = Node(
        package='slam_bot',
        executable='diff_drive_controller',
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

    # 4. rf2o Laser Odometry (scan-matching → /odom/rf2o for EKF)
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom/rf2o',
            'publish_tf': False,           # EKF owns the TF, not rf2o
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',    # Empty = start from origin, don't wait for ground truth
            'freq': 10.0,                 # Match LiDAR ~7.5 Hz, slight oversample
        }],
        output='screen',
    )

    # 5. Robot Localization EKF
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        remappings=[
            ('odometry/filtered', '/odom'),
        ],
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf_params.yaml')
        ],
        output='screen',
    )

    # 6. SLAM Toolbox (lifecycle node — must be configured + activated)
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[
            os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
        ],
        output='screen',
    )

    # Auto-activate slam_toolbox — retry loop until the node is available
    activate_slam = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'source /opt/ros/jazzy/setup.bash && '
                     'source ~/ros2_ws/install/setup.bash && '
                     'echo "Waiting for slam_toolbox lifecycle node..." && '
                     'for i in $(seq 1 30); do '
                     '  ros2 lifecycle set /slam_toolbox configure 2>/dev/null && break; '
                     '  echo "  Attempt $i: slam_toolbox not ready, retrying in 2s..."; '
                     '  sleep 2; '
                     'done && '
                     'sleep 2 && '
                     'ros2 lifecycle set /slam_toolbox activate && '
                     'echo "slam_toolbox ACTIVE — /map should be publishing"'],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        serial_port_arg,
        robot_state_publisher,
        custom_lidar_node,
        diff_drive_controller,
        rf2o_node,
        ekf_filter_node,
        slam_toolbox,
        activate_slam,
    ])
