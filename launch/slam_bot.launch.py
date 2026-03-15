"""
slam_bot.launch.py — Launch the full SLAM differential drive robot stack

Launches:
  1. robot_state_publisher     (URDF → static TF tree)
  2. custom_lidar_node         (LiDAR → /scan)
  3. diff_drive_controller     (cmd_vel → motors, IMU → /imu/data_raw, /odom/cmd_vel)
  4. robot_localization EKF    (fuses IMU + cmd_vel → /odom + odom→base_link TF)
  5. slam_toolbox              (lifecycle node, auto-activated → /map)

Usage:
  ros2 launch slam_bot slam_bot.launch.py

Optional overrides:
  ros2 launch slam_bot slam_bot.launch.py serial_port:=/dev/ttyAMA0
"""

import os
import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
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

    # 4. Robot Localization EKF
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

    # 5. SLAM Toolbox (lifecycle node — auto-configured and auto-activated)
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

    # ── Auto-activate slam_toolbox lifecycle node ─────────────────────────────
    # Step 1: Send CONFIGURE transition on launch
    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node.node_name == 'slam_toolbox',
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Step 2: When CONFIGURE succeeds (inactive state reached) → send ACTIVATE
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node.node_name == 'slam_toolbox',
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        serial_port_arg,
        robot_state_publisher,
        custom_lidar_node,
        diff_drive_controller,
        ekf_filter_node,
        slam_toolbox,
        configure_slam,
        activate_slam,
    ])
