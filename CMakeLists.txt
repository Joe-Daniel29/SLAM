cmake_minimum_required(VERSION 3.8)
project(slam_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ── Nodes ─────────────────────────────────────────────────────────────────────

# 1. Differential drive controller (cmd_vel → serial motor packets, serial IMU → /imu/data_raw, odometry)
add_executable(diff_drive_controller src/diff_drive_controller.cpp)
ament_target_dependencies(diff_drive_controller
  rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)

# 2. Custom LiDAR node (already exists, publishing /scan)
add_executable(custom_lidar_node src/custom_lidar_node.cpp)
ament_target_dependencies(custom_lidar_node
  rclcpp sensor_msgs
)

# 3. Autonomous explorer (reactive obstacle avoidance for auto-mapping)
add_executable(auto_explorer src/auto_explorer.cpp)
ament_target_dependencies(auto_explorer
  rclcpp sensor_msgs geometry_msgs
)

# 4. Frontier-based explorer (systematic exploration with loop closure)
add_executable(frontier_explorer src/frontier_explorer.cpp)
ament_target_dependencies(frontier_explorer
  rclcpp sensor_msgs geometry_msgs nav_msgs visualization_msgs
)

# 5. Explorer (wall-following + open-space seeking + IMU stall detection)
add_executable(explorer src/explorer.cpp)
ament_target_dependencies(explorer rclcpp sensor_msgs geometry_msgs nav_msgs)

# 6. Right wheel spinner (direct serial test node)
add_executable(right_wheel_spinner src/right_wheel_spinner.cpp)
ament_target_dependencies(right_wheel_spinner rclcpp)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  diff_drive_controller
  custom_lidar_node
  auto_explorer
  frontier_explorer
  explorer
  right_wheel_spinner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/calibrate_drive.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME calibrate_drive
)

ament_package()
