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

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  diff_drive_controller
  custom_lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
