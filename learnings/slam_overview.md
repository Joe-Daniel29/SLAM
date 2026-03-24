# SLAM — Simultaneous Localization and Mapping

## The chicken-and-egg problem

To know where you are, you need a map. To build a map, you need to know where you are. SLAM solves both at the same time.

## How it works (conceptual)

1. Robot takes a laser scan → sees walls, obstacles as a ring of distance measurements
2. Robot moves a bit → takes another scan
3. **Scan matching**: Compare the two scans. Figure out how the robot moved between them by aligning the point patterns. ("These two walls shifted 10cm to the right and rotated 2° — so the robot moved 10cm left and turned 2°")
4. Add the new scan's points to the growing map
5. Repeat

The tricky parts:
- Scan matching can be wrong (featureless hallways, symmetrical rooms)
- Small errors accumulate into big drift
- **Loop closure**: When the robot returns to a previously visited area, SLAM detects this and corrects the accumulated drift across the entire map

## slam_toolbox (what we use)

slam_toolbox is a ROS 2 package that implements graph-based SLAM. It:

1. Takes `/scan` (laser data) and the `odom → base_link` TF (from the EKF)
2. Performs scan matching to refine the pose estimate
3. Builds a pose graph (nodes = poses, edges = scan-match constraints)
4. Optimizes the graph (especially during loop closures)
5. Publishes: the `map → odom` TF and the `/map` occupancy grid

### What it needs from us

- **Good laser data** (`/scan`): Consistent, low-noise, 360° scans. Our custom LiDAR node provides this.
- **Reasonable odometry** (`odom → base_link` TF): Doesn't need to be perfect — slam_toolbox will correct drift. But if odometry is wildly wrong (like showing 180° when the robot turned 5°), SLAM can't match scans and will fail.

This is why fixing the IMU/EKF rotation issue is critical — slam_toolbox needs at least roughly correct odometry to function.

## The occupancy grid map

SLAM produces a 2D occupancy grid:
- Each cell represents a small area (typically 5cm × 5cm)
- Values: 0 = free space, 100 = occupied (wall), -1 = unknown
- Published on the `/map` topic
- Can be saved to disk for later use (navigation without SLAM)

## Online vs. offline SLAM

**Online (what we do):** Build the map while the robot is driving. Real-time constraints — must process scans as fast as they arrive.

**Offline:** Record all sensor data, then process it later with more computation. Can make multiple passes to refine the map.

## Why odometry quality matters

slam_toolbox uses odometry as an **initial guess** for scan matching. The process is:

1. Odometry says "robot moved ~0.1m forward and turned ~0.02 rad"
2. slam_toolbox uses this as a starting point to align the new scan with the map
3. Scan matching refines the estimate to the actual best alignment

If odometry is reasonably close, the scan matcher converges to the right answer quickly. If odometry is way off (like our 36x rotation error), the initial guess is so far from truth that the scan matcher may:
- Converge to a wrong local minimum (thinks it's in the wrong spot)
- Fail entirely (no good match found)
- Produce a garbled, overlapping map

## Our SLAM pipeline (end-to-end)

```
Physical world
    │
    ├── LiDAR sensor → custom_lidar_node → /scan
    │
    ├── MPU6050 → Arduino → diff_drive_controller → /imu/data_raw ──┐
    │                                                                 │
    ├── cmd_vel → diff_drive_controller → /odom/cmd_vel ─────────────┤
    │                                                                 │
    │                              robot_localization EKF ←───────────┘
    │                                   │
    │                            odom → base_link TF
    │                                   │
    └── ─ ─ ─ ─ ─ ─ ─ ─ ─ → slam_toolbox ← ── /scan
                                   │
                          map → odom TF + /map
```
