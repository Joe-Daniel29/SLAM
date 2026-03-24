# TF Frames — The ROS Transform Tree

## What is TF?

TF (transform) is ROS's system for tracking the spatial relationships between different parts of the robot and the world. Every sensor, every link, the map, the odometry origin — each lives in a **frame**, and TF describes how to convert coordinates between them.

Think of it like a family tree where each connection says "frame B is at position (x,y,z) and rotation (r,p,y) relative to frame A."

## Our robot's TF tree

```
map
 └── odom
      └── base_link
           ├── base_footprint
           ├── laser
           └── imu_link
```

### Who publishes each transform?

| Transform | Publisher | How |
|---|---|---|
| map → odom | slam_toolbox | Corrects long-term drift by matching laser scans to the map |
| odom → base_link | robot_localization (EKF) | Fused odometry from IMU + wheel odom |
| base_link → laser | robot_state_publisher | From URDF (static) |
| base_link → imu_link | robot_state_publisher | From URDF (static) |
| base_link → base_footprint | robot_state_publisher | From URDF (static) |

### What each frame means

**map** — The global, fixed reference frame. "Absolute" position in the world. Stays consistent — if the robot drives in a circle and returns to the start, it should be at the same map coordinates.

**odom** — The odometry origin. It's where the robot was when it started (or when the EKF initialized). The odom→base_link transform is smooth and continuous (no jumps), but it **drifts** over time. After driving around for a while, the odom frame's idea of "where I started" may be meters off from reality.

**base_link** — The robot's body. Attached rigidly to the center of the robot. All sensor frames are defined relative to this.

**laser** — Where the LiDAR sensor is. Defined in the URDF as an offset from base_link (e.g., 5cm forward, 13cm up). Every laser scan point is measured relative to this frame.

**imu_link** — Where the IMU chip is mounted. The EKF needs this to correctly interpret IMU measurements (if the IMU is offset from center, linear acceleration includes a centripetal component during turns).

## Why two "position" frames? (map vs odom)

This is the key insight of the ROS navigation architecture:

**odom → base_link** (from EKF):
- Updated at high frequency (30 Hz in our config)
- Smooth and continuous — never jumps
- But drifts over time (accumulated sensor error)
- Good for: local motion planning, obstacle avoidance

**map → odom** (from SLAM):
- Updated less frequently (whenever SLAM corrects its estimate)
- Can have discontinuous jumps (when SLAM realizes "oh, I was actually 0.5m off")
- Globally consistent — corrects drift
- Good for: knowing where you are in the building

The combination gives you the best of both worlds:
- `map → base_link` = globally accurate position (but may jump)
- `odom → base_link` = locally smooth motion (but drifts)

## Static vs dynamic transforms

**Static transforms** don't change — the laser is bolted to the robot at a fixed offset. These come from the URDF via `robot_state_publisher` and are published once.

**Dynamic transforms** change over time — the robot moves relative to the odom frame. These are published continuously by the EKF and SLAM.

## The URDF's role

The URDF (Unified Robot Description Format) file defines:
- The physical structure of the robot (links and joints)
- Where each sensor is relative to the body
- Visual and collision geometry (for simulation/visualization)

`robot_state_publisher` reads the URDF and broadcasts all the static (fixed joint) transforms. This is why the URDF must accurately describe where your LiDAR and IMU are physically mounted — wrong offsets mean the EKF and SLAM will misinterpret sensor data.

## Common TF problems

**"Could not find transform from laser to base_link"**: robot_state_publisher isn't running, or the URDF doesn't define the laser link.

**"Transform from odom to base_link is stale"**: The EKF node isn't running or isn't receiving any sensor data.

**Map jumps wildly**: slam_toolbox is making large corrections to map→odom because the odom drift has become significant. This usually means the odometry inputs to the EKF are poor.

**Robot appears rotated in visualization**: A frame's orientation is wrong — often the IMU axes don't match the expected orientation relative to base_link.
