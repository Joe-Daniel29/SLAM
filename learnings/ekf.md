# EKF — Extended Kalman Filter (robot_localization)

## The problem it solves

A robot has multiple sensors, each giving a noisy, partial picture of where it is:
- IMU gives rotation rate (noisy, drifts over time)
- Wheel odometry gives velocity (slips, drifts)
- Laser odometry gives position deltas (can fail with featureless walls)

No single sensor is reliable alone. The EKF combines ("fuses") all of them into one best estimate of the robot's state.

## What is a Kalman Filter?

At its core, it's a two-step loop:

### Step 1: Predict
Using a motion model, predict where the robot should be now based on where it was and what commands were sent:
```
predicted_state = previous_state + (velocity × dt)
predicted_uncertainty = previous_uncertainty + process_noise
```

The prediction always makes the uncertainty grow — the robot becomes less sure of where it is.

### Step 2: Update (correct)
When a sensor measurement arrives, compare it to the prediction and adjust:
```
error = measurement - predicted_state
kalman_gain = predicted_uncertainty / (predicted_uncertainty + sensor_noise)
corrected_state = predicted_state + kalman_gain × error
corrected_uncertainty = (1 - kalman_gain) × predicted_uncertainty
```

The **Kalman gain** determines how much to trust the sensor vs. the prediction:
- If sensor noise is low (reliable sensor) → gain is high → trust the sensor more
- If sensor noise is high (unreliable sensor) → gain is low → trust the prediction more

The update always *reduces* uncertainty — a measurement, even noisy, is better than no measurement.

### Why "Extended"?

The plain Kalman Filter only works for linear systems. Robot motion is nonlinear (sine/cosine in the motion model). The **Extended** Kalman Filter linearizes around the current estimate using Jacobians. It's an approximation, but works well for small time steps.

## The robot's state vector

robot_localization's EKF tracks a 15-dimensional state:

```
[x, y, z, roll, pitch, yaw, vx, vy, vz, v_roll, v_pitch, v_yaw, ax, ay, az]
 0  1  2   3      4     5   6   7   8    9       10       11     12  13  14
```

With `two_d_mode: true`, it constrains z, roll, and pitch to zero — we're a ground robot.

## Sensor configuration (the `_config` matrices)

Each sensor's config is a 15-element boolean array that tells the EKF which state variables this sensor provides. For example:

```yaml
imu0_config: [false, false, false,   # x, y, z — IMU can't measure position
              false, false, false,   # roll, pitch, yaw — we don't trust MPU6050 orientation
              false, false, false,   # vx, vy, vz — can't get velocity from gyro
              false, false, true,    # v_roll, v_pitch, v_yaw — YES, gyro measures yaw rate
              false, false, false]   # ax, ay, az — not using accelerometer
```

Only set `true` for things the sensor **actually measures reliably**. Setting too many to true feeds bad data into the filter.

## Differential mode (`_differential: true`)

When a sensor provides absolute values (like "I'm at x=3.5m"), set differential to false.

When a sensor provides values that drift over time or are in an arbitrary frame, set differential to true. The EKF then uses the **change between consecutive messages** rather than the absolute value. This is important for:
- Laser odometry (rf2o) — its frame can drift
- cmd_vel dead reckoning — it's open-loop and will drift

## Covariance — quantifying uncertainty

Every measurement in ROS carries a **covariance matrix** — a way of saying "I measured X, and I'm this confident."

For a single variable, covariance is just variance (σ²):
- Small variance (0.001) = "I'm very sure"
- Large variance (1.0) = "I'm not very sure"

For multiple variables, it's a matrix that also captures correlations ("if X is wrong, Y is probably also wrong").

In our IMU message:
```cpp
imu_msg.angular_velocity_covariance[0] = 0.02;  // gx variance
imu_msg.angular_velocity_covariance[4] = 0.02;  // gy variance
imu_msg.angular_velocity_covariance[8] = 0.02;  // gz variance
```

These tell the EKF how much to trust the gyro. Lower values = more trust = more influence on the fused estimate.

## Process noise covariance

This represents how much the robot's actual motion might deviate from the prediction model between updates. It's the "I don't trust my model perfectly" knob.

- Too low → filter is overconfident in prediction, slow to react to real motion, but smooth
- Too high → filter is jumpy, reacts fast but noisy

It's a 15×15 matrix (one entry per state variable). In our config, the diagonal values range from 0.01 to 0.06.

## Our specific EKF setup

```
Sensor sources:
├── odom0: /odom/rf2o     → x, y from laser scan matching (DEFERRED — no data)
├── imu0:  /imu/data_raw  → v_yaw from MPU6050 gyroscope
└── odom1: /odom/cmd_vel   → vx, v_yaw from motor commands (open-loop echo)

Output:
├── /odom topic (fused odometry)
└── odom → base_link TF transform
```

### Current problem with this setup

With rf2o deferred, the EKF has NO position-fixing source. It only has two velocity inputs. This means:
1. Position is purely integrated from velocity — drift is uncorrected
2. Yaw rate comes from both IMU and cmd_vel — these are not independent (see below)

### The double-counting problem

The cmd_vel odom publishes v_yaw computed from commanded wheel velocities — it's an echo of what was commanded, not a measurement of what happened. The IMU also reports the actual physical yaw rate.

The EKF assumes all inputs are **independent measurements**. When two "independent" sensors agree, the filter becomes more confident. But IMU and cmd_vel are correlated — the cmd_vel caused the rotation that the IMU measured. This makes the filter overconfident and can amplify errors.

**Fix:** Remove v_yaw from the cmd_vel source. Let the IMU be the sole authority on rotation. The cmd_vel source should only provide vx (forward velocity), since we have no other way to measure linear speed without encoders.

## Frames and TF tree

```
map → odom → base_link → [laser, imu_link, ...]
```

- **map → odom**: Published by slam_toolbox. Corrects for long-term drift.
- **odom → base_link**: Published by the EKF. Smooth, continuous, but drifts over time.
- **base_link → sensor frames**: Static transforms from the URDF.

The EKF operates in the `odom` frame (`world_frame: odom`). slam_toolbox provides the map→odom correction.
