# Differential Drive Kinematics

## What is a differential drive robot?

Two independently driven wheels on a common axis, plus one or more passive casters for balance. Steering is achieved by varying the speed of each wheel — no steering mechanism needed.

```
        Front
    ┌─────────────┐
    │             │
 [L]│      ●      │[R]    ● = center of rotation axis
    │             │
    └─────────────┘
         (caster)
```

- Both wheels same speed → straight line
- Right faster than left → turns left
- Left faster than right → turns right
- Wheels opposite speeds → spin in place

## Key parameters

- **wheel_radius (r)**: radius of each drive wheel (our robot: 0.09 m)
- **wheel_separation (d)**: distance between the two wheel contact points (our robot: 0.2 m)
- **v_linear**: forward/backward speed of the robot center (m/s)
- **v_angular**: rotation speed around the robot center (rad/s)

## Inverse kinematics (cmd_vel → wheel speeds)

Given a desired linear and angular velocity (from `/cmd_vel`), compute what each wheel needs to do:

```
v_left  = v_linear - (v_angular × d / 2)
v_right = v_linear + (v_angular × d / 2)
```

**Intuition:** To turn right (negative v_angular in ROS convention), the left wheel goes faster and the right wheel goes slower. The `d/2` term accounts for each wheel being half the separation distance from center.

**Example:** Drive forward at 0.3 m/s while turning left at 0.5 rad/s:
```
v_left  = 0.3 - (0.5 × 0.2 / 2) = 0.3 - 0.05 = 0.25 m/s
v_right = 0.3 + (0.5 × 0.2 / 2) = 0.3 + 0.05 = 0.35 m/s
```

## Forward kinematics (wheel speeds → robot motion)

Given measured (or assumed) wheel speeds, compute the robot's velocity:

```
v_linear  = (v_right + v_left) / 2
v_angular = (v_right - v_left) / d
```

**Intuition:** Linear velocity is the average of both wheels. Angular velocity comes from the difference — if both wheels go the same speed, the difference is zero (no turning).

## Converting to PWM

Wheel speeds are in m/s. Motors need a PWM signal (0-255 duty cycle). The conversion chain:

```
m/s → rad/s → fraction of max → PWM byte
```

1. **m/s → rad/s**: divide by wheel_radius (`v / r`)
2. **rad/s → fraction**: divide by max wheel angular velocity (`rad_s / max_rad_s`)
3. **fraction → PWM**: multiply by 127 (our protocol uses signed bytes, -127 to +127)

On the Arduino side, the signed byte is mapped to 0-255 PWM:
```
PWM = map(cmd_byte, -127, 127, -255, 255)
```

## Open-loop odometry (dead reckoning)

Without encoders, we can estimate position by assuming the wheels do exactly what we command. Each time step (dt):

```
delta_theta = v_angular × dt
delta_x = v_linear × cos(theta + delta_theta/2) × dt
delta_y = v_linear × sin(theta + delta_theta/2) × dt

x += delta_x
y += delta_y
theta += delta_theta
```

The `theta + delta_theta/2` trick (midpoint integration) is more accurate than using just `theta` — it accounts for the robot turning partway through the step.

### Why open-loop odometry drifts

This assumes the wheels do exactly what you command. In reality:
- One wheel may have more friction than the other
- Wheels slip on smooth/uneven surfaces
- Battery voltage drops → motor speed changes
- Motor response isn't perfectly linear with PWM

This is why open-loop odom is marked "low trust" in the EKF — it's a fallback, not a primary source.

## PID control (what we're missing)

Right now our motor control is **open-loop**: cmd_vel → math → PWM. There's no feedback.

A **PID controller** would close the loop using the IMU:

```
                    ┌──────────┐
cmd_vel ──→ [+] ──→│   PID    │──→ Motor PWM
             ↑  -  │Controller│
             │     └──────────┘
             │
          IMU yaw rate
          (actual rotation)
```

**P (Proportional):** Correct proportionally to the error.
  - Error = desired_yaw_rate - actual_yaw_rate
  - If the robot is drifting right, increase left motor / decrease right motor

**I (Integral):** Correct accumulated error over time.
  - If there's a persistent bias (one wheel always slower), the integral term builds up and compensates

**D (Derivative):** Dampen rapid changes in error.
  - Prevents overcorrection and oscillation

**Example for straight-line driving:**
- cmd_vel says: go straight (angular.z = 0, so desired yaw_rate = 0)
- IMU says: actually rotating at +0.05 rad/s (drifting left)
- Error = 0 - 0.05 = -0.05
- PID output: slightly increase right wheel / decrease left wheel
- Result: robot corrects back to straight

Without PID, the robot just drifts and nobody corrects it.
