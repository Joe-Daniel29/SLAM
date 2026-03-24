# PID Control — Closed-loop Motor Correction

## Open-loop vs closed-loop

**Open-loop (what we have now):**
```
Desired velocity → Math → Motor PWM
```
You send a command and hope for the best. No feedback, no correction. If the left wheel has more friction, the robot drifts right and nobody notices.

**Closed-loop (what we want):**
```
Desired velocity → [+] → PID → Motor PWM
                    ↑  -
                    │
              Measured velocity
              (from IMU/encoders)
```
You continuously measure what's actually happening and adjust the command to match what you wanted. This is feedback control.

## The PID controller

PID stands for **Proportional-Integral-Derivative** — three terms that each contribute to the correction signal.

### Error

First, define the error:
```
error(t) = desired_value - measured_value
```

For straight-line driving: `error = 0 (desired yaw rate) - actual_yaw_rate (from IMU gz)`

### P — Proportional

```
P_output = Kp × error
```

Corrects in proportion to how far off you are right now. Big error → big correction.

**Problem alone:** Can't fully eliminate steady-state error. If the system needs a constant correction (persistent drift), the P term can only provide that correction while there's still some error remaining. This leftover is called **steady-state error**.

**Example:** Robot drifting right at 0.05 rad/s. P says "turn left a bit." But the correction might not be strong enough to fully stop the drift — you end up with a small residual drift.

### I — Integral

```
I_output = Ki × ∫ error dt  (sum of error over time)
```

Accumulates past errors. If there's a persistent small error that P can't eliminate, the integral term builds up over time until the correction is large enough.

**Problem alone:** Slow to respond and can "wind up" — if the error persists for a long time (e.g., robot was stuck), the integral becomes huge and causes massive overcorrection when the error finally clears. This is called **integral windup**. Fix: clamp the integral to a maximum value.

**Example:** Robot has been drifting slightly right for 10 seconds. The integral has accumulated this history and pushes the left wheel progressively harder until the drift is eliminated.

### D — Derivative

```
D_output = Kd × d(error)/dt  (rate of change of error)
```

Reacts to how fast the error is changing. If the error is rapidly increasing, D provides a strong preemptive correction. If the error is decreasing (correction is working), D reduces the output to prevent overshoot.

**Problem alone:** Amplifies noise. If the sensor is jittery, the derivative of noise is even jittier.

**Example:** Robot suddenly hits a rough patch and starts veering. The D term reacts to the sudden change before the error gets large.

### Combined

```
output = Kp × error + Ki × ∫error dt + Kd × d(error)/dt
```

The three terms complement each other:
- P: handles the present (current error)
- I: handles the past (accumulated error)
- D: handles the future (rate of change, predicts where error is heading)

## Tuning PID gains

The gains (Kp, Ki, Kd) determine behavior:

| Gain | Too low | Too high |
|------|---------|----------|
| Kp | Slow response, large steady-state error | Oscillation, overshoot |
| Ki | Can't eliminate persistent bias | Integral windup, oscillation |
| Kd | Slow to react to sudden changes | Amplifies sensor noise, jittery |

### Simple tuning approach

1. Set Ki = 0, Kd = 0
2. Increase Kp until the robot corrects reasonably but starts to oscillate
3. Back off Kp by ~30%
4. Slowly increase Ki until steady-state error disappears
5. If there's overshoot/ringing, add a small Kd

## For our robot specifically

### Heading correction (yaw PID)

Use case: driving straight, correcting for surface-induced drift.

```
Input:      IMU gz (yaw rate in rad/s)
Setpoint:   desired yaw rate from cmd_vel angular.z
Error:      desired_yaw_rate - actual_yaw_rate
Output:     differential PWM adjustment to left/right motors
```

```
adjusted_left_pwm  = base_left_pwm  - pid_output
adjusted_right_pwm = base_right_pwm + pid_output
```

If PID says "you're drifting right" (positive pid_output), it reduces left motor and increases right motor to compensate.

### Speed correction (velocity PID) — needs encoders

For correcting forward speed, you'd need wheel encoders to measure actual velocity. The IMU can't reliably tell you forward speed (accelerometer double-integration drifts too fast). This is a future improvement once encoders are added.

## PID vs EKF — different jobs

These are often confused. They do fundamentally different things:

**PID:** Controls actuators (motors). Makes the robot DO what you want. "Turn left more to compensate for drift."

**EKF:** Estimates state (position, orientation). Tells you where the robot IS. "Based on all sensors, I think the robot is at (3.2, 1.5) facing 45°."

You need both:
- EKF without PID: knows the robot is drifting but can't fix it
- PID without EKF: corrects motors but doesn't know overall position
- Both together: PID keeps motion accurate, EKF tracks position, SLAM builds the map
