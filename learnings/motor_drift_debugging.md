# Motor Drift Debugging — Straight-line Bias

## Observed behavior (2026-03-16)

After deploying gyro bias calibration + PID heading correction (Kp=0.5, Ki=0, Kd=0), the robot still drifts to one side when commanded to drive straight.

## Possible causes

### 1. PID gain too low
Kp=0.5 may not produce enough corrective differential to overcome the mechanical asymmetry. Symptoms: IMU detects the yaw rate but correction is too small to counteract it.
**Fix:** Increase Kp incrementally (try 1.0, 1.5, 2.0) until the drift is corrected without oscillation.

### 2. Vibration-induced gyro bias
The gyro bias calibration runs at startup while motors are OFF. But when motors run, vibration changes the gyro's effective bias. The MPU6050 is particularly susceptible to this because it's a low-cost MEMS device.
**Symptoms:** IMU angular_velocity.z reads ~0 even while the robot is visibly curving.
**Fix:** Add a software low-pass filter to the gyro output, or mount the IMU on vibration dampeners (rubber standoffs).

### 3. Mechanical asymmetry
One motor is inherently stronger, one wheel has more friction, weight is unbalanced, or one wheel has slightly different diameter. This creates a constant torque bias.
**Fix:** A static PWM offset (trim) for one motor, or increase Ki in the PID so the integral term accumulates and compensates the persistent bias.

### 4. PWM dead-band asymmetry
The Arduino applies a deadband of ±15 PWM. If the PID correction is small, it might fall within the deadband and get zeroed out.
**Fix:** Check that the corrected PWM values are above the deadband threshold.

## Diagnostic steps

1. Echo `/imu/data_raw` angular_velocity.z while driving straight — is it non-zero?
2. If non-zero: increase Kp
3. If near-zero despite visible drift: vibration is masking the real yaw rate
4. Try driving at different speeds — if drift is worse at higher speed, it's likely mechanical
5. Swap left/right motor connections — if drift reverses direction, it's mechanical

## Key insight: Ki is probably what we need

For a persistent mechanical bias (one motor always slightly stronger), **Ki (integral gain)** is specifically designed to fix this. The integral accumulates the steady-state error over time and produces an increasing correction until the error is eliminated. Start with Ki=0.1 and adjust from there.
