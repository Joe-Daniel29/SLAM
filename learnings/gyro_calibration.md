# Gyro Bias Calibration — Why and How

## The problem

MEMS gyroscopes (like the MPU6050) have a non-zero output even when perfectly still. This is called **zero-rate offset** or **bias**. Typical MPU6050 bias is ±0.15°/s (around ±20 raw LSB at ±250°/s sensitivity).

When the EKF integrates this biased yaw rate to estimate heading:
```
yaw += biased_yaw_rate × dt
```

The error accumulates linearly with time:
- 0.15°/s × 60s = 9° of phantom rotation per minute
- 0.15°/s × 600s = 90° after 10 minutes

This was the primary cause of our "5° turn shows as 180°" bug — the bias was likely larger than typical, and the EKF had no corrective source to fight the drift (rf2o was deferred).

## The fix: startup calibration

At node startup, while the robot is stationary:

1. Collect N gyro samples (we use 100 samples ≈ 2 seconds at 50Hz)
2. Average each axis: `bias = sum / N`
3. Subtract bias from all future readings: `calibrated = raw - bias`

This eliminates the static (constant) component of the bias. It does NOT handle:
- **Temperature drift**: bias changes as the chip heats up (~0.015°/s per °C for MPU6050). For a hobby robot this is negligible.
- **In-run bias instability**: tiny random walk in the bias. Also negligible for our use case.

## Implementation details (our code)

In `diff_drive_controller.cpp`:

- During calibration (first ~2s), IMU messages are NOT published. This prevents the EKF from ingesting uncalibrated data during startup.
- The calibrated gz (yaw rate) is also stored in `imu_yaw_rate_` for the PID heading controller to use.
- `calib_done_` is `std::atomic<bool>` because it's written in the serial reader thread and read in the motor timer callback.

## Assumption

The robot MUST be stationary during the first ~2 seconds after launch. If it's moving, the calibration will capture real motion as "bias" and subtract it forever. The log message "Gyro bias calibrated" confirms when calibration is complete.
