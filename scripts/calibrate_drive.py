#!/usr/bin/env python3
"""
calibrate_drive.py — Automatic motor trim calibration for straight-line driving.

Procedure:
  1. Wait for gyro bias calibration to complete (~4s)
  2. Command the robot to drive straight forward at a test speed
  3. Record IMU yaw rate over several seconds to measure drift
  4. Compute the average yaw drift (positive = drifting left, negative = right)
  5. Repeat at multiple speeds
  6. Calculate optimal left/right PWM trim to counteract the drift
  7. Publish trim to /motor_trim and save to a YAML file

The robot needs ~2m of clear space ahead. Place it facing an open hallway.

Usage:
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  ros2 run slam_bot calibrate_drive
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import time
import math
import yaml
import os


class DriveCalibrater(Node):
    def __init__(self):
        super().__init__('calibrate_drive')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.trim_pub = self.create_publisher(Vector3, '/motor_trim', 10)

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # State
        self.gz_samples = []
        self.collecting = False
        self.imu_ready = False
        self.odom_vx_samples = []

        # Odom state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.get_logger().info('=== Drive Calibration Script ===')
        self.get_logger().info('Place the robot with ~2m clear space ahead.')
        self.get_logger().info('Waiting for IMU gyro calibration to complete...')

        # Start calibration after a delay (let gyro bias calibrate)
        self.create_timer(1.0, self.check_imu_ready)

    def imu_callback(self, msg):
        if not self.imu_ready:
            # The diff_drive_controller suppresses IMU during calibration.
            # Once we start receiving data, calibration is done.
            self.imu_ready = True
            self.get_logger().info('IMU data flowing — gyro calibration complete.')

        if self.collecting:
            self.gz_samples.append(msg.angular_velocity.z)

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.odom_yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

        if self.collecting:
            self.odom_vx_samples.append(msg.twist.twist.linear.x)

    def check_imu_ready(self):
        if not self.imu_ready:
            return
        # Cancel this timer and start the actual calibration
        self.get_logger().info('Starting calibration in 2 seconds...')
        self.create_timer(2.0, self.run_calibration_once)

    def run_calibration_once(self):
        """Run the full calibration sequence once."""
        import threading
        t = threading.Thread(target=self._calibration_sequence, daemon=True)
        t.start()

    def _calibration_sequence(self):
        test_speeds = [0.15, 0.25, 0.4]
        drive_duration = 3.0  # seconds per test
        results = []

        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║     STRAIGHT-LINE DRIFT CALIBRATION     ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')

        for speed in test_speeds:
            self.get_logger().info(f'')
            self.get_logger().info(f'── Test: forward at {speed:.2f} m/s for {drive_duration:.0f}s ──')

            # Record starting position
            start_x = self.odom_x
            start_y = self.odom_y
            start_yaw = self.odom_yaw

            # Start collecting IMU data
            self.gz_samples = []
            self.odom_vx_samples = []
            self.collecting = True

            # Drive forward
            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = 0.0

            start_time = time.time()
            while time.time() - start_time < drive_duration:
                self.cmd_pub.publish(cmd)
                time.sleep(0.05)  # 20 Hz

            # Stop
            self.cmd_pub.publish(Twist())
            self.collecting = False
            time.sleep(0.5)  # let it settle

            # Analyze
            if len(self.gz_samples) < 10:
                self.get_logger().warn(f'  Too few IMU samples ({len(self.gz_samples)}), skipping')
                continue

            # Trim first 10% of samples (ramp-up transient)
            trim_start = len(self.gz_samples) // 10
            steady_gz = self.gz_samples[trim_start:]

            avg_gz = sum(steady_gz) / len(steady_gz)
            max_gz = max(abs(g) for g in steady_gz)

            # Odom-based drift
            dx = self.odom_x - start_x
            dy = self.odom_y - start_y
            dist = math.hypot(dx, dy)
            yaw_drift = self.odom_yaw - start_yaw

            # Actual speed from odom
            avg_vx = sum(self.odom_vx_samples) / max(len(self.odom_vx_samples), 1)

            self.get_logger().info(f'  Samples: {len(steady_gz)} IMU readings')
            self.get_logger().info(f'  Avg gyro Z: {avg_gz:+.4f} rad/s  ({"LEFT" if avg_gz > 0 else "RIGHT"} drift)')
            self.get_logger().info(f'  Max gyro Z: {max_gz:.4f} rad/s')
            self.get_logger().info(f'  Odom distance: {dist:.3f}m, yaw drift: {math.degrees(yaw_drift):+.1f}°')
            self.get_logger().info(f'  Avg odom vx: {avg_vx:.3f} m/s (commanded {speed:.2f})')

            results.append({
                'speed': speed,
                'avg_gz': avg_gz,
                'yaw_drift_deg': math.degrees(yaw_drift),
                'distance': dist,
                'avg_vx': avg_vx,
                'n_samples': len(steady_gz),
            })

            # Pause between tests
            time.sleep(1.5)

        if not results:
            self.get_logger().error('No valid test results. Check IMU and motor connections.')
            rclpy.shutdown()
            return

        # ── Compute trim ──────────────────────────────────────────────────────
        # Average yaw drift across all speeds
        avg_drift = sum(r['avg_gz'] for r in results) / len(results)

        self.get_logger().info('')
        self.get_logger().info('═══════════════════════════════════════════')
        self.get_logger().info('              RESULTS                      ')
        self.get_logger().info('═══════════════════════════════════════════')
        self.get_logger().info(f'Average yaw drift: {avg_drift:+.4f} rad/s')

        # Convert drift to PWM trim.
        # Drift > 0 means robot turns left → right wheel is faster → reduce right or boost left.
        # Empirical: ~0.01 rad/s drift ≈ 1 PWM trim unit (tune based on your motors)
        DRIFT_TO_PWM = 300.0  # PWM trim per rad/s of drift (start conservative, tune up)

        trim_correction = int(round(avg_drift * DRIFT_TO_PWM))

        # Split correction: reduce the faster side, boost the slower side
        # Positive drift (left) → right is faster → trim_right negative, trim_left positive
        trim_left  = trim_correction // 2
        trim_right = -(trim_correction - trim_correction // 2)

        # Clamp to reasonable range
        trim_left  = max(-20, min(20, trim_left))
        trim_right = max(-20, min(20, trim_right))

        if abs(avg_drift) < 0.005:
            self.get_logger().info('Drift is negligible (<0.005 rad/s). No trim needed!')
            trim_left = 0
            trim_right = 0
        else:
            direction = "LEFT" if avg_drift > 0 else "RIGHT"
            self.get_logger().info(f'Robot drifts {direction}')
            self.get_logger().info(f'Computed trim: left={trim_left:+d}, right={trim_right:+d} PWM')

        # ── Publish trim ──────────────────────────────────────────────────────
        trim_msg = Vector3()
        trim_msg.x = float(trim_left)
        trim_msg.y = float(trim_right)
        self.trim_pub.publish(trim_msg)
        self.get_logger().info(f'Published to /motor_trim: left={trim_left}, right={trim_right}')

        # ── Save to YAML ──────────────────────────────────────────────────────
        config_dir = os.path.expanduser('~/ros2_ws/src/slam_bot/config')
        os.makedirs(config_dir, exist_ok=True)
        trim_file = os.path.join(config_dir, 'motor_trim.yaml')

        trim_data = {
            'diff_drive_controller': {
                'ros__parameters': {
                    'trim_left': trim_left,
                    'trim_right': trim_right,
                }
            },
            '_calibration_info': {
                'avg_drift_rad_s': round(avg_drift, 6),
                'tests': results,
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            }
        }

        with open(trim_file, 'w') as f:
            yaml.dump(trim_data, f, default_flow_style=False)
        self.get_logger().info(f'Saved trim to: {trim_file}')

        self.get_logger().info('')
        self.get_logger().info('To use these trims permanently, add to your launch:')
        self.get_logger().info(f"  parameters=['{trim_file}']")
        self.get_logger().info('Or pass as args:')
        self.get_logger().info(f'  trim_left:={trim_left} trim_right:={trim_right}')
        self.get_logger().info('')
        self.get_logger().info('Calibration complete!')

        # ── Verification run ──────────────────────────────────────────────────
        self.get_logger().info('')
        self.get_logger().info('── Verification: driving straight with new trim ──')
        time.sleep(1.0)

        self.gz_samples = []
        self.collecting = True
        start_yaw = self.odom_yaw

        cmd = Twist()
        cmd.linear.x = 0.25
        start_time = time.time()
        while time.time() - start_time < 3.0:
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())
        self.collecting = False
        time.sleep(0.5)

        if len(self.gz_samples) > 10:
            trim_start = len(self.gz_samples) // 10
            steady = self.gz_samples[trim_start:]
            verify_gz = sum(steady) / len(steady)
            verify_yaw = self.odom_yaw - start_yaw
            self.get_logger().info(f'  Post-trim gyro Z: {verify_gz:+.4f} rad/s (was {avg_drift:+.4f})')
            self.get_logger().info(f'  Post-trim yaw drift: {math.degrees(verify_yaw):+.1f}°')

            improvement = (1.0 - abs(verify_gz) / max(abs(avg_drift), 0.001)) * 100
            if improvement > 0:
                self.get_logger().info(f'  Improvement: {improvement:.0f}% less drift!')
            else:
                self.get_logger().info(f'  Trim may need further tuning. Try adjusting DRIFT_TO_PWM.')

        rclpy.shutdown()


def main():
    rclpy.init()
    node = DriveCalibrater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop motors on Ctrl+C
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        node.destroy_node()


if __name__ == '__main__':
    main()
