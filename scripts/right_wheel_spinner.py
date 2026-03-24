#!/usr/bin/env python3
"""
right_wheel_spinner.py

ROS 2 node that talks directly to the Arduino motor bridge on /dev/ttyAMA0 and
spins only the right wheel for a fixed duration.

Usage:
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  ros2 run slam_bot right_wheel_spinner
"""
import rclpy
from rclpy.node import Node
import serial
import time

SERIAL_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200
RIGHT_PWM = 110   # -127..+127 (positive = forward)
DURATION_S = 5.0  # seconds to keep the wheel running
SEND_HZ = 20.0


class RightWheelSpinner(Node):
    def __init__(self):
        super().__init__('right_wheel_spinner')

        try:
            # Open with DTR=False to avoid resetting the Arduino on connect
            self.ser = serial.Serial()
            self.ser.port = SERIAL_PORT
            self.ser.baudrate = BAUD_RATE
            self.ser.timeout = 0
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
        except serial.SerialException as exc:
            self.get_logger().error(f'Failed to open {SERIAL_PORT}: {exc}')
            raise SystemExit(1)

        self.get_logger().info(
            f'Opened {SERIAL_PORT} @ {BAUD_RATE}. Waiting 2s for Arduino to settle...')
        time.sleep(2.0)
        self.get_logger().info(
            f'Spinning right wheel PWM={RIGHT_PWM} for {DURATION_S}s')

        self.packet = self._build_packet(left_pwm=0, right_pwm=RIGHT_PWM)
        self.end_time = self.get_clock().now().nanoseconds / 1e9 + DURATION_S

        timer_period = 1.0 / SEND_HZ
        self.timer = self.create_timer(timer_period, self._send_packet)

    def _build_packet(self, left_pwm: int, right_pwm: int, yaw_mrad: int = 0) -> bytes:
        left = left_pwm & 0xFF
        right = right_pwm & 0xFF
        yaw_h = (yaw_mrad >> 8) & 0xFF
        yaw_l = yaw_mrad & 0xFF
        checksum = (0xAA + 0x55 + left + right + yaw_h + yaw_l) & 0xFF
        return bytes([0xAA, 0x55, left, right, yaw_h, yaw_l, checksum, 0x0D])

    def _send_packet(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now >= self.end_time:
            self.get_logger().info('Spin duration complete, stopping wheel')
            # send zero command once before shutting down
            stop_pkt = self._build_packet(0, 0)
            self.ser.write(stop_pkt)
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.ser.close()
            rclpy.shutdown()
            return

        self.ser.write(self.packet)


def main():
    rclpy.init()
    node = RightWheelSpinner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.write(node._build_packet(0, 0))
            node.ser.close()
        node.destroy_node()


if __name__ == '__main__':
    main()
