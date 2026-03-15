// ═══════════════════════════════════════════════════════════════════════════════
// diff_drive_controller.cpp — ROS 2 Jazzy
//
// Bridges /cmd_vel (Twist) → serial motor packets for BTS7960 differential drive
// Receives IMU packets from Arduino (MPU6050) → publishes /imu/data_raw
// Publishes open-loop odometry on /odom/cmd_vel (low-trust input for EKF)
// NOTE: The robot_localization EKF owns /odom and the odom→base_link TF
//
// Serial protocol (Pi → Arduino):
//   Motor packet [6 bytes]: 0xAA 0x55 <left_i8> <right_i8> <checksum> 0x0D
//
// Serial protocol (Arduino → Pi):
//   IMU packet [16 bytes]: 0xBB 0x66 <ax_h> <ax_l> <ay_h> <ay_l> <az_h> <az_l>
//                          <gx_h> <gx_l> <gy_h> <gy_l> <gz_h> <gz_l> <cksum> 0x0D
//
//   [FUTURE] Encoder packet [10 bytes]: 0xCC 0x77 <left_ticks_i32> <right_ticks_i32> <cksum> 0x0D
//
// Author: Principal Robotics Engineer Pipeline — auto-generated
// ═══════════════════════════════════════════════════════════════════════════════
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <cstdint>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// MPU6050 scale factors (default ±2g accel, ±250°/s gyro)
// ─────────────────────────────────────────────────────────────────────────────
static constexpr double ACCEL_SCALE = 9.80665 / 16384.0;   // raw → m/s²
static constexpr double GYRO_SCALE  = (M_PI / 180.0) / 131.0; // raw → rad/s

class DiffDriveController : public rclcpp::Node {
public:
    DiffDriveController() : Node("diff_drive_controller") {
        // ── Declare parameters ───────────────────────────────────────────────
        this->declare_parameter("serial_port",      "/dev/ttyACM0");
        this->declare_parameter("baud_rate",         115200);
        this->declare_parameter("wheel_radius",      0.09);
        this->declare_parameter("wheel_separation",  0.2);
        this->declare_parameter("max_rpm",           600.0);
        this->declare_parameter("cmd_vel_timeout_ms", 500);

        serial_port_     = this->get_parameter("serial_port").as_string();
        baud_rate_       = this->get_parameter("baud_rate").as_int();
        wheel_radius_    = this->get_parameter("wheel_radius").as_double();
        wheel_sep_       = this->get_parameter("wheel_separation").as_double();
        max_rpm_         = this->get_parameter("max_rpm").as_double();
        cmd_timeout_ms_  = this->get_parameter("cmd_vel_timeout_ms").as_int();

        // Derived
        max_wheel_vel_ = (max_rpm_ * 2.0 * M_PI) / 60.0; // rad/s

        // ── Open serial port ─────────────────────────────────────────────────
        serial_fd_ = open_serial(serial_port_, baud_rate_);
        if (serial_fd_ < 0) {
            RCLCPP_FATAL(get_logger(), "Cannot open serial port %s", serial_port_.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "Serial open: %s @ %d baud", serial_port_.c_str(), baud_rate_);

        // ── Publishers ───────────────────────────────────────────────────────
        // Publish to /odom/cmd_vel — the EKF fuses this as a low-trust source
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom/cmd_vel", 10);
        imu_pub_  = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);

        // ── Subscriber ───────────────────────────────────────────────────────
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&DiffDriveController::cmd_vel_callback, this, std::placeholders::_1));

        // NOTE: odom→base_link TF is published by the robot_localization EKF node

        // ── Timers ───────────────────────────────────────────────────────────
        // Motor send + watchdog @ 20 Hz
        motor_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DiffDriveController::motor_timer_callback, this));

        // Odometry @ 20 Hz
        odom_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DiffDriveController::odom_timer_callback, this));

        // ── Serial reader thread (for IMU + future encoder packets) ──────────
        running_.store(true);
        serial_reader_thread_ = std::thread(&DiffDriveController::serial_reader_loop, this);

        RCLCPP_INFO(get_logger(),
            "DiffDriveController ready — wheel_r=%.3fm, wheel_sep=%.3fm, max_rpm=%.0f",
            wheel_radius_, wheel_sep_, max_rpm_);
    }

    ~DiffDriveController() override {
        running_.store(false);
        if (serial_reader_thread_.joinable()) serial_reader_thread_.join();
        if (serial_fd_ >= 0) {
            // Send stop before closing
            send_motor_packet(0, 0);
            close(serial_fd_);
        }
    }

private:
    // ═════════════════════════════════════════════════════════════════════════
    // Parameters
    // ═════════════════════════════════════════════════════════════════════════
    std::string serial_port_;
    int         baud_rate_;
    double      wheel_radius_;
    double      wheel_sep_;
    double      max_rpm_;
    int         cmd_timeout_ms_;
    double      max_wheel_vel_; // rad/s

    // ═════════════════════════════════════════════════════════════════════════
    // Serial
    // ═════════════════════════════════════════════════════════════════════════
    int serial_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread serial_reader_thread_;
    std::mutex serial_write_mutex_;

    // ═════════════════════════════════════════════════════════════════════════
    // ROS interfaces
    // ═════════════════════════════════════════════════════════════════════════
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr        imu_pub_;
    rclcpp::TimerBase::SharedPtr motor_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    // ═════════════════════════════════════════════════════════════════════════
    // Motion state (protected by mutex)
    // ═════════════════════════════════════════════════════════════════════════
    std::mutex cmd_mutex_;
    double target_linear_  = 0.0;
    double target_angular_ = 0.0;
    rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
    bool   cmd_received_ = false;

    // Open-loop odometry state
    double odom_x_     = 0.0;
    double odom_y_     = 0.0;
    double odom_theta_ = 0.0;
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
    bool odom_initialized_ = false;

    // Current wheel velocities (for odometry)
    double current_left_vel_  = 0.0;  // m/s
    double current_right_vel_ = 0.0;  // m/s

    // ═════════════════════════════════════════════════════════════════════════
    // /cmd_vel callback
    // ═════════════════════════════════════════════════════════════════════════
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        target_linear_  = msg->linear.x;
        target_angular_ = msg->angular.z;
        last_cmd_time_  = this->now();
        cmd_received_   = true;
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Motor timer — inverse kinematics + serial TX (20 Hz)
    // ═════════════════════════════════════════════════════════════════════════
    void motor_timer_callback() {
        double v_lin, v_ang;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);

            // ── Watchdog: stop if no cmd_vel for timeout period ──────────────
            if (cmd_received_) {
                auto elapsed = (this->now() - last_cmd_time_).nanoseconds() / 1e6;
                if (elapsed > cmd_timeout_ms_) {
                    target_linear_  = 0.0;
                    target_angular_ = 0.0;
                    if (elapsed < cmd_timeout_ms_ + 100) { // log once
                        RCLCPP_WARN(get_logger(), "cmd_vel timeout — motors stopped");
                    }
                }
            }
            v_lin = target_linear_;
            v_ang = target_angular_;
        }

        // ── Differential-drive inverse kinematics ────────────────────────────
        //   v_left  = v_linear - (angular × wheel_sep / 2)
        //   v_right = v_linear + (angular × wheel_sep / 2)
        double v_left  = v_lin - (v_ang * wheel_sep_ / 2.0);
        double v_right = v_lin + (v_ang * wheel_sep_ / 2.0);

        // Store for open-loop odometry (m/s)
        current_left_vel_  = v_left;
        current_right_vel_ = v_right;

        // Convert m/s → wheel rad/s → fraction of max → PWM [-127, +127]
        double left_rad_s  = v_left  / wheel_radius_;
        double right_rad_s = v_right / wheel_radius_;

        int8_t left_pwm  = velocity_to_pwm(left_rad_s);
        int8_t right_pwm = velocity_to_pwm(right_rad_s);

        send_motor_packet(left_pwm, right_pwm);
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Odometry timer (open-loop dead reckoning, 20 Hz)
    // ═════════════════════════════════════════════════════════════════════════
    void odom_timer_callback() {
        rclcpp::Time now = this->now();

        if (!odom_initialized_) {
            last_odom_time_   = now;
            odom_initialized_ = true;
            return;
        }

        double dt = (now - last_odom_time_).seconds();
        last_odom_time_ = now;
        if (dt <= 0.0 || dt > 1.0) return; // guard

        // ── Forward kinematics ───────────────────────────────────────────────
        double v_lin = (current_right_vel_ + current_left_vel_) / 2.0;
        double v_ang = (current_right_vel_ - current_left_vel_) / wheel_sep_;

        double delta_theta = v_ang * dt;
        double delta_x     = v_lin * cos(odom_theta_ + delta_theta / 2.0) * dt;
        double delta_y     = v_lin * sin(odom_theta_ + delta_theta / 2.0) * dt;

        odom_x_     += delta_x;
        odom_y_     += delta_y;
        odom_theta_ += delta_theta;

        // ── Publish odometry message ─────────────────────────────────────────
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp    = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id  = "base_link";

        odom_msg.pose.pose.position.x = odom_x_;
        odom_msg.pose.pose.position.y = odom_y_;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, odom_theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x  = v_lin;
        odom_msg.twist.twist.angular.z = v_ang;

        odom_pub_->publish(odom_msg);
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Serial reader thread — parses incoming IMU packets (and future encoders)
    // ═════════════════════════════════════════════════════════════════════════
    void serial_reader_loop() {
        std::vector<uint8_t> buf;
        buf.reserve(64);
        uint8_t byte;

        while (running_.load()) {
            int n = read(serial_fd_, &byte, 1);
            if (n <= 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            buf.push_back(byte);

            // Keep buffer from growing unbounded
            if (buf.size() > 64) {
                buf.erase(buf.begin(), buf.end() - 32);
            }

            // ── Try to match IMU packet: 0xBB 0x66 ... (16 bytes total) ──────
            if (buf.size() >= 16) {
                size_t start = buf.size() - 16;
                if (buf[start] == 0xBB && buf[start + 1] == 0x66 && buf[start + 15] == 0x0D) {
                    // Validate checksum
                    uint8_t cksum = 0;
                    for (int i = 0; i < 14; ++i) {
                        cksum += buf[start + i];
                    }
                    if (cksum == buf[start + 14]) {
                        parse_and_publish_imu(&buf[start + 2]);
                        buf.clear();
                        continue;
                    }
                }
            }

            // ── [FUTURE] Encoder packet: 0xCC 0x77 ... (10 bytes total) ──────
            // When you add encoders, uncomment and implement:
            // if (buf.size() >= 10) {
            //     size_t start = buf.size() - 10;
            //     if (buf[start] == 0xCC && buf[start + 1] == 0x77 && buf[start + 9] == 0x0D) {
            //         uint8_t cksum = 0;
            //         for (int i = 0; i < 8; ++i) cksum += buf[start + i];
            //         if (cksum == buf[start + 8]) {
            //             parse_encoder_ticks(&buf[start + 2]);
            //             buf.clear();
            //             continue;
            //         }
            //     }
            // }
        }
    }

    void parse_and_publish_imu(const uint8_t* data) {
        // data[0..11] = ax_h ax_l ay_h ay_l az_h az_l gx_h gx_l gy_h gy_l gz_h gz_l
        auto to_int16 = [](uint8_t h, uint8_t l) -> int16_t {
            return static_cast<int16_t>((h << 8) | l);
        };

        int16_t ax_raw = to_int16(data[0],  data[1]);
        int16_t ay_raw = to_int16(data[2],  data[3]);
        int16_t az_raw = to_int16(data[4],  data[5]);
        int16_t gx_raw = to_int16(data[6],  data[7]);
        int16_t gy_raw = to_int16(data[8],  data[9]);
        int16_t gz_raw = to_int16(data[10], data[11]);

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp    = this->now();
        imu_msg.header.frame_id = "imu_link";

        imu_msg.linear_acceleration.x = ax_raw * ACCEL_SCALE;
        imu_msg.linear_acceleration.y = ay_raw * ACCEL_SCALE;
        imu_msg.linear_acceleration.z = az_raw * ACCEL_SCALE;

        imu_msg.angular_velocity.x = gx_raw * GYRO_SCALE;
        imu_msg.angular_velocity.y = gy_raw * GYRO_SCALE;
        imu_msg.angular_velocity.z = gz_raw * GYRO_SCALE;

        // Orientation not provided by MPU6050 raw data — mark unknown
        imu_msg.orientation_covariance[0] = -1.0;

        // Set covariance (diagonal) — approximate for MPU6050
        imu_msg.linear_acceleration_covariance[0] = 0.04;
        imu_msg.linear_acceleration_covariance[4] = 0.04;
        imu_msg.linear_acceleration_covariance[8] = 0.04;

        imu_msg.angular_velocity_covariance[0] = 0.02;
        imu_msg.angular_velocity_covariance[4] = 0.02;
        imu_msg.angular_velocity_covariance[8] = 0.02;

        imu_pub_->publish(imu_msg);
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Helpers
    // ═════════════════════════════════════════════════════════════════════════

    int8_t velocity_to_pwm(double wheel_rad_s) {
        // Clamp to max
        double ratio = wheel_rad_s / max_wheel_vel_;
        ratio = std::clamp(ratio, -1.0, 1.0);
        return static_cast<int8_t>(ratio * 127.0);
    }

    void send_motor_packet(int8_t left, int8_t right) {
        uint8_t packet[6];
        packet[0] = 0xAA;                        // Header
        packet[1] = 0x55;                        // Header
        packet[2] = static_cast<uint8_t>(left);  // Left speed (signed, reinterpreted)
        packet[3] = static_cast<uint8_t>(right); // Right speed
        packet[4] = (packet[0] + packet[1] + packet[2] + packet[3]) & 0xFF; // Checksum
        packet[5] = 0x0D;                        // Tail

        std::lock_guard<std::mutex> lock(serial_write_mutex_);
        ::write(serial_fd_, packet, 6);
    }

    static int open_serial(const std::string& port, int baud) {
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) return -1;

        struct termios tty{};
        if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }

        speed_t speed;
        switch (baud) {
            case 9600:   speed = B9600;   break;
            case 19200:  speed = B19200;  break;
            case 38400:  speed = B38400;  break;
            case 57600:  speed = B57600;  break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            case 460800: speed = B460800; break;
            default:     speed = B115200; break;
        }
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        // Raw mode
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK |
                          ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1; // 100ms read timeout

        if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
        tcflush(fd, TCIOFLUSH);

        return fd;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiffDriveController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
