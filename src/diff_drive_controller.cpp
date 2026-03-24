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
//   NOTE: Motors are negated and swapped so "forward" = camera direction
//
// Serial protocol (Arduino → Pi):
//   IMU packet [16 bytes]: 0xBB 0x66 <ax_h> <ax_l> <ay_h> <ay_l> <az_h> <az_l>
//                          <gx_h> <gx_l> <gy_h> <gy_l> <gz_h> <gz_l> <cksum> 0x0D
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

static constexpr double ACCEL_SCALE = 9.80665 / 16384.0;
static constexpr double GYRO_SCALE  = (M_PI / 180.0) / 131.0;

class DiffDriveController : public rclcpp::Node {
public:
    DiffDriveController() : Node("diff_drive_controller") {
        this->declare_parameter("serial_port",       "/dev/ttyACM0");
        this->declare_parameter("baud_rate",          115200);
        this->declare_parameter("wheel_radius",       0.09);
        this->declare_parameter("wheel_separation",   0.2);
        this->declare_parameter("max_rpm",            600.0);
        this->declare_parameter("cmd_vel_timeout_ms", 500);

        serial_port_    = this->get_parameter("serial_port").as_string();
        baud_rate_      = this->get_parameter("baud_rate").as_int();
        wheel_radius_   = this->get_parameter("wheel_radius").as_double();
        wheel_sep_      = this->get_parameter("wheel_separation").as_double();
        max_rpm_        = this->get_parameter("max_rpm").as_double();
        cmd_timeout_ms_ = this->get_parameter("cmd_vel_timeout_ms").as_int();
        max_wheel_vel_  = (max_rpm_ * 2.0 * M_PI) / 60.0;

        serial_fd_ = open_serial(serial_port_, baud_rate_);
        if (serial_fd_ < 0) {
            RCLCPP_FATAL(get_logger(), "Cannot open serial port %s", serial_port_.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "Serial open: %s @ %d baud", serial_port_.c_str(), baud_rate_);

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom/cmd_vel", 10);
        imu_pub_  = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&DiffDriveController::cmd_vel_callback, this, std::placeholders::_1));

        // Motor loop at 50 Hz (fast enough to be smooth, matches Arduino IMU rate)
        motor_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&DiffDriveController::motor_timer_callback, this));

        odom_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DiffDriveController::odom_timer_callback, this));

        running_.store(true);
        serial_reader_thread_ = std::thread(&DiffDriveController::serial_reader_loop, this);

        RCLCPP_INFO(get_logger(),
            "DiffDriveController ready — r=%.3fm sep=%.3fm rpm=%.0f | 50Hz motor + EMA smoothing",
            wheel_radius_, wheel_sep_, max_rpm_);
    }

    ~DiffDriveController() override {
        running_.store(false);
        if (serial_reader_thread_.joinable()) serial_reader_thread_.join();
        if (serial_fd_ >= 0) {
            send_motor_packet(0, 0);
            close(serial_fd_);
        }
    }

private:
    std::string serial_port_;
    int    baud_rate_, cmd_timeout_ms_;
    double wheel_radius_, wheel_sep_, max_rpm_, max_wheel_vel_;

    int serial_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread serial_reader_thread_;
    std::mutex serial_write_mutex_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr        imu_pub_;
    rclcpp::TimerBase::SharedPtr motor_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    std::mutex cmd_mutex_;
    double target_linear_  = 0.0;
    double target_angular_ = 0.0;
    rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
    bool cmd_received_ = false;

    // ── Velocity smoothing state ─────────────────────────────────────────────
    double smooth_lin_ = 0.0;
    double smooth_ang_ = 0.0;

    // Smoothing: simple EMA only (no rate limiter — it was strangling the signal)
    // At 50Hz with alpha=0.3: reaches 95% of target in ~0.2s (smooth but responsive)
    static constexpr double EMA_LIN = 0.3;
    static constexpr double EMA_ANG = 0.3;

    // Odometry state
    double odom_x_ = 0.0, odom_y_ = 0.0, odom_theta_ = 0.0;
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
    bool odom_initialized_ = false;
    double current_left_vel_ = 0.0, current_right_vel_ = 0.0;

    // Gyro bias calibration
    static constexpr int CALIB_SAMPLES = 150; // ~3s at 50Hz
    int calib_count_ = 0;
    std::atomic<bool> calib_done_{false};
    double gyro_bias_x_ = 0.0, gyro_bias_y_ = 0.0, gyro_bias_z_ = 0.0;
    double gyro_sum_x_  = 0.0, gyro_sum_y_  = 0.0, gyro_sum_z_  = 0.0;

    // ═════════════════════════════════════════════════════════════════════════
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        target_linear_  = msg->linear.x;
        target_angular_ = msg->angular.z;
        last_cmd_time_  = this->now();
        cmd_received_   = true;
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Motor timer — 50 Hz with rate limiting + EMA smoothing
    // ═════════════════════════════════════════════════════════════════════════
    void motor_timer_callback() {
        double v_lin, v_ang;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            if (cmd_received_) {
                auto elapsed = (this->now() - last_cmd_time_).nanoseconds() / 1e6;
                if (elapsed > cmd_timeout_ms_) {
                    target_linear_  = 0.0;
                    target_angular_ = 0.0;
                    if (elapsed < cmd_timeout_ms_ + 100)
                        RCLCPP_WARN(get_logger(), "cmd_vel timeout — motors stopped");
                }
            }
            v_lin = target_linear_;
            v_ang = target_angular_;
        }

        // ── EMA smoothing (simple, one-stage) ─────────────────────────────────
        smooth_lin_ += EMA_LIN * (v_lin - smooth_lin_);
        smooth_ang_ += EMA_ANG * (v_ang - smooth_ang_);

        // ── Inverse kinematics ───────────────────────────────────────────────
        double v_left  = smooth_lin_ - (smooth_ang_ * wheel_sep_ / 2.0);
        double v_right = smooth_lin_ + (smooth_ang_ * wheel_sep_ / 2.0);

        current_left_vel_  = v_left;
        current_right_vel_ = v_right;

        double left_rad_s  = v_left  / wheel_radius_;
        double right_rad_s = v_right / wheel_radius_;

        int8_t left_pwm  = velocity_to_pwm(left_rad_s);
        int8_t right_pwm = velocity_to_pwm(right_rad_s);

        send_motor_packet(left_pwm, right_pwm);
    }

    // ═════════════════════════════════════════════════════════════════════════
    void odom_timer_callback() {
        rclcpp::Time now = this->now();
        if (!odom_initialized_) {
            last_odom_time_ = now;
            odom_initialized_ = true;
            return;
        }
        double dt = (now - last_odom_time_).seconds();
        last_odom_time_ = now;
        if (dt <= 0.0 || dt > 1.0) return;

        double v_lin = (current_right_vel_ + current_left_vel_) / 2.0;
        double v_ang = (current_right_vel_ - current_left_vel_) / wheel_sep_;

        odom_theta_ += v_ang * dt;
        odom_x_ += v_lin * cos(odom_theta_) * dt;
        odom_y_ += v_lin * sin(odom_theta_) * dt;

        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = now;
        msg.header.frame_id = "odom";
        msg.child_frame_id  = "base_link";
        msg.pose.pose.position.x = odom_x_;
        msg.pose.pose.position.y = odom_y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_theta_);
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
        msg.twist.twist.linear.x  = v_lin;
        msg.twist.twist.angular.z = v_ang;
        odom_pub_->publish(msg);
    }

    // ═════════════════════════════════════════════════════════════════════════
    void serial_reader_loop() {
        std::vector<uint8_t> buf;
        buf.reserve(64);
        uint8_t byte;
        while (running_.load()) {
            int n = read(serial_fd_, &byte, 1);
            if (n <= 0) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
            buf.push_back(byte);
            if (buf.size() > 64) buf.erase(buf.begin(), buf.end() - 32);

            if (buf.size() >= 16) {
                size_t s = buf.size() - 16;
                if (buf[s] == 0xBB && buf[s+1] == 0x66 && buf[s+15] == 0x0D) {
                    uint8_t ck = 0;
                    for (int i = 0; i < 14; ++i) ck += buf[s+i];
                    if (ck == buf[s+14]) {
                        parse_and_publish_imu(&buf[s+2]);
                        buf.clear();
                    }
                }
            }
        }
    }

    void parse_and_publish_imu(const uint8_t* d) {
        auto i16 = [](uint8_t h, uint8_t l) -> int16_t {
            return static_cast<int16_t>((h << 8) | l);
        };
        int16_t ax = i16(d[0],d[1]), ay = i16(d[2],d[3]), az = i16(d[4],d[5]);
        int16_t gx = i16(d[6],d[7]), gy = i16(d[8],d[9]), gz = i16(d[10],d[11]);

        double gxd = gx * GYRO_SCALE, gyd = gy * GYRO_SCALE, gzd = gz * GYRO_SCALE;

        // Gyro bias calibration (first ~3s while stationary)
        if (!calib_done_.load()) {
            gyro_sum_x_ += gxd; gyro_sum_y_ += gyd; gyro_sum_z_ += gzd;
            if (++calib_count_ >= CALIB_SAMPLES) {
                gyro_bias_x_ = gyro_sum_x_ / CALIB_SAMPLES;
                gyro_bias_y_ = gyro_sum_y_ / CALIB_SAMPLES;
                gyro_bias_z_ = gyro_sum_z_ / CALIB_SAMPLES;
                calib_done_.store(true);
                RCLCPP_INFO(get_logger(), "Gyro calibrated: bz=%.5f rad/s", gyro_bias_z_);
            }
            return; // don't publish until calibrated
        }

        gxd -= gyro_bias_x_; gyd -= gyro_bias_y_; gzd -= gyro_bias_z_;
        if (std::abs(gxd) < 0.01) gxd = 0.0;
        if (std::abs(gyd) < 0.01) gyd = 0.0;
        if (std::abs(gzd) < 0.01) gzd = 0.0;

        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";
        msg.linear_acceleration.x = ax * ACCEL_SCALE;
        msg.linear_acceleration.y = ay * ACCEL_SCALE;
        msg.linear_acceleration.z = az * ACCEL_SCALE;
        msg.angular_velocity.x = gxd;
        msg.angular_velocity.y = gyd;
        msg.angular_velocity.z = gzd;
        msg.orientation_covariance[0] = -1.0;
        msg.linear_acceleration_covariance[0] = 0.04;
        msg.linear_acceleration_covariance[4] = 0.04;
        msg.linear_acceleration_covariance[8] = 0.04;
        msg.angular_velocity_covariance[0] = 0.02;
        msg.angular_velocity_covariance[4] = 0.02;
        msg.angular_velocity_covariance[8] = 0.02;
        imu_pub_->publish(msg);
    }

    // ═════════════════════════════════════════════════════════════════════════
    int8_t velocity_to_pwm(double wheel_rad_s) {
        double ratio = std::clamp(wheel_rad_s / max_wheel_vel_, -1.0, 1.0);
        return static_cast<int8_t>(ratio * 127.0);
    }

    // 8-byte packet matching flashed Arduino firmware:
    // [0xAA][0x55][left][right][yaw_h][yaw_l][checksum][0x0D]
    // checksum = sum of bytes 0-5
    void send_motor_packet(int8_t left, int8_t right) {
        uint8_t packet[8];
        packet[0] = 0xAA;
        packet[1] = 0x55;
        packet[2] = static_cast<uint8_t>(-right);  // swap + negate: camera side is forward
        packet[3] = static_cast<uint8_t>(-left);
        packet[4] = 0x00;  // yaw_h (unused for now)
        packet[5] = 0x00;  // yaw_l
        packet[6] = static_cast<uint8_t>(packet[0] + packet[1] + packet[2] + packet[3] + packet[4] + packet[5]);
        packet[7] = 0x0D;
        std::lock_guard<std::mutex> lock(serial_write_mutex_);
        ::write(serial_fd_, packet, 8);
    }

    static int open_serial(const std::string& port, int baud) {
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) return -1;
        struct termios tty{};
        if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }
        speed_t speed;
        switch (baud) {
            case 9600:   speed = B9600;   break;
            case 57600:  speed = B57600;  break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            default:     speed = B115200; break;
        }
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB; tty.c_cflag &= ~CSTOPB; tty.c_cflag &= ~CRTSCTS;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK |
                          ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
        if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
        tcflush(fd, TCIOFLUSH);
        return fd;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveController>());
    rclcpp::shutdown();
    return 0;
}
