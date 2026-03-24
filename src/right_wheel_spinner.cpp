// right_wheel_spinner.cpp
// Simple ROS 2 node that streams motor packets to spin only the right wheel.

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <thread>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>

using namespace std::chrono_literals;

namespace {
int open_serial(const std::string &port, int baud) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        return -1;
    }

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return -1;
    }

    speed_t speed;
    switch (baud) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default: speed = B115200; break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK |
                      ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}
}

class RightWheelSpinner : public rclcpp::Node {
public:
    RightWheelSpinner() : Node("right_wheel_spinner") {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<int>("left_pwm", 0);
        this->declare_parameter<int>("right_pwm", 110);
        this->declare_parameter<double>("duration_s", 5.0);
        this->declare_parameter<double>("send_hz", 20.0);

        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_   = this->get_parameter("baud_rate").as_int();
        left_pwm_    = this->get_parameter("left_pwm").as_int();
        right_pwm_   = this->get_parameter("right_pwm").as_int();
        duration_s_  = this->get_parameter("duration_s").as_double();
        send_hz_     = this->get_parameter("send_hz").as_double();

        serial_fd_ = open_serial(serial_port_, baud_rate_);
        if (serial_fd_ < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to open %s", serial_port_.c_str());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(get_logger(), "Streaming left/right PWM=%d/%d on %s for %.2f s",
                    left_pwm_, right_pwm_, serial_port_.c_str(), duration_s_);

        build_packets();

        end_time_ = this->now() + rclcpp::Duration::from_seconds(duration_s_);
        auto period = std::chrono::duration<double>(1.0 / send_hz_);
        timer_ = this->create_wall_timer(period, std::bind(&RightWheelSpinner::timer_cb, this));
    }

    ~RightWheelSpinner() override {
        stop_wheel();
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
    }

private:
    void build_packets() {
        auto build = [](int8_t left, int8_t right, int16_t yaw_mrad = 0) {
            uint8_t pkt[8];
            pkt[0] = 0xAA;
            pkt[1] = 0x55;
            pkt[2] = static_cast<uint8_t>(left);
            pkt[3] = static_cast<uint8_t>(right);
            pkt[4] = static_cast<uint8_t>((yaw_mrad >> 8) & 0xFF);
            pkt[5] = static_cast<uint8_t>(yaw_mrad & 0xFF);
            uint8_t checksum = 0;
            for (int i = 0; i < 6; i++) checksum += pkt[i];
            pkt[6] = checksum;
            pkt[7] = 0x0D;
            return std::array<uint8_t, 8>{pkt[0], pkt[1], pkt[2], pkt[3], pkt[4], pkt[5], pkt[6], pkt[7]};
        };

        cmd_packet_  = build(static_cast<int8_t>(left_pwm_), static_cast<int8_t>(right_pwm_));
        stop_packet_ = build(0, 0);
    }

    void timer_cb() {
        if (serial_fd_ < 0) return;

        if (this->now() >= end_time_) {
            RCLCPP_INFO(get_logger(), "Spin duration done; stopping");
            stop_wheel();
            timer_->cancel();
            rclcpp::shutdown();
            return;
        }

        ::write(serial_fd_, cmd_packet_.data(), cmd_packet_.size());
    }

    void stop_wheel() {
        if (serial_fd_ >= 0) {
            ::write(serial_fd_, stop_packet_.data(), stop_packet_.size());
        }
    }

    std::string serial_port_;
    int serial_fd_{-1};
    int baud_rate_;
    int left_pwm_;
    int right_pwm_;
    double duration_s_;
    double send_hz_;
    rclcpp::Time end_time_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::array<uint8_t, 8> cmd_packet_{};
    std::array<uint8_t, 8> stop_packet_{};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RightWheelSpinner>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
