#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <cmath>
#include <thread>
#include <limits> // Required for Infinity

class CustomLidarNode : public rclcpp::Node {
public:
    CustomLidarNode() : Node("custom_lidar_node") {
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/ttyUSB0.");
            return;
        }

        struct termios tty;
        tcgetattr(serial_fd_, &tty);
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        // Hardened timeouts
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;

        tcsetattr(serial_fd_, TCSANOW, &tty);

        RCLCPP_INFO(this->get_logger(), "Hardware connected. Broadcasting to /scan...");

        read_thread_ = std::thread(&CustomLidarNode::read_serial_loop, this);
    }

    ~CustomLidarNode() {
        if (serial_fd_ >= 0) close(serial_fd_);
        if (read_thread_.joinable()) read_thread_.join();
    }

private:
    int serial_fd_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    std::thread read_thread_;
    float scan_360[360] = {0.0f};
    float last_angle = 0.0f; // Tracks laser rotation

    void read_serial_loop() {
        std::vector<uint8_t> header_buffer;
        uint8_t byte;

        while (rclcpp::ok()) {
            if (read(serial_fd_, &byte, 1) > 0) {
                header_buffer.push_back(byte);

                if (header_buffer.size() > 8) {
                    header_buffer.erase(header_buffer.begin());
                }

                if (header_buffer.size() == 8 && header_buffer[5] == 0xAD) {
                    uint16_t payload_length = (header_buffer[6] << 8) | header_buffer[7];

                    if (payload_length > 200) {
                        header_buffer.clear();
                        continue;
                    }

                    std::vector<uint8_t> payload_data(payload_length);
                    int bytes_read = 0;
                    bool timeout_occurred = false;

                    while (bytes_read < payload_length) {
                        int r = read(serial_fd_, &payload_data[bytes_read], payload_length - bytes_read);
                        if (r > 0) {
                            bytes_read += r;
                        } else {
                            timeout_occurred = true;
                            break;
                        }
                    }

                    if (timeout_occurred) {
                        header_buffer.clear();
                        continue;
                    }

                    uint8_t crc[2];
                    bytes_read = 0;
                    while (bytes_read < 2) {
                        int r = read(serial_fd_, &crc[bytes_read], 2 - bytes_read);
                        if (r > 0) {
                            bytes_read += r;
                        } else {
                            timeout_occurred = true;
                            break;
                        }
                    }

                    if (timeout_occurred) {
                        header_buffer.clear();
                        continue;
                    }

                    // --- NEW MATH & ROTATION PUBLISHING ---
                    if (payload_length >= 5) {
                        uint16_t start_angle_raw = (payload_data[3] << 8) | payload_data[4];
                        float start_angle = start_angle_raw * 0.01f;
                        int sample_count = (payload_length - 5) / 3;

                        for (int i = 0; i < sample_count; ++i) {
                            float angle = start_angle + i * (360.0f / (16.0f * sample_count));
                            while (angle >= 360.0f) angle -= 360.0f;
                            while (angle < 0.0f) angle += 360.0f;

                            int idx = 5 + (i * 3);
                            if (idx + 2 < payload_length) {
                                uint16_t distance_raw = (payload_data[idx + 1] << 8) | payload_data[idx + 2];
                                float distance = distance_raw * 0.25f; // in mm

                                int angle_int = (int)angle % 360;

                                // Trigger publish exactly when the laser crosses from 350+ degrees back to 0
                                if (angle_int < 45 && last_angle > 315) {
                                    publish_scan();
                                }

                                // Only save valid, non-zero distances
                                if (angle_int >= 0 && angle_int < 360 && distance > 0.0f) {
                                    scan_360[angle_int] = distance;
                                }

                                last_angle = angle_int;
                            }
                        }
                    }

                    header_buffer.clear();
                }
            }
        }
    }

    void publish_scan() {
        auto msg = sensor_msgs::msg::LaserScan();
        msg.header.stamp = this->now();
        msg.header.frame_id = "laser";

        msg.angle_min = 0.0;
        msg.angle_max = 2.0 * M_PI;
        msg.angle_increment = (2.0 * M_PI) / 360.0;

        msg.range_min = 0.15; // 15cm minimum
        msg.range_max = 8.0;  // 8m maximum

        // Convert to meters and handle missing data
        for (int i = 0; i < 360; i++) {
            if (scan_360[i] > 0.0f) {
                msg.ranges.push_back(scan_360[i] / 1000.0f);
            } else {
                // Tell ROS to ignore this angle instead of drawing a line to 0,0
                msg.ranges.push_back(std::numeric_limits<float>::infinity());
            }
            // Wipe the array clean for the next physical rotation
            scan_360[i] = 0.0f;
        }

        scan_pub_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomLidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}