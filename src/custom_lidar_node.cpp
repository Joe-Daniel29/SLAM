#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <cmath>
#include <thread>
#include <limits>

class CustomLidarNode : public rclcpp::Node {
public:
    CustomLidarNode() : Node("custom_lidar_node") {
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/ttyUSB0.");
            return;
        }

        // Raw serial setup — matches pyserial defaults exactly
        struct termios tty;
        tcgetattr(serial_fd_, &tty);

        // Input flags: disable ALL input processing (critical for binary data)
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                         INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);

        // Output flags: disable ALL output processing
        tty.c_oflag &= ~OPOST;

        // Local flags: raw mode, no echo
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

        // Control flags: 8N1, no flow control
        tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        tty.c_cflag |= (CS8 | CLOCAL | CREAD);

        // Baud rate
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        // Blocking read with 500ms timeout (matches Python timeout=0.5)
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;  // 500ms in deciseconds

        tcflush(serial_fd_, TCIOFLUSH);
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

    // Persistent scan cache (like Python angle_distance_cache)
    float scan_cache_[360] = {0.0f};
    uint8_t scan_age_[360] = {0};
    static constexpr uint8_t MAX_AGE = 10;
    float last_angle_ = 0.0f;

    static constexpr uint8_t CMDTYPE_HEALTH = 0xAE;
    static constexpr uint8_t CMDTYPE_MEASUREMENT = 0xAD;

    // Read exactly n bytes (blocking with retry, like Python ser.read(n))
    bool read_exact(uint8_t* buf, int n) {
        int total = 0;
        while (total < n && rclcpp::ok()) {
            int r = read(serial_fd_, buf + total, n - total);
            if (r > 0) {
                total += r;
            } else {
                // Timeout or error
                return false;
            }
        }
        return total == n;
    }

    // ── Main read loop — mirrors lidar.py exactly ────────────────────────────
    void read_serial_loop() {
        uint8_t header[8];

        while (rclcpp::ok()) {
            // 1. Read 8-byte header (like Python: data = ser.read(8))
            if (!read_exact(header, 8)) continue;

            // Parse header fields (matching Python)
            // uint8_t  chunk_header  = header[0];
            // uint16_t chunk_length  = (header[1] << 8) | header[2];
            // uint8_t  chunk_version = header[3];
            // uint8_t  chunk_type    = header[4];
            uint8_t  command_type  = header[5];
            uint16_t payload_length = (header[6] << 8) | header[7];

            // Sanity check
            if (payload_length > 200) continue;

            // 2. Read payload + CRC for ALL packet types (like Python)
            std::vector<uint8_t> payload_data(payload_length);
            if (!read_exact(payload_data.data(), payload_length)) continue;

            uint8_t crc[2];
            if (!read_exact(crc, 2)) continue;

            // 3. Only process measurement packets (like Python: if command_type == CMDTYPE_MEASUREMENT)
            if (command_type != CMDTYPE_MEASUREMENT) continue;

            // motor_rpm = payload_data[0] * 3  (not needed for ROS, but matches Python)
            // offset_angle = (payload_data[1:3]) * 0.01  (Python uses this but we don't need it)

            if (payload_length < 5) continue;

            uint16_t start_angle_raw = (payload_data[3] << 8) | payload_data[4];
            float start_angle = start_angle_raw * 0.01f;
            int sample_count = (payload_length - 5) / 3;

            for (int i = 0; i < sample_count; ++i) {
                float angle = start_angle + i * (360.0f / (16.0f * sample_count));
                while (angle >= 360.0f) angle -= 360.0f;
                while (angle < 0.0f) angle += 360.0f;

                int idx = 5 + (i * 3);
                if (idx + 2 < (int)payload_length) {
                    // signal_quality = payload_data[idx]  (Python reads this but doesn't use it)
                    uint16_t distance_raw = (payload_data[idx + 1] << 8) | payload_data[idx + 2];
                    float distance = distance_raw * 0.25f; // in mm

                    // Round angle (matches Python: round(angle))
                    int angle_int = ((int)(angle + 0.5f)) % 360;

                    // Trigger publish when laser wraps 350+ → 0
                    if (angle_int < 45 && last_angle_ > 315) {
                        publish_scan();
                    }

                    // Direct overwrite (matches Python: angle_distance_cache[round(angle)] = round(distance))
                    if (angle_int >= 0 && angle_int < 360 && distance > 0.0f) {
                        scan_cache_[angle_int] = distance;
                        scan_age_[angle_int] = 0;
                    }

                    last_angle_ = angle_int;
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

        msg.range_min = 0.15;
        msg.range_max = 8.0;

        msg.scan_time = 1.0 / 7.5;
        msg.time_increment = msg.scan_time / 360.0;

        // Build scan from persistent cache (reversed + 180° rotation so LiDAR "forward" = camera side)
        for (int i = 359; i >= 0; i--) {
            int idx = (i + 90) % 360;
            if (scan_cache_[idx] > 0.0f && scan_age_[idx] <= MAX_AGE) {
                float range_m = scan_cache_[idx] / 1000.0f;
                if (range_m >= msg.range_min && range_m <= msg.range_max) {
                    msg.ranges.push_back(range_m);
                } else {
                    msg.ranges.push_back(std::numeric_limits<float>::infinity());
                }
            } else {
                msg.ranges.push_back(std::numeric_limits<float>::infinity());
            }
            if (scan_age_[i] < 255) scan_age_[i]++;
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
