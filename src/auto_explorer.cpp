// ═══════════════════════════════════════════════════════════════════════════════
// auto_explorer.cpp — Autonomous exploration using reactive obstacle avoidance
//
// Subscribes to /scan, publishes /cmd_vel.
// Behavior:
//   - Drive forward when the path ahead is clear
//   - Turn toward the more open side when an obstacle is detected
//   - Rotate in place when boxed in
//   - Slight random bias to avoid getting stuck in loops
// ═══════════════════════════════════════════════════════════════════════════════
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <algorithm>
#include <random>
#include <limits>

class AutoExplorer : public rclcpp::Node {
public:
    AutoExplorer() : Node("auto_explorer"), rng_(std::random_device{}()) {
        // Parameters
        this->declare_parameter("linear_speed", 0.8);        // m/s forward speed (~PWM 18)
        this->declare_parameter("angular_speed", 10.0);      // rad/s turn speed (high torque needed to spin)
        this->declare_parameter("obstacle_dist", 0.35);      // m — stop and turn threshold
        this->declare_parameter("slowdown_dist", 0.6);       // m — start slowing down
        this->declare_parameter("side_obstacle_dist", 0.25); // m — side clearance
        this->declare_parameter("timer_hz", 10.0);

        linear_speed_     = this->get_parameter("linear_speed").as_double();
        angular_speed_    = this->get_parameter("angular_speed").as_double();
        obstacle_dist_    = this->get_parameter("obstacle_dist").as_double();
        slowdown_dist_    = this->get_parameter("slowdown_dist").as_double();
        side_obstacle_dist_ = this->get_parameter("side_obstacle_dist").as_double();
        double hz         = this->get_parameter("timer_hz").as_double();

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&AutoExplorer::scan_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
            std::bind(&AutoExplorer::control_loop, this));

        RCLCPP_INFO(this->get_logger(),
            "AutoExplorer ready — speed=%.2f m/s, obstacle=%.2fm, slowdown=%.2fm",
            linear_speed_, obstacle_dist_, slowdown_dist_);
    }

private:
    // Publishers / subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double linear_speed_;
    double angular_speed_;
    double obstacle_dist_;
    double slowdown_dist_;
    double side_obstacle_dist_;

    // Scan sectors (min range in each zone)
    double front_dist_  = 999.0;  // -30° to +30°
    double left_dist_   = 999.0;  // +30° to +90°
    double right_dist_  = 999.0;  // -90° to -30°
    double fl_dist_     = 999.0;  // front-left: +15° to +45°
    double fr_dist_     = 999.0;  // front-right: -45° to -15°
    bool scan_received_ = false;

    // State
    enum State { DRIVE_FORWARD, TURN_LEFT, TURN_RIGHT, REVERSE_TURN };
    State state_ = DRIVE_FORWARD;
    int turn_ticks_ = 0;  // how long to keep turning

    // Random
    std::mt19937 rng_;

    // ── Compute min range in an angular sector ───────────────────────────────
    double sector_min(const sensor_msgs::msg::LaserScan& scan, double angle_min_deg, double angle_max_deg) {
        double min_range = 999.0;
        for (size_t i = 0; i < scan.ranges.size(); i++) {
            double angle_deg = (scan.angle_min + i * scan.angle_increment) * 180.0 / M_PI;
            // Normalize to -180..180
            while (angle_deg > 180.0) angle_deg -= 360.0;
            while (angle_deg < -180.0) angle_deg += 360.0;

            if (angle_deg >= angle_min_deg && angle_deg <= angle_max_deg) {
                float r = scan.ranges[i];
                if (std::isfinite(r) && r >= scan.range_min && r <= scan.range_max) {
                    min_range = std::min(min_range, static_cast<double>(r));
                }
            }
        }
        return min_range;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        front_dist_ = sector_min(*msg, -30.0, 30.0);
        left_dist_  = sector_min(*msg, 30.0, 90.0);
        right_dist_ = sector_min(*msg, -90.0, -30.0);
        fl_dist_    = sector_min(*msg, 15.0, 45.0);
        fr_dist_    = sector_min(*msg, -45.0, -15.0);
        scan_received_ = true;
    }

    void control_loop() {
        if (!scan_received_) return;

        auto cmd = geometry_msgs::msg::Twist();

        // Decrement turn counter
        if (turn_ticks_ > 0) {
            turn_ticks_--;
            // Continue current turn
            if (state_ == TURN_LEFT) {
                cmd.angular.z = angular_speed_;
            } else if (state_ == TURN_RIGHT) {
                cmd.angular.z = -angular_speed_;
            } else if (state_ == REVERSE_TURN) {
                cmd.linear.x = -0.4;
                cmd.angular.z = angular_speed_;
            }
            cmd_pub_->publish(cmd);
            return;
        }

        // ── Decision logic ───────────────────────────────────────────────────
        bool front_blocked = front_dist_ < obstacle_dist_;
        bool front_close   = front_dist_ < slowdown_dist_;
        bool left_tight    = left_dist_ < side_obstacle_dist_;
        bool right_tight   = right_dist_ < side_obstacle_dist_;

        if (front_blocked && left_tight && right_tight) {
            // Boxed in — back up and turn
            state_ = REVERSE_TURN;
            turn_ticks_ = random_ticks(15, 25);
            cmd.linear.x = -0.05;
            cmd.angular.z = angular_speed_;
            RCLCPP_DEBUG(this->get_logger(), "Boxed in — reversing");
        }
        else if (front_blocked) {
            // Turn toward the more open side
            if (left_dist_ > right_dist_) {
                state_ = TURN_LEFT;
                cmd.angular.z = angular_speed_;
            } else if (right_dist_ > left_dist_) {
                state_ = TURN_RIGHT;
                cmd.angular.z = -angular_speed_;
            } else {
                // Equal — pick randomly
                state_ = (rng_() % 2 == 0) ? TURN_LEFT : TURN_RIGHT;
                cmd.angular.z = (state_ == TURN_LEFT) ? angular_speed_ : -angular_speed_;
            }
            turn_ticks_ = random_ticks(5, 15);
            RCLCPP_DEBUG(this->get_logger(), "Obstacle ahead (%.2fm) — turning %s",
                front_dist_, (state_ == TURN_LEFT) ? "left" : "right");
        }
        else {
            // Path is clear — drive forward
            state_ = DRIVE_FORWARD;
            if (front_close) {
                // Proportional slowdown as we approach obstacle
                double scale = (front_dist_ - obstacle_dist_) / (slowdown_dist_ - obstacle_dist_);
                scale = std::clamp(scale, 0.2, 1.0);
                cmd.linear.x = linear_speed_ * scale;
            } else {
                cmd.linear.x = linear_speed_;
            }

            // Gentle steering away from close side walls
            if (fl_dist_ < slowdown_dist_ && fr_dist_ > fl_dist_) {
                cmd.angular.z = -4.0;  // nudge right
            } else if (fr_dist_ < slowdown_dist_ && fl_dist_ > fr_dist_) {
                cmd.angular.z = 4.0;   // nudge left
            }
        }

        cmd_pub_->publish(cmd);
    }

    int random_ticks(int min_t, int max_t) {
        std::uniform_int_distribution<int> dist(min_t, max_t);
        return dist(rng_);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
