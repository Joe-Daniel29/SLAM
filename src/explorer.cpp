// ═══════════════════════════════════════════════════════════════════════════════
// explorer.cpp v3 — Autonomous exploration with IMU-based stall detection
//
// Strategy:
//   WANDER       → drive toward open space
//   FOLLOW_LEFT  → PD wall-following, left wall
//   FOLLOW_RIGHT → PD wall-following, right wall  
//   ESCAPE       → full-power reverse+turn (bypasses EMA smoothing)
//
// Stall detection: compares commanded velocity vs EKF odometry velocity.
// If motors are commanded but odom shows no movement → physical obstacle
// that LiDAR missed (e.g. low object, ramp, cable). Triggers ESCAPE.
//
// Motor output is EMA-smoothed except during ESCAPE (instant response).
// ═══════════════════════════════════════════════════════════════════════════════
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <algorithm>
#include <random>

class Explorer : public rclcpp::Node {
public:
    Explorer() : Node("explorer"), rng_(std::random_device{}()) {
        this->declare_parameter("linear_speed", 0.3);
        this->declare_parameter("angular_speed", 2.0);
        this->declare_parameter("obstacle_dist", 0.30);
        this->declare_parameter("wall_follow_dist", 0.45);
        this->declare_parameter("smoothing", 0.15);
        this->declare_parameter("timer_hz", 10.0);
        this->declare_parameter("stall_threshold", 0.03);     // m/s — below this = not moving
        this->declare_parameter("stall_cmd_threshold", 0.15); // m/s — above this = we WANT to move
        this->declare_parameter("stall_ticks", 20);           // ~1.3s of stall before escape

        linear_speed_      = this->get_parameter("linear_speed").as_double();
        angular_speed_     = this->get_parameter("angular_speed").as_double();
        obstacle_dist_     = this->get_parameter("obstacle_dist").as_double();
        wall_dist_         = this->get_parameter("wall_follow_dist").as_double();
        alpha_             = this->get_parameter("smoothing").as_double();
        stall_vel_thresh_  = this->get_parameter("stall_threshold").as_double();
        stall_cmd_thresh_  = this->get_parameter("stall_cmd_threshold").as_double();
        stall_limit_       = this->get_parameter("stall_ticks").as_int();
        double hz          = this->get_parameter("timer_hz").as_double();

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&Explorer::scan_cb, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&Explorer::odom_cb, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / hz)),
            std::bind(&Explorer::tick, this));

        RCLCPP_INFO(get_logger(),
            "Explorer v3 — speed=%.2f ang=%.1f obstacle=%.2f wall=%.2f stall_limit=%d",
            linear_speed_, angular_speed_, obstacle_dist_, wall_dist_, stall_limit_);
    }

private:
    // ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Params
    double linear_speed_, angular_speed_, obstacle_dist_, wall_dist_, alpha_;
    double stall_vel_thresh_, stall_cmd_thresh_;
    int stall_limit_;

    // Smoothed output
    double smooth_lin_ = 0.0, smooth_ang_ = 0.0;

    // Scan sectors (EMA-filtered)
    double front_ = 5.0, front_l_ = 5.0, front_r_ = 5.0;
    double left_ = 5.0, right_ = 5.0;
    double left_far_ = 5.0, right_far_ = 5.0;
    bool scan_ok_ = false;

    // Odometry (from EKF — fused IMU + laser odom)
    double odom_vx_ = 0.0;    // actual linear velocity from EKF
    double odom_wz_ = 0.0;    // actual angular velocity from EKF

    // Stall detection
    int stall_counter_ = 0;
    double last_cmd_lin_ = 0.0;  // what we last commanded

    // State
    enum State { WANDER, FOLLOW_LEFT, FOLLOW_RIGHT, ESCAPE };
    State state_ = WANDER;
    int ticks_ = 0;
    int escape_count_ = 0;
    int escape_dir_ = 1;

    std::mt19937 rng_;

    // ── Scan processing (EMA filtered) ───────────────────────────────────────
    void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto sec = [&](double lo_deg, double hi_deg) {
            double best = 50.0;
            for (size_t i = 0; i < msg->ranges.size(); i++) {
                float r = msg->ranges[i];
                if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) continue;
                double deg = (msg->angle_min + i * msg->angle_increment) * 180.0 / M_PI;
                while (deg > 180) deg -= 360;
                while (deg < -180) deg += 360;
                if (deg >= lo_deg && deg <= hi_deg && r < best) best = r;
            }
            return best;
        };
        auto ema = [this](double& v, double raw) { v = v * (1.0 - alpha_) + raw * alpha_; };

        ema(front_,    sec(-20, 20));
        ema(front_l_,  sec(20, 50));
        ema(front_r_,  sec(-50, -20));
        ema(left_,     sec(50, 100));
        ema(right_,    sec(-100, -50));
        ema(left_far_, sec(80, 130));
        ema(right_far_,sec(-130, -80));
        scan_ok_ = true;
    }

    // ── Odometry callback (EKF fused velocity) ──────────────────────────────
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_vx_ = msg->twist.twist.linear.x;
        odom_wz_ = msg->twist.twist.angular.z;
    }

    // ── Main tick ────────────────────────────────────────────────────────────
    void tick() {
        if (!scan_ok_) return;
        ticks_++;

        double cmd_lin = 0.0, cmd_ang = 0.0;

        bool front_blocked = front_ < obstacle_dist_;
        bool front_close   = front_ < wall_dist_ * 1.5;
        bool fl_blocked    = front_l_ < obstacle_dist_;
        bool fr_blocked    = front_r_ < obstacle_dist_;
        bool left_wall     = left_ < wall_dist_;
        bool right_wall    = right_ < wall_dist_;

        // ── ESCAPE (bypasses EMA — instant full-power) ───────────────────────
        if (state_ == ESCAPE) {
            escape_count_--;
            if (escape_count_ <= 0) {
                set_state(WANDER);
                smooth_lin_ = 0.0;
                smooth_ang_ = 0.0;
                stall_counter_ = 0;
            } else {
                auto cmd = geometry_msgs::msg::Twist();
                cmd.linear.x = -0.4;
                cmd.angular.z = escape_dir_ * angular_speed_ * 0.8;
                cmd_pub_->publish(cmd);
                smooth_lin_ = cmd.linear.x;
                smooth_ang_ = cmd.angular.z;
                last_cmd_lin_ = cmd.linear.x;
                return;
            }
        }

        // ── Trigger escape: LiDAR says blocked on all sides ──────────────────
        if (front_blocked && (fl_blocked || left_wall) && (fr_blocked || right_wall)) {
            trigger_escape("lidar blocked");
            return;
        }

        // ── IMU stall detection ──────────────────────────────────────────────
        // If we're commanding forward motion but EKF says we're not moving
        // → something physical is blocking us that LiDAR can't see
        if (std::abs(last_cmd_lin_) > stall_cmd_thresh_ &&
            std::abs(odom_vx_) < stall_vel_thresh_ &&
            state_ != ESCAPE)
        {
            stall_counter_++;
            if (stall_counter_ >= stall_limit_) {
                RCLCPP_WARN(get_logger(),
                    "IMU STALL: cmd=%.2f but odom_vx=%.3f — invisible obstacle! Escaping.",
                    last_cmd_lin_, odom_vx_);
                trigger_escape("imu stall");
                return;
            }
        } else {
            // Decay stall counter when moving normally
            stall_counter_ = std::max(0, stall_counter_ - 1);
        }

        // ── State behaviors ──────────────────────────────────────────────────
        switch (state_) {
        case WANDER: {
            if (front_blocked) {
                double bias = (left_ - right_);
                cmd_ang = std::clamp(bias * 3.0, -angular_speed_, angular_speed_);
                if (std::abs(cmd_ang) < angular_speed_ * 0.3)
                    cmd_ang = (bias >= 0 ? 1 : -1) * angular_speed_ * 0.5;
                cmd_lin = 0.0;
            } else if (front_close) {
                double clearance = (front_ - obstacle_dist_) / (wall_dist_ * 1.5 - obstacle_dist_);
                clearance = std::clamp(clearance, 0.1, 1.0);
                cmd_lin = linear_speed_ * clearance;
                double bias = (front_l_ - front_r_);
                cmd_ang = std::clamp(bias * 2.0, -angular_speed_ * 0.4, angular_speed_ * 0.4);
            } else {
                cmd_lin = linear_speed_;
                double bias = (front_l_ - front_r_) * 0.3;
                cmd_ang = std::clamp(bias, -angular_speed_ * 0.15, angular_speed_ * 0.15);
            }

            // Wall-follow transition (2s cooldown after state change)
            if (left_wall && !right_wall && !front_blocked && ticks_ > 30) {
                set_state(FOLLOW_LEFT);
            } else if (right_wall && !left_wall && !front_blocked && ticks_ > 30) {
                set_state(FOLLOW_RIGHT);
            }
            break;
        }

        case FOLLOW_LEFT: {
            auto [l, a] = wall_follow(left_, front_l_, front_, true);
            cmd_lin = l; cmd_ang = a;

            if (left_ > wall_dist_ * 3.0 && left_far_ > wall_dist_ * 3.0) {
                RCLCPP_INFO(get_logger(), "Opening on left");
                cmd_ang = angular_speed_ * 0.35;
                cmd_lin = linear_speed_ * 0.4;
                set_state(WANDER);
            }
            if (ticks_ > 250) set_state(WANDER);
            break;
        }

        case FOLLOW_RIGHT: {
            auto [l, a] = wall_follow(right_, front_r_, front_, false);
            cmd_lin = l; cmd_ang = a;

            if (right_ > wall_dist_ * 3.0 && right_far_ > wall_dist_ * 3.0) {
                RCLCPP_INFO(get_logger(), "Opening on right");
                cmd_ang = -angular_speed_ * 0.35;
                cmd_lin = linear_speed_ * 0.4;
                set_state(WANDER);
            }
            if (ticks_ > 250) set_state(WANDER);
            break;
        }

        case ESCAPE:
            break;
        }

        publish_smooth(cmd_lin, cmd_ang);
        last_cmd_lin_ = smooth_lin_;
    }

    // ── Wall-follow PD controller ────────────────────────────────────────────
    std::pair<double, double> wall_follow(double wall_d, double front_wall_d,
                                           double front_d, bool left_side)
    {
        double cmd_lin, cmd_ang;

        if (front_d < obstacle_dist_) {
            cmd_ang = left_side ? -angular_speed_ * 0.7 : angular_speed_ * 0.7;
            cmd_lin = 0.0;
            return {cmd_lin, cmd_ang};
        }

        double error = wall_d - wall_dist_;
        double deriv = front_wall_d - wall_d;

        double kp = 4.0, kd = 2.0;
        double steer = kp * error + kd * deriv;
        if (!left_side) steer = -steer;

        cmd_ang = std::clamp(-steer, -angular_speed_ * 0.5, angular_speed_ * 0.5);

        if (front_d < wall_dist_ * 1.5) {
            double s = (front_d - obstacle_dist_) / (wall_dist_ * 1.5 - obstacle_dist_);
            cmd_lin = linear_speed_ * std::clamp(s, 0.1, 0.6);
        } else {
            cmd_lin = linear_speed_ * 0.6;
        }

        return {cmd_lin, cmd_ang};
    }

    // ── Trigger escape maneuver ──────────────────────────────────────────────
    void trigger_escape(const char* reason) {
        set_state(ESCAPE);
        escape_count_ = rng_() % 20 + 20;  // 1.3-2.7s
        escape_dir_ = (left_ > right_) ? 1 : -1;
        stall_counter_ = 0;

        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = -0.4;
        cmd.angular.z = escape_dir_ * angular_speed_ * 0.8;
        cmd_pub_->publish(cmd);
        smooth_lin_ = -0.4;
        smooth_ang_ = escape_dir_ * angular_speed_ * 0.8;
        last_cmd_lin_ = -0.4;

        RCLCPP_WARN(get_logger(), "ESCAPE triggered: %s (dir=%s)",
            reason, escape_dir_ > 0 ? "left" : "right");
    }

    // ── Smooth publish (EMA) ─────────────────────────────────────────────────
    void publish_smooth(double lin, double ang) {
        double lin_alpha = alpha_ * 0.7;
        double ang_alpha = alpha_ * 1.2;

        smooth_lin_ = smooth_lin_ * (1.0 - lin_alpha) + lin * lin_alpha;
        smooth_ang_ = smooth_ang_ * (1.0 - ang_alpha) + ang * ang_alpha;

        if (std::abs(smooth_lin_) < 0.02) smooth_lin_ = 0.0;
        if (std::abs(smooth_ang_) < 0.05) smooth_ang_ = 0.0;

        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = smooth_lin_;
        cmd.angular.z = smooth_ang_;
        cmd_pub_->publish(cmd);
    }

    void set_state(State s) {
        if (s != state_) {
            const char* n[] = {"WANDER", "FOLLOW_LEFT", "FOLLOW_RIGHT", "ESCAPE"};
            RCLCPP_INFO(get_logger(), "%s → %s (%d ticks)", n[state_], n[s], ticks_);
            state_ = s;
            ticks_ = 0;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Explorer>());
    rclcpp::shutdown();
    return 0;
}
