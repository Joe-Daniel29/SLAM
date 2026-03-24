// ═══════════════════════════════════════════════════════════════════════════════
// frontier_explorer.cpp — Frontier-based autonomous exploration
//
// Algorithm: Yamauchi (1997) "A Frontier-Based Approach for Autonomous Exploration"
// with efficient frontier clustering and reactive obstacle avoidance.
//
// Subscribes: /map (OccupancyGrid), /scan (LaserScan), /odom (Odometry)
// Publishes:  /cmd_vel (Twist), /frontiers (MarkerArray for visualization)
//
// Behavior:
//   1. Extract frontiers from occupancy grid (free cells adjacent to unknown)
//   2. Cluster nearby frontier cells into groups
//   3. Select best frontier: nearest cluster above minimum size
//   4. Navigate toward frontier centroid with reactive obstacle avoidance
//   5. When no frontiers remain → exploration complete
//
// Loop closure is handled by slam_toolbox automatically when the robot
// revisits previously mapped areas.
// ═══════════════════════════════════════════════════════════════════════════════
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include <random>
#include <limits>

struct Point2D {
    double x, y;
};

struct FrontierCluster {
    std::vector<std::pair<int, int>> cells; // grid coords
    Point2D centroid;                        // world coords
    double distance;                         // from robot
    double size;                             // number of cells
};

class FrontierExplorer : public rclcpp::Node {
public:
    FrontierExplorer() : Node("frontier_explorer"), rng_(std::random_device{}()) {
        // Parameters
        this->declare_parameter("linear_speed", 0.8);
        this->declare_parameter("angular_speed", 10.0);
        this->declare_parameter("obstacle_dist", 0.35);
        this->declare_parameter("slowdown_dist", 0.6);
        this->declare_parameter("goal_tolerance", 0.5);       // m — close enough to frontier
        this->declare_parameter("min_frontier_size", 8);       // cells — ignore tiny frontiers
        this->declare_parameter("frontier_update_hz", 1.0);    // how often to recompute frontiers
        this->declare_parameter("blacklist_radius", 0.8);      // m — blacklist unreachable goals

        linear_speed_      = this->get_parameter("linear_speed").as_double();
        angular_speed_     = this->get_parameter("angular_speed").as_double();
        obstacle_dist_     = this->get_parameter("obstacle_dist").as_double();
        slowdown_dist_     = this->get_parameter("slowdown_dist").as_double();
        goal_tolerance_    = this->get_parameter("goal_tolerance").as_double();
        min_frontier_size_ = this->get_parameter("min_frontier_size").as_int();
        blacklist_radius_  = this->get_parameter("blacklist_radius").as_double();
        double frontier_hz = this->get_parameter("frontier_update_hz").as_double();

        // Publishers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/frontiers", 10);

        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&FrontierExplorer::map_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&FrontierExplorer::scan_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&FrontierExplorer::odom_callback, this, std::placeholders::_1));

        // Timers
        frontier_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / frontier_hz)),
            std::bind(&FrontierExplorer::update_frontiers, this));

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz control loop
            std::bind(&FrontierExplorer::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "FrontierExplorer ready — waiting for /map...");
    }

private:
    // ── ROS interfaces ───────────────────────────────────────────────────────
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr frontier_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ── Parameters ───────────────────────────────────────────────────────────
    double linear_speed_;
    double angular_speed_;
    double obstacle_dist_;
    double slowdown_dist_;
    double goal_tolerance_;
    int    min_frontier_size_;
    double blacklist_radius_;

    // ── State ────────────────────────────────────────────────────────────────
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    bool map_received_ = false;
    bool scan_received_ = false;
    bool odom_received_ = false;
    bool exploration_complete_ = false;

    // Robot pose
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    // Scan sectors
    double front_dist_ = 999.0;
    double left_dist_  = 999.0;
    double right_dist_ = 999.0;

    // Current goal
    bool has_goal_ = false;
    Point2D goal_;
    int stuck_counter_ = 0;
    double last_robot_x_ = 0.0;
    double last_robot_y_ = 0.0;
    int no_progress_ticks_ = 0;

    // Blacklisted goals (unreachable frontiers)
    std::vector<Point2D> blacklist_;

    // Recently visited goals (avoid oscillation)
    std::vector<Point2D> recent_goals_;

    // Goal attempt tracking — blacklist zones where robot oscillates
    std::vector<Point2D> completed_goals_history_;  // last N completed goals
    static constexpr int OSCILLATION_WINDOW = 8;    // check last 8 completed goals
    static constexpr double OSCILLATION_ZONE = 2.0; // if all within 2m box = oscillating

    // Frontier clusters
    std::vector<FrontierCluster> clusters_;

    // Random
    std::mt19937 rng_;

    // ── Callbacks ────────────────────────────────────────────────────────────
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = msg;
        if (!map_received_) {
            RCLCPP_INFO(this->get_logger(), "Map received: %dx%d, resolution=%.2fm",
                msg->info.width, msg->info.height, msg->info.resolution);
            map_received_ = true;
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        // Extract yaw from quaternion
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        robot_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy),
                                1.0 - 2.0 * (qy * qy + qz * qz));
        odom_received_ = true;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        front_dist_ = sector_min(*msg, -30.0, 30.0);
        left_dist_  = sector_min(*msg, 30.0, 90.0);
        right_dist_ = sector_min(*msg, -90.0, -30.0);
        scan_received_ = true;
    }

    double sector_min(const sensor_msgs::msg::LaserScan& scan, double ang_min_deg, double ang_max_deg) {
        double min_r = 999.0;
        for (size_t i = 0; i < scan.ranges.size(); i++) {
            double angle_deg = (scan.angle_min + i * scan.angle_increment) * 180.0 / M_PI;
            while (angle_deg > 180.0) angle_deg -= 360.0;
            while (angle_deg < -180.0) angle_deg += 360.0;
            if (angle_deg >= ang_min_deg && angle_deg <= ang_max_deg) {
                float r = scan.ranges[i];
                if (std::isfinite(r) && r >= scan.range_min && r <= scan.range_max) {
                    min_r = std::min(min_r, static_cast<double>(r));
                }
            }
        }
        return min_r;
    }

    // ── Frontier detection ───────────────────────────────────────────────────
    void update_frontiers() {
        if (!map_received_ || !odom_received_ || exploration_complete_) return;

        auto& grid = map_->data;
        int w = map_->info.width;
        int h = map_->info.height;
        double res = map_->info.resolution;
        double ox = map_->info.origin.position.x;
        double oy = map_->info.origin.position.y;

        // Find frontier cells: free cells (0) adjacent to unknown (-1)
        std::vector<bool> is_frontier(w * h, false);

        for (int y = 1; y < h - 1; y++) {
            for (int x = 1; x < w - 1; x++) {
                int idx = y * w + x;
                if (grid[idx] != 0) continue; // must be free space

                // Check 4-connected neighbors for unknown
                static const int dx[] = {-1, 1, 0, 0};
                static const int dy[] = {0, 0, -1, 1};
                for (int d = 0; d < 4; d++) {
                    int ni = (y + dy[d]) * w + (x + dx[d]);
                    if (grid[ni] == -1) {
                        is_frontier[idx] = true;
                        break;
                    }
                }
            }
        }

        // BFS clustering of frontier cells
        std::vector<bool> visited(w * h, false);
        clusters_.clear();

        for (int y = 1; y < h - 1; y++) {
            for (int x = 1; x < w - 1; x++) {
                int idx = y * w + x;
                if (!is_frontier[idx] || visited[idx]) continue;

                // BFS to collect cluster
                FrontierCluster cluster;
                std::queue<std::pair<int, int>> q;
                q.push({x, y});
                visited[idx] = true;

                double cx_sum = 0.0, cy_sum = 0.0;

                while (!q.empty()) {
                    auto [cx, cy] = q.front();
                    q.pop();
                    cluster.cells.push_back({cx, cy});

                    // World coordinates
                    double wx = ox + (cx + 0.5) * res;
                    double wy = oy + (cy + 0.5) * res;
                    cx_sum += wx;
                    cy_sum += wy;

                    // Expand to 8-connected neighbors
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            int nx = cx + dx, ny = cy + dy;
                            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                            int ni = ny * w + nx;
                            if (is_frontier[ni] && !visited[ni]) {
                                visited[ni] = true;
                                q.push({nx, ny});
                            }
                        }
                    }
                }

                int n = cluster.cells.size();
                cluster.centroid = {cx_sum / n, cy_sum / n};
                cluster.size = n;
                cluster.distance = std::hypot(cluster.centroid.x - robot_x_,
                                              cluster.centroid.y - robot_y_);
                clusters_.push_back(std::move(cluster));
            }
        }

        // Filter: remove small clusters and blacklisted goals
        std::vector<FrontierCluster> valid;
        for (auto& c : clusters_) {
            if (static_cast<int>(c.size) < min_frontier_size_) continue;
            bool blacklisted = false;
            for (auto& bl : blacklist_) {
                if (std::hypot(c.centroid.x - bl.x, c.centroid.y - bl.y) < blacklist_radius_) {
                    blacklisted = true;
                    break;
                }
            }
            if (!blacklisted) valid.push_back(std::move(c));
        }
        clusters_ = std::move(valid);

        // Score frontiers: prefer LARGE frontiers, penalize very close ones
        // (close tiny frontiers cause oscillation — they get "reached" but not cleared)

        // If we already have a goal and are making progress, don't switch
        if (has_goal_) {
            double dist_to_current = std::hypot(goal_.x - robot_x_, goal_.y - robot_y_);
            if (dist_to_current > goal_tolerance_) {
                // Still pursuing current goal — don't re-select
                RCLCPP_INFO(this->get_logger(),
                    "Pursuing: (%.2f, %.2f) dist=%.2fm | %zu frontiers",
                    goal_.x, goal_.y, dist_to_current, clusters_.size());
                publish_frontier_markers();
                return;
            }
            // Track completed goals to detect oscillation
            completed_goals_history_.push_back(goal_);
            if (static_cast<int>(completed_goals_history_.size()) > OSCILLATION_WINDOW * 2) {
                completed_goals_history_.erase(completed_goals_history_.begin());
            }

            // Oscillation detection: if last N goals are all within a small zone, blacklist the zone center
            if (static_cast<int>(completed_goals_history_.size()) >= OSCILLATION_WINDOW) {
                size_t start = completed_goals_history_.size() - OSCILLATION_WINDOW;
                double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
                double sum_x = 0, sum_y = 0;
                for (size_t k = start; k < completed_goals_history_.size(); k++) {
                    min_x = std::min(min_x, completed_goals_history_[k].x);
                    max_x = std::max(max_x, completed_goals_history_[k].x);
                    min_y = std::min(min_y, completed_goals_history_[k].y);
                    max_y = std::max(max_y, completed_goals_history_[k].y);
                    sum_x += completed_goals_history_[k].x;
                    sum_y += completed_goals_history_[k].y;
                }
                double span = std::max(max_x - min_x, max_y - min_y);
                if (span < OSCILLATION_ZONE) {
                    // Robot is trapped in a small zone — blacklist the center
                    Point2D center{sum_x / OSCILLATION_WINDOW, sum_y / OSCILLATION_WINDOW};
                    RCLCPP_WARN(this->get_logger(),
                        "OSCILLATION detected! Last %d goals within %.1fm zone. "
                        "Blacklisting center (%.2f, %.2f)",
                        OSCILLATION_WINDOW, span, center.x, center.y);
                    blacklist_.push_back(center);
                    completed_goals_history_.clear();
                }
            }

            recent_goals_.push_back(goal_);
            if (recent_goals_.size() > 15) {
                recent_goals_.erase(recent_goals_.begin());
            }
        }

        // Need a new goal — score: sqrt(size) / distance
        // sqrt(size) prevents huge frontiers from dominating too much
        // minimum distance threshold prevents micro-oscillation
        int selected = -1;
        double best_score = -1.0;
        for (size_t i = 0; i < clusters_.size(); i++) {
            if (clusters_[i].distance <= goal_tolerance_) continue;
            // Skip if too close to a recently visited goal
            bool recently_visited = false;
            for (auto& rg : recent_goals_) {
                if (std::hypot(clusters_[i].centroid.x - rg.x,
                               clusters_[i].centroid.y - rg.y) < blacklist_radius_ * 0.6) {
                    recently_visited = true;
                    break;
                }
            }
            if (recently_visited) continue;

            // Score: prefer large clusters at moderate distance
            double dist = std::max(clusters_[i].distance, 0.5); // floor distance to prevent close-bias
            double score = std::sqrt(clusters_[i].size) / dist;
            if (score > best_score) {
                best_score = score;
                selected = static_cast<int>(i);
            }
        }

        // If all non-recent frontiers are exhausted, clear recent history and retry
        if (selected < 0 && !recent_goals_.empty() && !clusters_.empty()) {
            recent_goals_.clear();
            for (size_t i = 0; i < clusters_.size(); i++) {
                if (clusters_[i].distance <= goal_tolerance_) continue;
                double dist = std::max(clusters_[i].distance, 0.5);
                double score = std::sqrt(clusters_[i].size) / dist;
                if (score > best_score) {
                    best_score = score;
                    selected = static_cast<int>(i);
                }
            }
        }

        if (selected < 0) {
            if (clusters_.empty()) {
                RCLCPP_INFO(this->get_logger(),
                    "=== EXPLORATION COMPLETE === No more reachable frontiers!");
                exploration_complete_ = true;
                has_goal_ = false;
                auto stop = geometry_msgs::msg::Twist();
                cmd_pub_->publish(stop);
            }
            // else: all frontiers too close, wait for map to update
        } else {
            goal_ = clusters_[selected].centroid;
            has_goal_ = true;
            no_progress_ticks_ = 0;
            last_robot_x_ = robot_x_;
            last_robot_y_ = robot_y_;
            RCLCPP_INFO(this->get_logger(),
                "New goal: (%.2f, %.2f) dist=%.2fm size=%.0f | %zu frontiers",
                goal_.x, goal_.y, clusters_[selected].distance,
                clusters_[selected].size, clusters_.size());
        }

        publish_frontier_markers();
    }

    // ── Control loop — navigate to goal with obstacle avoidance ──────────────
    void control_loop() {
        if (!scan_received_ || exploration_complete_) return;

        auto cmd = geometry_msgs::msg::Twist();

        if (!has_goal_) {
            cmd_pub_->publish(cmd); // stop
            return;
        }

        // Check if we reached the goal
        double dist_to_goal = std::hypot(goal_.x - robot_x_, goal_.y - robot_y_);
        if (dist_to_goal < goal_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "Reached frontier goal");
            has_goal_ = false;
            no_progress_ticks_ = 0;
            cmd_pub_->publish(cmd);
            return;
        }

        // Stuck detection: if robot hasn't moved much in ~5 seconds, blacklist goal
        double moved = std::hypot(robot_x_ - last_robot_x_, robot_y_ - last_robot_y_);
        if (moved < 0.02) {
            no_progress_ticks_++;
        } else {
            no_progress_ticks_ = 0;
            last_robot_x_ = robot_x_;
            last_robot_y_ = robot_y_;
        }

        if (no_progress_ticks_ > 30) { // ~3s at 10Hz
            RCLCPP_WARN(this->get_logger(),
                "Stuck — blacklisting goal (%.2f, %.2f)", goal_.x, goal_.y);
            blacklist_.push_back(goal_);
            has_goal_ = false;
            no_progress_ticks_ = 0;
            cmd_pub_->publish(cmd);
            return;
        }

        // ── Reactive navigation toward goal ──────────────────────────────────
        // Angle to goal in robot frame
        double angle_to_goal = std::atan2(goal_.y - robot_y_, goal_.x - robot_x_) - robot_yaw_;
        // Normalize to [-pi, pi]
        while (angle_to_goal > M_PI) angle_to_goal -= 2.0 * M_PI;
        while (angle_to_goal < -M_PI) angle_to_goal += 2.0 * M_PI;

        bool front_blocked = front_dist_ < obstacle_dist_;
        bool front_close   = front_dist_ < slowdown_dist_;

        if (front_blocked) {
            // Obstacle ahead — turn toward goal direction but away from obstacle
            if (left_dist_ > right_dist_) {
                cmd.angular.z = angular_speed_;
            } else {
                cmd.angular.z = -angular_speed_;
            }
        }
        else if (std::abs(angle_to_goal) > 0.5) {
            // Need to turn toward goal first
            cmd.angular.z = (angle_to_goal > 0) ? angular_speed_ : -angular_speed_;
            // Slow forward motion while turning if roughly facing goal
            if (std::abs(angle_to_goal) < 1.2) {
                cmd.linear.x = linear_speed_ * 0.3;
            }
        }
        else {
            // Heading roughly toward goal — drive forward
            if (front_close) {
                double scale = (front_dist_ - obstacle_dist_) / (slowdown_dist_ - obstacle_dist_);
                scale = std::clamp(scale, 0.2, 1.0);
                cmd.linear.x = linear_speed_ * scale;
            } else {
                cmd.linear.x = linear_speed_;
            }
            // Proportional steering toward goal
            cmd.angular.z = std::clamp(angle_to_goal * 4.0, -angular_speed_, angular_speed_);
        }

        cmd_pub_->publish(cmd);
    }

    // ── Visualization ────────────────────────────────────────────────────────
    void publish_frontier_markers() {
        visualization_msgs::msg::MarkerArray markers;

        // Delete old markers
        visualization_msgs::msg::Marker del;
        del.header.frame_id = "map";
        del.header.stamp = this->now();
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(del);

        int id = 0;
        for (auto& c : clusters_) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = this->now();
            m.ns = "frontiers";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = c.centroid.x;
            m.pose.position.y = c.centroid.y;
            m.pose.position.z = 0.1;
            m.pose.orientation.w = 1.0;
            // Size proportional to cluster size
            double s = std::clamp(c.size * 0.01, 0.05, 0.3);
            m.scale.x = s;
            m.scale.y = s;
            m.scale.z = s;
            m.color.r = 0.0;
            m.color.g = 1.0;
            m.color.b = 0.0;
            m.color.a = 0.8;
            m.lifetime = rclcpp::Duration::from_seconds(2.0);
            markers.markers.push_back(m);
        }

        // Highlight current goal in red
        if (has_goal_) {
            visualization_msgs::msg::Marker gm;
            gm.header.frame_id = "map";
            gm.header.stamp = this->now();
            gm.ns = "goal";
            gm.id = id++;
            gm.type = visualization_msgs::msg::Marker::SPHERE;
            gm.action = visualization_msgs::msg::Marker::ADD;
            gm.pose.position.x = goal_.x;
            gm.pose.position.y = goal_.y;
            gm.pose.position.z = 0.2;
            gm.pose.orientation.w = 1.0;
            gm.scale.x = 0.2;
            gm.scale.y = 0.2;
            gm.scale.z = 0.2;
            gm.color.r = 1.0;
            gm.color.g = 0.0;
            gm.color.b = 0.0;
            gm.color.a = 1.0;
            gm.lifetime = rclcpp::Duration::from_seconds(2.0);
            markers.markers.push_back(gm);
        }

        marker_pub_->publish(markers);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontierExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
