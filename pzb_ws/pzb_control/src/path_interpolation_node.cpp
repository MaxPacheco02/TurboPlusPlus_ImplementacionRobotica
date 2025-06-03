#include <cmath>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

struct Wp
{
    double x, y;
};

class PathInterpolationNode : public rclcpp::Node
{
public:
    PathInterpolationNode() : Node("path_interpolation_node")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PathInterpolationNode::locate_wp, this, _1));

        in_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/pzb/a_star_path", 10, std::bind(&PathInterpolationNode::interpolate, this, _1));

        out_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/pzb/path_to_follow", 10);

        nav_ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/avoidance/goal_pose", 10);

        updateTimer =
            this->create_wall_timer(100ms, std::bind(&PathInterpolationNode::update, this));

        // Declare parameters for interpolation
        this->declare_parameter("interpolation_distance", 0.05); // meters between points
        this->declare_parameter("min_segment_length", 0.01);     // minimum segment length to interpolate

        // Declare parameters for lookahead
        this->declare_parameter("lookahead_distance", 0.7);     // lookahead distance in meters
        this->declare_parameter("min_lookahead_distance", 0.7); // minimum lookahead distance
        this->declare_parameter("max_lookahead_distance", 1.0); // maximum lookahead distance
        this->declare_parameter("goal_tolerance", 0.25);         // distance to consider goal reached
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr out_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_ref_pub_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr in_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::TimerBase::SharedPtr updateTimer;

    nav_msgs::msg::Path out_;
    geometry_msgs::msg::PoseStamped nav_ref_msg;

    void update()
    {
        out_.header.stamp = this->get_clock()->now();
        out_path_pub_->publish(out_);
    }

    void locate_wp(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Get a lookahead point from the publishing path and publish as reference
        if (out_.poses.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "No interpolated path available for lookahead");
            return;
        }

        // Get lookahead parameters
        double lookahead_dist = this->get_parameter("lookahead_distance").as_double();
        double min_lookahead = this->get_parameter("min_lookahead_distance").as_double();
        double max_lookahead = this->get_parameter("max_lookahead_distance").as_double();
        double goal_tolerance = this->get_parameter("goal_tolerance").as_double();

        // Current robot position
        double robot_x = msg->pose.pose.position.x;
        double robot_y = msg->pose.pose.position.y;

        // Find the closest point on the path
        int closest_idx = findClosestPointIndex(robot_x, robot_y);
        if (closest_idx == -1)
        {
            RCLCPP_WARN(this->get_logger(), "Could not find closest point on path");
            return;
        }

        // Check if we're close to the goal
        const auto &goal_pose = out_.poses.back();
        double dist_to_goal = calculateDistance(robot_x, robot_y,
                                                goal_pose.pose.position.x,
                                                goal_pose.pose.position.y);

        if (dist_to_goal < goal_tolerance)
        {
            // We're close to the goal, use the goal as the reference
            nav_ref_msg = goal_pose;
            nav_ref_msg.header.stamp = this->get_clock()->now();
            nav_ref_pub_->publish(nav_ref_msg);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Goal reached! Distance to goal: %.3f", dist_to_goal);
            return;
        }

        // Find lookahead point
        geometry_msgs::msg::PoseStamped lookahead_pose;
        bool found_lookahead = findLookaheadPoint(robot_x, robot_y, closest_idx,
                                                  lookahead_dist, lookahead_pose);

        if (!found_lookahead)
        {
            // If no lookahead point found, use the goal
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "No lookahead point found, using goal");
            lookahead_pose = out_.poses.back();
        }

        // Clamp lookahead distance to bounds
        double actual_dist = calculateDistance(robot_x, robot_y,
                                               lookahead_pose.pose.position.x,
                                               lookahead_pose.pose.position.y);

        if (actual_dist < min_lookahead || actual_dist > max_lookahead)
        {
            // Adjust lookahead point to be within bounds
            adjustLookaheadDistance(robot_x, robot_y, closest_idx,
                                    std::max(min_lookahead, std::min(max_lookahead, actual_dist)),
                                    lookahead_pose);
        }

        // Publish the lookahead point as navigation reference
        nav_ref_msg = lookahead_pose;
        nav_ref_msg.header.stamp = this->get_clock()->now();
        nav_ref_msg.header.frame_id = out_.header.frame_id;
        nav_ref_pub_->publish(nav_ref_msg);

        RCLCPP_DEBUG(this->get_logger(),
                     "Lookahead point: (%.3f, %.3f), distance: %.3f",
                     lookahead_pose.pose.position.x,
                     lookahead_pose.pose.position.y,
                     actual_dist);
    }

    int findClosestPointIndex(double robot_x, double robot_y)
    {
        if (out_.poses.empty())
            return -1;

        double min_dist = std::numeric_limits<double>::max();
        int closest_idx = 0;

        for (size_t i = 0; i < out_.poses.size(); ++i)
        {
            double dist = calculateDistance(robot_x, robot_y,
                                            out_.poses[i].pose.position.x,
                                            out_.poses[i].pose.position.y);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }

        return closest_idx;
    }

    bool findLookaheadPoint(double robot_x, double robot_y, int start_idx,
                            double target_dist, geometry_msgs::msg::PoseStamped &lookahead_pose)
    {
        double accumulated_dist = 0.0;

        // Start from the closest point and move forward along the path
        for (size_t i = start_idx; i < out_.poses.size() - 1; ++i)
        {
            const auto &current_pose = out_.poses[i];
            const auto &next_pose = out_.poses[i + 1];

            // Distance from current point to robot
            double dist_to_robot = calculateDistance(robot_x, robot_y,
                                                     current_pose.pose.position.x,
                                                     current_pose.pose.position.y);

            // Distance between consecutive path points
            double segment_length = calculateDistance(current_pose.pose.position.x,
                                                      current_pose.pose.position.y,
                                                      next_pose.pose.position.x,
                                                      next_pose.pose.position.y);

            // If we're at the start, add distance from robot to current point
            if (i == start_idx)
            {
                accumulated_dist = dist_to_robot;
            }

            // Check if target distance is within this segment
            if (accumulated_dist + segment_length >= target_dist)
            {
                // Interpolate within this segment
                double remaining_dist = target_dist - accumulated_dist;
                double t = remaining_dist / segment_length;

                lookahead_pose.pose.position.x = current_pose.pose.position.x +
                                                 t * (next_pose.pose.position.x - current_pose.pose.position.x);
                lookahead_pose.pose.position.y = current_pose.pose.position.y +
                                                 t * (next_pose.pose.position.y - current_pose.pose.position.y);
                lookahead_pose.pose.position.z = 0.0;

                // Interpolate orientation
                lookahead_pose.pose.orientation = next_pose.pose.orientation;

                return true;
            }

            accumulated_dist += segment_length;
        }

        // If we reach here, target distance extends beyond the path
        // Return the last point
        lookahead_pose = out_.poses.back();
        return true;
    }

    void adjustLookaheadDistance(double robot_x, double robot_y, int start_idx,
                                 double desired_dist, geometry_msgs::msg::PoseStamped &lookahead_pose)
    {
        // This is a simplified version that just uses the desired distance
        findLookaheadPoint(robot_x, robot_y, start_idx, desired_dist, lookahead_pose);
    }

    double calculateDistance(double x1, double y1, double x2, double y2)
    {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    void interpolate(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.size() < 2)
        {
            RCLCPP_WARN(this->get_logger(), "Path has less than 2 points, cannot interpolate");
            return;
        }

        // Get interpolation parameters
        double interp_dist = this->get_parameter("interpolation_distance").as_double();
        double min_segment_length = this->get_parameter("min_segment_length").as_double();

        // Clear the output path and copy header
        out_.poses.clear();
        out_.header = msg->header;

        // Convert poses to waypoints for easier processing
        std::vector<Wp> waypoints;
        for (const auto &pose : msg->poses)
        {
            Wp wp;
            wp.x = pose.pose.position.x;
            wp.y = pose.pose.position.y;
            waypoints.push_back(wp);
        }

        // Always add the first point
        geometry_msgs::msg::PoseStamped first_pose = msg->poses[0];
        out_.poses.push_back(first_pose);

        // Interpolate between consecutive waypoints
        for (size_t i = 0; i < waypoints.size() - 1; ++i)
        {
            const Wp &start = waypoints[i];
            const Wp &end = waypoints[i + 1];

            // Calculate distance between points
            double segment_length = distance(start, end);

            // Skip interpolation for very short segments
            if (segment_length < min_segment_length)
            {
                continue;
            }

            // Calculate number of interpolation points needed
            int num_points = static_cast<int>(std::ceil(segment_length / interp_dist));

            // Create interpolated points
            for (int j = 1; j < num_points; ++j)
            {
                double t = static_cast<double>(j) / num_points;
                Wp interpolated_point = linearInterpolate(start, end, t);

                // Create pose message
                geometry_msgs::msg::PoseStamped interpolated_pose;
                interpolated_pose.header = msg->header;
                interpolated_pose.pose.position.x = interpolated_point.x;
                interpolated_pose.pose.position.y = interpolated_point.y;
                interpolated_pose.pose.position.z = 0.0;

                // For orientation, you might want to calculate heading based on direction
                double heading = std::atan2(end.y - start.y, end.x - start.x);
                interpolated_pose.pose.orientation.z = std::sin(heading / 2.0);
                interpolated_pose.pose.orientation.w = std::cos(heading / 2.0);

                out_.poses.push_back(interpolated_pose);
            }

            // Add the end point (except for the last iteration to avoid duplication)
            if (i == waypoints.size() - 2)
            {
                geometry_msgs::msg::PoseStamped last_pose = msg->poses.back();
                out_.poses.push_back(last_pose);
            }
        }

        RCLCPP_INFO(this->get_logger(),
                    "Interpolated path: %zu input points -> %zu output points",
                    msg->poses.size(), out_.poses.size());
    }

    // Helper function to calculate distance between two waypoints
    double distance(const Wp &a, const Wp &b)
    {
        return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
    }

    // Linear interpolation between two waypoints
    Wp linearInterpolate(const Wp &start, const Wp &end, double t)
    {
        Wp result;
        result.x = start.x + t * (end.x - start.x);
        result.y = start.y + t * (end.y - start.y);
        return result;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathInterpolationNode>());
    rclcpp::shutdown();
    return 0;
}