#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"

#include "pzb_msgs/msg/signal.hpp"

#include "std_srvs/srv/empty.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

#include "PID.cpp"

using namespace std::chrono_literals;

struct Waypoint
{
    double x, y;
};

struct Error
{
    double x, y, z;
};

class PIDGuidance : public rclcpp::Node
{
public:
    PIDGuidance() : Node("pid_guidance")
    {
        using namespace std::placeholders;

        this->declare_parameter("KP", 1.0); // default KP
        KP = this->get_parameter("KP").as_double();
        this->declare_parameter("KI", 1.0); // default KI
        KI = this->get_parameter("KI").as_double();
        this->declare_parameter("KD", 1.0); // default KD
        KD = this->get_parameter("KD").as_double();
        this->declare_parameter("u_max", 1.0); // default u_max
        u_max = this->get_parameter("u_max").as_double();
        this->declare_parameter("u_min", -1.0); // default u_min
        u_min = this->get_parameter("u_min").as_double();

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](const nav_msgs::msg::Odometry &msg)
            { this->pose = msg.pose.pose; });

        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped &msg)
            { goal_pose = msg.pose; });

        // path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        //     "/pzb/path_to_follow", 10,
        //     [this](const nav_msgs::msg::Path &msg) {
        //         if(!same_msg(wp_list, msg)){
        //             // RCLCPP_INFO(get_logger(), "new wp!");
        //             wp_i = 0;
        //         }
        //         wp_list = get_wp_list(msg);
        //     });

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        updateTimer =
            this->create_wall_timer(10ms, std::bind(&PIDGuidance::update, this));

        pid_linear = PID(dt, KP, KI, KD, u_max, u_min);
        pid_angular = PID(dt, KP, KI, KD, u_max, u_min);
        pid_linear.updateReferences(0.0);
        pid_angular.updateReferences(0.0);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr w1_des_pub_, w2_des_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_srv, stop_srv;

    rclcpp::TimerBase::SharedPtr updateTimer;

    std_msgs::msg::Float32 w1_des_msg, w2_des_msg;
    geometry_msgs::msg::PoseStamped wp_stamped_msg;
    geometry_msgs::msg::Pose pose, goal_pose;
    geometry_msgs::msg::Twist cmd_vel_msg;

    PID pid_linear, pid_angular;

    double motors_enabled{1.};

    double psi_d{0.0}, lin_vel_d{0.0}, ang_vel_d{0.0}, psi_e{0.0};

    const double change_wp_dist{0.1};
    const double r{0.05}, l{0.08};
    double wheel_relation;
    double last_lin_vel_d{0.};
    double last_ang_vel_d{0.};
    double dt{0.01};
    double KP{1.0}, KI{1.0}, KD{1.0};
    double u_max{10.0}, u_min{-10.0};

    double min_surge_ref{-0.1}, max_surge_ref{0.1};
    double min_yaw_ref{-1.0}, max_yaw_ref{1.0};

    int wp_i{0};
    double vel_multiplier{1.};

    std::vector<Waypoint> wp_list;

    double distance(geometry_msgs::msg::Vector3 pos, Waypoint wp)
    {
        return sqrt(pow(pos.x - wp.x, 2) + pow(pos.y - wp.y, 2));
    }

    double get_angle_diff(double psi_1, double psi_2)
    {
        double angle_diff = std::fmod((psi_1 - psi_2 + M_PI), 2 * M_PI) - M_PI;
        return angle_diff < -M_PI ? angle_diff + 2 * M_PI : angle_diff;
    }

    double normalize_angle(double x)
    {
        x = std::fmod(x + M_PI, M_PI * 2);
        if (x < 0)
            x += M_PI * 2;
        return x - M_PI;
    }

    std::vector<Waypoint> get_wp_list(nav_msgs::msg::Path path)
    {
        std::vector<Waypoint> wp_l;
        for (int i = 0; i < path.poses.size(); i++)
        {
            Waypoint wp{path.poses[i].pose.position.x, path.poses[i].pose.position.y};
            wp_l.push_back(wp);
        }
        return wp_l;
    }

    Error calculatePoseError(
        const geometry_msgs::msg::Pose &reference_pose,
        const geometry_msgs::msg::Pose &current_pose)
    {
        // Get the positions
        double ref_x = reference_pose.position.x;
        double ref_y = reference_pose.position.y;
        double current_x = current_pose.position.x;
        double current_y = current_pose.position.y;

        // Calculate translation difference in global frame
        double dx = current_x - ref_x;
        double dy = current_y - ref_y;

        // Get reference orientation as yaw angle
        double ref_yaw = tf2::getYaw(reference_pose.orientation);
        double current_yaw = tf2::getYaw(current_pose.orientation);

        // Calculate angular error and normalize to [-π, π]
        double angular_error = normalize_angle(current_yaw - ref_yaw);
        if(std::fabs(angular_error) < 0.05){
            angular_error = 0;
        }

        // Transform the translation difference to the reference pose's frame
        double cos_yaw = std::cos(ref_yaw);
        double sin_yaw = std::sin(ref_yaw);

        // Rotate the global difference by -ref_yaw to get it in reference frame
        double forward_error = dx * cos_yaw + dy * sin_yaw;  // Forward (x) in reference frame
        double lateral_error = -dx * sin_yaw + dy * cos_yaw; // Lateral (y) in reference frame

        if(std::fabs(forward_error) < 0.05){
            forward_error = 0;
        }

        // if(std::fabs(lateral_error) < 0.005){
        //     lateral_error = 0;
        // }

        return Error{forward_error, lateral_error, angular_error};
    }

    void update()
    {
        Error e = calculatePoseError(pose, goal_pose);

        psi_d = std::atan2((goal_pose.position.y - pose.position.y), 
        (goal_pose.position.x - pose.position.x));

        double trans_e = std::sqrt(e.x*e.x + e.y*e.y);

        // If it's not there, go. Else, fix orientation
        if(trans_e > 0.05)
            psi_e = get_angle_diff(psi_d, tf2::getYaw(pose.orientation));
        else
            psi_e = e.z;

        pid_linear.saturateManipulation(std::clamp(e.x / (1 + 3 * std::fabs(psi_e)), min_surge_ref, max_surge_ref));
        pid_angular.saturateManipulation(psi_e);

        lin_vel_d = -pid_linear.u_;
        ang_vel_d = -pid_angular.u_;

        lin_vel_d = last_lin_vel_d + std::clamp(lin_vel_d - last_lin_vel_d, -0.01, 0.01);
        ang_vel_d = last_ang_vel_d + std::clamp(ang_vel_d - last_ang_vel_d, -0.01, 0.01);

        RCLCPP_INFO(get_logger(), "e: %f, %f, %f", e.x, e.y, e.z);
        RCLCPP_INFO(get_logger(), "lin_d: %f, u: %f", lin_vel_d, pid_linear.u_);
        RCLCPP_INFO(get_logger(), "ang_d: %f, u: %f", ang_vel_d, pid_angular.u_);
        // RCLCPP_INFO(get_logger(), "wp_i: %d, size: %d, vel: %f, ang_v: %f", wp_i, wp_list.size(), vel_d, ang_vel_d);
        // RCLCPP_INFO(get_logger(), "From %f, %f to %f, %f", this->pose.x, this->pose.y, wp_list[wp_i].x, wp_list[wp_i].y);

        this->cmd_vel_msg.linear.x = lin_vel_d;
        this->cmd_vel_msg.angular.z = ang_vel_d;
        cmd_vel_pub_->publish(this->cmd_vel_msg);

        last_lin_vel_d = lin_vel_d;
        last_ang_vel_d = ang_vel_d;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDGuidance>());
    rclcpp::shutdown();
    return 0;
}
