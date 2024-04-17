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
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"


using namespace std::chrono_literals;

class StateEstimateNode : public rclcpp::Node
{
public:
    StateEstimateNode() : Node("state_estimate")
    {
        using namespace std::placeholders;

        l_enc_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/VelocityEncL", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            [this](const std_msgs::msg::Float32 &msg){this->w1 = msg.data;});
        r_enc_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/VelocityEncR", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            [this](const std_msgs::msg::Float32 &msg){this->w2 = msg.data * 0.62;});

        pose_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/pzb_pose", 10);
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/pzb_vel", 10);
        pose_rviz_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_rviz", 10);
        pose_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/pose_path", 10);

        updateTimer =
            this->create_wall_timer(100ms, std::bind(&StateEstimateNode::update, this));

        xi << 0, 0, 0;
        xi_dot << 0, 0, 0;

        pose_stamped_msg.header.frame_id = "world";
        pose_path.header.frame_id = "world";
    }

private:

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr l_enc_sub_, r_enc_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pose_pub_, vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_rviz_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pose_path_pub_;


    rclcpp::TimerBase::SharedPtr updateTimer;

    geometry_msgs::msg::Vector3 pose_msg, vel_msg;
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    nav_msgs::msg::Path pose_path;

    tf2::Quaternion quat;

    float w1{0.0}, w2{0.0};

    const float r{0.05}, l{0.08}, dt{0.1};

    Eigen::Vector3f xi, xi_dot;

    void update() {       
        
        // Obtain velocity vector
        xi_dot << 
                (r/2) * (this->w1 + this->w2) * cos(xi(2)),
                (r/2) * (this->w1 + this->w2) * sin(xi(2)),
                (r/(2*l)) * (this->w2 - this->w1);

        xi_dot(2) /= 0.75;

        // Integrate
        xi << xi + xi_dot * dt;

        // Make theta [-pi, pi]
        xi(2) = std::fmod(xi(2) + M_PI, 2*M_PI) - M_PI;

        // Set and publish ROS msgs
        pose_msg.x = xi(0);
        pose_msg.y = xi(1);
        pose_msg.z = xi(2);

        vel_msg.x = xi_dot(0);
        vel_msg.y = xi_dot(1);
        vel_msg.z = xi_dot(2);

        pose_stamped_msg.pose.position.x = xi(0);
        pose_stamped_msg.pose.position.y = xi(1);
        pose_stamped_msg.pose.position.z = 0.0;
        quat.setRPY(0, 0, xi(2));
        tf2::convert(quat, pose_stamped_msg.pose.orientation);
        pose_path.poses.push_back(pose_stamped_msg);

        pose_pub_->publish(pose_msg);
        pose_rviz_pub_->publish(pose_stamped_msg);
        vel_pub_->publish(vel_msg);
        pose_path_pub_->publish(pose_path);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimateNode>());
    rclcpp::shutdown();
    return 0;
}
