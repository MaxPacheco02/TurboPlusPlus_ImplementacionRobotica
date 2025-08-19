#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class MasterNode : public rclcpp::Node
{
public:
    MasterNode() : Node("master_node")
    {
        using namespace std::placeholders;

        pid_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_pid", 10,
            [this](const geometry_msgs::msg::Twist &msg)
            { cmd_vels[1] = msg; });

        teleop_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_teleop", 10,
            [this](const geometry_msgs::msg::Twist &msg)
            { cmd_vels[0] = msg; });

        cmd_vel_sel_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/cmd_vel_sel", 10,
            [this](const std_msgs::msg::Int8 &msg)
            { cmd_vel_selector = msg.data % cmd_vels.size(); });

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        updateTimer = this->create_wall_timer(50ms, std::bind(&MasterNode::update, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pid_cmd_vel_sub_, teleop_cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr cmd_vel_sel_sub_;
    
    rclcpp::TimerBase::SharedPtr updateTimer;

    std::vector<geometry_msgs::msg::Twist> cmd_vels{2};
    int cmd_vel_selector{0};

    geometry_msgs::msg::Twist cmd_vel_msg;

    void update()
    {
        cmd_vel_msg = cmd_vels[cmd_vel_selector];
        cmd_vel_pub_->publish(cmd_vel_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MasterNode>());
    rclcpp::shutdown();
    return 0;
}
