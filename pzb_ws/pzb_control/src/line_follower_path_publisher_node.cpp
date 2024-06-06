#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class LineFollowerPathPublisherNode : public rclcpp::Node
{
public:
    LineFollowerPathPublisherNode() : Node("line_follower_path_publisher_node")
    {
        using namespace std::placeholders;

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/pzb_pose", 10,
            [this](const geometry_msgs::msg::Vector3 &msg){ 
                pose << msg.x, msg.y, -msg.z; 
                rotM << std::cos(-pose(2)), - std::sin(-pose(2)), 0, 
                        std::sin(-pose(2)), std::cos(-pose(2)), 0, 
                        0, 0, 1;
            });

        lf_err_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/error", 10,
            [this](const std_msgs::msg::Int32 &msg){ lf_err = std::clamp(-msg.data / 1000.0, -0.5, 0.5); });

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/pzb/path_to_follow", 10);

        updateTimer = this->create_wall_timer(50ms, std::bind(&LineFollowerPathPublisherNode::update, this));

        path_msg.header.frame_id = "world";
        for(int i = 0 ; i < 2 ; i++){
            geometry_msgs::msg::PoseStamped pose_stamped_msg;
            path_msg.poses.push_back(pose_stamped_msg);
        }
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lf_err_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::TimerBase::SharedPtr updateTimer;

    nav_msgs::msg::Path path_msg;

    Eigen::Matrix3f rotM;
    Eigen::Vector3f pose, v, r;

    float lf_err{0.};

    void update() {

        v << 0.7, lf_err, 0;
        r = rotM * v + pose;

        path_msg.poses[0].pose.position = 
            geometry_msgs::build<geometry_msgs::msg::Point>().x(pose(0)).y(pose(1)).z(0.);
        path_msg.poses[1].pose.position = 
            geometry_msgs::build<geometry_msgs::msg::Point>().x(r(0)).y(r(1)).z(0.);

        path_pub_->publish(path_msg);
        
        // RCLCPP_INFO(get_logger(), "u: %f", ang_vel_d);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollowerPathPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
