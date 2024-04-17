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

#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

struct Waypoint {
  double x, y;
};

class GuidanceNode : public rclcpp::Node
{
public:
    GuidanceNode() : Node("guidance_node")
    {
        using namespace std::placeholders;

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/pzb_pose", 10,
            [this](const geometry_msgs::msg::Vector3 &msg){this->pose = msg;});

        w1_des_pub_ = this->create_publisher<std_msgs::msg::Float32>("/VelocitySetL", 10);
        w2_des_pub_ = this->create_publisher<std_msgs::msg::Float32>("/VelocitySetR", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        updateTimer =
            this->create_wall_timer(100ms, std::bind(&GuidanceNode::update, this));

        wp_list.push_back(Waypoint{1,0});
        wp_list.push_back(Waypoint{1,1});
        wp_list.push_back(Waypoint{0,1});
        wp_list.push_back(Waypoint{0,0});
    }

private:

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pose_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr w1_des_pub_, w2_des_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::TimerBase::SharedPtr updateTimer;

    std_msgs::msg::Float32 w1_des_msg, w2_des_msg;
    geometry_msgs::msg::Vector3 pose;
    geometry_msgs::msg::Twist cmd_vel_msg;

    double psi_d{0.0}, vel_d{0.0}, ang_vel_d{0.0};

    const double change_wp_dist{0.01};
    const double r{0.05}, l{0.08};

    int wp_i{0};

    std::vector<Waypoint> wp_list;

    float distance(geometry_msgs::msg::Vector3 pos, Waypoint wp){
        return sqrt(pow(pos.x - wp.x, 2) + pow(pos.y - wp.y, 2));   
    }

    float get_angle_diff(double psi_1, double psi_2){
        double angle_diff = std::fmod((psi_1 - psi_2 + M_PI), 2*M_PI) - M_PI;
        return angle_diff < -M_PI ? angle_diff + 2 * M_PI : angle_diff;        
    }

    void update() {        

        // Look for the waypoint indexes        
        if(wp_i < wp_list.size() && 
            (distance(this->pose, wp_list[wp_i]) < change_wp_dist))
            wp_i++;

        // Update the desired psi
        if(wp_list.size() != 0){
            psi_d = std::atan2((wp_list[wp_i].y - this->pose.y), 
                (wp_list[wp_i].x - this->pose.x));

            vel_d = 0.05;
            ang_vel_d = -get_angle_diff(this->pose.z, psi_d) * 0.5;

            if(std::fabs(ang_vel_d) > 0.4)
                vel_d = 0;
            
            if(wp_i == wp_list.size()) {
                vel_d = 0.0;
                ang_vel_d = 0.0;
            }

            RCLCPP_INFO(get_logger(), "wp_i: %d, size: %d, vel: %f, ang_v: %f", wp_i, wp_list.size(), vel_d, ang_vel_d);
            RCLCPP_INFO(get_logger(), "From %f, %f to %f, %f", this->pose.x, this->pose.y, wp_list[wp_i].x, wp_list[wp_i].y);
            
            this->w1_des_msg.data = 0.615 * (vel_d - ang_vel_d * l) / r;
            this->w2_des_msg.data = (vel_d + ang_vel_d * l) / r;

            this->cmd_vel_msg.linear.x = vel_d;
            this->cmd_vel_msg.angular.z = ang_vel_d;

            w1_des_pub_->publish(this->w1_des_msg);
            w2_des_pub_->publish(this->w2_des_msg);

            // cmd_vel_pub_->publish(this->cmd_vel_msg);

        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuidanceNode>());
    rclcpp::shutdown();
    return 0;
}
