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
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "pzb_msgs/msg/object_detected.hpp"
#include "pzb_msgs/msg/object_detected_vector.hpp"

using namespace std::chrono_literals;

class MasterNode : public rclcpp::Node
{
public:
    MasterNode() : Node("master_node")
    {
        using namespace std::placeholders;

        obj_vector_sub_ = this->create_subscription<pzb_msgs::msg::ObjectDetectedVector>(
            "/pzb_pose", 10,
            [this](const pzb_msgs::msg::ObjectDetectedVector &msg){
                switch(color_to_int[msg.object_detected_vector[0].color]){
                    case 0:
                        RCLCPP_ERROR(get_logger(), "STOP");
                        multiplier = 0;
                        break;
                    case 1:
                        RCLCPP_ERROR(get_logger(), "GO");
                        multiplier = 1;
                        break;
                    case 2:
                        RCLCPP_ERROR(get_logger(), "SLOW DOWN");
                        multiplier = 0.5;
                        break;
                }
        });

        w1_des_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/w1_d", 10,
            [this](const std_msgs::msg::Float32 &msg){this->w1_msg.data = msg.data * multiplier;});
        w2_des_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/w2_d", 10,
            [this](const std_msgs::msg::Float32 &msg){this->w2_msg.data = msg.data * multiplier;});

        w1_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32>("/VelocitySetL", 10);
        w2_cmd_pub_ = this->create_publisher<std_msgs::msg::Float32>("/VelocitySetR", 10);

        updateTimer = this->create_wall_timer(100ms, std::bind(&MasterNode::update, this));
    }

private:

    rclcpp::Subscription<pzb_msgs::msg::ObjectDetectedVector>::SharedPtr obj_vector_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr w1_des_sub_, w2_des_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr w1_cmd_pub_, w2_cmd_pub_;

    rclcpp::TimerBase::SharedPtr updateTimer;

    std_msgs::msg::Float32 w1_msg, w2_msg;

    double multiplier{1.0};

    std::map<std::string, int> color_to_int = {
        {"red", 0},
        {"green", 1},
        {"yellow", 2},
    };

    void update() {        
        w1_cmd_pub_->publish(this->w1_msg);
        w2_cmd_pub_->publish(this->w2_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MasterNode>());
    rclcpp::shutdown();
    return 0;
}
