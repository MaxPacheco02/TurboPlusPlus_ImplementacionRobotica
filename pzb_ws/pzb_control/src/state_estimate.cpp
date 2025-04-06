#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <iomanip>
#include <queue>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


using namespace std::chrono_literals;

class StateEstimateNode : public rclcpp::Node
{
public:
    StateEstimateNode() : Node("state_estimate")
    {
        using namespace std::placeholders;

        this->declare_parameter("wheel_relation2", 0.0); // default wheel_relation2
        wheel_relation2 = this->get_parameter("wheel_relation2").as_double();
        this->declare_parameter("angle_ponder", 0.0); // default angle_ponder
        angle_ponder = this->get_parameter("angle_ponder").as_double();

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            [this](const geometry_msgs::msg::Twist &msg){
                update_xidot_cmdvel(msg.linear.x, msg.angular.z);
            });

        l_enc_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/VelocityEncL", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            [this](const std_msgs::msg::Float32 &msg){
                if(q1.size() <= q_length)
                    q1.push(msg.data / wheel_relation2);
                if(q1.size() > q_length)
                    q1.pop();
                w1 = average(q1);
                w1_msg.data = w1;
                update_xidot_ws(w1, w2);
            });

        r_enc_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/VelocityEncR", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            [this](const std_msgs::msg::Float32 &msg){
                if(q2.size() <= q_length)
                    q2.push(msg.data);
                if(q2.size() > q_length)
                    q2.pop();
                w2 = average(q2);
                w2_msg.data = w2;
                update_xidot_ws(w1, w2);
            });

        w1_smooth_pub_ = this->create_publisher<std_msgs::msg::Float32>("/w1_smooth", 10);
        w2_smooth_pub_ = this->create_publisher<std_msgs::msg::Float32>("/w2_smooth", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/pzb_pose", 10);
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/pzb_vel", 10);
        odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/pzb/odom", 10);

        updateTimer =
            this->create_wall_timer(10ms, std::bind(&StateEstimateNode::update, this));

        xi << 0, 0, 0;
        xi_dot << 0, 0, 0;

        pose_stamped_msg.header.frame_id = "odom";
        odometry_msg.header.frame_id = "odom";
    }

private:

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr l_enc_sub_, r_enc_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pose_pub_, vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr w1_smooth_pub_, w2_smooth_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

    rclcpp::TimerBase::SharedPtr updateTimer;

    geometry_msgs::msg::Vector3 pose_msg, vel_msg;
    geometry_msgs::msg::PoseStamped pose_stamped_msg;

    std_msgs::msg::Float32 w1_msg, w2_msg;

    nav_msgs::msg::Odometry odometry_msg;

    tf2::Quaternion quat;

    double w1{0.0}, w2{0.0};

    const double r{0.05}, l{0.08}, dt{0.01};

    Eigen::Vector3f xi, xi_dot;

    double wheel_relation2{0.0};
    double angle_ponder{0.0};

    std::queue<double> q1, q2;
    int q_length{10};

    void update() {       
        
        xi_dot(2) /= angle_ponder;

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
        
        quat.setRPY(0, 0, xi(2));
        odometry_msg.pose.pose.position.x = xi(0);
        odometry_msg.pose.pose.position.y = xi(1);
        tf2::convert(quat, odometry_msg.pose.pose.orientation);

        pose_pub_->publish(pose_msg);
        vel_pub_->publish(vel_msg);
        odometry_pub_->publish(odometry_msg);
        w1_smooth_pub_->publish(w1_msg);
        w2_smooth_pub_->publish(w2_msg);
    }

    double average(std::queue<double> q){
        int siz = q.size();
        double sum{0};
        while(q.size() > 0){
            sum += q.front();
            q.pop();
        }
        if(siz > 0)
            return sum / siz;
        else
            return 0;
    }

    // Obtain velocity vector with Twist msg
    void update_xidot_cmdvel(double lin_x, double ang_z){
        xi_dot << 
            lin_x * cos(xi(2)),
            lin_x * sin(xi(2)),
            ang_z;
    }

    // Obtain velocity vector with wheels' ang. vel
    void update_xidot_ws(double w_l, double w_r){
        xi_dot << 
            (r/2) * (w1 + w2) * cos(xi(2)),
            (r/2) * (w1 + w2) * sin(xi(2)),
            (r/(2*l)) * (w2 - w1);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimateNode>());
    rclcpp::shutdown();
    return 0;
}
