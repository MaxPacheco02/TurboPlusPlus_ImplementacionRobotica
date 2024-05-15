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
#include "nav_msgs/msg/path.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"

#include "std_srvs/srv/empty.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "PID.cpp"

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

        this->declare_parameter("sides", 3); // default sides
        int sides = this->get_parameter("sides").as_int();
        this->declare_parameter("length", 1.0); // default length
        double length = this->get_parameter("length").as_double();
        this->declare_parameter("wheel_relation", 1.0); // default wheel_relation
        wheel_relation = this->get_parameter("wheel_relation").as_double();
        this->declare_parameter("KP", 1.0); // default KP
        KP = this->get_parameter("KP").as_double();
        this->declare_parameter("KI", 1.0); // default KI
        KI = this->get_parameter("KI").as_double();
        this->declare_parameter("KD", 1.0); // default KD
        KD = this->get_parameter("KD").as_double();
        this->declare_parameter("u_max", 1.0); // default u_max
        u_max = this->get_parameter("u_max").as_double();
        this->declare_parameter("u_min", 1.0); // default u_min
        u_min = this->get_parameter("u_min").as_double();

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/pzb_pose", 10,
            [this](const geometry_msgs::msg::Vector3 &msg){this->pose = msg;});

        w1_des_pub_ = this->create_publisher<std_msgs::msg::Float32>("/w1_d", 10);
        w2_des_pub_ = this->create_publisher<std_msgs::msg::Float32>("/w2_d", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        wp_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/wp_path", 10);

        start_srv = this->create_service<std_srvs::srv::Empty>("pzb_start", std::bind(&GuidanceNode::pzb_start, this, _1, _2));
        stop_srv = this->create_service<std_srvs::srv::Empty>("pzb_stop", std::bind(&GuidanceNode::pzb_stop, this, _1, _2));

        updateTimer =
            this->create_wall_timer(50ms, std::bind(&GuidanceNode::update, this));

        wp_stamped_msg.header.frame_id = "world";
        wp_path.header.frame_id = "world";

        double n{50};

        double  angle_diff{2 * M_PI / sides}, 
                dir{0.0}, 
                wp_x{0.0}, wp_y{0.0};

        for(int i = 0 ; i < sides ; i++){
            for(int j = 0 ; j < n ; j++){
                wp_x += length / n * cos(dir);
                wp_y += length / n * sin(dir);
                wp_list.push_back(Waypoint{wp_x, wp_y});

                wp_stamped_msg.pose.position.x = wp_x;
                wp_stamped_msg.pose.position.y = wp_y;
                wp_path.poses.push_back(wp_stamped_msg);
            }
            dir += angle_diff;
        }

        controller = PID(dt, KP, KI, KD, u_max, u_min);
        controller.updateReferences(0);
    }

private:

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pose_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr w1_des_pub_, w2_des_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr wp_path_pub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_srv, stop_srv;

    rclcpp::TimerBase::SharedPtr updateTimer;

    std_msgs::msg::Float32 w1_des_msg, w2_des_msg;
    nav_msgs::msg::Path wp_path;
    geometry_msgs::msg::PoseStamped wp_stamped_msg;
    geometry_msgs::msg::Vector3 pose;
    geometry_msgs::msg::Twist cmd_vel_msg;

    PID controller;

    float motors_enabled{false};

    double psi_d{0.0}, vel_d{0.0}, ang_vel_d{0.0};

    const double change_wp_dist{0.05};
    const double r{0.05}, l{0.08};
    double wheel_relation;
    double dt{0.05};
    double KP{1.0}, KI{1.0}, KD{1.0};
    double u_max{10.0}, u_min{-10.0};

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
        while(wp_i < wp_list.size() && 
            (distance(this->pose, wp_list[wp_i]) < change_wp_dist))
            wp_i++;

        // Update the desired psi
        if(wp_list.size() != 0){
            psi_d = std::atan2((wp_list[wp_i].y - this->pose.y), 
                (wp_list[wp_i].x - this->pose.x));

            vel_d = 0.15;
            controller.saturateManipulation(get_angle_diff(psi_d, this->pose.z));
            ang_vel_d = -controller.u_;

            if(std::fabs(ang_vel_d) > 0.3)
                vel_d = 0;

            if(wp_i == wp_list.size()) {
                vel_d = 0.0;
                ang_vel_d = 0.0;
            }

            // RCLCPP_INFO(get_logger(), "u: %f", ang_vel_d);
            // RCLCPP_INFO(get_logger(), "wp_i: %d, size: %d, vel: %f, ang_v: %f", wp_i, wp_list.size(), vel_d, ang_vel_d);
            // RCLCPP_INFO(get_logger(), "From %f, %f to %f, %f", this->pose.x, this->pose.y, wp_list[wp_i].x, wp_list[wp_i].y);

            if(motors_enabled){
                this->w1_des_msg.data = wheel_relation * (vel_d - ang_vel_d * l) / r;
                this->w2_des_msg.data = (vel_d + ang_vel_d * l) / r;
            } else {
                this->w1_des_msg.data = 0;
                this->w2_des_msg.data = 0;
            }

            w1_des_pub_->publish(this->w1_des_msg);
            w2_des_pub_->publish(this->w2_des_msg);
            wp_path_pub_->publish(wp_path);

            this->cmd_vel_msg.linear.x = vel_d;
            this->cmd_vel_msg.angular.z = ang_vel_d;
            cmd_vel_pub_->publish(this->cmd_vel_msg);
        }
    }

    void pzb_start(const std::shared_ptr<std_srvs::srv::Empty::Request> request, 
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        motors_enabled = true;
    }

    void pzb_stop(const std::shared_ptr<std_srvs::srv::Empty::Request> request, 
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        motors_enabled = false;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuidanceNode>());
    rclcpp::shutdown();
    return 0;
}
