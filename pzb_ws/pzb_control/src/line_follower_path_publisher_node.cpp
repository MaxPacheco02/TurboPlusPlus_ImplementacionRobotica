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

#include "pzb_msgs/msg/signal.hpp"

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
            });

        lf_err_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/error", 10,
            [this](const std_msgs::msg::Int32 &msg){ lf_err = std::clamp(-msg.data / 1000.0, -0.5, 0.5); });

        signal_sub_ = this->create_subscription<pzb_msgs::msg::Signal>(
            "/signal_detected", 10,
            [this](const pzb_msgs::msg::Signal &msg){ signal = msg.signal;});

        dotted_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/dotted", 10,
            [this](const std_msgs::msg::Int32 &msg){ 
                if(msg.data)
                    in_dotted = true;
            });

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/pzb/path_to_follow", 10);

        updateTimer = this->create_wall_timer(50ms, std::bind(&LineFollowerPathPublisherNode::update, this));

        path_msg.header.frame_id = "world";
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lf_err_sub_, dotted_sub_;
    rclcpp::Subscription<pzb_msgs::msg::Signal>::SharedPtr signal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::TimerBase::SharedPtr updateTimer;

    nav_msgs::msg::Path path_msg;

    Eigen::Matrix3f rotM;
    Eigen::Vector3f pose, v, r, path_base;

    std::vector<Eigen::Vector3f> v_vec;

    float lf_err{0.};
    int in_dotted{0};
    int signal{pzb_msgs::msg::Signal::CIRCLE};
    int cooldown{0};
    float ahead{0.85};

    void update() {
        cooldown++;

        if(!in_dotted){
            path_base << pose(0), pose(1), 0.;
            rotM << std::cos(-pose(2)), - std::sin(-pose(2)), 0, 
                    std::sin(-pose(2)), std::cos(-pose(2)), 0, 
                    0, 0, 1;
            v << ahead, lf_err, 0.;
            v_vec.clear();
            v_vec.push_back(v);
        }
        else{
            v_vec = v_for_signal();
        }

        path_msg.poses.clear();
        geometry_msgs::msg::PoseStamped pose_stamped_msg;

        pose_stamped_msg.pose.position = 
            geometry_msgs::build<geometry_msgs::msg::Point>()
            .x(path_base(0)).y(path_base(1)).z(0.);
        path_msg.poses.push_back(pose_stamped_msg);

        for(int i = 0 ; i < v_vec.size(); i++){
            r = rotM * v_vec[i] + path_base;

            pose_stamped_msg.pose.position = 
                geometry_msgs::build<geometry_msgs::msg::Point>()
                .x(r(0)).y(r(1)).z(0.);
            path_msg.poses.push_back(pose_stamped_msg);

        }

        path_pub_->publish(path_msg);
        // RCLCPP_INFO(get_logger(), "in dotted: %d, signal: %d", in_dotted, signal);
    }

    std::vector<Eigen::Vector3f> v_for_signal(){
        std::vector<Eigen::Vector3f> vs;
        Eigen::Vector3f v_tmp;
        switch(signal){
            case pzb_msgs::msg::Signal::STOP:
                v_tmp << 0., 0., 0.;
                vs.push_back(v_tmp);
                break;
            case pzb_msgs::msg::Signal::LEFT:
                v_tmp << 0.2, 0., 0.;
                vs.push_back(v_tmp);
                v_tmp << 0.4, 0.15, 0.;
                vs.push_back(v_tmp);
                v_tmp << 0.4, 0.4, 0.;
                vs.push_back(v_tmp);
                break;
            case pzb_msgs::msg::Signal::CIRCLE:
                v_tmp << 0.2, 0., 0.;
                vs.push_back(v_tmp);
                v_tmp << 0.4, -0.1, 0.;
                vs.push_back(v_tmp);
                v_tmp << 0.4, -0.4, 0.;
                vs.push_back(v_tmp);
                break;
            case pzb_msgs::msg::Signal::SLOW:
                v_tmp << 0.2, lf_err, 0.;
                vs.push_back(v_tmp);
                break;
            default:
                v_tmp << ahead, lf_err, 0.;
                vs.push_back(v_tmp);
                break;
        }
        return vs;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollowerPathPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
