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

        op_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/op_cmd", 10,
            [this](const geometry_msgs::msg::Vector3 &msg){ 
                tele_x = msg.x;
                tele_y = msg.y;
                auto_mode = (bool)(msg.z);
            });

        signal_sub_ = this->create_subscription<pzb_msgs::msg::Signal>(
            "/signal_detected", 10,
            [this](const pzb_msgs::msg::Signal &msg){ 
                real_signal = msg.signal;
                if(msg.signal!= -1)
                    signal = msg.signal; 
            });

        dotted_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/dotted", 10,
            [this](const std_msgs::msg::Int32 &msg){ 
                if(msg.data){
                    if(dotted_cooldown > 0 && stop_cooldown > 100)
                        dotted_cooldown = 0;
                    in_dotted = true;
                }
            });

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/pzb/path_to_follow", 10);

        updateTimer = this->create_wall_timer(50ms, std::bind(&LineFollowerPathPublisherNode::update, this));

        path_msg.header.frame_id = "world";
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pose_sub_, op_cmd_sub_;
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
    int signal{pzb_msgs::msg::Signal::SLOW}, real_signal{pzb_msgs::msg::Signal::SLOW};
    int dotted_cooldown{80}, stop_cooldown{200};
    float tele_x{0.}, tele_y{0.}, ahead{2.};
    bool auto_mode{false};
    bool ending{false};

    void update() {
        dotted_cooldown++;
        stop_cooldown++;
        if(dotted_cooldown > 80 ){
            in_dotted = false;
            // RCLCPP_INFO(get_logger(), "DOTTED CD %d", dotted_cooldown);
        } else {
            RCLCPP_INFO(get_logger(), "WAITING DOTTED CD %d", dotted_cooldown);
        }

        
        if(stop_cooldown > 100){
            if(auto_mode){
                // RCLCPP_INFO(get_logger(), "AUTO");
                if(!in_dotted){
                    // RCLCPP_INFO(get_logger(), "FOLLOWING LINE");
                    path_base << pose(0), pose(1), 0.;
                    rotM << std::cos(-pose(2)), - std::sin(-pose(2)), 0, 
                            std::sin(-pose(2)), std::cos(-pose(2)), 0, 
                            0, 0, 1;
                    v << ahead, lf_err, 0.;
                    if(ending)
                        v << 0., 0., 0.;
                    v_vec.clear();
                    v_vec.push_back(v);
                } else {
                    // RCLCPP_INFO(get_logger(), "IN DOTTED");
                    v_vec = v_for_signal();
                    RCLCPP_INFO(get_logger(), "v signal: %f, %f", v_vec[0](0), v_vec[0](1));
                    if(v_vec[0](0) == 0){
                        if(stop_cooldown < 200){
                            stop_cooldown = -80;
                            // stop_cooldown = 10;
                            v << ahead, 0., 0.;
                            v_vec.clear();
                            v_vec.push_back(v);
                            // RCLCPP_INFO(get_logger(), "AHEAD SENT!");
                        }
                        else{
                            // RCLCPP_INFO(get_logger(), "RESET COOLDOWN AFTER RED DETECTED!");
                            // RCLCPP_INFO(get_logger(), "count: %d", stop_cooldown);
                            stop_cooldown = 0;
                        }
                    } else {
                        if(signal == pzb_msgs::msg::Signal::CIRCLE)
                            stop_cooldown = -150;
                        else
                            stop_cooldown = -150;

                        // stop_cooldown = -90;
                    // RCLCPP_INFO(get_logger(), "NADAAAAA!!!!!!!!!");
                    }
                }
            } else {
                // RCLCPP_INFO(get_logger(), "IN TELEOP");
                ending = false;
                dotted_cooldown = 80;
                path_base << pose(0), pose(1), 0.;
                rotM << std::cos(-pose(2)), - std::sin(-pose(2)), 0, 
                        std::sin(-pose(2)), std::cos(-pose(2)), 0, 
                        0, 0, 1;
                v << tele_x, tele_y, 0.;
                v_vec.clear();
                v_vec.push_back(v);
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
        } else {
            RCLCPP_INFO(get_logger(), "WAITING STOP CD %d", stop_cooldown);
        }

        path_pub_->publish(path_msg);
        // RCLCPP_INFO(get_logger(), "in dotted: %d, signal: %d", in_dotted, signal);
    }

    std::vector<Eigen::Vector3f> v_for_signal(){
        std::vector<Eigen::Vector3f> vs;
        Eigen::Vector3f v_tmp;
        switch(real_signal){
            case pzb_msgs::msg::Signal::STOP:
                RCLCPP_INFO(get_logger(), "RED DETECTED");
                v_tmp << 0., 0., 0.;
                vs.push_back(v_tmp);
                break;
            case pzb_msgs::msg::Signal::LEFT:
                v_tmp << 0.2, 0., 0.;
                vs.push_back(v_tmp);
                v_tmp << 0.3, 0.1, 0.;
                vs.push_back(v_tmp);
                v_tmp << 0.35, 0.3, 0.;
                vs.push_back(v_tmp);
                break;
            case pzb_msgs::msg::Signal::CIRCLE:
                v_tmp << 0.2, 0., 0.;
                vs.push_back(v_tmp);
                v_tmp << 0.3, -0.05, 0.;
                vs.push_back(v_tmp);
                v_tmp << 0.35, -0.25, 0.;
                vs.push_back(v_tmp);
                break;
            default:
                RCLCPP_INFO(get_logger(), "FOLLOWING LINE, no signal: %d", signal);
                v_tmp << ahead, lf_err, 0.;
                vs.push_back(v_tmp);
                ending = true;
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
