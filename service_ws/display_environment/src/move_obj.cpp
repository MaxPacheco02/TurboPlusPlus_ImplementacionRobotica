#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_stamped.h"
// Messages
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion>
#include <sensor_msgs/msg/joint_state>
// Dependencies
#include <cmath>


using namespace std::chrono_literals;

class RobotPublisher : public rclcpp::Node {
    public:
        RobotPublisher() : Node("robot_publisher") {
            // Publisher
            joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            // Variables
            float degree = M_PI/180;
            // robot state
            float tilt = 0.0; float tinc = degree; float swivel = 0.0;
            float angle = 0.0; float height = 0.0; float hinc = 0.005;

            RCLCPP_INFO(this->get_logger(), "Node[robot_publisher] initialized");
        }

    private:
        // Declarations of Publisher
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
        // Declarations fo Messages

};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotPublisher>());
    rclcpp::shutdown();
    return 0;
}
