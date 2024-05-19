#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"

#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;


class ObjectTracking : public rclcpp::Node
{
  public:
    ObjectTracking() : Node("object_tracking") {
        
        // Instance Publishers
        pose_rviz_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/my_pose_rviz", 10); 
        object_marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("/my_object", 10);
        
        // Instance Timers
        pose_timer = this->create_wall_timer(50ms, std::bind(&ObjectTracking::pose_callback, this));
        
        // Instance msg
        pose_stamped_msg.header.frame_id = "world";
        // marker
        double color_list[4][4]{
        {1,0,0,1},
        {0,1,0,1},
        {1,1,0,1},
        {0.1,0.1,0.1,1},
        };
        object_marker_msg.header.frame_id = "world";
        object_marker_msg.action = 0;
        object_marker_msg.id = 0;
        object_marker_msg.type = object_marker_msg.CUBE;
        object_marker_msg.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.2).y(0.5).z(0.5); 
        object_marker_msg.pose.position.x = -1;
        object_marker_msg.pose.position.y = 0;
        object_marker_msg.pose.position.z = 0;
        object_marker_msg.color = std_msgs::build<std_msgs::msg::ColorRGBA>().
                        r(color_list[0][0]).
                        g(color_list[0][1]).
                        b(color_list[0][2]).
                        a(color_list[0][3]);

        RCLCPP_INFO(this->get_logger(), "Node[object_tracking] Initialized =)");
    }

  private:
    // Declaring timers
    rclcpp::TimerBase::SharedPtr pose_timer;  
    // Declaring publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_rviz_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr object_marker_publisher;
    // Declaring messages
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    visualization_msgs::msg::Marker object_marker_msg;

    int resolution = 100;
    int radius = 50;
    int i=0;
    tf2::Quaternion quat;
    void pose_callback() { 
        // position poseStamped
        pose_stamped_msg.pose.position.x = cos(2*M_PI/resolution * i);
        pose_stamped_msg.pose.position.y = sin(2*M_PI/resolution * i);
        pose_stamped_msg.pose.position.z = 0.0;
        // orientation poseStamped
        quat.setRPY(0, 0, 0);
        tf2::convert(quat, pose_stamped_msg.pose.orientation);

        // position Marker
        object_marker_msg.pose.position.x = cos(2*M_PI/resolution * i);
        object_marker_msg.pose.position.y = sin(2*M_PI/resolution * i);
        object_marker_msg.pose.position.z = 0.0;

        if(i<100){
            i++;
        }else{
            i=0;
        }

        pose_rviz_publisher->publish(pose_stamped_msg);
        object_marker_publisher->publish(object_marker_msg);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectTracking>());
  rclcpp::shutdown();
  return 0;
}
