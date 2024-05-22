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

// Package obj node
#include "std_msgs/msg/float32_multi_array.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace std::chrono_literals;


class ObjectTracking : public rclcpp::Node
{
  public:
    ObjectTracking() : Node("object_tracking") {
        
        // Declare parameters
        this->declare_parameter("scale", rclcpp::PARAMETER_DOUBLE);
        // Get parameters
        scale_param = this->get_parameter("scale");

        // Variables
        scale = scale_param.as_double();

        objPos = Eigen::VectorXd::Zero(6);
        refPos = Eigen::VectorXd::Zero(12);
        
        // Subscribers
        ref_pos_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "ref_pos", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg){
                for(int i=0; i<int(msg->data.size()); i++){
                    refPos(i) = msg->data[i]*scale;
                }
            }
        );
        obj_pos_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "obj_pos", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg){
                for(int i=0; i<int(msg->data.size()); i++){
                    objPos(i) = msg->data[i]*scale;
                }
            }
        );

        
        // Instance Publishers
        pose_rviz_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/my_pose_rviz", 10); 
        ref_rviz_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/my_ref_rviz", 10); 
        object_marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("/my_object", 10);

        
        // Instance Timers
        pose_timer = this->create_wall_timer(50ms, std::bind(&ObjectTracking::pose_callback, this));
        
        // Instance msg
        pose_stamped_msg.header.frame_id = "world";
        ref_stamped_msg.header.frame_id = "world";
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
        //object_marker_msg.type = object_marker_msg.CUBE;
        object_marker_msg.type = object_marker_msg.SPHERE;
        object_marker_msg.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.2).y(0.2).z(0.2); 
        object_marker_msg.pose.position.x = 0;
        object_marker_msg.pose.position.y = 0;
        object_marker_msg.pose.position.z = 0;
        object_marker_msg.color = std_msgs::build<std_msgs::msg::ColorRGBA>().
                        r(color_list[0][0]).
                        g(color_list[0][1]).
                        b(color_list[0][2]).
                        a(color_list[0][3]);

        // Finish initializing
        RCLCPP_INFO(this->get_logger(), "Node[object_tracking] Initialized =)");
    }

  private:
    // Declaring timers
    rclcpp::TimerBase::SharedPtr pose_timer;  
    // Declaring publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_rviz_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_rviz_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr object_marker_publisher;
    // Declaring subscribers
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ref_pos_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obj_pos_sub;
    // Parameters
    rclcpp::Parameter scale_param;


    // Declaring messages
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    geometry_msgs::msg::PoseStamped ref_stamped_msg;
    visualization_msgs::msg::Marker object_marker_msg;
    // Object position
    Eigen::VectorXd objPos;
    // Reference frame
    Eigen::VectorXd refPos;


    // For pose_callback()
    int resolution = 100;
    int radius = 50;
    int i=0;
    tf2::Quaternion quat;
    tf2::Quaternion quatOrigin;
    tf2::Quaternion quatObject;

    float scale;

    void pose_callback() { 
        float sy = sqrt( (refPos(0)*refPos(0)) + (refPos(1)*refPos(1)) );
        bool singular = sy < 1e-6;
        float thetaX, thetaY, thetaZ;

        if(!singular){
            thetaX = atan2(refPos(5),refPos(8));
            thetaY = atan2(-refPos(2),sy);
            thetaZ = atan2(refPos(1), refPos(0));
        }else{
            thetaX = atan2(refPos(3),refPos(0));
            thetaY = atan2(-refPos(2),sy);
            thetaZ = 0;
        }

        //float thetaX = M_PI/4;
        //float thetaY = M_PI/4;
        //float thetaZ = M_PI/4;

        // Reference of Camera poseStamped
        quat.setRPY(thetaX, thetaY, thetaZ);
        tf2::convert(quat, pose_stamped_msg.pose.orientation);
        pose_stamped_msg.pose.position.x = refPos(9);
        pose_stamped_msg.pose.position.y = refPos(10);
        pose_stamped_msg.pose.position.z = refPos(11);


        // Object tracking Marker
        quatObject.setRPY(0, 0, 0);
        tf2::convert(quatObject, object_marker_msg.pose.orientation);
        object_marker_msg.pose.position.y = -1*((objPos(0) + objPos(3)) /2); // Changing y to x
        object_marker_msg.pose.position.x = -1*((objPos(1) + objPos(4)) /2);
        object_marker_msg.pose.position.z = ((objPos(2)+objPos(5))/2);

        //object_marker_msg.pose.position.x = objPos(0);
        //object_marker_msg.pose.position.y = objPos(1);
        //object_marker_msg.pose.position.z = objPos(2);

        // Origin
        quatOrigin.setRPY(0,0,0);
        tf2::convert(quatOrigin, ref_stamped_msg.pose.orientation);
        ref_stamped_msg.pose.position.x = 0;
        ref_stamped_msg.pose.position.y = 0;
        ref_stamped_msg.pose.position.z = 0;

        // RCLCPP_INFO(this->get_logger(), "Ref: %f %f %f", refPos(9), refPos(10),refPos(11));

        pose_rviz_publisher->publish(pose_stamped_msg);
        object_marker_publisher->publish(object_marker_msg);
        ref_rviz_publisher->publish(ref_stamped_msg);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectTracking>());
  rclcpp::shutdown();
  return 0;
}
