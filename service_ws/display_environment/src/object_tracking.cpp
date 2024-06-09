
// ROS2
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// Messages
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

// Package obj node
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

// table: 6 rot + 3 pos + 1 flag

Eigen::VectorXd rotMat_to_angles(Eigen::VectorXd rotMat){
    float sy = sqrt( (rotMat(0)*rotMat(0)) + (rotMat(1)*rotMat(1)) );
    bool singular = sy < 1e-6;
    float thetaX, thetaY, thetaZ;

    if(!singular){
        thetaX = atan2(rotMat(5),rotMat(8));
        thetaY = atan2(-rotMat(2),sy);
        thetaZ = atan2(rotMat(1), rotMat(0));
    }else{
        thetaX = atan2(rotMat(3),rotMat(0));
        thetaY = atan2(-rotMat(2),sy);
        thetaZ = 0;
    }

    Eigen::VectorXd angles = Eigen::VectorXd::Zero(3);
    angles(0) = thetaX; angles(1) = thetaY; angles(2) = thetaZ;

    return angles;
}


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
        
        // Subscribers
        obj_pos_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>
            ("obj_pos", 10, std::bind(&ObjectTracking::append_object_callback,this,_1));
        
        // Instance Publishers
        object_marker_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/my_object_array", 10);
        
        // Instance Timers
        pose_timer = this->create_wall_timer(50ms, std::bind(&ObjectTracking::pose_callback, this));
        
        // Finish initializing
        RCLCPP_INFO(this->get_logger(), "Node[object_tracking] Initialized =)");
    }

  private:
    // Declaring timers
    rclcpp::TimerBase::SharedPtr pose_timer;  
    // Declaring publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_marker_array_publisher;
    // Declaring subscribers
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obj_pos_sub;
    // Parameters
    rclcpp::Parameter scale_param;


    // Declaring messages
    visualization_msgs::msg::MarkerArray object_marker_array_msg;


    // pose_callback()
    int resolution = 100;
    int radius = 50;
    int i=0;
    void pose_callback() { 

    }

    // append_object_callback()
    // Object position
    Eigen::VectorXd objPos = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd rotMat = Eigen::VectorXd::Zero(9);
    Eigen::VectorXd angles = Eigen::VectorXd::Zero(3);
    tf2::Quaternion quat;
    float scale;
    int id = 0;
    void append_object_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){

        // Saving pose in vector
        for(int i=0; i<int(msg->data.size()-1); i++) {
            if(i>=9 && i<=11){
                objPos(i-9) = msg->data[i]*scale;
            }
            if(i<9){
                rotMat(i) = msg->data[i];
            }
        }

        RCLCPP_INFO(this->get_logger(), "received");

        // If flag 1 or 2 -> object
        if(msg->data[msg->data.size()-1] > 0){
            // Creating marker
            visualization_msgs::msg::Marker marker;
            double chair_color[4][4]{
                {1,0,0,1},
                    {0,1,0,1},
                    {1,1,0,1},
                    {0.1,0.1,0.1,1},
            };
            double table_color[4][4]{
                {0,1,0,1},
                    {0,1,0,1},
                    {1,1,0,1},
                    {0.1,0.1,0.1,1},
            };

            // Getting Angles
            angles = rotMat_to_angles(rotMat);
            // Setting orientation
            quat.setRPY(angles(0), angles(1), angles(2));
            tf2::convert(quat, marker.pose.orientation);

            // Setting marker params
            marker.header.frame_id = "world";
            marker.action = 0;
            marker.id = id;
            id += 1;
            marker.type = marker.CUBE;
            marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(1.0).y(1.0).z(1.0); 
            marker.pose.position.x = objPos(0);
            marker.pose.position.y = objPos(1);
            marker.pose.position.z = objPos(2);

            if (msg->data[msg->data.size()-1] == 1) {
                marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(table_color[0][0]).g(table_color[0][1]).b(table_color[0][2]).a(table_color[0][3]);
            }
            if (msg->data[msg->data.size()-1] == 2) {
                marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(chair_color[0][0]).g(chair_color[0][1]).b(chair_color[0][2]).a(chair_color[0][3]);
            }

            object_marker_array_msg.markers.push_back(marker);
            object_marker_array_publisher->publish(object_marker_array_msg);
        }

    }

    // 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectTracking>());
  rclcpp::shutdown();
  return 0;
}
