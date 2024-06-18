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
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Package obj node
#include "std_msgs/msg/float32_multi_array.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "std_msgs/msg/int32.hpp"

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

        objPos = Eigen::VectorXd::Zero(13);
        refPos = Eigen::VectorXd::Zero(12);

        // Max Distance
        distanceMax = 290 * scale;

        
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
                    if(i < 12){
                        objPos(i) = msg->data[i]*scale;
                    }else{
                        objPos(i) = msg->data[i];
                    }
                    if (msg->data[12] > 0){
                        curr = (int)msg->data[12];
                    }
                    if ( msg->data[12] == 0){
                        curr = (int)msg->data[12];
                        prev = curr;
                    }
                }
            }
        );

        // Instance Publishers
        pose_rviz_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/my_pose_rviz", 10); 
        ref_rviz_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/my_ref_rviz", 10); 
        object_rviz_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/my_object", 10);
        object_marker_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/my_object_array", 10);
        close_publisher = this->create_publisher<std_msgs::msg::Int32>("/alarm",10);

        // Instance Timers
        pose_timer = this->create_wall_timer(50ms, std::bind(&ObjectTracking::pose_callback, this));
        dist_timer = this->create_wall_timer(50ms, std::bind(&ObjectTracking::distance_callback, this));
       
        // Instance msg
        pose_stamped_msg.header.frame_id = "world";
        ref_stamped_msg.header.frame_id = "world";
        object_stamped_msg.header.frame_id = "world";

        // Finish initializing
        RCLCPP_INFO(this->get_logger(), "Node[object_tracking] Initialized =)");
    }


  private:

    // Declaring timers
    rclcpp::TimerBase::SharedPtr pose_timer;  
    rclcpp::TimerBase::SharedPtr dist_timer;  
    // Declaring publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_rviz_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_rviz_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_rviz_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_marker_array_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr close_publisher;

    // Declaring subscribers
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ref_pos_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obj_pos_sub;
    // Parameters
    rclcpp::Parameter scale_param;

    // Declaring messages
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    geometry_msgs::msg::PoseStamped ref_stamped_msg;
    geometry_msgs::msg::PoseStamped object_stamped_msg;
    visualization_msgs::msg::MarkerArray object_marker_array_msg;

    // Object position
    Eigen::VectorXd objPos;
    // Reference frame
    Eigen::VectorXd refPos;
    // Angles
    Eigen::VectorXd angles;

    // For pose_callback()
    int resolution = 100;
    int radius = 50;
    int id = 0;
    int prev = 0;
    int curr = 0;
    tf2::Quaternion quat;
    tf2::Quaternion quatOrigin;
    tf2::Quaternion quatObject;
    tf2::Quaternion quatMue;

    // For distance_callback()
    float scale; // For coordinate received to graph in Rviz
    float distanceMax;
    float distance;
    float xO=0, yO=0, xP=0, yP=0;


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

    void distance_callback(){
        int counter = 0;
        std_msgs::msg::Int32 n;
        for(int i=0; i<(int)object_marker_array_msg.markers.size();i++){
            xO = object_marker_array_msg.markers[i].pose.position.x;
            yO = object_marker_array_msg.markers[i].pose.position.z;
            xP = object_stamped_msg.pose.position.x;
            yP = object_stamped_msg.pose.position.z;
            distance = sqrt( (xO-xP)*(xO-xP) +  (yO-yP)*(yO-yP));
            RCLCPP_INFO(this->get_logger(), "Distance: %f, Distance Max: %f", distance, distanceMax);
            if(distance < distanceMax) {
                RCLCPP_ERROR(this->get_logger(), "Object %d near", i+1);
                counter++;
            }
        }
        if(counter>0){
            n.data = 1;
            close_publisher->publish(n);
        }else if(counter == 0){
            n.data = 0;
            close_publisher->publish(n);

        }
    }

    void pose_callback() {

        // Getting Angles
        angles = rotMat_to_angles(refPos);
        // Reference of Camera poseStamped
        quat.setRPY(angles[0], angles[1], angles[2]);
        tf2::convert(quat, pose_stamped_msg.pose.orientation);
        pose_stamped_msg.pose.position.x = refPos(9);
        pose_stamped_msg.pose.position.y = refPos(10);
        pose_stamped_msg.pose.position.z = refPos(11);

        // Object tracking Marker
        angles = rotMat_to_angles(objPos);
        quatObject.setRPY(-angles[0], -angles[1], -angles[2]);
        tf2::convert(quatObject, object_stamped_msg.pose.orientation);
        object_stamped_msg.pose.position.x = objPos(9);
        object_stamped_msg.pose.position.y = objPos(10);
        object_stamped_msg.pose.position.z = objPos(11);

        // Origin
        quatOrigin.setRPY(0,0,0);
        tf2::convert(quatOrigin, ref_stamped_msg.pose.orientation);
        ref_stamped_msg.pose.position.x = 0;
        ref_stamped_msg.pose.position.y = 0;
        ref_stamped_msg.pose.position.z = 0;
        std::cout<<"Afuera"<<std::endl;

        // OBJECT DETECTION (SILLAS Y MESAS)
        // **** FOR DEMO PURPOSES ONLY I SWEAR ****
        // If flag 1 or 2 -> object
        if(objPos(12) != 0 && prev == 0){
            // Creating m   arker
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
            angles = rotMat_to_angles(objPos);
            // Setting orientation
            quatMue.setRPY(angles[0], angles[1], angles[2]);
            tf2::convert(quatMue, marker.pose.orientation);

            // Setting marker params
            marker.header.frame_id = "world";
            marker.action = 0;
            marker.id = id;
            id += 1;
            marker.type = marker.CUBE;
            marker.pose.position.x = objPos(9);
            marker.pose.position.y = objPos(10);
            marker.pose.position.z = objPos(11);

            if (objPos(12) == 1) {
                marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(1.0).y(1.0).z(1.0); 
                marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(chair_color[0][0]).g(chair_color[0][1]).b(chair_color[0][2]).a(chair_color[0][3]);
            }
            if (objPos(12) == 2) {
                marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(3.0).y(1.0).z(1.0); 
                marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(table_color[0][0]).g(table_color[0][1]).b(table_color[0][2]).a(table_color[0][3]);
            }

            // PUBLISH OBJECTS
            object_marker_array_msg.markers.push_back(marker);
            object_marker_array_publisher->publish(object_marker_array_msg);
            prev = curr;
        }

        // PUBLISH 
        pose_rviz_publisher->publish(pose_stamped_msg);
        object_rviz_publisher->publish(object_stamped_msg);
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
