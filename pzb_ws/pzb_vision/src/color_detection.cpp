#include "color_detection.h"
#include <chrono>
//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
//ROS
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
//Custom Messages
#include "pzb_msgs/msg/object_detected.hpp"
#include "pzb_msgs/msg/object_detected_vector.hpp"

using namespace std::chrono_literals;


ColorDetection::ColorDetection():Node("color_detection"){
    // Creating publishers
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    publisher_image = this->create_publisher<sensor_msgs::msg::Image>("camera_frame", 10);
    publisher_objectedDetected = this->create_publisher<pzb_msgs::msg::ObjectDetectedVector>("object_detected_vector", 10);

    // Creating timers
    sendFrame_timer = this->create_wall_timer(50ms, std::bind(&ColorDetection::sendingFrame_callback, this));
    colorDetection_timer = this->create_wall_timer(50ms, std::bind(&ColorDetection::colorDetection_callback, this));
    updateObjectsDetected_timer = this->create_wall_timer(25ms, std::bind(&ColorDetection::updateObjectsDetected_callback, this));

    // Declare parameters
    this->declare_parameter("red_values", rclcpp::PARAMETER_INTEGER_ARRAY);
    this->declare_parameter("yellow_values", rclcpp::PARAMETER_INTEGER_ARRAY);
    this->declare_parameter("green_values", rclcpp::PARAMETER_INTEGER_ARRAY);

    // Get parameters
    red_values_param = this->get_parameter("red_values");
    yellow_values_param = this->get_parameter("yellow_values");
    green_values_param = this->get_parameter("green_values");

    // Initiating vectors
    red_values_array = red_values_param.as_integer_array();
    yellow_values_array = yellow_values_param.as_integer_array();
    green_values_array = green_values_param.as_integer_array();

    // Create pipeline for raspberrypi camera
    std::string pipeline = this->gstreamer_pipeline(capture_width,
        capture_height,
        display_width,
        display_height,
        framerate,
        flip_method);
    

    //cap.open(pipeline, cv::CAP_GSTREAMER); //open raspberrypi camera
    cap.open(apiID, deviceID); //open camera in laptop
    // check if we succeeded
    if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Could not open Camera!!!");
    }

    // Initializing time of last_detected for reference
    last_detected = this->now();
    // Initializing colorObjects with one void value
    colorObjects.push_back(std::make_pair("",0));
    prevColorObjects = colorObjects;
    // Initializing objectDetectedVector_msg
    for(int i=0; i<3; i++){
        pzb_msgs::msg::ObjectDetected msg;
        msg.color = "";
        msg.area = 0;
        objectDetectedVector_msg.object_detected_vector.push_back(msg);
    }

    RCLCPP_INFO(this->get_logger(), "Node: color_detecion Initialized =)!!!");


}

// Sending video frame with cv_bridge through topic
void ColorDetection::sendingFrame_callback() {
    cap.read(frame);
    if (frame.empty()) {
         RCLCPP_ERROR(this->get_logger(), "[ERROR] Blank frame grabbed!!!");
     }else{
         cameraFrame_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
         publisher_image->publish(*cameraFrame_msg.get());
         RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
         count_++;
     }
}

void ColorDetection::colorDetection_callback(){
    // Reading frame from camera
    cap.read(frame);  

    // Finding color in matrix
    cv::Mat convert_to_HSV;
    cvtColor(frame, convert_to_HSV, cv::COLOR_BGR2HSV);
    cv::Mat detectionScreen_red;
    cv::Mat detectionScreen_yellow;
    cv::Mat detectionScreen_green;

    // Finding red values
    inRange(convert_to_HSV,cv::Scalar(red_values_array[0], red_values_array[2], red_values_array[4]),
            cv::Scalar(red_values_array[1],red_values_array[3], red_values_array[5]), detectionScreen_red);
    // Finding yellow values
    inRange(convert_to_HSV,cv::Scalar(yellow_values_array[0], yellow_values_array[2], yellow_values_array[4]),
            cv::Scalar(yellow_values_array[1],yellow_values_array[3], yellow_values_array[5]), detectionScreen_yellow);
    // Finding green values
    inRange(convert_to_HSV,cv::Scalar(green_values_array[0], green_values_array[2], green_values_array[4]),
            cv::Scalar(green_values_array[1],green_values_array[3], green_values_array[5]), detectionScreen_green);

    // Cleaning matrix red
    erode(detectionScreen_red, detectionScreen_red, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    dilate(detectionScreen_red, detectionScreen_red, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    dilate(detectionScreen_red, detectionScreen_red, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    erode(detectionScreen_red, detectionScreen_red, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    // Cleaning matrix yellow 
    erode(detectionScreen_yellow, detectionScreen_yellow, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    dilate(detectionScreen_yellow, detectionScreen_yellow, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    dilate(detectionScreen_yellow, detectionScreen_yellow, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    erode(detectionScreen_yellow, detectionScreen_yellow, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    // Cleaning matrix green 
    erode(detectionScreen_green, detectionScreen_green, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    dilate(detectionScreen_green, detectionScreen_green, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    dilate(detectionScreen_green, detectionScreen_green, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    erode(detectionScreen_green, detectionScreen_green, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    // Finding red color (with contours) 
    findContours(detectionScreen_red, contoursRed, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    for(int i=0; i<(int)contoursRed.size(); i++){
        int area = cv::contourArea(contoursRed[i]);
        if (area >2000){
            last_detected = this->now();
            addObject(std::make_pair("red", area), colorObjects);
            RCLCPP_INFO(this->get_logger(), "Red area %d", area);
        }
    }
    // Finding yellow color (with contours) 
    findContours(detectionScreen_yellow, contoursYellow, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    for(int i=0; i<(int)contoursYellow.size(); i++){
        int area = cv::contourArea(contoursYellow[i]);
        if (area >2000){
            last_detected = this->now();
            addObject(std::make_pair("yellow", area), colorObjects);
            RCLCPP_INFO(this->get_logger(), "Yellow area %d", area);
        }
    }
    // Finding green color (with contours) 
    findContours(detectionScreen_green, contoursGreen, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    for(int i=0; i<(int)contoursGreen.size(); i++){
        int area = cv::contourArea(contoursGreen[i]);
        if (area >2000){
            last_detected = this->now();
            addObject(std::make_pair("green", area), colorObjects);
            RCLCPP_INFO(this->get_logger(), "Green area %d", area);
        }
    }
}

void ColorDetection::addObject(std::pair<std::string, int> object, std::vector<std::pair<std::string,int>> &v){
    // Only a 3 size vector, from bigger to smaller area of object
    std::pair<std::string, int> temp;
    if(v.size() < 3) {
        v.push_back(object);
        sort(v.begin(), v.end(), [](auto& a, auto& b) {
            return a.second > b.second;
        });
    }else{
        v.push_back(object);
        sort(v.begin(), v.end(), [](auto& a, auto& b) {
            return a.second > b.second;
        });
        v.pop_back();
    }
}

void ColorDetection::removeObject(std::string color, std::vector<std::pair<std::string,int>> &v){
    // remove all objects "color"
    std::pair<std::string, int> tempObject;
    int j=0;
    for(int i=0; i<(int)v.size(); i++){
        if(v[i].first == color){
            v[i].second = 0;
            j++;
        }
    }
    sort(v.begin(), v.end(), [](auto& a, auto& b) {
        return a.second > b.second;
    });
    for(int i=0; i<j; i++){
        v.pop_back();
    }
}

void ColorDetection::updateObjectsDetected_callback(){
        
    rclcpp::Duration diff = this->now() - last_detected;
    int elapsed_ms = diff.nanoseconds() / 1e6;
    if (elapsed_ms > 250){
        // Set to none object detected
        for(int i=0; i<(int)colorObjects.size(); i++){
            colorObjects[i].first = "";
            colorObjects[i].second = 0;
        }
    }

    for(int i=0; i<(int)colorObjects.size(); i++){
        if(colorObjects[i] == prevColorObjects[i]) continue;
        objectDetectedVector_msg.object_detected_vector[i].color = colorObjects[i].first;
        objectDetectedVector_msg.object_detected_vector[i].area = colorObjects[i].second;
    }

    publisher_objectedDetected->publish(objectDetectedVector_msg); // publish msg with 3 objects detected
    prevColorObjects = colorObjects; // update previous objects detected
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetection>());
    rclcpp::shutdown();
    return 0;
}
