#pragma once
//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

//ROS
#include <opencv2/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
//Custom Messages
#include "pzb_msgs/msg/object_detected.hpp"
#include "pzb_msgs/msg/object_detected_vector.hpp"

using namespace std::chrono_literals;


class ColorDetection : public rclcpp::Node
{

    // OpenCV Variables
    int deviceID = 0; // 0 = open default camera
    int apiID = cv::CAP_ANY; // 0 = autodetect default API

    // Camera values
    int capture_width = 1280 ;
    int capture_height = 720 ;
    int display_width = 1280 ;
    int display_height = 720 ;
    int framerate = 30 ;
    int flip_method = 0 ;

    public:
        ColorDetection();

    private:
        // Return string of pipeline for camera
        std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
            return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
                std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
                "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
                std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
        }


        // Parameters
        rclcpp::Parameter red_values_param;
        rclcpp::Parameter yellow_values_param;
        rclcpp::Parameter green_values_param;

        // Callbacks
        void sendingFrame_callback();
        void colorDetection_callback();
        void updateObjectsDetected_callback();

        // Timers
        rclcpp::TimerBase::SharedPtr colorDetection_timer;
        rclcpp::TimerBase::SharedPtr updateObjectsDetected_timer;

        // Publishers
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image;
        rclcpp::Publisher<pzb_msgs::msg::ObjectDetectedVector>::SharedPtr publisher_objectedDetected;

        // Variables
        cv::Mat frame;
        cv::VideoCapture cap;
        size_t count_;
        std::vector<long int> red_values_array;
        std::vector<long int> yellow_values_array;
        std::vector<long int> green_values_array;
        std::vector<std::vector<cv::Point>> contoursRed;
        std::vector<std::vector<cv::Point>> contoursYellow;
        std::vector<std::vector<cv::Point>> contoursGreen;
        std::vector< std::pair<std::string, int> > colorObjects; // for storing the 3 biggest objects
        std::vector< std::pair<std::string, int> > prevColorObjects;
        rclcpp::Time last_detected;

        //Functions
        void addObject(std::pair<std::string, int> object, std::vector<std::pair<std::string,int>> &v);
        void removeObject(std::string color, std::vector<std::pair<std::string,int>> &v);

        // Msg
        sensor_msgs::msg::Image::SharedPtr cameraFrame_msg;
        pzb_msgs::msg::ObjectDetectedVector objectDetectedVector_msg;
};


