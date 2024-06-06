#pragma once
//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

//ROS
#include <opencv2/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;


class CameraView : public rclcpp::Node
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
        CameraView();

    private:
        // Return string of pipeline for camera
        std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
            return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
                std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
                "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
                std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
        }

        // Callbacks
        void sendingFrame_callback();

        // Timers
        rclcpp::TimerBase::SharedPtr sendFrame_timer;

        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr floor_publisher_image;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr signal_publisher_image;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dotted_publisher_image;

        // Variables
        cv::Mat frame;
        cv::Mat frame_resized;
        cv::Mat floor_frame, signal_frame, dotted_frame;
        cv::Size frame_size;
        cv::Rect floor_r, signal_r, dotted_r;

        cv::VideoCapture cap;
        size_t count_;

        // Msg
        sensor_msgs::msg::Image::SharedPtr cameraFrame_msg;
        sensor_msgs::msg::Image::SharedPtr floorFrame_msg;
        sensor_msgs::msg::Image::SharedPtr signalFrame_msg;
        sensor_msgs::msg::Image::SharedPtr dottedFrame_msg;
};
