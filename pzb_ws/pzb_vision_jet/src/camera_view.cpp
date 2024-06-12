#include "camera_view.h"
#include <chrono>
//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
//ROS
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
//Dependencies
#include <vector>



using namespace std::chrono_literals;

CameraView::CameraView():Node("camera_view"){
    // Creating publishers
    floor_publisher_image = this->create_publisher<sensor_msgs::msg::CompressedImage>("floor_frame", 10);
    dotted_publisher_image = this->create_publisher<sensor_msgs::msg::CompressedImage>("dotted_frame", 10);
    signal_publisher_image = this->create_publisher<sensor_msgs::msg::CompressedImage>("signal_frame", 10);

    // Creating timers
    sendFrame_timer = this->create_wall_timer(10ms, std::bind(&CameraView::sendingFrame_callback, this));

    // Create pipeline for raspberrypi camera
    std::string pipeline = this->gstreamer_pipeline(capture_width,
        capture_height,
        display_width,
        display_height,
        framerate,
        flip_method);

    cap.open(pipeline, cv::CAP_GSTREAMER); //open raspberrypi camera
    // cap.open(apiID, deviceID); //open camera in laptop
    if (!cap.isOpened())
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Could not open Camera!!!");
    RCLCPP_INFO(this->get_logger(), "Node initialized JE :)");

    floorFrame_msg.format=".jpeg";
    dottedFrame_msg.format=".jpeg";
    signalFrame_msg.format=".jpeg";
}

// Sending video frame with cv_bridge through topic
void CameraView::sendingFrame_callback() {
    cap.read(frame);
    if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Blank frame grabbed!!!");
     }else{

        cv::resize(frame, frame_resized, cv::Size(), 0.2, 0.2);

        frame_size = frame_resized.size();

        dotted_r = cv::Rect( 0, frame_size.height * 3 / 5, frame_size.width, frame_size.height / 5);
        dotted_frame = frame_resized(dotted_r).clone();
        floor_r = cv::Rect( 0, 4 * frame_size.height / 5, frame_size.width, frame_size.height / 5);
        floor_frame = frame_resized(floor_r).clone();
        signal_r = cv::Rect( 0, 0, frame_size.width, frame_size.height / 2);
        signal_frame = frame_resized(signal_r).clone();

        cv::imencode(".jpeg", dotted_frame, dotted_img);
        dottedFrame_msg.data=dotted_img;
        cv::imencode(".jpeg", signal_frame, signal_img);
        signalFrame_msg.data=signal_img;
        cv::imencode(".jpeg", floor_frame, floor_img);
        floorFrame_msg.data=floor_img;

        dotted_publisher_image->publish(dottedFrame_msg);
        signal_publisher_image->publish(signalFrame_msg);
        floor_publisher_image->publish(floorFrame_msg);

     }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraView>());
    rclcpp::shutdown();
    return 0;
}
