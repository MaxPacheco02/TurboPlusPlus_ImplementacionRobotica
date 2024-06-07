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
    publisher_image = this->create_publisher<sensor_msgs::msg::Image>("camera_frame", 10);
    floor_publisher_image = this->create_publisher<sensor_msgs::msg::Image>("floor_frame", 10);
    dotted_publisher_image = this->create_publisher<sensor_msgs::msg::Image>("dotted_frame", 10);
    signal_publisher_image = this->create_publisher<sensor_msgs::msg::Image>("signal_frame", 10);
    publisher_compressed_image = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera_frame_compressed", 10);


    // Creating timers
    sendFrame_timer = this->create_wall_timer(50ms, std::bind(&CameraView::sendingFrame_callback, this));

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
}

// Sending video frame with cv_bridge through topic
void CameraView::sendingFrame_callback() {
    cap.read(frame);
    if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Blank frame grabbed!!!");
     }else{
        cv::imencode(".jpeg", frame, buff_img);
        compressed_img_msg.format=".jpeg";
        compressed_img_msg.data=buff_img;

        cv::resize(frame, frame_resized, cv::Size(), 0.2, 0.2);

        frame_size = frame_resized.size();

        dotted_r = cv::Rect( 0, frame_size.height / 2, frame_size.width, frame_size.height / 5);
        dotted_frame = frame_resized(dotted_r).clone();
        floor_r = cv::Rect( 0, 4 * frame_size.height / 5, frame_size.width, frame_size.height / 5);
        floor_frame = frame_resized(floor_r).clone();
        signal_r = cv::Rect( 0, 0, frame_size.width, frame_size.height / 2);
        signal_frame = frame_resized(signal_r).clone();

        cameraFrame_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_resized).toImageMsg();
        floorFrame_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", floor_frame).toImageMsg();
        dottedFrame_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", dotted_frame).toImageMsg();
        signalFrame_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", signal_frame).toImageMsg();
        publisher_image->publish(*cameraFrame_msg.get());
        floor_publisher_image->publish(*floorFrame_msg.get());
        dotted_publisher_image->publish(*dottedFrame_msg.get());
        signal_publisher_image->publish(*signalFrame_msg.get());
        publisher_compressed_image->publish(compressed_img_msg);

     }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraView>());
    rclcpp::shutdown();
    return 0;
}
