#include "camera_view.h"
#include <chrono>
//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
//ROS
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
//Dependencies
#include <vector>
#include <typeinfo>
#include <algorithm>

using namespace std::chrono_literals;
using std::placeholders::_1;

CameraView::CameraView():Node("camera_view"){
    // Creating subscribers
    frame_subscriber = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "Image", 10,
        [this](const sensor_msgs::msg::CompressedImage &msg) { 
            try {
                buff_img = msg.data;
                frame = cv::imdecode(buff_img, cv::IMREAD_COLOR);

                std::cout << "before" << frame.size() << std::endl;
                // frame = cv_bridge::toCvCopy(msg)->image;
                if(!frame.size().empty()){
                    cv::resize(frame, frame_resized, cv::Size(), 0.8, 0.6, 6);
                    frame = frame_resized.clone();
                    frame_size = frame.size();
                    std::cout << "after" << frame.size() << std::endl;

                    dotted_r = cv::Rect( 0, frame_size.height / 2, frame_size.width, frame_size.height / 5);
                    dotted_frame = frame(dotted_r).clone();
                    floor_r = cv::Rect( 0, 16 * frame_size.height / 20, frame_size.width, 2 * frame_size.height / 20);
                    floor_frame = frame(floor_r).clone();
                    signal_r = cv::Rect( 0, 0, frame_size.width, frame_size.height / 2);
                    signal_frame = frame(signal_r).clone();

                    cv::imencode(".jpeg", dotted_frame, dotted_img);
                    dottedFrame_msg.data=dotted_img;
                    cv::imencode(".jpeg", signal_frame, signal_img);
                    signalFrame_msg.data=signal_img;
                    cv::imencode(".jpeg", floor_frame, floor_img);
                    floorFrame_msg.data=floor_img;
                }

                dotted_publisher_image->publish(dottedFrame_msg);
                signal_publisher_image->publish(signalFrame_msg);
                floor_publisher_image->publish(floorFrame_msg);
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "[ERROR] NULL Buffer!!!");
            }
        });

    // Creating publishers
    floor_publisher_image = this->create_publisher<sensor_msgs::msg::CompressedImage>("/floor_frame", 10);
    dotted_publisher_image = this->create_publisher<sensor_msgs::msg::CompressedImage>("/dotted_frame", 10);
    signal_publisher_image = this->create_publisher<sensor_msgs::msg::CompressedImage>("/signal_frame", 10);

    floorFrame_msg.format=".jpeg";
    dottedFrame_msg.format=".jpeg";
    signalFrame_msg.format=".jpeg";
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraView>());
    rclcpp::shutdown();
    return 0;
}
