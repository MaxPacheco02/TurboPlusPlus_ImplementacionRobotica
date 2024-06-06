//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
//ROS
#include <opencv2/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <cv_bridge/cv_bridge.h>

//Dependencies
#include <vector>

using std::placeholders::_1;

class DecodeImg : public rclcpp::Node
{
  public:
    DecodeImg() : Node("decode_img"){
      publisher_image = this->create_publisher<sensor_msgs::msg::Image>("camera_decompressed_frame", 10);
      sub_compressed_image = this->create_subscription<sensor_msgs::msg::CompressedImage>("camera_frame_compressed", 10, std::bind(&DecodeImg::show_compressed_img_callback, this, _1));
    
      RCLCPP_INFO(this->get_logger(), "Node: decode img =)!!!");
    }

  private:
    cv::Mat frame;
    std::vector<uchar> buff_img;

    void show_compressed_img_callback( sensor_msgs::msg::CompressedImage msg) {
      try {
        buff_img = msg.data;
        frame = cv::imdecode(buff_img, cv::IMREAD_COLOR);
        cameraFrame_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_image->publish(*cameraFrame_msg.get());  
      } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] NULL Buffer!!!");
      }
    } 
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed_image;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image;
    sensor_msgs::msg::Image::SharedPtr cameraFrame_msg;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DecodeImg>());
  rclcpp::shutdown();
  return 0;
}