// ROS
#include <chrono>
#include "rclcpp/rclcpp.hpp"
// Messages
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/image.hpp"
// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
// Dependencies
#include <vector>


using namespace std::chrono_literals;
using std::placeholders::_1;

//void pure_ring(cv::Mat mat, int ring, std::vector<int> lims, n){
//    int in_p = 0;
//    int out_p = 0;
//    int last_p = 0;
//
//    j
//
//}
//
//void pure_thresh(cv::Mat mat){
//    
//}

class LineDetection : public rclcpp::Node
{
  public:
    LineDetection() : Node("line_detection_node_cpp") {
        // Publishers
        error_pub = this->create_publisher<std_msgs::msg::Int32>("error_cpp", 10);

        // Subscribers
        frame_sub = this->create_subscription<sensor_msgs::msg::Image>("floor_frame",10, std::bind(&LineDetection::subscriber_callback, this, _1));

        // Variables
        error = 0;

        RCLCPP_INFO(this->get_logger(), "Node[line_detection_node] initialized =)");
    }

  private:
    // Variables
    int error;
    cv::Mat img;
    cv::Mat img_weighted;
    cv::Mat img_gray;
    cv::Mat thresh; cv::Mat pure_thresh;
    
    // Edit image
    int min_thresh = 143;
    int chosen_idx = 0;
    int brightness = -277;
    float contrast = 3.5;
    int low_canny = 50;
    int high_canny = 150;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr error_pub;
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr frame_sub;

    void subscriber_callback(const sensor_msgs::msg::Image & msg) {
        // Converting from msg to cv_ptr
        cv_bridge::CvImagePtr img_ptr;
        try {
            img = cv_bridge::toCvCopy(msg)->image;
        }
        catch (cv_bridge::Exception& e){
            RCLCPP_ERROR(this->get_logger(), "Failed msg to cv");
        }

        // Resizing image
        int w = img.cols * 5;
        int h = img.rows * 5;
        resize(img, img, cv::Size(w, h), cv::INTER_LINEAR);

        // Cutting image 
        int x_c = round(w/2);
        int x_t = round(w*.3);
        cv::Rect rect( x_c - x_t, 0, x_t * 2, img.rows);
        img = img(rect).clone();

        // Blending image
        addWeighted( img, contrast, cv::Mat::zeros(img.size().height, img.size().width, img.type()) , 0, brightness, img_weighted);
        // Converting to gray
        cvtColor(img_weighted, img_gray, cv::COLOR_BGR2GRAY); 
        // Applying threshold
        cv::threshold(img_gray, thresh, min_thresh, 255, cv::THRESH_BINARY_INV);
        // my Pure thresh

        cv::imshow("Weighted",thresh);
        cv::waitKey(1);


    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineDetection>());
  rclcpp::shutdown();
  return 0;
}
