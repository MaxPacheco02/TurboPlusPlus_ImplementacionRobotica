#include <typeinfo>
// ROS
#include <chrono>
#include <algorithm>
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

class LineDetection : public rclcpp::Node
{
  public:
    LineDetection() : Node("line_detection") {
        error_pub_ = this->create_publisher<std_msgs::msg::Int32>("error", 10);
        processed_frame_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/line_detection_processed_frame", 10);
        frame_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("floor_frame", 10, 
                        std::bind(&LineDetection::subscriber_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Node[line_detection_node] initialized =)");

        processed_frame_msg.format=".jpeg";
    }

  private:
    std::vector<uchar> buff_img;
    // Variables
    int error{0};
    cv::Mat img;
    cv::Mat img_org;
    cv::Mat img_weighted;
    cv::Mat img_gray;
    cv::Mat frames;
    cv::Mat thresh;
    cv::Mat thresh_pure;
    
    std::vector<uchar> processed_img;

    // Edit image
    int min_thresh{143};
    int chosen_idx{0};
    int brightness{-454};
    // int brightness{-644};
    float contrast{4.5};
    int low_canny{50};
    int high_canny{150};

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr error_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr processed_frame_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr frame_sub_;

    sensor_msgs::msg::CompressedImage processed_frame_msg;
    std_msgs::msg::Int32 error_msg;

    void subscriber_callback(const sensor_msgs::msg::CompressedImage & msg) {

      try {
        buff_img = msg.data;
        img = cv::imdecode(buff_img, cv::IMREAD_COLOR);
      } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] NULL Buffer!!!");
      }
      
      int w = img.cols*5;
      int h = img.rows*5;
      resize(img, img, cv::Size(w, h), cv::INTER_LINEAR);

      // Cutting image 
      int x_c = round(w/2);
      int x_t = round(w*.2);
      cv::Rect rect( x_c - x_t, 0, x_t * 2, img.rows);
      img = img(rect).clone();
      img_org = img.clone();

      // Blending image
      addWeighted(img, contrast, cv::Mat::zeros(img.size().height, img.size().width, img.type()) , 0, brightness, img_weighted);
      // Converting to gray
      cvtColor(img_weighted, img_gray, cv::COLOR_BGR2GRAY); 
      // Applying threshold
      cv::threshold(img_gray, thresh, min_thresh, 255, cv::THRESH_BINARY_INV);
      // my Pure thresh
      thresh_pure = pure_thresh(thresh);
      
      cv::Moments M = cv::moments(thresh_pure, true);
      float cx{thresh_pure.cols/2}, cy{thresh_pure.rows/2};
      if(M.m00 != 0.){
        cx = M.m10 / M.m00;
        cy = M.m01 / M.m00;
      }
      // std::cout << cx << "," << cy << std::endl;

      if(cx != thresh_pure.cols/2 || cy != thresh_pure.rows/2){
        cv::Point pt(cx, cy);
        cv::circle(img, pt, 5, (0,0,255), -1);
      }

      error_msg.data = (cx - thresh_pure.cols/2) * 5.;
      error_pub_->publish(error_msg);

      cv::imencode(".jpeg", img, processed_img);
      processed_frame_msg.data=processed_img;
      processed_frame_pub_->publish(processed_frame_msg);

      // frames = cv2_conc(std::vector<cv::Mat>{img_org, img, img_weighted, thresh, thresh_pure});
      // cv::imshow("Weighted",frames);
      // cv::imshow("Weighted",img);
      // cv::waitKey(1);
    }

    cv::Mat cv2_conc(std::vector<cv::Mat> vec){
      cv::Mat tmp, tmp2;
      for(int i = 0 ; i < vec.size(); i++){

        if(vec[i].channels() == 1)
          cvtColor(vec[i], tmp2, cv::COLOR_GRAY2RGB);
        else
          tmp2 = vec[i];

        if(tmp.size().width == 0)
          tmp = tmp2;
        else
          cv::vconcat(tmp,tmp2,tmp);
      }
      return tmp;
    }

    int validate_window(int in_p, int out_p, int lims[2]){
      int diff = out_p - in_p;
      return (diff < lims[1] && diff > lims[0]) ? 1 : 0;
    }

    std::vector<std::array<int, 3>> pure_ring(cv::Mat mat, int ring, int n, int limits[2]){
      int in_p{0}, out_p{0}, last_p{0}, pixel{0}, one_count{0}, zero_count{0};
      std::vector<std::array<int, 3>> windows;
      
      for(int i = 0 ; i < mat.size().width ; i++){
        pixel = cv::countNonZero(mat.row(mat.size().height * ring / n).col(i));
        if(pixel){
          if(last_p==0)
            in_p = i;
          out_p = i;
          

          if(windows.size() > 0 && i == mat.size().width - 1 && out_p != windows[windows.size() - 1][1]){
            std::array<int, 3> wdw = {in_p, out_p, validate_window(in_p, out_p, limits)};
            windows.push_back(wdw);
          }

        } else {
          if(last_p != 0){
            std::array<int, 3> wdw = {in_p, out_p, validate_window(in_p, out_p, limits)};
            windows.push_back(wdw);
          }
        }
        last_p = pixel;
      }
      return windows;
    }

    bool is_joined_window(std::array<int, 3> w1, std::array<int, 3> w2){
      if(std::max(w1[0], w2[0]) < std::min(w1[1], w2[1]))
          return true;
      return false; 
    }

    bool is_in_list(std::array<int, 3> window, std::vector<std::array<int, 3>> window_list){
      for(int i = 0 ; i < window_list.size() ; i++){
        if(is_joined_window(window, window_list[i]))
          return true;
      }
      return false;
    }

    cv::Mat pure_thresh(cv::Mat mat){
      std::vector<std::vector<std::array<int, 3>>> windows;
      std::vector<std::array<int, 3>> windows_pure;
      int n = 30;
      int limits[2]{30, 300};
      for(int i = 0 ; i < n - 1 ; i++){
        windows.push_back(pure_ring(mat, i+1, n, limits));
      }

      for(int i = 0 ; i < windows.size() ; i++){
        for(int j = 0 ; j < windows[i].size() ; j++){
          if(windows[i][j][2]){
            if(windows_pure.size() == 0){
              windows_pure.push_back(windows[i][j]);
            }

            int k = 0;
            while(k < windows_pure.size()){
              if(is_joined_window(windows[i][j], windows_pure[k])){
                windows_pure[k][0] = std::min(windows_pure[k][0], windows[i][j][0]);
                windows_pure[k][1] = std::max(windows_pure[k][1], windows[i][j][1]);
                windows_pure[k][2] = windows_pure[k][2] + windows[i][j][2];
              } else {
                if(!is_in_list(windows[i][j], windows_pure)){
                  windows_pure.push_back(windows[i][j]);
                }
              }
              k++;
            }
          }
        }
      }

      cv::Mat pure_img = cv::Mat::zeros(cv::Size(mat.cols, mat.rows), CV_8U);
      for(int i = 0 ; i < windows_pure.size() ; i++){
        if(windows_pure[i][2] > n * 0.2){
          // std::cout << "pure window in " << windows_pure[i][0] << " , " << 
            // windows_pure[i][1] << " , " << windows_pure[i][2] << std::endl;
            // cv::Rect rect(0,0,mat.cols/2,mat.rows/2);
            cv::Rect rect(windows_pure[i][0],0,windows_pure[i][1] - windows_pure[i][0],mat.rows);
            mat(rect).copyTo(pure_img(rect));
        }
      }
      return pure_img;
    }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineDetection>());
  rclcpp::shutdown();
  return 0;
}