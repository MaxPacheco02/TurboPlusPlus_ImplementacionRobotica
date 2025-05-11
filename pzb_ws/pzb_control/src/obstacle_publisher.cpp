#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class ObstaclePublisher : public rclcpp::Node {
public:
    ObstaclePublisher() : Node("obstacle_publisher") {
        // Initialize publishers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());
        pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("obstacle_positions", 10);

        // Create timer for periodic publishing
        timer_ = this->create_wall_timer(500ms, std::bind(&ObstaclePublisher::publish_obstacles, this));

        // Initialize obstacle positions (5 obstacles with x,y coordinates)
        obstacle_positions_ = {
            1.0, 0.0
        };

        RCLCPP_INFO(this->get_logger(), "ObstaclePublisher initialized");
    }

private:
    void publish_obstacles() {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < obstacle_positions_.size() / 2 ; ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = this->now();
            marker.ns = "obstacles";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set position
            marker.pose.position.x = obstacle_positions_[i * 2];
            marker.pose.position.y = obstacle_positions_[i * 2 + 1];
            marker.pose.position.z = 0.0;
            
            // Set orientation
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            
            // Set scale (radius = 0.01m = 1cm)
            marker.scale.x = 0.2;  // Made bigger for visibility
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            
            // Set color (red)
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            // Set lifetime (0 = forever)
            marker.lifetime = rclcpp::Duration(0, 0);

            marker_array.markers.push_back(marker);
        }
        
        // Publish positions as Float64MultiArray
        auto positions = std_msgs::msg::Float64MultiArray();
        positions.data = obstacle_positions_;
        
        marker_pub_->publish(marker_array);
        pos_pub_->publish(positions);

        // Log publishing
        static int count = 0;
        if (count++ % 10 == 0) {  // Log every 10th publish
            RCLCPP_INFO(this->get_logger(), "Published markers. Count: %d", count);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pos_pub_;
    std::vector<double> obstacle_positions_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstaclePublisher>();
    RCLCPP_INFO(node->get_logger(), "Starting ObstaclePublisher node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}