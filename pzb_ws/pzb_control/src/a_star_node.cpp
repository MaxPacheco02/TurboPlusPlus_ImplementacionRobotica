#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "nav_msgs/msg/grid_cells.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "AStar.hpp"

using namespace std::chrono_literals;

struct Wp
{
    double x, y;
};

class ObstacleAvoidanceNode : public rclcpp::Node
{
public:
    ObstacleAvoidanceNode() : Node("obstacle_avoidance_node")
    {
        multiplier = map_size / (2 * map_magnitude);

        // map_size = multiplier * map_magnitude * 2;

        generator.setWorldSize({int(map_size), int(map_size)});
        generator.setHeuristic(AStar::Heuristic::euclidean);
        generator.setDiagonalMovement(true);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            [this](const nav_msgs::msg::Odometry &msg)
            { pose_ = msg.pose.pose; });

        obstacle_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10,
            [this](const nav_msgs::msg::OccupancyGrid &msg)
            {
                RCLCPP_INFO(this->get_logger(), "OccupancyGrid received!");
                generator.clearCollisions();

                int width = msg.info.width;
                int height = msg.info.height;
                float resolution = msg.info.resolution;
                auto origin = msg.info.origin.position;

                for (int y = 0; y < height; ++y)
                {
                    for (int x = 0; x < width; ++x)
                    {
                        int index = y * width + x;
                        int8_t value = msg.data[index];

                        // You can check for -1 (unknown), 0 (free), 100 (occupied)
                        if (value > 50) // Threshold for obstacle
                        {
                            float wx = origin.x + x * resolution;
                            float wy = origin.y + y * resolution;

                            generator.addCollision({int(wx * multiplier + map_size / 2), int(wy * multiplier + map_size / 2)});
                        }
                    }
                }
            });

        nav_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "nav/goal_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped &msg)
            {
                Wp wp1{pose_.position.x, pose_.position.y};
                Wp wp2{msg.pose.position.x, msg.pose.position.y};

                a_star_path.poses.clear();

                auto path = generator.findPath({wp2.x * multiplier + map_size / 2,
                                                wp2.y * multiplier + map_size / 2},
                                               {wp1.x * multiplier + map_size / 2,
                                                wp1.y * multiplier + map_size / 2});

                for (auto &coordinate : path)
                {
                    // std::cout << coordinate.x << " " << coordinate.y << "\n";
                    pose_stamped_tmp_.pose.position.x = (coordinate.x - map_size / 2) / multiplier;
                    pose_stamped_tmp_.pose.position.y = (coordinate.y - map_size / 2) / multiplier;
                    pose_stamped_tmp_.pose.orientation = msg.pose.orientation;
                    a_star_path.poses.push_back(pose_stamped_tmp_);
                }
            });

        a_star_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/pzb/a_star_path", 10);

        a_star_path.header.frame_id = "odom";
        a_star_path.header.stamp = ObstacleAvoidanceNode::now();
        pose_stamped_tmp_.header.frame_id = "odom";

        updateTimer =
            this->create_wall_timer(100ms, std::bind(&ObstacleAvoidanceNode::update, this));
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr a_star_path_pub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::TimerBase::SharedPtr updateTimer;

    geometry_msgs::msg::PoseStamped pose_stamped_tmp_;
    geometry_msgs::msg::Pose pose_;
    nav_msgs::msg::Path a_star_path;

    AStar::Generator generator;

    // Squared map with center on 0,0 and going from -mag,-mag to mag,mag
    double map_magnitude{4.0};

    // Total number of cells used per axis
    // Multiple of 2 * map_magnitude
    int map_size{80};

    // Number of cells per meter
    double multiplier;

    void update()
    {
        a_star_path_pub_->publish(a_star_path);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}