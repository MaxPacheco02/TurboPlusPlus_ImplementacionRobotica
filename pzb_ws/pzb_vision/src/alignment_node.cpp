#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <iomanip>
#include <queue>
#include <iostream>
#include <vector>
#include <optional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.hpp"
#include "std_srvs/srv/empty.hpp"

#include "pzb_msgs/msg/trailer_bearing.hpp"
#include "pzb_msgs/srv/trailer_align.hpp"
#include "pzb_msgs/srv/approach.hpp"

using namespace std::chrono_literals;

struct MarkerProps
{
    int type;
    double x, y, z, z_trans;
};

class AlignmentNode : public rclcpp::Node
{
public:
    AlignmentNode() : Node("alignment_node")
    {
        using namespace std::placeholders;

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar", 10,
            [this](const sensor_msgs::msg::LaserScan &msg)
            {
                angle_min = msg.angle_min;
                angle_max = msg.angle_max;
                angle_increment = msg.angle_increment;

                laserscan_vector = msg.ranges;
            });

        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](const nav_msgs::msg::Odometry &msg)
            {
                pose_ << msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    tf2::getYaw(msg.pose.pose.orientation);

                double theta = pose_(2);
                rotM_ << std::cos(theta), -std::sin(theta), 0,
                    std::sin(theta), std::cos(theta), 0,
                    0, 0, 1;
            });

        // Get a trailer's relative heading angle from the camera
        yolo_bearing_sub_ = this->create_subscription<pzb_msgs::msg::TrailerBearing>(
            "/yolo_bearing_angles", 10,
            [this](const pzb_msgs::msg::TrailerBearing &msg)
            {
                last_yolo_update_ = this->get_clock()->now();
                cam_bearing_angles["diagonal"] = msg.diagonal;
                cam_bearing_angles["oxidado"] = msg.oxidado;
                cam_bearing_angles["rojo"] = msg.diagonal;
            });

        // Get a pallet's relative pose from the camera
        pallet_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pallet_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped &msg)
            {
                last_pallet_update_ = this->get_clock()->now();
                pallet_v = pose2vec(msg.pose);
            });

        trailer_service_ = this->create_service<pzb_msgs::srv::TrailerAlign>(
            "trailer_align_srv", std::bind(
                                     &AlignmentNode::align_trailer, this, _1, _2));

        pallet_service_ = this->create_service<std_srvs::srv::Empty>(
            "pallet_align_srv", std::bind(
                                    &AlignmentNode::align_pallet, this, _1, _2));

        approach_service_ = this->create_service<pzb_msgs::srv::Approach>(
            "approach_srv", std::bind(
                                &AlignmentNode::approach, this, _1, _2));

        pid_ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/marker_array", 10);

        // update_timer_ = this->create_wall_timer(100ms, std::bind(&AlignmentNode::update, this));
        pid_ref_msg.header.frame_id = "odom";

        axis2cam_v << axis2cam, 0;

        last_yolo_update_ = this->get_clock()->now();
        last_pallet_update_ = this->get_clock()->now();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_sub_;
    rclcpp::Subscription<pzb_msgs::msg::TrailerBearing>::SharedPtr yolo_bearing_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_pose_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pid_ref_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Service<pzb_msgs::srv::TrailerAlign>::SharedPtr trailer_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pallet_service_;
    rclcpp::Service<pzb_msgs::srv::Approach>::SharedPtr approach_service_;

    visualization_msgs::msg::MarkerArray marker_arr;
    rclcpp::TimerBase::SharedPtr update_timer_;
    tf2::Quaternion quat;
    geometry_msgs::msg::PoseStamped pid_ref_msg;

    rclcpp::Time last_yolo_update_;
    rclcpp::Time last_pallet_update_;

    double scan_filter_angle{5.0};
    std::vector<float> laserscan_vector;
    double angle_min, angle_max, angle_increment;
    int nearest_index{-1};
    double offset = 0.2;

    std::map<std::string, double> cam_bearing_angles = {
        {"diagonal", NAN},
        {"oxidado", NAN},
        {"rojo", NAN},
    };

    double axis2cam{0.056};
    Eigen::Vector2f axis2cam_v;

    std::string disp_frame{"laser"};

    Eigen::Vector3f pose_, pallet_v;

    Eigen::Matrix3f rotM_;

    double received_goal{false};
    int bug_algorithm{2};

    int skip_rays{1};

    int color_list[6][4]{
        {1, 0, 0, 1},
        {0, 1, 0, 1},
        {0, 0, 1, 1},
        {1, 1, 0, 1},
        {0, 0, 0, 1},
        {0, 0, 0, 0}};

    std::map<std::string, MarkerProps> marker_type = {
        {"marker", MarkerProps{3, 0.02, 0.02, 0.02, 0.25}},
    };

    // Trailer aligning service call. Assumes the trailer is being detected.
    void align_trailer(const std::shared_ptr<pzb_msgs::srv::TrailerAlign::Request> request,
                       std::shared_ptr<pzb_msgs::srv::TrailerAlign::Response> response)
    {
        // Ignore if a yolo detection hasn't been recently found (100 ms)
        if (this->get_clock()->now() - last_yolo_update_ > rclcpp::Duration(0, 100 * 1e6))
            return;

        calc_trailer_ref(cam_bearing_angles[request->trailer_type]);
        pid_ref_pub_->publish(pid_ref_msg);
    }

    // Pallet aligning service call.
    void align_pallet(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                      std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        // Ignore if a pallet detection hasn't been recently found (100 ms)
        if (this->get_clock()->now() - last_pallet_update_ > rclcpp::Duration(0, 100 * 1e6))
            return;

        calc_pallet_ref();
        pid_ref_pub_->publish(pid_ref_msg);
    }

    // Pallet aligning service call.
    void approach(const std::shared_ptr<pzb_msgs::srv::Approach::Request> request,
                      std::shared_ptr<pzb_msgs::srv::Approach::Response> response)
    {
        set_pid_ref(forward(pose_, request->distance));
        pid_ref_pub_->publish(pid_ref_msg);
    }

    void add_marker(Eigen::Vector2f p, int color)
    {
        visualization_msgs::msg::Marker marker;

        std::string type = "marker";

        int r{color_list[color][0]},
            g{color_list[color][1]},
            b{color_list[color][2]},
            a{color_list[color][3]};

        marker.header.frame_id = disp_frame;
        marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(r).g(g).b(b).a(a);
        marker.action = 0;
        marker.id = marker_arr.markers.size();
        marker.type = marker_type[type].type;
        marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(marker_type[type].x).y(marker_type[type].y).z(marker_type[type].z);
        marker.pose.position.x = p(0);
        marker.pose.position.y = p(1);
        marker.pose.position.z = marker_type[type].z_trans;
        marker_arr.markers.push_back(marker);
    }

    void add_linestrip(Eigen::Vector2f p, int color)
    {
        visualization_msgs::msg::Marker marker;

        int r{color_list[color][0]},
            g{color_list[color][1]},
            b{color_list[color][2]},
            a{color_list[color][3]};

        marker.header.frame_id = disp_frame;
        marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(r).g(g).b(b).a(a);
        marker.action = 0;
        marker.id = marker_arr.markers.size();
        marker.type = 4;
        marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.005).y(0.0).z(0.0);
        marker.points.push_back(geometry_msgs::build<geometry_msgs::msg::Point>().x(axis2cam).y(0.0).z(0.0));
        marker.points.push_back(geometry_msgs::build<geometry_msgs::msg::Point>().x(p(0)).y(p(1)).z(0.0));

        marker_arr.markers.push_back(marker);
    }

    void calc_pallet_ref()
    {
        set_pid_ref(pose_ + rotM_ * forward(pallet_v, -offset));
    }

    void calc_trailer_ref(double cam_bearing_angle)
    {
        if (laserscan_vector.size() == 0)
            return;

        marker_arr.markers.clear();

        int inc = -sign(cam_bearing_angle);
        Eigen::Vector2f bearing_v;
        bearing_v << 1.0, tan(cam_bearing_angle);
        bearing_v = bearing_v * 5 + axis2cam_v;
        add_linestrip(bearing_v, 3);

        /*
            Find the laserscan's index that best intersects with the bounding box's center
            ang = min + idx * increment
        */
        int idx = (cam_bearing_angle - angle_min) / angle_increment; // Start with the idx looking at the camera's angle.

        // Find the laserscan's relative bearing angle. Commented because not doing this worked better. TODO: Debug why.
        // double min_ce = calculate_crosstrack_error(get_point(idx), axis2cam_v, bearing_v);

        // // The real index always has a lower abs angle than the camera's angle because its further.
        // int tmp_idx = idx;
        // bool is_decreasing{true};
        // while (is_decreasing)
        // {
        //     tmp_idx += inc;
        //     double tmp_ce = calculate_crosstrack_error(get_point(tmp_idx), axis2cam_v, bearing_v);
        //     if (tmp_ce < min_ce)
        //     {
        //         min_ce = tmp_ce;
        //         idx = tmp_idx;
        //     }
        //     else
        //     {
        //         is_decreasing = false;
        //     }
        // }

        if (idx < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Not suitable idx.");
            return;
        }

        Eigen::Vector2f midpoint = get_point(idx);
        Eigen::Vector2f p1 = get_point(idx - get_sep_from_dist(laserscan_vector[idx]));
        Eigen::Vector2f p2 = get_point(idx + get_sep_from_dist(laserscan_vector[idx]));

        // Calculate tangent and normal
        Eigen::Vector2f tangent = (p2 - p1).normalized();
        Eigen::Vector2f normal(-tangent(1), tangent(0)); // Left-hand normal

        // First calculate the offset perpendicular to the wall
        Eigen::Vector2f offset_point = midpoint + offset * normal;

        Eigen::Vector3f local_wp;
        local_wp.head<2>() = offset_point;
        local_wp(2) = calculate_heading(offset_point, midpoint);

        Eigen::Vector3f wp = pose_ + rotM_ * local_wp;

        set_pid_ref(wp);

        add_marker(p1, 2);
        add_marker(p2, 2);
        add_marker(midpoint, 1);
        add_marker(offset_point, 2);

        marker_pub_->publish(marker_arr);
    }

    int get_sep_from_dist(double d)
    {
        double sep = -20.0 * d + 22.0;
        return static_cast<int>(std::clamp(sep, 2.0, 18.0));
    }

    double calculate_crosstrack_error(const Eigen::Vector2f &p,
                                      const Eigen::Vector2f &a,
                                      const Eigen::Vector2f &b)
    {
        // Vector from start point to end point
        Eigen::Vector2f ab = b - a;

        // Vector from start point to pose
        Eigen::Vector2f ap = p - a;

        // Check if line segment has zero length
        double ab_length = ab.norm();
        if (ab_length < 1e-6)
        {
            // Return distance to either point if segment is degenerate
            return ap.norm();
        }

        // Calculate normalized projection
        double t = ap.dot(ab) / (ab_length * ab_length);

        // If projection is before start point
        if (t < 0.0)
        {
            return ap.norm();
        }
        // If projection is after end point
        else if (t > 1.0)
        {
            return (p - b).norm();
        }

        // Project point onto line segment
        Eigen::Vector2f projection = a + t * ab;

        // Return perpendicular distance
        return (p - projection).norm();
    }

    double dist(Eigen::Vector3f p1, Eigen::Vector3f p2)
    {
        return std::sqrt(
            std::pow(p1(0) - p2(0), 2) +
            std::pow(p1(1) - p2(1), 2));
    }

    double sign(double val)
    {
        return (0 < val) - (val < 0);
    }

    double ang_distance(double ang1, double ang2)
    {
        return normalize_angle(ang1 - ang2);
    }

    double calculate_heading(double x1, double y1, double x2, double y2)
    {
        double dx = x2 - x1;
        double dy = y2 - y1;

        return std::atan2(dy, dx);
    }

    double calculate_heading(Eigen::Vector2f p1, Eigen::Vector2f p2)
    {
        Eigen::Vector2f diff = p2 - p1;
        return std::atan2(diff(1), diff(0));
    }

    double calculate_heading(Eigen::Vector3f p1, Eigen::Vector3f p2)
    {
        Eigen::Vector3f diff = p2 - p1;
        return std::atan2(diff(1), diff(0));
    }

    Eigen::Vector2f get_point(int idx)
    {
        if (laserscan_vector.size() == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Vector is EMPTY!");
            return Eigen::Vector2f::Zero();
        }

        idx = std::fmod(idx + laserscan_vector.size(), laserscan_vector.size());

        double r = laserscan_vector[idx];
        double theta = angle_min + angle_increment * idx;

        Eigen::Vector2f point;
        point << r * std::cos(theta),
            r * std::sin(theta);

        return point;
    }

    Eigen::Vector3f pose2vec(geometry_msgs::msg::Pose pose)
    {
        Eigen::Vector3f v;
        v << pose.position.x,
            pose.position.y,
            tf2::getYaw(pose.orientation);
        return v;
    }

    Eigen::Vector2f midpoint(Eigen::Vector2f p1, Eigen::Vector2f p2)
    {
        Eigen::Vector2f p = (p1 + p2) / 2;
        return p;
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }

    Eigen::Vector3f forward(const Eigen::Vector3f &base, double distance)
    {
        Eigen::Vector3f p;
        p << std::cos(base(2)), std::sin(base(2)), 0.0;
        return base + distance * p;
    }

    void set_pid_ref(Eigen::Vector3f v)
    {
        pid_ref_msg.header.stamp = this->get_clock()->now();

        pid_ref_msg.pose.position.x = v(0);
        pid_ref_msg.pose.position.y = v(1);
        quat.setRPY(0, 0, v(2));
        tf2::convert(quat, pid_ref_msg.pose.orientation);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AlignmentNode>());
    rclcpp::shutdown();
    return 0;
}