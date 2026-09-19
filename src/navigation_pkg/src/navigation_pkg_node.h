#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace navigation_pkg
{

class AreaSelector : public rclcpp::Node
{
public:
    AreaSelector();

private:
    // --- callbacks ---
    void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

    // --- services ---
    void srvClearTrajectory(const std_srvs::srv::Trigger::Request::SharedPtr,
                            std_srvs::srv::Trigger::Response::SharedPtr resp);
    void srvClearStart(const std_srvs::srv::Trigger::Request::SharedPtr,
                       std_srvs::srv::Trigger::Response::SharedPtr resp);
    void srvClearAll(const std_srvs::srv::Trigger::Request::SharedPtr,
                     std_srvs::srv::Trigger::Response::SharedPtr resp);

    // --- helpers ---
    void buildArea();
    void publishArea();
    void publishStart();
    void publishTrajectory();
    void publishStartPose();
    std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a) const;
    bool isInsideArea(double x, double y) const;

    // --- params ---
    std::string frame_id_;
    double area_width_{20.0};
    double area_height_{20.0};
    double area_origin_x_{0.0};
    double area_origin_y_{0.0};
    double traj_min_dist_{0.05};    // м — минимальный шаг между точками траектории
    int    traj_max_points_{10000}; // максимум точек в буфере

    // --- state ---
    std::vector<geometry_msgs::msg::Point> area_corners_;

    geometry_msgs::msg::Point start_point_;
    geometry_msgs::msg::Quaternion start_orientation_;
    bool has_start_{false};

    std::deque<geometry_msgs::msg::Point> trajectory_;
    geometry_msgs::msg::Point last_odom_point_;
    bool has_last_odom_{false};

    // --- ROS ---
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr area_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr start_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_marker_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_trajectory_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_start_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_all_;
};

}  // namespace navigation_pkg
