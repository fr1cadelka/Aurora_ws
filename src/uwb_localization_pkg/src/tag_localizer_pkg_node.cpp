#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <deque>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nlink_parser2/msg/linktrack_nodeframe2.hpp"

static double normalize_angle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

class TagLocalizer : public rclcpp::Node
{
public:
    TagLocalizer() : Node("tag_localizer_pkg_node")
    {
        // ===== Параметры =====
        frame_id_map_   = declare_parameter<std::string>("frame_id_map",   "map");
        frame_id_odom_  = declare_parameter<std::string>("frame_id_odom",  "odom");
        frame_id_robot_ = declare_parameter<std::string>("frame_id_robot", "base_link");
        publish_tf_     = declare_parameter<bool>       ("publish_tf",     true);
        marker_scale_   = declare_parameter<double>     ("marker_scale",   0.5);
        marker_z_       = declare_parameter<double>     ("marker_z",       0.3);
        ema_alpha_      = declare_parameter<double>     ("ema_alpha",      0.3);

        kf_q_pos_       = declare_parameter<double>("kalman_q_pos", 0.05);
        kf_q_vel_       = declare_parameter<double>("kalman_q_vel", 0.5);
        kf_r_pos_       = declare_parameter<double>("kalman_r_pos", 1.0);

        heading_min_dist_ = declare_parameter<double>("heading_min_dist", 0.15);
        heading_alpha_    = declare_parameter<double>("heading_alpha",    0.3);

        yaw_comp_alpha_    = declare_parameter<double>("yaw_comp_alpha",    0.05);
        yaw_min_speed_     = declare_parameter<double>("yaw_min_speed",     0.2);
        yaw_update_min_dt_ = declare_parameter<double>("yaw_update_min_dt", 0.5);

        uwb_timeout_s_     = declare_parameter<double>("uwb_timeout_s", 1.0);

        lane_width_       = declare_parameter<double>("lane_width",       1.0);
        coverage_margin_  = declare_parameter<double>("coverage_margin",  0.5);
        coverage_exit_m_  = declare_parameter<double>("coverage_exit_m",  2.0);
        field_x_min_      = declare_parameter<double>("field_x_min",      0.0);
        field_x_max_      = declare_parameter<double>("field_x_max",      3.0);
        field_y_min_      = declare_parameter<double>("field_y_min",      0.0);
        field_y_max_      = declare_parameter<double>("field_y_max",      8.0);

        anchors_x_ = declare_parameter<std::vector<double>>("anchors_x", {0.0, 4.67, 4.67, 0.0});
        anchors_y_ = declare_parameter<std::vector<double>>("anchors_y", {0.0, 0.0, 10.10, 10.10});
        anchors_z_ = declare_parameter<std::vector<double>>("anchors_z", {1.70, 1.56, 1.82, 1.82});

        traversed_path_max_ = declare_parameter<int>("traversed_path_max", 5000);

        num_anchors_ = anchors_x_.size();

        RCLCPP_INFO(get_logger(), "=== Tag Localizer (UWB + VESC fusion) ===");
        RCLCPP_INFO(get_logger(), "field: [%.2f..%.2f] x [%.2f..%.2f] | lane=%.2f margin=%.2f exit=%.2f",
                    field_x_min_, field_x_max_, field_y_min_, field_y_max_,
                    lane_width_, coverage_margin_, coverage_exit_m_);
        for (size_t i = 0; i < num_anchors_; ++i)
            RCLCPP_INFO(get_logger(), "  A%zu: (%.2f, %.2f, %.2f)",
                        i, anchors_x_[i], anchors_y_[i], anchors_z_[i]);

        // ===== Подписки =====
        uwb_sub_ = create_subscription<nlink_parser2::msg::LinktrackNodeframe2>(
            "/nlink_linktrack_nodeframe2", rclcpp::SensorDataQoS(),
            std::bind(&TagLocalizer::uwb_callback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(20),
            std::bind(&TagLocalizer::odom_callback, this, std::placeholders::_1));

        clicked_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", rclcpp::QoS(10),
            std::bind(&TagLocalizer::clicked_callback, this, std::placeholders::_1));

        // ===== Publishers =====
        pose_pub_          = create_publisher<geometry_msgs::msg::PoseStamped>("/uwb/pose", 10);
        marker_pub_        = create_publisher<visualization_msgs::msg::MarkerArray>("/uwb/tag_marker", 10);
        traj_pub_          = create_publisher<nav_msgs::msg::Path>("/uwb/trajectory", 10);
        traversed_pub_     = create_publisher<nav_msgs::msg::Path>("/uwb/traversed_path", 10);
        points_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/uwb/clicked_points", 10);

        // ===== TF =====
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // ===== Сервисы =====
        build_cov_srv_ = create_service<std_srvs::srv::Empty>(
            "~/build_coverage_trajectory",
            [this](const std_srvs::srv::Empty::Request::SharedPtr,
                   const std_srvs::srv::Empty::Response::SharedPtr) {
                build_coverage_trajectory();
            });
        build_click_srv_ = create_service<std_srvs::srv::Empty>(
            "~/build_trajectory_from_clicks",
            [this](const std_srvs::srv::Empty::Request::SharedPtr,
                   const std_srvs::srv::Empty::Response::SharedPtr) {
                build_trajectory_from_clicks();
            });
        clear_pts_srv_ = create_service<std_srvs::srv::Empty>(
            "~/clear_points",
            [this](const std_srvs::srv::Empty::Request::SharedPtr,
                   const std_srvs::srv::Empty::Response::SharedPtr) {
                clicked_points_.clear();
                publish_clicked_markers();
                RCLCPP_WARN(get_logger(), "Clicked points cleared");
            });

        // ===== Состояние =====
        x_filt_ = 0.0;
        y_filt_ = 0.0;
        filter_init_ = false;

        kf_init_ = false;
        for (int i = 0; i < 4; ++i) kf_x_[i] = 0.0;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) kf_P_[i][j] = (i == j) ? 1.0 : 0.0;

        uwb_heading_init_ = false;
        uwb_heading_ = 0.0;
        last_uwb_pos_[0] = 0.0;
        last_uwb_pos_[1] = 0.0;
        last_uwb_heading_time_ = this->now();

        odom_received_ = false;
        odom_x_ = odom_y_ = odom_yaw_ = odom_v_ = 0.0;
        odom_time_ = this->now();

        yaw_offset_ = 0.0;
        yaw_fused_ = 0.0;
        yaw_fused_valid_ = false;
        last_yaw_update_time_ = this->now();

        last_uwb_time_ = this->now();
        last_uwb_set_  = false;

        ema_dist_.assign(num_anchors_, 0.0);
        ema_dist_init_.assign(num_anchors_, false);

        traversed_path_.header.frame_id = frame_id_map_;
        traversed_init_ = false;
        last_traversed_pos_[0] = 0.0;
        last_traversed_pos_[1] = 0.0;

        // ===== Таймеры =====
        tf_timer_ = create_wall_timer(
            std::chrono::milliseconds(40),
            std::bind(&TagLocalizer::publish_tf, this));

        republish_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
                if (!last_trajectory_.poses.empty()) {
                    last_trajectory_.header.stamp = this->now();
                    traj_pub_->publish(last_trajectory_);
                }
            });

        // ===== Автопостроение покрытия =====
        build_coverage_trajectory();

        RCLCPP_INFO(get_logger(), "Ready.");
    }

private:
    // ============================================================
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        odom_yaw_ = y;

        odom_v_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
        odom_time_ = this->now();
        odom_received_ = true;
    }

    // ============================================================
    // CLICKED POINT
    // ============================================================
    void clicked_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        clicked_points_.push_back({msg->point.x, msg->point.y});
        RCLCPP_INFO(get_logger(), "Click #%zu: (%.2f, %.2f)",
                    clicked_points_.size(), msg->point.x, msg->point.y);
        publish_clicked_markers();
    }

    void publish_clicked_markers()
    {
        visualization_msgs::msg::MarkerArray arr;

        visualization_msgs::msg::Marker del;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);
        points_marker_pub_->publish(arr);
        arr.markers.clear();

        for (size_t i = 0; i < clicked_points_.size(); ++i) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_map_;
            m.header.stamp = this->now();
            m.ns = "clicked_points";
            m.id = (int)i;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = clicked_points_[i].first;
            m.pose.position.y = clicked_points_[i].second;
            m.pose.position.z = 0.15;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = 0.3;
            m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 1.0;
            m.lifetime = rclcpp::Duration::from_seconds(0.0);
            arr.markers.push_back(m);

            visualization_msgs::msg::Marker t = m;
            t.ns = "clicked_points_labels";
            t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            t.pose.position.z = 0.7;
            t.scale.z = 0.4;
            t.color.r = t.color.g = t.color.b = 1.0;
            t.text = std::to_string(i + 1);
            arr.markers.push_back(t);
        }

        points_marker_pub_->publish(arr);
    }

    // ============================================================
    // BUILD COVERAGE TRAJECTORY (змейка через всё поле в одном направлении)
    // ============================================================
    void build_coverage_trajectory()
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = frame_id_map_;
        path.header.stamp = this->now();

        double margin = coverage_margin_;
        double exit_e = coverage_exit_m_;
        double x_start = field_x_min_ + margin;
        double x_end   = field_x_max_ - margin;
        double y_bot   = field_y_min_ + margin;
        double y_top   = field_y_max_ - margin;
        double y_top_e = field_y_max_ + exit_e;
        double y_bot_e = field_y_min_ - exit_e;

        int n_lanes = (int)std::floor((x_end - x_start) / lane_width_ + 1e-6) + 1;
        if (n_lanes < 1) n_lanes = 1;

        auto add_pt = [&](double x, double y) {
            geometry_msgs::msg::PoseStamped p;
            p.header = path.header;
            p.pose.position.x = x;
            p.pose.position.y = y;
            p.pose.position.z = 0.0;
            p.pose.orientation.w = 1.0;
            path.poses.push_back(p);
        };
        auto add_lerp = [&](double x0, double y0, double x1, double y1) {
            double d = std::hypot(x1 - x0, y1 - y0);
            int n = std::max(1, (int)std::ceil(d / 0.2));
            for (int k = 1; k <= n; ++k) {
                double t = (double)k / n;
                add_pt(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t);
            }
        };

        add_pt(x_start, y_bot);
        double px = x_start, py = y_bot;

        for (int i = 0; i < n_lanes; ++i) {
            double x_lane = x_start + i * lane_width_;

            // 1) Подъём вдоль дорожки
            add_lerp(px, py, x_lane, y_top);
            px = x_lane; py = y_top;

            // 2) Выезд за пределы поля
            add_lerp(px, py, x_lane, y_top_e);
            px = x_lane; py = y_top_e;

            if (i < n_lanes - 1) {
                double x_next = x_start + (i + 1) * lane_width_;
                double x_mid  = (x_lane + x_next) / 2.0;

                // 3) Сдвиг к середине (по выезду)
                add_lerp(px, py, x_mid, y_top_e);
                px = x_mid; py = y_top_e;

                // 4) Спуск по середине (между дорожками)
                add_lerp(px, py, x_mid, y_bot_e);
                px = x_mid; py = y_bot_e;

                // 5) Сдвиг к X следующей дорожки
                add_lerp(px, py, x_next, y_bot_e);
                px = x_next; py = y_bot_e;

                // 6) Подъём к началу следующей дорожки
                add_lerp(px, py, x_next, y_bot);
                px = x_next; py = y_bot;
            }
        }

        last_trajectory_ = path;
        traj_pub_->publish(path);
        RCLCPP_INFO(get_logger(),
                    "Coverage trajectory: %zu points, %d lanes",
                    path.poses.size(), n_lanes);
    }

    // ============================================================
    // BUILD TRAJECTORY FROM CLICKS
    // ============================================================
    void build_trajectory_from_clicks()
    {
        if (clicked_points_.size() < 2) {
            RCLCPP_ERROR(get_logger(), "Need >=2 clicks, have %zu", clicked_points_.size());
            return;
        }
        nav_msgs::msg::Path path;
        path.header.frame_id = frame_id_map_;
        path.header.stamp = this->now();

        auto add_pt = [&](double x, double y) {
            geometry_msgs::msg::PoseStamped p;
            p.header = path.header;
            p.pose.position.x = x;
            p.pose.position.y = y;
            p.pose.position.z = 0.0;
            p.pose.orientation.w = 1.0;
            path.poses.push_back(p);
        };
        for (size_t i = 0; i + 1 < clicked_points_.size(); ++i) {
            double x0 = clicked_points_[i].first,   y0 = clicked_points_[i].second;
            double x1 = clicked_points_[i+1].first, y1 = clicked_points_[i+1].second;
            double d = std::hypot(x1 - x0, y1 - y0);
            int n = std::max(1, (int)std::ceil(d / 0.2));
            for (int k = (i == 0 ? 0 : 1); k <= n; ++k) {
                double t = (double)k / n;
                add_pt(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t);
            }
        }
        last_trajectory_ = path;
        traj_pub_->publish(path);
        RCLCPP_INFO(get_logger(), "Click trajectory: %zu points from %zu clicks",
                    path.poses.size(), clicked_points_.size());
    }

    // ============================================================
    // UWB CALLBACK
    // ============================================================
    void uwb_callback(const nlink_parser2::msg::LinktrackNodeframe2::SharedPtr msg)
    {
        rclcpp::Time now = this->now();
        double dt_uwb = 0.0;
        if (last_uwb_set_) {
            dt_uwb = (now - last_uwb_time_).seconds();
            if (dt_uwb > 0.5) dt_uwb = 0.5;
        }
        last_uwb_time_ = now;
        last_uwb_set_  = true;

        std::vector<double> dist(num_anchors_, -1.0);
        for (const auto &node : msg->nodes) {
            uint8_t id = node.id;
            if (id < num_anchors_) dist[id] = node.dis;
        }

        for (size_t i = 0; i < num_anchors_; ++i) {
            if (dist[i] > 0.0) {
                if (!ema_dist_init_[i]) {
                    ema_dist_[i] = dist[i];
                    ema_dist_init_[i] = true;
                } else {
                    ema_dist_[i] = ema_alpha_ * dist[i] + (1.0 - ema_alpha_) * ema_dist_[i];
                }
            }
        }

        int valid = 0;
        for (double d : ema_dist_) if (d > 0.0) valid++;
        if (valid < 3) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Not enough anchors: %d/4", valid);
            return;
        }

        double x_raw, y_raw;
        if (!trilaterate(x_raw, y_raw)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Trilateration failed");
            return;
        }

        kalman_update(x_raw, y_raw, dt_uwb > 0.0 ? dt_uwb : 0.04);
        x_filt_ = kf_x_[0];
        y_filt_ = kf_x_[1];
        filter_init_ = true;

        // UWB heading from motion
        if (!uwb_heading_init_) {
            last_uwb_pos_[0] = x_filt_;
            last_uwb_pos_[1] = y_filt_;
            uwb_heading_init_ = true;
        } else {
            double dx = x_filt_ - last_uwb_pos_[0];
            double dy = y_filt_ - last_uwb_pos_[1];
            double dist_moved = std::hypot(dx, dy);
            double dt_h = (now - last_uwb_heading_time_).seconds();
            if (dist_moved > heading_min_dist_ && dt_h > 0.3) {
                double new_heading = std::atan2(dy, dx);
                if (!yaw_fused_valid_) {
                    uwb_heading_ = new_heading;
                } else {
                    double diff = normalize_angle(new_heading - uwb_heading_);
                    uwb_heading_ = normalize_angle(uwb_heading_ + heading_alpha_ * diff);
                }
                last_uwb_pos_[0] = x_filt_;
                last_uwb_pos_[1] = y_filt_;
                last_uwb_heading_time_ = now;
            }
        }

        // Yaw fusion
        if (odom_received_) {
            bool can_update = (odom_v_ > yaw_min_speed_) &&
                              ((now - last_yaw_update_time_).seconds() > yaw_update_min_dt_) &&
                              uwb_heading_init_;
            if (can_update) {
                double offset_meas = normalize_angle(uwb_heading_ - odom_yaw_);
                double offset_err  = normalize_angle(offset_meas - yaw_offset_);
                yaw_offset_ = normalize_angle(yaw_offset_ + yaw_comp_alpha_ * offset_err);
                last_yaw_update_time_ = now;
            }
            yaw_fused_ = normalize_angle(odom_yaw_ + yaw_offset_);
            yaw_fused_valid_ = true;
        } else if (!yaw_fused_valid_) {
            yaw_fused_ = uwb_heading_;
        }

        double yaw = yaw_fused_;
        double z_robot = 0.0;

        // Traversed path
        if (!traversed_init_) {
            traversed_init_ = true;
            last_traversed_pos_[0] = x_filt_;
            last_traversed_pos_[1] = y_filt_;
            append_to_traversed(x_filt_, y_filt_);
        } else {
            double dx = x_filt_ - last_traversed_pos_[0];
            double dy = y_filt_ - last_traversed_pos_[1];
            if (std::hypot(dx, dy) > 0.05) {
                append_to_traversed(x_filt_, y_filt_);
                last_traversed_pos_[0] = x_filt_;
                last_traversed_pos_[1] = y_filt_;
            }
        }
        traversed_pub_->publish(traversed_path_);

        // /uwb/pose
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = frame_id_map_;
        pose.pose.position.x = x_filt_;
        pose.pose.position.y = y_filt_;
        pose.pose.position.z = z_robot;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        pose_pub_->publish(pose);

        publish_marker(x_filt_, y_filt_, z_robot, yaw);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "POS: x=%.2f y=%.2f yaw=%.2f | odom_yaw=%.2f | uwb_yaw=%.2f | off=%.2f",
                             x_filt_, y_filt_, yaw, odom_yaw_, uwb_heading_, yaw_offset_);
    }

    void append_to_traversed(double x, double y)
    {
        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = this->now();
        p.header.frame_id = frame_id_map_;
        p.pose.position.x = x;
        p.pose.position.y = y;
        p.pose.position.z = 0.0;
        p.pose.orientation.w = 1.0;
        traversed_path_.poses.push_back(p);
        traversed_path_.header.stamp = p.header.stamp;
        while (traversed_path_.poses.size() > (size_t)traversed_path_max_) {
            traversed_path_.poses.erase(traversed_path_.poses.begin());
        }
    }

    // ============================================================
    void publish_tf()
    {
        if (!publish_tf_ || !yaw_fused_valid_ || !odom_received_) return;

        double yaw_off = yaw_offset_;
        double tx = x_filt_ - std::cos(yaw_off) * odom_x_ + std::sin(yaw_off) * odom_y_;
        double ty = y_filt_ - std::sin(yaw_off) * odom_x_ - std::cos(yaw_off) * odom_y_;

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = this->now();
        t.header.frame_id = frame_id_map_;
        t.child_frame_id  = frame_id_odom_;
        t.transform.translation.x = tx;
        t.transform.translation.y = ty;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_off);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }

    // ============================================================
    void kalman_update(double z_x, double z_y, double dt)
    {
        if (!kf_init_) {
            kf_x_[0] = z_x; kf_x_[1] = z_y;
            kf_x_[2] = 0.0; kf_x_[3] = 0.0;
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    kf_P_[i][j] = (i == j) ? 1.0 : 0.0;
            kf_init_ = true;
            return;
        }

        double px = kf_x_[0] + kf_x_[2] * dt;
        double py = kf_x_[1] + kf_x_[3] * dt;

        double F[4][4] = {{1,0,dt,0},{0,1,0,dt},{0,0,1,0},{0,0,0,1}};
        double FP[4][4] = {};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                for (int k = 0; k < 4; ++k) FP[i][j] += F[i][k] * kf_P_[k][j];

        double FPFt[4][4] = {};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                for (int k = 0; k < 4; ++k) FPFt[i][j] += FP[i][k] * F[j][k];

        double Q[4][4] = {{kf_q_pos_,0,0,0},{0,kf_q_pos_,0,0},
                          {0,0,kf_q_vel_,0},{0,0,0,kf_q_vel_}};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) kf_P_[i][j] = FPFt[i][j] + Q[i][j];

        double H[2][4] = {{1,0,0,0},{0,1,0,0}};
        double R[2][2] = {{kf_r_pos_,0},{0,kf_r_pos_}};
        double y_[2] = {z_x - px, z_y - py};

        double HP[2][4] = {};
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 4; ++j)
                for (int k = 0; k < 4; ++k) HP[i][j] += H[i][k] * kf_P_[k][j];

        double S[2][2] = {};
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j) {
                S[i][j] = R[i][j];
                for (int k = 0; k < 4; ++k) S[i][j] += HP[i][k] * H[j][k];
            }

        double detS = S[0][0]*S[1][1] - S[0][1]*S[1][0];
        if (std::abs(detS) < 1e-12) return;
        double Sinv[2][2] = {{S[1][1]/detS, -S[0][1]/detS},{-S[1][0]/detS, S[0][0]/detS}};

        double PHt[4][2] = {};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 4; ++k) PHt[i][j] += kf_P_[i][k] * H[j][k];

        double K[4][2] = {};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 2; ++k) K[i][j] += PHt[i][k] * Sinv[k][j];

        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 2; ++j) kf_x_[i] += K[i][j] * y_[j];

        double KH[4][4] = {};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                for (int k = 0; k < 2; ++k) KH[i][j] += K[i][k] * H[k][j];

        double I_KH[4][4] = {};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) I_KH[i][j] = ((i==j)?1.0:0.0) - KH[i][j];

        double newP[4][4] = {};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                for (int k = 0; k < 4; ++k) newP[i][j] += I_KH[i][k] * kf_P_[k][j];

        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) kf_P_[i][j] = newP[i][j];
    }

    // ============================================================
    bool trilaterate(double &x_out, double &y_out)
    {
        int ref = -1;
        for (size_t i = 0; i < num_anchors_; ++i)
            if (ema_dist_[i] > 0.0) { ref = (int)i; break; }
        if (ref < 0) return false;

        double x0 = anchors_x_[ref];
        double y0 = anchors_y_[ref];
        double d0 = ema_dist_[ref];

        std::vector<double> A11, A12, b1;
        for (size_t i = 0; i < num_anchors_; ++i) {
            if ((int)i == ref) continue;
            if (ema_dist_[i] <= 0.0) continue;
            double xi = anchors_x_[i], yi = anchors_y_[i], di = ema_dist_[i];
            A11.push_back(2.0 * (xi - x0));
            A12.push_back(2.0 * (yi - y0));
            b1.push_back((d0*d0 - di*di) + (xi*xi - x0*x0) + (yi*yi - y0*y0));
        }

        size_t N = b1.size();
        if (N < 2) return false;

        double a11=0,a12=0,a22=0,bx=0,by=0;
        for (size_t i = 0; i < N; ++i) {
            a11 += A11[i]*A11[i];
            a12 += A11[i]*A12[i];
            a22 += A12[i]*A12[i];
            bx  += A11[i]*b1[i];
            by  += A12[i]*b1[i];
        }

        double det = a11*a22 - a12*a12;
        if (std::abs(det) < 1e-9) return false;
        x_out = (bx*a22 - by*a12) / det;
        y_out = (a11*by - a12*bx) / det;
        return true;
    }

    // ============================================================
    void publish_marker(double x, double y, double z, double yaw)
    {
        visualization_msgs::msg::MarkerArray arr;

        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = frame_id_map_;
        arrow.header.stamp = this->now();
        arrow.ns = "tag"; arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose.position.x = x;
        arrow.pose.position.y = y;
        arrow.pose.position.z = z + marker_z_;
        tf2::Quaternion q; q.setRPY(0.0, 0.0, yaw);
        arrow.pose.orientation.x = q.x();
        arrow.pose.orientation.y = q.y();
        arrow.pose.orientation.z = q.z();
        arrow.pose.orientation.w = q.w();
        arrow.scale.x = marker_scale_ * 2.0;
        arrow.scale.y = marker_scale_ * 0.4;
        arrow.scale.z = marker_scale_ * 0.4;
        arrow.color.r = 0.0; arrow.color.g = 1.0; arrow.color.b = 0.0; arrow.color.a = 1.0;
        arrow.lifetime = rclcpp::Duration::from_seconds(0.5);
        arr.markers.push_back(arrow);

        visualization_msgs::msg::Marker dot;
        dot.header.frame_id = frame_id_map_;
        dot.header.stamp = this->now();
        dot.ns = "tag_dot"; dot.id = 1;
        dot.type = visualization_msgs::msg::Marker::SPHERE;
        dot.action = visualization_msgs::msg::Marker::ADD;
        dot.pose.position.x = x;
        dot.pose.position.y = y;
        dot.pose.position.z = z + marker_z_;
        dot.pose.orientation.w = 1.0;
        dot.scale.x = dot.scale.y = dot.scale.z = marker_scale_ * 0.5;
        dot.color.r = 1.0; dot.color.g = 1.0; dot.color.b = 0.0; dot.color.a = 1.0;
        dot.lifetime = rclcpp::Duration::from_seconds(0.5);
        arr.markers.push_back(dot);

        marker_pub_->publish(arr);
    }

    // ============================================================
    // MEMBERS
    // ============================================================
    rclcpp::Subscription<nlink_parser2::msg::LinktrackNodeframe2>::SharedPtr uwb_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traversed_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr points_marker_pub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr build_cov_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr build_click_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_pts_srv_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr republish_timer_;

    std::string frame_id_map_, frame_id_odom_, frame_id_robot_;
    bool publish_tf_;
    double marker_scale_, marker_z_, ema_alpha_;
    double kf_q_pos_, kf_q_vel_, kf_r_pos_;
    double heading_min_dist_, heading_alpha_;
    double yaw_comp_alpha_, yaw_min_speed_, yaw_update_min_dt_;
    double uwb_timeout_s_;
    double lane_width_, coverage_margin_, coverage_exit_m_;
    double field_x_min_, field_x_max_, field_y_min_, field_y_max_;
    std::vector<double> anchors_x_, anchors_y_, anchors_z_;
    size_t num_anchors_;
    int traversed_path_max_;

    std::vector<double> ema_dist_;
    std::vector<bool>   ema_dist_init_;

    bool kf_init_;
    double kf_x_[4];
    double kf_P_[4][4];

    bool   uwb_heading_init_;
    double uwb_heading_;
    double last_uwb_pos_[2];
    rclcpp::Time last_uwb_heading_time_;

    bool   odom_received_;
    double odom_x_, odom_y_, odom_yaw_, odom_v_;
    rclcpp::Time odom_time_;

    double yaw_offset_;
    double yaw_fused_;
    bool   yaw_fused_valid_;
    rclcpp::Time last_yaw_update_time_;

    rclcpp::Time last_uwb_time_;
    bool last_uwb_set_;

    double x_filt_, y_filt_;
    bool   filter_init_;

    nav_msgs::msg::Path traversed_path_;
    bool traversed_init_;
    double last_traversed_pos_[2];

    nav_msgs::msg::Path last_trajectory_;
    std::vector<std::pair<double,double>> clicked_points_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TagLocalizer>());
    rclcpp::shutdown();
    return 0;
}
