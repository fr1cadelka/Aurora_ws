#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nlink_parser2/msg/linktrack_nodeframe2.hpp"

class TagLocalizer : public rclcpp::Node
{
public:
    TagLocalizer() : Node("tag_localizer_pkg_node")
    {
        // === Параметры ===
        this->declare_parameter<std::string>("frame_id_map",     "map");
        this->declare_parameter<std::string>("frame_id_robot",   "base_link");
        this->declare_parameter<bool>       ("publish_tf",       true);
        this->declare_parameter<double>     ("marker_scale",     0.5);
        this->declare_parameter<double>     ("marker_z",         0.3);
        this->declare_parameter<double>     ("ema_alpha",        0.3);

        // Фильтр Калмана
        this->declare_parameter<double>("kalman_q_pos",    0.05);
        this->declare_parameter<double>("kalman_q_vel",    0.5);
        this->declare_parameter<double>("kalman_r_pos",    1.0);

        // Мёртвая зона — не обновлять позицию при малых сдвигах
        this->declare_parameter<double>("deadband_m",      0.05);

        // Ориентация
        this->declare_parameter<double>("heading_min_dist", 0.10); // 10 см
        this->declare_parameter<double>("heading_alpha",    0.3);

        // Параметры траектории
        this->declare_parameter<double>("lane_width",        1.0);
        this->declare_parameter<double>("field_margin",      1.0);
        this->declare_parameter<double>("field_x_min",       0.0);
        this->declare_parameter<double>("field_x_max",       3.0);
        this->declare_parameter<double>("field_y_min",       0.0);
        this->declare_parameter<double>("field_y_max",       8.0);

        // Координаты якорей
        this->declare_parameter<std::vector<double>>("anchors_x", {0.0, 3.0, 3.0, 0.0});
        this->declare_parameter<std::vector<double>>("anchors_y", {0.0, 0.0, 8.0, 8.0});
        this->declare_parameter<std::vector<double>>("anchors_z", {0.5, 0.5, 0.5, 0.5});

        // === Считываем ===
        this->get_parameter("frame_id_map",     frame_id_map_);
        this->get_parameter("frame_id_robot",   frame_id_robot_);
        this->get_parameter("publish_tf",       publish_tf_);
        this->get_parameter("marker_scale",     marker_scale_);
        this->get_parameter("marker_z",         marker_z_);
        this->get_parameter("ema_alpha",        ema_alpha_);
        this->get_parameter("kalman_q_pos",     kf_q_pos_);
        this->get_parameter("kalman_q_vel",     kf_q_vel_);
        this->get_parameter("kalman_r_pos",     kf_r_pos_);
        this->get_parameter("deadband_m",       deadband_);
        this->get_parameter("heading_min_dist", heading_min_dist_);
        this->get_parameter("heading_alpha",    heading_alpha_);
        this->get_parameter("lane_width",       lane_width_);
        this->get_parameter("field_margin",     field_margin_);
        this->get_parameter("field_x_min",      field_x_min_);
        this->get_parameter("field_x_max",      field_x_max_);
        this->get_parameter("field_y_min",      field_y_min_);
        this->get_parameter("field_y_max",      field_y_max_);
        this->get_parameter("anchors_x",        anchors_x_);
        this->get_parameter("anchors_y",        anchors_y_);
        this->get_parameter("anchors_z",        anchors_z_);

        num_anchors_ = anchors_x_.size();

        RCLCPP_INFO(this->get_logger(), "=== Tag Localizer (KF + trajectory) ===");
        for (size_t i = 0; i < num_anchors_; ++i) {
            RCLCPP_INFO(this->get_logger(), "  A%zu: (%.2f, %.2f, %.2f)",
                        i, anchors_x_[i], anchors_y_[i], anchors_z_[i]);
        }

        // === Подписки ===
        uwb_sub_ = this->create_subscription<nlink_parser2::msg::LinktrackNodeframe2>(
            "/nlink_linktrack_nodeframe2",
            rclcpp::SensorDataQoS(),
            std::bind(&TagLocalizer::uwb_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10),
            std::bind(&TagLocalizer::odom_callback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", rclcpp::QoS(10),
            std::bind(&TagLocalizer::goal_callback, this, std::placeholders::_1));

        // === Publishers ===
        pose_pub_       = this->create_publisher<geometry_msgs::msg::PoseStamped>("/uwb/pose", 10);
        marker_pub_     = this->create_publisher<visualization_msgs::msg::MarkerArray>("/uwb/tag_marker", 10);
        traj_pub_       = this->create_publisher<nav_msgs::msg::Path>("/uwb/trajectory", 10);

        // === TF ===
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // === Состояние ===
        last_yaw_       = 0.0;
        x_filt_         = 0.0;
        y_filt_         = 0.0;
        filter_init_    = false;

        // Калман: состояние [x, y, vx, vy]
        kf_init_        = false;
        for (int i = 0; i < 4; ++i) kf_x_[i] = 0.0;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) kf_P_[i][j] = (i == j) ? 1.0 : 0.0;

        // Ориентация
        heading_init_   = false;
        heading_        = 0.0;
        last_pos_for_heading_[0] = 0.0;
        last_pos_for_heading_[1] = 0.0;

        // Последний timestamp UWB
        last_uwb_time_  = this->now();
        last_uwb_set_   = false;

        // EMA дистанций
        ema_dist_.assign(num_anchors_, 0.0);
        ema_dist_init_.assign(num_anchors_, false);

        RCLCPP_INFO(this->get_logger(), "Ready. Waiting for UWB data...");
    }

private:
    // ==================== KALMAN FILTER ====================
    // Простой 2D-фильтр Калмана с моделью постоянной скорости
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

        // Predict: x = F*x
        double px = kf_x_[0] + kf_x_[2] * dt;
        double py = kf_x_[1] + kf_x_[3] * dt;
        double pvx = kf_x_[2];
        double pvy = kf_x_[3];

        // P = F*P*F^T + Q
        double F[4][4] = {
            {1, 0, dt, 0},
            {0, 1, 0, dt},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        };
        double FP[4][4] = {};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                for (int k = 0; k < 4; ++k)
                    FP[i][j] += F[i][k] * kf_P_[k][j];

        double FPFt[4][4] = {};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                for (int k = 0; k < 4; ++k)
                    FPFt[i][j] += FP[i][k] * F[j][k];

        double Q[4][4] = {
            {kf_q_pos_, 0, 0, 0},
            {0, kf_q_pos_, 0, 0},
            {0, 0, kf_q_vel_, 0},
            {0, 0, 0, kf_q_vel_}
        };
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                kf_P_[i][j] = FPFt[i][j] + Q[i][j];

        // Update
        double H[2][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}};
        double R[2][2] = {{kf_r_pos_, 0}, {0, kf_r_pos_}};

        double y[2] = {z_x - px, z_y - py};

        double HP[2][4] = {};
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 4; ++j)
                for (int k = 0; k < 4; ++k)
                    HP[i][j] += H[i][k] * kf_P_[k][j];

        double S[2][2] = {};
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j) {
                S[i][j] = R[i][j];
                for (int k = 0; k < 4; ++k) S[i][j] += HP[i][k] * H[j][k];
            }

        double detS = S[0][0]*S[1][1] - S[0][1]*S[1][0];
        if (std::abs(detS) < 1e-12) return;

        double Sinv[2][2] = {
            { S[1][1] / detS, -S[0][1] / detS},
            {-S[1][0] / detS,  S[0][0] / detS}
        };

        double PHt[4][2] = {};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 4; ++k)
                    PHt[i][j] += kf_P_[i][k] * H[j][k];

        double K[4][2] = {};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 2; ++k)
                    K[i][j] += PHt[i][k] * Sinv[k][j];

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 2; ++j) kf_x_[i] += K[i][j] * y[j];
        }

        // P = (I - K*H) * P
        double KH[4][4] = {};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                for (int k = 0; k < 2; ++k)
                    KH[i][j] += K[i][k] * H[k][j];

        double I_KH[4][4] = {};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                I_KH[i][j] = ((i == j) ? 1.0 : 0.0) - KH[i][j];

        double newP[4][4] = {};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                for (int k = 0; k < 4; ++k)
                    newP[i][j] += I_KH[i][k] * kf_P_[k][j];
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) kf_P_[i][j] = newP[i][j];
    }

    // ==================== UWB CALLBACK ====================
    void uwb_callback(const nlink_parser2::msg::LinktrackNodeframe2::SharedPtr msg)
    {
        // Проверяем таймаут
        rclcpp::Time now = this->now();
        double dt = 0.0;
        if (last_uwb_set_) {
            dt = (now - last_uwb_time_).seconds();
            if (dt > 0.5) dt = 0.5;
        }
        last_uwb_time_ = now;
        last_uwb_set_  = true;

        // === 1. Считываем дистанции ===
        std::vector<double> dist(num_anchors_, -1.0);
        for (const auto &node : msg->nodes) {
            uint8_t id = node.id;
            if (id < num_anchors_) {
                dist[id] = node.dis;
            }
        }

        // === 2. EMA фильтр дистанций ===
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

        // === 3. Проверка валидности ===
        int valid = 0;
        for (double d : ema_dist_) if (d > 0.0) valid++;
        if (valid < 3) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Not enough anchors: %d/4", valid);
            return;
        }

        // === 4. Трилатерация ===
        double x_raw, y_raw;
        if (!trilaterate(x_raw, y_raw)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Trilateration failed");
            return;
        }

        // === 5. Фильтр Калмана ===
        kalman_update(x_raw, y_raw, dt > 0.0 ? dt : 0.04);

        double x_kf = kf_x_[0];
        double y_kf = kf_x_[1];

        // === 6. Мёртвая зона ===
        if (!filter_init_) {
            x_filt_ = x_kf;
            y_filt_ = y_kf;
            filter_init_ = true;
        } else {
            double dx = x_kf - x_filt_;
            double dy = y_kf - y_filt_;
            if (std::sqrt(dx*dx + dy*dy) > deadband_) {
                x_filt_ = x_kf;
                y_filt_ = y_kf;
            }
        }

        // === 7. Ориентация по направлению движения ===
        if (!heading_init_) {
            last_pos_for_heading_[0] = x_filt_;
            last_pos_for_heading_[1] = y_filt_;
            heading_init_ = true;
        } else {
            double dx = x_filt_ - last_pos_for_heading_[0];
            double dy = y_filt_ - last_pos_for_heading_[1];
            double dist = std::sqrt(dx*dx + dy*dy);
            if (dist > heading_min_dist_) {
                double new_heading = std::atan2(dy, dx);
                // Сглаживание угла
                double diff = new_heading - heading_;
                while (diff >  M_PI) diff -= 2*M_PI;
                while (diff < -M_PI) diff += 2*M_PI;
                heading_ += heading_alpha_ * diff;
                last_pos_for_heading_[0] = x_filt_;
                last_pos_for_heading_[1] = y_filt_;
            }
        }

        double z = anchors_z_[0];
        double yaw = heading_;

        // === 8. TF ===
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = frame_id_map_;
            t.child_frame_id  = frame_id_robot_;
            t.transform.translation.x = x_filt_;
            t.transform.translation.y = y_filt_;
            t.transform.translation.z = z;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(t);
        }

        // === 9. PoseStamped ===
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = frame_id_map_;
        pose.pose.position.x = x_filt_;
        pose.pose.position.y = y_filt_;
        pose.pose.position.z = z;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        pose_pub_->publish(pose);

        // === 10. Маркер ===
        publish_marker(x_filt_, y_filt_, z, yaw);

        // === 11. Диагностика ===
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "POS: x=%.2f y=%.2f yaw=%.2f | dist=[%.2f %.2f %.2f %.2f]",
                             x_filt_, y_filt_, yaw,
                             ema_dist_[0], ema_dist_[1], ema_dist_[2], ema_dist_[3]);
    }

    // ==================== ТРИЛАТЕРАЦИЯ ====================
    bool trilaterate(double &x_out, double &y_out)
    {
        int ref = -1;
        for (size_t i = 0; i < num_anchors_; ++i) {
            if (ema_dist_[i] > 0.0) { ref = static_cast<int>(i); break; }
        }
        if (ref < 0) return false;

        double x0 = anchors_x_[ref];
        double y0 = anchors_y_[ref];
        double d0 = ema_dist_[ref];

        std::vector<double> A11, A12, b1;
        for (size_t i = 0; i < num_anchors_; ++i) {
            if (static_cast<int>(i) == ref) continue;
            if (ema_dist_[i] <= 0.0) continue;

            double xi = anchors_x_[i];
            double yi = anchors_y_[i];
            double di = ema_dist_[i];

            A11.push_back(2.0 * (xi - x0));
            A12.push_back(2.0 * (yi - y0));
            b1.push_back((d0*d0 - di*di) + (xi*xi - x0*x0) + (yi*yi - y0*y0));
        }

        size_t N = b1.size();
        if (N < 2) return false;

        double a11 = 0, a12 = 0, a22 = 0, b_x = 0, b_y = 0;
        for (size_t i = 0; i < N; ++i) {
            a11 += A11[i]*A11[i];
            a12 += A11[i]*A12[i];
            a22 += A12[i]*A12[i];
            b_x += A11[i]*b1[i];
            b_y += A12[i]*b1[i];
        }

        double det = a11*a22 - a12*a12;
        if (std::abs(det) < 1e-9) return false;

        x_out = (b_x*a22 - b_y*a12) / det;
        y_out = (a11*b_y - a12*b_x) / det;
        return true;
    }

    // ==================== GOAL CALLBACK — построение траектории ====================
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
    {
        RCLCPP_INFO(this->get_logger(), "GOAL: (%.2f, %.2f)",
                    goal->pose.position.x, goal->pose.position.y);
        build_trajectory(goal->pose.position.x, goal->pose.position.y);
    }

    void build_trajectory(double gx, double gy)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = frame_id_map_;
        path.header.stamp = this->now();

        // Змейка: идём по X, шагаем по Y с lane_width
        double x1 = field_x_min_ + field_margin_;
        double x2 = field_x_max_ - field_margin_;
        double y1 = field_y_min_ + field_margin_;
        double y2 = field_y_max_ - field_margin_;

        // Начинаем с ближайшего края к цели
        double y_start = y1;
        double y_end   = y2;

        double y = y_start;
        bool left_to_right = true;

        while (y <= y_end) {
            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = frame_id_map_;
            p.header.stamp = this->now();
            p.pose.position.z = 0.0;

            if (left_to_right) {
                p.pose.position.x = x2;
                p.pose.position.y = y;
                path.poses.push_back(p);

                p.pose.position.x = x1;
                path.poses.push_back(p);
            } else {
                p.pose.position.x = x1;
                p.pose.position.y = y;
                path.poses.push_back(p);

                p.pose.position.x = x2;
                path.poses.push_back(p);
            }

            left_to_right = !left_to_right;
            y += lane_width_;
        }

        traj_pub_->publish(path);
        RCLCPP_INFO(this->get_logger(), "Trajectory published: %zu points", path.poses.size());
    }

    // ==================== ODOM ====================
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        last_yaw_ = y;
    }

    // ==================== МАРКЕР ====================
    void publish_marker(double x, double y, double z, double yaw)
    {
        visualization_msgs::msg::MarkerArray arr;

        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = frame_id_map_;
        arrow.header.stamp = this->now();
        arrow.ns = "tag";
        arrow.id = 0;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.pose.position.x = x;
        arrow.pose.position.y = y;
        arrow.pose.position.z = z + marker_z_;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
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
        dot.ns = "tag_dot";
        dot.id = 1;
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

    // ==================== MEMBERS ====================
    rclcpp::Subscription<nlink_parser2::msg::LinktrackNodeframe2>::SharedPtr uwb_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string frame_id_map_;
    std::string frame_id_robot_;
    bool publish_tf_;
    double marker_scale_;
    double marker_z_;
    double ema_alpha_;

    double kf_q_pos_, kf_q_vel_, kf_r_pos_;
    double deadband_;
    double heading_min_dist_, heading_alpha_;

    double lane_width_, field_margin_;
    double field_x_min_, field_x_max_;
    double field_y_min_, field_y_max_;

    std::vector<double> anchors_x_, anchors_y_, anchors_z_;
    size_t num_anchors_;

    std::vector<double> ema_dist_;
    std::vector<bool>   ema_dist_init_;

    // KF
    bool   kf_init_;
    double kf_x_[4];          // [x, y, vx, vy]
    double kf_P_[4][4];

    // Heading
    bool   heading_init_;
    double heading_;
    double last_pos_for_heading_[2];

    // Позиция (фильтрованная)
    double x_filt_, y_filt_;
    bool   filter_init_;

    // Timeout
    rclcpp::Time last_uwb_time_;
    bool last_uwb_set_;

    double last_yaw_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TagLocalizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
