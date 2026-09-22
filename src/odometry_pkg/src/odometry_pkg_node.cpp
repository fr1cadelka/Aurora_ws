#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <memory>

static double normalize_angle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

class OdometryPkgNode : public rclcpp::Node
{
public:
    OdometryPkgNode() : Node("odometry_pkg_node")
    {
        // === Параметры ===
        wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.1);
        wheel_base_   = this->declare_parameter<double>("wheel_base",   0.5);
        erpm_to_rads_ = this->declare_parameter<double>("erpm_to_rads", 0.001383);
        publish_tf_   = this->declare_parameter<bool>  ("publish_tf",   true);

        RCLCPP_INFO(this->get_logger(), "=== Odometry Node (VESC-only) ===");
        RCLCPP_INFO(this->get_logger(), "wheel_radius: %.4f m", wheel_radius_);
        RCLCPP_INFO(this->get_logger(), "wheel_base:   %.4f m", wheel_base_);
        RCLCPP_INFO(this->get_logger(), "erpm_to_rads: %.6f",   erpm_to_rads_);
        RCLCPP_INFO(this->get_logger(), "publish_tf:   %s",     publish_tf_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "=================================");

        // === Состояние ===
        x_ = 0.0; y_ = 0.0; theta_ = 0.0; distance_ = 0.0;
        last_time_ = this->now();
        left_rpm_ = 0.0; right_rpm_ = 0.0;
        velocity_received_ = false;

        // === Подписчики VESC ===
        left_state_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
            "/left/sensors/core", rclcpp::QoS(10),
            [this](const vesc_msgs::msg::VescStateStamped::SharedPtr msg) {
                left_rpm_ = msg->state.speed;
                velocity_received_ = true;
            });

        right_state_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
            "/right/sensors/core", rclcpp::QoS(10),
            [this](const vesc_msgs::msg::VescStateStamped::SharedPtr msg) {
                right_rpm_ = msg->state.speed;
            });

        // === Издатель /odom ===
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // === TF broadcaster (одноразово — только если включено) ===
        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        // === Таймер 50 Гц ===
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&OdometryPkgNode::publish_odometry, this));

        // === Сервис сброса ===
        reset_service_ = this->create_service<std_srvs::srv::Empty>(
            "~/reset_odometry",
            [this](const std_srvs::srv::Empty::Request::SharedPtr,
                   const std_srvs::srv::Empty::Response::SharedPtr) {
                x_ = 0.0; y_ = 0.0; theta_ = 0.0; distance_ = 0.0;
                last_time_ = this->now();
                RCLCPP_WARN(this->get_logger(), "========== ODOMETRY RESET ==========");
            });

        RCLCPP_INFO(this->get_logger(),
                    "Subscribed: /left/sensors/core, /right/sensors/core");
        RCLCPP_INFO(this->get_logger(),
                    "Publishing: /odom%s", publish_tf_ ? ", TF odom->base_link" : "");
        RCLCPP_INFO(this->get_logger(),
                    "Service: ~/reset_odometry");
    }

private:
    void publish_odometry()
    {
        if (!velocity_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for VESC data...");
            return;
        }

        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt > 0.1) dt = 0.1;
        if (dt <= 0.0) return;

        // === Скорости колёс ===
        double left_rads  = left_rpm_  * erpm_to_rads_;
        double right_rads = right_rpm_ * erpm_to_rads_;
        double v_left     = left_rads  * wheel_radius_;
        double v_right    = right_rads * wheel_radius_;

        // === Дифференциальная кинематика ===
        double v = (v_left + v_right) / 2.0;
        double w = (v_right - v_left) / wheel_base_;

        // === Интегрирование позы ===
        double delta_theta = w * dt;
        theta_ = normalize_angle(theta_ + delta_theta);

        double delta_x, delta_y;
        if (std::abs(delta_theta) < 1e-6) {
            delta_x = v * std::cos(theta_) * dt;
            delta_y = v * std::sin(theta_) * dt;
        } else {
            double avg_theta = theta_ - delta_theta / 2.0;
            delta_x = v * std::cos(avg_theta) * dt;
            delta_y = v * std::sin(avg_theta) * dt;
        }
        x_ += delta_x;
        y_ += delta_y;
        distance_ += std::sqrt(delta_x * delta_x + delta_y * delta_y);

        // === Кватернион курса ===
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);

        // === Публикация /odom ===
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp    = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id  = "base_link";

        odom_msg.pose.pose.position.x    = x_;
        odom_msg.pose.pose.position.y    = y_;
        odom_msg.pose.pose.position.z    = 0.0;
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x  = v;
        odom_msg.twist.twist.angular.z = w;

        odom_pub_->publish(odom_msg);

        // === TF ===
        if (publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp    = current_time;
            t.header.frame_id = "odom";
            t.child_frame_id  = "base_link";
            t.transform.translation.x = x_;
            t.transform.translation.y = y_;
            t.transform.translation.z = 0.0;
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(t);
        }

        // === Лог ===
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "ODOM | x=%.3f y=%.3f th=%.2f | v=%.2f w=%.2f | "
                             "L=%.0f R=%.0f ERPM | DIST=%.2f m",
                             x_, y_, theta_, v, w,
                             left_rpm_, right_rpm_, distance_);
    }

    // === Члены ===
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr left_state_sub_;
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr right_state_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr             odom_pub_;
    rclcpp::TimerBase::SharedPtr                                      timer_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                  reset_service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                    tf_broadcaster_;

    double wheel_radius_, wheel_base_, erpm_to_rads_;
    bool   publish_tf_;

    double x_, y_, theta_, distance_;
    rclcpp::Time last_time_;
    double left_rpm_, right_rpm_;
    bool   velocity_received_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryPkgNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}