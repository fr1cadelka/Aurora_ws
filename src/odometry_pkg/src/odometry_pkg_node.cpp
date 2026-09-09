#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

class OdometryPkgNode : public rclcpp::Node
{
public:
    OdometryPkgNode()
        : Node("odometry_pkg_node")
    {
        // === Параметры ===
        this->declare_parameter<double>("wheel_radius", 0.1);
        this->declare_parameter<double>("wheel_base", 0.5);
        this->declare_parameter<double>("erpm_to_rads", 0.10472);

        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        erpm_to_rads_ = this->get_parameter("erpm_to_rads").as_double();

        RCLCPP_INFO(this->get_logger(), "=== Odometry Node Parameters ===");
        RCLCPP_INFO(this->get_logger(), "wheel_radius: %.3f m", wheel_radius_);
        RCLCPP_INFO(this->get_logger(), "wheel_base:   %.3f m", wheel_base_);
        RCLCPP_INFO(this->get_logger(), "erpm_to_rads: %.5f", erpm_to_rads_);
        RCLCPP_INFO(this->get_logger(), "=================================");

        // === Подписчики на состояния VESC ===
        left_state_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
            "/left/sensors/core", 10,
            std::bind(&OdometryPkgNode::left_state_callback, this, std::placeholders::_1));

        right_state_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
            "/right/sensors/core", 10,
            std::bind(&OdometryPkgNode::right_state_callback, this, std::placeholders::_1));

        // === Издатель ===
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // === Таймер ===
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&OdometryPkgNode::publish_odometry, this));

        // === Сервис сброса ===
        reset_service_ = this->create_service<std_srvs::srv::Empty>(
            "reset_odometry",
            std::bind(&OdometryPkgNode::reset_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // === Инициализация ===
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        distance_ = 0.0;  // <-- НОВОЕ: Суммарное пройденное расстояние
        last_time_ = this->now();
        left_rpm_ = 0.0;
        right_rpm_ = 0.0;
        velocity_received_ = false;

        RCLCPP_INFO(this->get_logger(), "Odometry node started!");
        RCLCPP_INFO(this->get_logger(), "Subscribed to: /left/sensors/core and /right/sensors/core");
    }

private:
    void left_state_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
    {
        left_rpm_ = msg->state.speed; // RPM из состояния VESC
        velocity_received_ = true;
        // Логируем сырые данные с левого мотора
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[RAW L] RPM: %.1f", left_rpm_);
    }

    void right_state_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
    {
        right_rpm_ = msg->state.speed;
        // Логируем сырые данные с правого мотора
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[RAW R] RPM: %.1f", right_rpm_);
    }

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

        // === Вычисление скоростей ===
        double left_rads = left_rpm_ * erpm_to_rads_;
        double right_rads = right_rpm_ * erpm_to_rads_;

        double v_left = left_rads * wheel_radius_;
        double v_right = right_rads * wheel_radius_;

        double linear_x = (v_left + v_right) / 2.0;
        double angular_z = (v_right - v_left) / wheel_base_;

        // === Интегрирование ===
        double delta_theta = angular_z * dt;
        double delta_x, delta_y;

        if (std::abs(angular_z) < 0.001) {
            delta_x = linear_x * cos(theta_) * dt;
            delta_y = linear_x * sin(theta_) * dt;
        } else {
            double avg_theta = theta_ + delta_theta / 2.0;
            delta_x = linear_x * cos(avg_theta) * dt;
            delta_y = linear_x * sin(avg_theta) * dt;
        }

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // === НОВОЕ: Вычисление суммарного пройденного расстояния ===
        // Сумма модулей перемещений за каждый шаг времени
        distance_ += std::sqrt(delta_x * delta_x + delta_y * delta_y);

        while (theta_ > M_PI) theta_ -= 2 * M_PI;
        while (theta_ < -M_PI) theta_ += 2 * M_PI;

        // === Создание сообщения ===
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = linear_x;
        odom_msg.twist.twist.angular.z = angular_z;

        odom_pub_->publish(odom_msg);

        // === Детальное логирование в терминал ===
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, // Каждые 100 мс
                             "ODOM | x=%.3f m, y=%.3f m, theta=%.2f rad | "
                             "v=%.2f m/s, w=%.2f rad/s | "
                             "L=%.2f rad/s, R=%.2f rad/s | "
                             "DIST=%.2f m",
                             x_, y_, theta_,
                             linear_x, angular_z,
                             left_rads, right_rads,
                             distance_);
    }

    void reset_callback(
        const std_srvs::srv::Empty::Request::SharedPtr request,
        const std_srvs::srv::Empty::Response::SharedPtr response)
    {
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        distance_ = 0.0;  // <-- НОВОЕ: Сбрасываем расстояние
        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "========== ODOMETRY RESET! ==========");
    }

    // === Члены класса ===
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr left_state_sub_;
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr right_state_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;

    double wheel_radius_;
    double wheel_base_;
    double erpm_to_rads_;

    double x_, y_, theta_;
    double distance_;  // <-- НОВОЕ: переменная для расстояния
    rclcpp::Time last_time_;
    double left_rpm_, right_rpm_;
    bool velocity_received_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryPkgNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
