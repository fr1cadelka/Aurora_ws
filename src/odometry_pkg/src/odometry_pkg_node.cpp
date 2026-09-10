#include <functional>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
        this->declare_parameter<double>("kp", 1.0);               // Коэффициент регулятора курса
        this->declare_parameter<bool>("use_compass", false);      // Использовать компас для коррекции

        this->get_parameter("wheel_radius", wheel_radius_);
        this->get_parameter("wheel_base", wheel_base_);
        this->get_parameter("erpm_to_rads", erpm_to_rads_);
        this->get_parameter("kp", kp_);
        this->get_parameter("use_compass", use_compass_);

        RCLCPP_INFO(this->get_logger(), "=== Odometry Node Parameters ===");
        RCLCPP_INFO(this->get_logger(), "wheel_radius: %.3f m", wheel_radius_);
        RCLCPP_INFO(this->get_logger(), "wheel_base:   %.3f m", wheel_base_);
        RCLCPP_INFO(this->get_logger(), "erpm_to_rads: %.5f", erpm_to_rads_);
        RCLCPP_INFO(this->get_logger(), "kp:           %.2f", kp_);
        RCLCPP_INFO(this->get_logger(), "use_compass:  %s", use_compass_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "=================================");

        // === Подписчики ===
        left_state_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
            "/left/sensors/core", rclcpp::QoS(10),
            [this](const vesc_msgs::msg::VescStateStamped::SharedPtr msg) {
                left_rpm_ = msg->state.speed;
                velocity_received_ = true;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "[RAW L] RPM: %.1f", left_rpm_);
            });

        right_state_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
            "/right/sensors/core", rclcpp::QoS(10),
            [this](const vesc_msgs::msg::VescStateStamped::SharedPtr msg) {
                right_rpm_ = msg->state.speed;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "[RAW R] RPM: %.1f", right_rpm_);
            });

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data",
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                if (msg->orientation.x == 0.0 && msg->orientation.y == 0.0 &&
                    msg->orientation.z == 0.0 && msg->orientation.w == 0.0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "IMU orientation is zero! Waiting for valid data...");
                    return;
                }
                current_orientation_ = msg->orientation;
                imu_ready_ = true;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "IMU data received: orientation (x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
                                     msg->orientation.x, msg->orientation.y,
                                     msg->orientation.z, msg->orientation.w);
            });

        // === НОВОЕ: Подписка на компас ===
        compass_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mavros/global_position/compass_hdg",
            rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                compass_deg_ = msg->data;
                compass_ready_ = true;
                // Логируем компас для проверки (раз в 2 секунды)
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Compass: %.1f deg (%.2f rad)",
                                     compass_deg_, compass_deg_ * M_PI / 180.0);
            });

        // === Издатель одометрии ===
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // === НОВОЕ: Издатель поправки (коррекции) ===
        correction_pub_ = this->create_publisher<std_msgs::msg::Float64>("/correction", 10);

        // === Таймер (50 Гц) ===
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&OdometryPkgNode::publish_odometry, this));

        // === Сервис сброса (обнуляет и сбрасывает целевой курс) ===
        reset_service_ = this->create_service<std_srvs::srv::Empty>(
            "reset_odometry",
            [this](const std_srvs::srv::Empty::Request::SharedPtr /*req*/,
                   const std_srvs::srv::Empty::Response::SharedPtr /*res*/) {
                x_ = 0.0;
                y_ = 0.0;
                theta_ = 0.0;
                distance_ = 0.0;
                previous_yaw_ = 0.0;
                target_yaw_set_ = false;   // Сбросить цель, чтобы переустановить при следующем получении IMU
                last_time_ = this->now();
                RCLCPP_INFO(this->get_logger(), "========== ODOMETRY RESET! ==========");
            });

        // === Инициализация ===
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        distance_ = 0.0;
        last_time_ = this->now();
        left_rpm_ = 0.0;
        right_rpm_ = 0.0;
        velocity_received_ = false;
        imu_ready_ = false;
        compass_ready_ = false;
        previous_yaw_ = 0.0;
        target_yaw_set_ = false;
        compass_deg_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "Odometry node started with IMU and Compass support!");
        RCLCPP_INFO(this->get_logger(), "Subscribed to: /left/sensors/core, /right/sensors/core, /mavros/imu/data, /mavros/global_position/compass_hdg");
    }

private:
    void publish_odometry()
    {
        if (!velocity_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for VESC data...");
            return;
        }
        if (!imu_ready_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for IMU data...");
            return;
        }

        // --- Установка целевого курса (при первом получении IMU) ---
        if (!target_yaw_set_) {
            // Получаем текущий yaw из IMU
            tf2::Quaternion q;
            tf2::fromMsg(current_orientation_, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            target_yaw_ = yaw;
            target_yaw_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Target yaw set to: %.2f rad (%.1f deg)",
                        target_yaw_, target_yaw_ * 180.0 / M_PI);
        }

        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt > 0.1) dt = 0.1;
        if (dt <= 0.0) return;

        // === Скорости колёс ===
        double left_rads = left_rpm_ * erpm_to_rads_;
        double right_rads = right_rpm_ * erpm_to_rads_;
        double v_left = left_rads * wheel_radius_;
        double v_right = right_rads * wheel_radius_;
        double linear_x = (v_left + v_right) / 2.0;

        // === Ориентация из IMU (текущий yaw) ===
        tf2::Quaternion q;
        tf2::fromMsg(current_orientation_, q);
        double roll, pitch, current_yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw);

        // === Коррекция курса ===
        double error = target_yaw_ - current_yaw;
        // Нормализация ошибки в [-PI, PI]
        while (error > M_PI) error -= 2.0 * M_PI;
        while (error < -M_PI) error += 2.0 * M_PI;

        // Можно дополнительно использовать компас для коррекции дрейфа (опционально)
        if (use_compass_ && compass_ready_) {
            // Преобразуем компас (градусы) в радианы
            double compass_rad = compass_deg_ * M_PI / 180.0;
            // Здесь можно сделать коррекцию target_yaw или error
            // Например, если компас показывает абсолютный курс, можно корректировать target_yaw
            // Но пока просто выводим в лог
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Compass correction enabled. Compass rad: %.2f", compass_rad);
        }

        // П-регулятор: корректирующая угловая скорость (положительная = поворот вправо)
        double correction = kp_ * error;

        // Публикуем поправку (можно использовать для управления моторами)
        auto corr_msg = std_msgs::msg::Float64();
        corr_msg.data = correction;
        correction_pub_->publish(corr_msg);

        // === Интегрирование перемещения (оставляем как было) ===
        double delta_theta = current_yaw - previous_yaw_;
        while (delta_theta > M_PI) delta_theta -= 2.0 * M_PI;
        while (delta_theta < -M_PI) delta_theta += 2.0 * M_PI;

        theta_ += delta_theta;
        while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
        previous_yaw_ = current_yaw;

        double delta_x, delta_y;
        if (std::abs(delta_theta) < 0.001) {
            delta_x = linear_x * cos(theta_) * dt;
            delta_y = linear_x * sin(theta_) * dt;
        } else {
            double avg_theta = theta_ - delta_theta / 2.0;
            delta_x = linear_x * cos(avg_theta) * dt;
            delta_y = linear_x * sin(avg_theta) * dt;
        }
        x_ += delta_x;
        y_ += delta_y;
        distance_ += std::sqrt(delta_x * delta_x + delta_y * delta_y);

        // === Публикация Odometry ===
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q_odom;
        q_odom.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q_odom.x();
        odom_msg.pose.pose.orientation.y = q_odom.y();
        odom_msg.pose.pose.orientation.z = q_odom.z();
        odom_msg.pose.pose.orientation.w = q_odom.w();

        odom_msg.twist.twist.linear.x = linear_x;
        double angular_z = (v_right - v_left) / wheel_base_;
        odom_msg.twist.twist.angular.z = angular_z;

        odom_pub_->publish(odom_msg);

        // === Логирование ===
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                             "ODOM | x=%.3f m, y=%.3f m, theta=%.2f rad | "
                             "v=%.2f m/s, w=%.2f rad/s | "
                             "L=%.2f rad/s, R=%.2f rad/s | "
                             "DIST=%.2f m | IMU_yaw=%.2f rad | "
                             "corr=%.3f",
                             x_, y_, theta_,
                             linear_x, angular_z,
                             left_rads, right_rads,
                             distance_, current_yaw, correction);
    }

    // === Члены ===
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr left_state_sub_;
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr right_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_sub_;   // <-- НОВОЕ
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr correction_pub_;   // <-- НОВОЕ
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;

    double wheel_radius_, wheel_base_, erpm_to_rads_;
    double kp_;                     // Коэффициент регулятора
    bool use_compass_;              // Флаг использования компаса

    double x_, y_, theta_, distance_;
    rclcpp::Time last_time_;
    double left_rpm_, right_rpm_;
    bool velocity_received_;
    geometry_msgs::msg::Quaternion current_orientation_;
    bool imu_ready_;
    double previous_yaw_;

    double compass_deg_;            // Текущее показание компаса (градусы)
    bool compass_ready_;            // Флаг получения данных компаса

    double target_yaw_;             // Целевой курс
    bool target_yaw_set_;           // Установлен ли целевой курс
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryPkgNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
