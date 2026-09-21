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

static double normalize_angle(double a)
{
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

class OdometryPkgNode : public rclcpp::Node
{
public:
    OdometryPkgNode() : Node("odometry_pkg_node")
    {
        // === Параметры (существующие — не тронуты) ===
        this->declare_parameter<double>("wheel_radius", 0.1);
        this->declare_parameter<double>("wheel_base",   0.5);
        this->declare_parameter<double>("erpm_to_rads", 0.10472);
        this->declare_parameter<double>("kp",           1.0);
        this->declare_parameter<bool>  ("use_compass",  false);

        // === НОВЫЕ параметры фьюжена (дефолты безопасные) ===
        this->declare_parameter<double>("compass_alpha",      0.95); // 0..1, чем больше — тем больше доверия гироскопу
        this->declare_parameter<double>("compass_reject_rad", 0.5);  // отбрасывать скачки компаса > ~28°
        this->declare_parameter<double>("tilt_gate_rad",      0.5);  // игнор компаса при крене > ~28°
        this->declare_parameter<bool>  ("compass_invert",     false);// если компас растёт CCW — true

        this->get_parameter("wheel_radius",       wheel_radius_);
        this->get_parameter("wheel_base",         wheel_base_);
        this->get_parameter("erpm_to_rads",       erpm_to_rads_);
        this->get_parameter("kp",                 kp_);
        this->get_parameter("use_compass",        use_compass_);
        this->get_parameter("compass_alpha",      compass_alpha_);
        this->get_parameter("compass_reject_rad", compass_reject_rad_);
        this->get_parameter("tilt_gate_rad",      tilt_gate_rad_);
        this->get_parameter("compass_invert",     compass_invert_);

        RCLCPP_INFO(this->get_logger(), "=== Odometry Node Parameters ===");
        RCLCPP_INFO(this->get_logger(), "wheel_radius:      %.3f m", wheel_radius_);
        RCLCPP_INFO(this->get_logger(), "wheel_base:        %.3f m", wheel_base_);
        RCLCPP_INFO(this->get_logger(), "erpm_to_rads:      %.5f",    erpm_to_rads_);
        RCLCPP_INFO(this->get_logger(), "kp:                %.2f",    kp_);
        RCLCPP_INFO(this->get_logger(), "use_compass:       %s",      use_compass_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "compass_alpha:     %.3f",    compass_alpha_);
        RCLCPP_INFO(this->get_logger(), "compass_reject_rad:%.3f",    compass_reject_rad_);
        RCLCPP_INFO(this->get_logger(), "tilt_gate_rad:     %.3f",    tilt_gate_rad_);
        RCLCPP_INFO(this->get_logger(), "compass_invert:    %s",      compass_invert_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "===============================");

        // === Инициализация состояния ===
        x_ = y_ = theta_ = distance_ = 0.0;
        last_time_ = this->now();
        left_rpm_ = right_rpm_ = 0.0;
        velocity_received_ = false;
        imu_ready_         = false;
        compass_ready_     = false;
        compass_fresh_     = false;
        yaw_fused_init_    = false;
        yaw_fused_         = 0.0;
        gyro_z_            = 0.0;
        roll_ = pitch_     = 0.0;
        previous_yaw_      = 0.0;
        compass_deg_       = 0.0;
        target_yaw_        = 0.0;
        target_set_        = false;

        // === Подписчики ===
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
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", rclcpp::SensorDataQoS(),
            std::bind(&OdometryPkgNode::imu_callback, this, std::placeholders::_1));
        compass_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
            std::bind(&OdometryPkgNode::compass_callback, this, std::placeholders::_1));

        // === Издатели ===
        odom_pub_       = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        correction_pub_ = this->create_publisher<std_msgs::msg::Float64>("/correction", 10);

        // === Сервисы фиксации/сброса эталона ===
        set_target_srv_ = this->create_service<std_srvs::srv::Empty>(
            "~/set_target_heading",
            [this](const std_srvs::srv::Empty::Request::SharedPtr,
                   std_srvs::srv::Empty::Response::SharedPtr) {
                if (!yaw_fused_init_) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Cannot set target: yaw not initialized yet (IMU not ready)");
                    return;
                }
                target_yaw_ = yaw_fused_;
                target_set_ = true;
                RCLCPP_WARN(this->get_logger(),
                            "TARGET HEADING SET: %.3f rad (%.1f deg)",
                            target_yaw_, target_yaw_ * 180.0 / M_PI);
            });

        clear_target_srv_ = this->create_service<std_srvs::srv::Empty>(
            "~/clear_target_heading",
            [this](const std_srvs::srv::Empty::Request::SharedPtr,
                   std_srvs::srv::Empty::Response::SharedPtr) {
                target_set_ = false;
                RCLCPP_WARN(this->get_logger(), "TARGET HEADING CLEARED");
            });

        // === Таймер 50 Гц ===
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&OdometryPkgNode::publish_odometry, this));

        RCLCPP_INFO(this->get_logger(),
                    "Odometry node started (fused IMU + compass + VESC)");
        RCLCPP_INFO(this->get_logger(),
                    "Services: ~/set_target_heading, ~/clear_target_heading");
    }

private:
    // ==================== CALLBACKS ====================
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (msg->orientation.x == 0.0 && msg->orientation.y == 0.0 &&
            msg->orientation.z == 0.0 && msg->orientation.w == 0.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "IMU orientation is zero! Waiting for valid data...");
            return;
        }

        current_orientation_ = msg->orientation;
        gyro_z_ = msg->angular_velocity.z;
        imu_ready_ = true;

        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        roll_  = r;
        pitch_ = p;

        if (!yaw_fused_init_) {
            yaw_fused_      = y;
            previous_yaw_   = y;
            yaw_fused_init_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "yaw_fused initialized from IMU: %.3f rad", y);
        }
    }

    void compass_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        compass_deg_   = msg->data;
        compass_ready_ = true;
        compass_fresh_ = true;
    }

    // === Конвертация компас (0=N, CW) → ROS-ENU yaw (0=E, CCW) ===
    double compass_hdg_to_yaw(double hdg_deg) const
    {
        double hdg_rad = hdg_deg * M_PI / 180.0;
        if (compass_invert_)
            return normalize_angle(M_PI / 2.0 + hdg_rad);
        return normalize_angle(M_PI / 2.0 - hdg_rad);
    }

    // ==================== MAIN LOOP ====================
    void publish_odometry()
    {
        if (!velocity_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for VESC data...");
            return;
        }
        if (!imu_ready_ || !yaw_fused_init_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for IMU data...");
            return;
        }

        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        if (dt > 0.1) dt = 0.1;
        if (dt <= 0.0) return;

        // ============ FUSION ============
        // 1) Predict via gyro
        double yaw_pred = yaw_fused_ + gyro_z_ * dt;

        // 2) Correct with compass on fresh message
        double compass_yaw = 0.0;
        bool   compass_applied = false;

        if (compass_fresh_ && compass_ready_) {
            compass_fresh_ = false;
            compass_yaw = compass_hdg_to_yaw(compass_deg_);

            double err = normalize_angle(compass_yaw - yaw_pred);
            bool tilt_ok = (std::abs(roll_)  < tilt_gate_rad_ &&
                            std::abs(pitch_) < tilt_gate_rad_);

            if (tilt_ok && std::abs(err) < compass_reject_rad_) {
                yaw_pred += (1.0 - compass_alpha_) * err;
                compass_applied = true;
            } else if (std::abs(err) >= compass_reject_rad_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Compass jump rejected: err=%.2f rad (tilt_ok=%d)",
                                     err, tilt_ok ? 1 : 0);
            }
        }
        yaw_fused_ = normalize_angle(yaw_pred);
        double current_yaw = yaw_fused_;

        // ============ Диагностический лог ===
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "FUSION yaw=%.2f | compass=%.1fdeg (%.2frad) | gyro_z=%.3f | "
                             "roll=%.2f pitch=%.2f | applied=%d",
                             current_yaw, compass_deg_, compass_yaw, gyro_z_,
                             roll_, pitch_, compass_applied ? 1 : 0);

        // ============ Correction ============
        double correction = 0.0;
        if (target_set_) {
            double error = normalize_angle(target_yaw_ - current_yaw);
            correction = kp_ * error;
        }
        auto corr_msg = std_msgs::msg::Float64();
        corr_msg.data = correction;
        correction_pub_->publish(corr_msg);

        // ============ Odometry integration ============
        double left_rads  = left_rpm_  * erpm_to_rads_;
        double right_rads = right_rpm_ * erpm_to_rads_;
        double v_left     = left_rads  * wheel_radius_;
        double v_right    = right_rads * wheel_radius_;
        double linear_x   = (v_left + v_right) / 2.0;

        double delta_theta = normalize_angle(current_yaw - previous_yaw_);
        theta_ = normalize_angle(theta_ + delta_theta);
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

        // ============ Publish /odom ============
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp    = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id  = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q_odom;
        q_odom.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q_odom.x();
        odom_msg.pose.pose.orientation.y = q_odom.y();
        odom_msg.pose.pose.orientation.z = q_odom.z();
        odom_msg.pose.pose.orientation.w = q_odom.w();

        odom_msg.twist.twist.linear.x  = linear_x;
        odom_msg.twist.twist.angular.z = (v_right - v_left) / wheel_base_;
        odom_pub_->publish(odom_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "ODOM | x=%.2f y=%.2f th=%.2f | v=%.2f w=%.2f | L=%.0f R=%.0f | "
                             "tgt=%d corr=%.3f",
                             x_, y_, theta_, linear_x, odom_msg.twist.twist.angular.z,
                             left_rpm_, right_rpm_, target_set_ ? 1 : 0, correction);
    }

    // ==================== MEMBERS ====================
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr left_state_sub_;
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr right_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           compass_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr   odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    correction_pub_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr set_target_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_target_srv_;

    rclcpp::TimerBase::SharedPtr timer_;

    double wheel_radius_, wheel_base_, erpm_to_rads_, kp_;
    bool   use_compass_;
    double compass_alpha_, compass_reject_rad_, tilt_gate_rad_;
    bool   compass_invert_;

    double x_, y_, theta_, distance_;
    rclcpp::Time last_time_;
    double left_rpm_, right_rpm_;
    bool   velocity_received_;
    geometry_msgs::msg::Quaternion current_orientation_;
    bool   imu_ready_;

    double gyro_z_;
    double roll_, pitch_;
    double yaw_fused_;
    bool   yaw_fused_init_;
    double previous_yaw_;

    double compass_deg_;
    bool   compass_ready_;
    bool   compass_fresh_;

    double target_yaw_;
    bool   target_set_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryPkgNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}