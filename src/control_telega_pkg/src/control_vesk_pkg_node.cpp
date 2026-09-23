#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <atomic>
#include <memory>
#include <mutex>

using namespace std::chrono_literals;

static double normalize_angle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

int getch() {
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

class VescTeleopNode : public rclcpp::Node {
public:
    VescTeleopNode() : Node("control_vesk_pkg_node") {
        // ===== Параметры (не тронуты) =====
        declare_parameter<double>("correction_scale", 500.0);
        declare_parameter<bool>  ("invert_correction", false);
        declare_parameter<bool>  ("invert_left", false);
        declare_parameter<bool>  ("invert_right", false);
        declare_parameter<bool>  ("invert_angular", false);
        declare_parameter<double>("max_erpm", 30000.0);
        declare_parameter<double>("erpm_step", 250.0);
        declare_parameter<double>("min_erpm", 300.0);
        declare_parameter<double>("erpm_slew_rate", 4000.0);

        // ===== НОВЫЕ параметры автонома =====
        declare_parameter<double>("auto_speed_erpm",    1750.0);  // ≈ 1750 ERPM
        declare_parameter<double>("auto_lookahead_m",   1.5);
        declare_parameter<double>("auto_kp_heading",    1.2);
        declare_parameter<double>("auto_max_w_erpm",    5000.0);
        declare_parameter<double>("auto_uwb_timeout_s", 2.0);     // отвал UWB
        declare_parameter<double>("auto_odom_timeout_s",1.0);
        declare_parameter<double>("auto_goal_tol_m",    0.5);     // достигли конца траектории

        get_parameter("correction_scale", correction_scale_);
        get_parameter("invert_correction", invert_correction_);
        get_parameter("invert_left", invert_left_);
        get_parameter("invert_right", invert_right_);
        get_parameter("invert_angular", invert_angular_);
        get_parameter("max_erpm", max_erpm_);
        get_parameter("erpm_step", erpm_step_);
        get_parameter("min_erpm", min_erpm_);
        get_parameter("erpm_slew_rate", erpm_slew_rate_);
        get_parameter("auto_speed_erpm", auto_speed_erpm_);
        get_parameter("auto_lookahead_m", auto_lookahead_m_);
        get_parameter("auto_kp_heading", auto_kp_heading_);
        get_parameter("auto_max_w_erpm", auto_max_w_erpm_);
        get_parameter("auto_uwb_timeout_s", auto_uwb_timeout_s_);
        get_parameter("auto_odom_timeout_s", auto_odom_timeout_s_);
        get_parameter("auto_goal_tol_m", auto_goal_tol_m_);

        RCLCPP_INFO(get_logger(), "=== VESC Teleop ===");
        RCLCPP_INFO(get_logger(), "max_erpm       = %.0f", max_erpm_);
        RCLCPP_INFO(get_logger(), "auto_speed     = %.0f ERPM", auto_speed_erpm_);
        RCLCPP_INFO(get_logger(), "auto_lookahead = %.2f m", auto_lookahead_m_);
        RCLCPP_INFO(get_logger(), "auto_kp        = %.2f", auto_kp_heading_);

        // ===== Состояние =====
        is_armed_     = false;
        linear_cmd_   = 0.0;
        angular_cmd_  = 0.0;
        speed_step_   = 3000.0;
        turn_factor_  = 0.6;
        cruise_active_ = false;
        cruise_speed_  = 0.0;
        auto_mode_    = false;
        smooth_left_  = 0.0;
        smooth_right_ = 0.0;

        // ===== Publishers =====
        left_pub_  = create_publisher<std_msgs::msg::Float64>("/left/commands/motor/speed", 10);
        right_pub_ = create_publisher<std_msgs::msg::Float64>("/right/commands/motor/speed", 10);
        publishMotors(0.0, 0.0);

        // ===== Subscribers =====
        cmd_sub_ = create_subscription<std_msgs::msg::String>(
            "/telega_commands", 10,
            std::bind(&VescTeleopNode::cmdCallback, this, std::placeholders::_1));

        uwb_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/uwb/pose", rclcpp::QoS(10),
            std::bind(&VescTeleopNode::uwb_pose_callback, this, std::placeholders::_1));

        trajectory_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/uwb/trajectory", rclcpp::QoS(10),
            std::bind(&VescTeleopNode::trajectory_callback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10),
            std::bind(&VescTeleopNode::odom_callback, this, std::placeholders::_1));

        // ===== Таймер 50 Гц =====
        timer_ = create_wall_timer(20ms, std::bind(&VescTeleopNode::controlLoop, this));

        RCLCPP_INFO(get_logger(), "Subscribed: VESC core, /telega_commands, /uwb/pose, /uwb/trajectory, /odom");
        RCLCPP_INFO(get_logger(), "Publishing: /left|right/commands/motor/speed");
        printHelp();
        printStatus();
    }

    void readKeyboard() {
        while (rclcpp::ok()) {
            int ch = getch();
            handleKey(ch);
        }
    }

private:
    // ============================================================
    // CALLBACKS
    // ============================================================
    void uwb_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        uwb_x_ = msg->pose.position.x;
        uwb_y_ = msg->pose.position.y;
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        uwb_yaw_ = y;
        uwb_pose_time_ = this->now();
        uwb_pose_received_ = true;
    }

    void trajectory_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        trajectory_ = *msg;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        odom_time_ = this->now();
        odom_received_ = true;
        odom_v_ = msg->twist.twist.linear.x;
    }

    // ============================================================
    // KEYBOARD HANDLER
    // ============================================================
    void handleKey(int ch) {
        std::lock_guard<std::mutex> lk(mtx_);

        if (ch == 'e' || ch == 'E') {
            is_armed_ = !is_armed_;
            if (is_armed_) RCLCPP_WARN(get_logger(), "ARMED");
            else { resetMotion(); RCLCPP_WARN(get_logger(), "DISARMED"); }
            printStatus();
            return;
        }
        if (ch == ' ') { emergencyStop(); return; }

        if (!is_armed_) { std::cout << "\n[!] Сначала ARM (E)\n"; return; }

        // ===== АВТОНОМ =====
        if (ch == 'm' || ch == 'M') {
            if (!auto_mode_) {
                if (!check_auto_ready()) {
                    return;   // причина уже в логе
                }
                auto_mode_ = true;
                auto_start_time_ = this->now();
                linear_cmd_ = auto_speed_erpm_;
                angular_cmd_ = 0.0;
                RCLCPP_WARN(get_logger(),
                            "AUTO MODE ON @ %.0f ERPM | traj: %zu pts | lookahead: %.2f m",
                            auto_speed_erpm_, trajectory_.poses.size(), auto_lookahead_m_);
            } else {
                auto_mode_ = false;
                linear_cmd_ = 0.0;
                angular_cmd_ = 0.0;
                RCLCPP_WARN(get_logger(), "AUTO MODE OFF");
            }
            printStatus();
            return;
        }

        if (ch == 'q' || ch == 'Q') {
            if (auto_mode_) { std::cout << "\n[!] Круиз недоступен в авто\n"; return; }
            cruise_active_ = !cruise_active_;
            if (cruise_active_) {
                cruise_speed_ = speed_step_;
                linear_cmd_ = cruise_speed_;
                angular_cmd_ = 0.0;
                RCLCPP_WARN(get_logger(), "CRUISE ON @ %.0f ERPM", cruise_speed_);
            } else {
                linear_cmd_ = 0.0;
                angular_cmd_ = 0.0;
                RCLCPP_WARN(get_logger(), "CRUISE OFF");
            }
            printStatus();
            return;
        }

        if (ch == 'r' || ch == 'R') {
            speed_step_ = std::min(speed_step_ + erpm_step_, max_erpm_);
            if (cruise_active_) linear_cmd_ = speed_step_;
            printStatus();
            return;
        }
        if (ch == 'f' || ch == 'F') {
            speed_step_ = std::max(erpm_step_, speed_step_ - erpm_step_);
            if (cruise_active_) linear_cmd_ = speed_step_;
            printStatus();
            return;
        }

        if (ch == ',' || ch == '<') {
            turn_factor_ = std::max(0.05, turn_factor_ - 0.05);
            std::cout << "\n[TURN] factor = " << turn_factor_ << "\n";
            printStatus(); return;
        }
        if (ch == '.' || ch == '>') {
            turn_factor_ = std::min(2.0, turn_factor_ + 0.05);
            std::cout << "\n[TURN] factor = " << turn_factor_ << "\n";
            printStatus(); return;
        }

        if (auto_mode_) { std::cout << "\n[!] Ручные команды в AUTO заблокированы\n"; return; }

        if (ch == 'w' || ch == 'W') {
            linear_cmd_ = (linear_cmd_ > 0.0) ? 0.0 : speed_step_;
            printStatus(); return;
        }
        if (ch == 's' || ch == 'S') {
            linear_cmd_ = (linear_cmd_ < 0.0) ? 0.0 : -speed_step_;
            printStatus(); return;
        }
        if (ch == 'a' || ch == 'A') {
            angular_cmd_ = (angular_cmd_ > 0.0) ? 0.0 : speed_step_ * turn_factor_;
            printStatus(); return;
        }
        if (ch == 'd' || ch == 'D') {
            angular_cmd_ = (angular_cmd_ < 0.0) ? 0.0 : -speed_step_ * turn_factor_;
            printStatus(); return;
        }
    }

    // ============================================================
    // ПРОВЕРКА ГОТОВНОСТИ ДАННЫХ ПРИ ВХОДЕ В АВТОНОМ
    // ============================================================
    bool check_auto_ready() {
        rclcpp::Time now = this->now();
        bool ok = true;

        if (!uwb_pose_received_) {
            RCLCPP_ERROR(get_logger(), "[AUTO] /uwb/pose никогда не приходил");
            ok = false;
        } else {
            double age = (now - uwb_pose_time_).seconds();
            if (age > 1.0) {
                RCLCPP_ERROR(get_logger(), "[AUTO] /uwb/pose устарел (%.1f s)", age);
                ok = false;
            }
        }

        if (!odom_received_) {
            RCLCPP_ERROR(get_logger(), "[AUTO] /odom никогда не приходил");
            ok = false;
        } else {
            double age = (now - odom_time_).seconds();
            if (age > auto_odom_timeout_s_) {
                RCLCPP_ERROR(get_logger(), "[AUTO] /odom устарел (%.1f s)", age);
                ok = false;
            }
        }

        if (trajectory_.poses.empty()) {
            RCLCPP_ERROR(get_logger(), "[AUTO] /uwb/trajectory пустая (%zu точек)",
                         trajectory_.poses.size());
            ok = false;
        }

        if (!ok) {
            RCLCPP_ERROR(get_logger(), "[AUTO] НЕ ВХОДИМ В АВТОНОМ");
        }
        return ok;
    }

    // ============================================================
    // PURE PURSUIT — вычисление (v, w) для автонома
    // ============================================================
    std::pair<double, double> compute_auto_control() {
        // Копируем всё, что нужно, под мьютексом
        double cur_x, cur_y, cur_yaw;
        rclcpp::Time uwb_t, odom_t;
        nav_msgs::msg::Path traj;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            cur_x = uwb_x_;
            cur_y = uwb_y_;
            cur_yaw = uwb_yaw_;
            uwb_t = uwb_pose_time_;
            odom_t = odom_time_;
            traj = trajectory_;
        }

        rclcpp::Time now = this->now();

        // === Проверка отвала UWB ===
        if ((now - uwb_t).seconds() > auto_uwb_timeout_s_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[AUTO] UWB timeout %.1f s — STOPPING", (now - uwb_t).seconds());
            return {0.0, 0.0};
        }
        // === Проверка отвала одометрии (нужна для yaw fusion) ===
        if ((now - odom_t).seconds() > auto_odom_timeout_s_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[AUTO] odom timeout %.1f s — STOPPING", (now - odom_t).seconds());
            return {0.0, 0.0};
        }

        if (traj.poses.size() < 2) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[AUTO] trajectory empty — STOPPING");
            return {0.0, 0.0};
        }

        // === Финальная точка: если близко — считаем что доехали ===
        const auto& last = traj.poses.back().pose.position;
        double d_end = std::hypot(last.x - cur_x, last.y - cur_y);

        // === Lookahead поиск ===
        double look = auto_lookahead_m_;
        double gx = 0.0, gy = 0.0;
        bool have_goal = false;

        // Найти ближайшую точку
        size_t nearest = 0;
        double best_d2 = 1e18;
        for (size_t i = 0; i < traj.poses.size(); ++i) {
            double dx = traj.poses[i].pose.position.x - cur_x;
            double dy = traj.poses[i].pose.position.y - cur_y;
            double d2 = dx*dx + dy*dy;
            if (d2 < best_d2) { best_d2 = d2; nearest = i; }
        }

        // Идём вперёд до lookahead
        for (size_t i = nearest; i < traj.poses.size(); ++i) {
            double dx = traj.poses[i].pose.position.x - cur_x;
            double dy = traj.poses[i].pose.position.y - cur_y;
            if (dx*dx + dy*dy >= look*look) {
                gx = traj.poses[i].pose.position.x;
                gy = traj.poses[i].pose.position.y;
                have_goal = true;
                break;
            }
        }
        // Если не нашли — берём последнюю (но только если ещё не достигли)
        if (!have_goal) {
            bool in_startup = (this->now() - auto_start_time_).seconds() < 1.5;
            if (!in_startup && d_end < auto_goal_tol_m_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "[AUTO] Goal reached! Stopping.");
                return {0.0, 0.0};
            }
            gx = last.x;
            gy = last.y;
        }

        // === Угол к цели в системе робота ===
        double dx = gx - cur_x;
        double dy = gy - cur_y;
        double angle_to_goal = std::atan2(dy, dx);   // в map
        double alpha = normalize_angle(angle_to_goal - cur_yaw);

        // === Pure Pursuit: w = kp * alpha ===
        // (стандартная формула w = 2v sin(alpha)/L сводится к этому же при малых alpha)
        double w = auto_kp_heading_ * alpha * 1000.0;  // * 1000 — чтобы получить ERPM в разумном диапазоне
        w = std::clamp(w, -auto_max_w_erpm_, auto_max_w_erpm_);

        double v = auto_speed_erpm_;

        // Тормозим если очень крутой угол (>90°)
        if (std::abs(alpha) > M_PI_2) {
            v *= 0.4;
        } else if (std::abs(alpha) > M_PI_4) {
            v *= 0.7;
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "[AUTO] pos=(%.2f,%.2f) yaw=%.2f goal=(%.2f,%.2f) "
                             "alpha=%.2f v=%.0f w=%.0f d_end=%.2f",
                             cur_x, cur_y, cur_yaw, gx, gy, alpha, v, w, d_end);

        return {v, w};
    }

    // ============================================================
    // UTILS
    // ============================================================
    void resetMotion() {
        linear_cmd_ = angular_cmd_ = 0.0;
        cruise_active_ = auto_mode_ = false;
        cruise_speed_ = 0.0;
    }

    std::pair<double, double> mixMotors(double v, double w) const {
        if (invert_angular_) w = -w;
        double left  = v + w;
        double right = v - w;
        if (invert_left_)  left  = -left;
        if (invert_right_) right = -right;
        left  = std::clamp(left,  -max_erpm_, max_erpm_);
        right = std::clamp(right, -max_erpm_, max_erpm_);
        return {left, right};
    }

    void publishMotors(double left, double right) {
        std_msgs::msg::Float64 l, r;
        l.data = left; r.data = right;
        left_pub_->publish(l);
        right_pub_->publish(r);
    }

    void emergencyStop() {
        resetMotion();
        is_armed_ = false;
        smooth_left_ = smooth_right_ = 0.0;
        RCLCPP_ERROR(get_logger(), "EMERGENCY STOP");
        publishMotors(0.0, 0.0);
        printStatus();
    }

    // ============================================================
    // TCP
    // ============================================================
    void cmdCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;
        if (cmd.size() == 1) { handleKey(cmd[0]); return; }

        std::lock_guard<std::mutex> lk(mtx_);
        if (cmd == "arm" || cmd == "ARM") {
            is_armed_ = true;
            RCLCPP_WARN(get_logger(), "ARMED (TCP)");
            printStatus(); return;
        }
        if (cmd == "disarm" || cmd == "DISARM" || cmd == "stop" || cmd == "STOP") {
            resetMotion(); is_armed_ = false;
            publishMotors(0.0, 0.0);
            RCLCPP_WARN(get_logger(), "DISARMED (TCP)");
            printStatus(); return;
        }
        RCLCPP_WARN(get_logger(), "Неизвестная команда: '%s'", cmd.c_str());
    }

    // ============================================================
    // ГЛАВНЫЙ ЦИКЛ (50 Гц)
    // ============================================================
    void controlLoop() {
        double v = 0.0, w = 0.0;
        bool armed, auto_m;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            armed = is_armed_;
            auto_m = auto_mode_;
        }

        if (armed && auto_m) {
            auto [va, wa] = compute_auto_control();
            v = va; w = wa;
        } else if (armed) {
            std::lock_guard<std::mutex> lk(mtx_);
            v = linear_cmd_;
            w = angular_cmd_;
        }

        auto [tl, tr] = mixMotors(v, w);

        const double dt = 0.02;
        const double max_delta = erpm_slew_rate_ * dt;
        smooth_left_  += std::clamp(tl - smooth_left_,  -max_delta, max_delta);
        smooth_right_ += std::clamp(tr - smooth_right_, -max_delta, max_delta);

        if (std::abs(tl) < min_erpm_ && std::abs(smooth_left_)  < min_erpm_) smooth_left_  = 0.0;
        if (std::abs(tr) < min_erpm_ && std::abs(smooth_right_) < min_erpm_) smooth_right_ = 0.0;

        publishMotors(smooth_left_, smooth_right_);

        static int cnt = 0;
        if (++cnt >= 25) {
            cnt = 0;
            const char* mode = auto_mode_ ? "AUTO" : (cruise_active_ ? "CRUISE" : "MANUAL");
            RCLCPP_INFO(get_logger(),
                        "%s | L=%.0f R=%.0f ERPM | v=%.0f w=%.0f step=%.0f turn=%.2f",
                        mode, smooth_left_, smooth_right_, v, w, speed_step_, turn_factor_);
        }
    }

    // ============================================================
    // ПЕЧАТЬ
    // ============================================================
    void printHelp() {
        std::cout << "\n=======================================================\n"
                  << "  VESC Teleop — 2 мотора, режим СКОРОСТИ (ERPM)\n"
                  << "-------------------------------------------------------\n"
                  << "  [E]        — ARM / DISARM\n"
                  << "  [M]        — АВТОНОМ (follow /uwb/trajectory)\n"
                  << "  [R] / [F]  — Больше/меньше скорость\n"
                  << "  [Q]        — Круиз\n"
                  << "  [W] [S]    — Вперёд / Назад\n"
                  << "  [A] [D]    — Поворот влево/вправо\n"
                  << "  [,] [.]    — Острота поворота\n"
                  << "  [Пробел]   — АВАРИЙНЫЙ СТОП\n"
                  << "=======================================================\n\n"
                  << "  [Пробел]   — АВАРИЙНЫЙ СТОП\n"
                  << "-------------------------------------------------------\n"
                  << "  Сервисы (из другого терминала):\n"
                  << "    ros2 service call /tag_localizer_pkg_node/build_coverage_trajectory std_srvs/srv/Empty\n"
                  << "    ros2 service call /tag_localizer_pkg_node/build_trajectory_from_clicks std_srvs/srv/Empty\n"
                  << "    ros2 service call /tag_localizer_pkg_node/clear_points std_srvs/srv/Empty\n"
                  << "=======================================================\n\n";
    }

    void printStatus() {
        std::cout << "\r[Статус] "
                  << (is_armed_ ? "ARMED   " : "DISARMED")
                  << " | " << (auto_mode_ ? "AUTO " : "     ")
                  << " | " << (cruise_active_ ? "CRUISE" : "      ")
                  << " | step = " << speed_step_ << " ERPM"
                  << " | turn = " << turn_factor_
                  << "          " << std::flush;
    }

    // ============================================================
    // MEMBERS
    // ============================================================
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_, right_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr uwb_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex mtx_;
    rclcpp::Time auto_start_time_;

    // === Автоном ===
    double auto_speed_erpm_, auto_lookahead_m_, auto_kp_heading_;
    double auto_max_w_erpm_, auto_uwb_timeout_s_, auto_odom_timeout_s_;
    double auto_goal_tol_m_;

    double uwb_x_, uwb_y_, uwb_yaw_;
    rclcpp::Time uwb_pose_time_;
    bool uwb_pose_received_ = false;

    nav_msgs::msg::Path trajectory_;

    double odom_v_ = 0.0;
    rclcpp::Time odom_time_;
    bool odom_received_ = false;

    // === Остальное ===
    bool   is_armed_;
    double linear_cmd_, angular_cmd_;
    double speed_step_, max_erpm_, erpm_step_, min_erpm_, erpm_slew_rate_;
    double turn_factor_;
    bool   cruise_active_;
    double cruise_speed_;
    bool   auto_mode_;
    double correction_;
    double correction_scale_;
    bool   invert_correction_, invert_left_, invert_right_, invert_angular_;
    double smooth_left_, smooth_right_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VescTeleopNode>();
    std::thread keyboard_thread(&VescTeleopNode::readKeyboard, node);
    rclcpp::spin(node);
    if (keyboard_thread.joinable()) keyboard_thread.join();
    rclcpp::shutdown();
    return 0;
}
