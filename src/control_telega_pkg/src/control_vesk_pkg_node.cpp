#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
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
        // === Параметры (всё в узле, VESC Tool не трогаем) ===
        this->declare_parameter<double>("correction_scale", 500.0);
        this->declare_parameter<bool>("invert_correction", false);
        this->declare_parameter<bool>("invert_left", false);
        this->declare_parameter<bool>("invert_right", false);
        this->declare_parameter<bool>("invert_angular", false);

        // ERPM — единицы скорости вращения, которые понимает VESC в режиме PID Speed.
        // 100000 ERPM — это порядка 10000-15000 RPM на 14-полюсном моторе.
        this->declare_parameter<double>("max_erpm", 30000.0);
        this->declare_parameter<double>("erpm_step", 250.0);
        this->declare_parameter<double>("min_erpm", 300.0);       // мёртвая зона
        this->declare_parameter<double>("erpm_slew_rate", 4000.0); // ERPM/с — плавность

        // === НОВЫЙ параметр автонома ===
        this->declare_parameter<double>("auto_speed_erpm", 2000.0);

        this->get_parameter("correction_scale", correction_scale_);
        this->get_parameter("invert_correction", invert_correction_);
        this->get_parameter("invert_left", invert_left_);
        this->get_parameter("invert_right", invert_right_);
        this->get_parameter("invert_angular", invert_angular_);
        this->get_parameter("max_erpm", max_erpm_);
        this->get_parameter("erpm_step", erpm_step_);
        this->get_parameter("min_erpm", min_erpm_);
        this->get_parameter("erpm_slew_rate", erpm_slew_rate_);
        this->get_parameter("auto_speed_erpm", auto_speed_erpm_);

        RCLCPP_INFO(this->get_logger(), "max_erpm         = %.0f", max_erpm_);
        RCLCPP_INFO(this->get_logger(), "erpm_step        = %.0f", erpm_step_);
        RCLCPP_INFO(this->get_logger(), "min_erpm         = %.0f", min_erpm_);
        RCLCPP_INFO(this->get_logger(), "erpm_slew_rate   = %.0f ERPM/s", erpm_slew_rate_);
        RCLCPP_INFO(this->get_logger(), "auto_speed_erpm  = %.0f", auto_speed_erpm_);

        is_armed_     = false;
        linear_cmd_   = 0.0;
        angular_cmd_  = 0.0;
        speed_step_   = 3000.0;    // стартовая скорость, ERPM
        turn_factor_  = 0.6;

        cruise_active_ = false;
        cruise_speed_  = 0.0;

        auto_mode_  = false;
        correction_ = 0.0;

        smooth_left_  = 0.0;
        smooth_right_ = 0.0;

        // ВАЖНО: режим скорости
        left_pub_  = this->create_publisher<std_msgs::msg::Float64>("/left/commands/motor/speed", 10);
        right_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right/commands/motor/speed", 10);

        publishMotors(0.0, 0.0);

        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/telega_commands", 10,
            std::bind(&VescTeleopNode::cmdCallback, this, std::placeholders::_1));

        correction_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/correction", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { correction_ = msg->data; });

        // === НОВОЕ: клиенты сервисов одометрии ===
        set_target_client_ = this->create_client<std_srvs::srv::Empty>(
            "/odometry_pkg_node/set_target_heading");
        clear_target_client_ = this->create_client<std_srvs::srv::Empty>(
            "/odometry_pkg_node/clear_target_heading");

        timer_ = this->create_wall_timer(20ms, std::bind(&VescTeleopNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "==============================================");
        RCLCPP_INFO(this->get_logger(), "  VESC Teleop — 2 мотора, режим СКОРОСТИ (ERPM)");
        RCLCPP_INFO(this->get_logger(), "  Топики: /left|right/commands/motor/speed");
        RCLCPP_INFO(this->get_logger(), "==============================================");
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
    // ================= КЛАВИШИ =================
    void handleKey(int ch) {
        std::lock_guard<std::mutex> lk(mtx_);

        if (ch == 'e' || ch == 'E') {
            is_armed_ = !is_armed_;
            if (is_armed_) RCLCPP_WARN(this->get_logger(), "ARMED");
            else { resetMotion(); RCLCPP_WARN(this->get_logger(), "DISARMED"); }
            printStatus();
            return;
        }
        if (ch == ' ') { emergencyStop(); return; }

        if (!is_armed_) { std::cout << "\n[!] Сначала ARM (E)\n"; return; }

        if (ch == 'm' || ch == 'M') {
            if (!auto_mode_) {
                // === ВХОД В АВТОНОМ: фиксируем эталон курса ===
                if (!set_target_client_->service_is_ready()) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "AUTO: сервис /odometry_pkg_node/set_target_heading недоступен!");
                    return;
                }
                auto req = std::make_shared<std_srvs::srv::Empty::Request>();
                set_target_client_->async_send_request(req);

                auto_mode_   = true;
                linear_cmd_  = auto_speed_erpm_;
                angular_cmd_ = 0.0;
                RCLCPP_WARN(this->get_logger(),
                            "AUTO MODE ON @ %.0f ERPM (heading locked)", auto_speed_erpm_);
            } else {
                // === ВЫХОД ИЗ АВТОНОМА ===
                if (clear_target_client_->service_is_ready()) {
                    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
                    clear_target_client_->async_send_request(req);
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "AUTO: сервис clear_target_heading недоступен, сбрасываем локально");
                }
                auto_mode_   = false;
                linear_cmd_  = 0.0;
                angular_cmd_ = 0.0;
                correction_  = 0.0;
                RCLCPP_WARN(this->get_logger(), "AUTO MODE OFF");
            }
            printStatus();
            return;
        }

        if (ch == 'q' || ch == 'Q') {
            if (auto_mode_) { std::cout << "\n[!] Круиз недоступен в авто\n"; return; }
            cruise_active_ = !cruise_active_;
            if (cruise_active_) {
                cruise_speed_ = speed_step_;       // базовая скорость = текущая
                linear_cmd_   = cruise_speed_;
                angular_cmd_  = 0.0;
                RCLCPP_WARN(this->get_logger(), "CRUISE ON @ %.0f ERPM", cruise_speed_);
            } else {
                linear_cmd_  = 0.0;
                angular_cmd_ = 0.0;
                RCLCPP_WARN(this->get_logger(), "CRUISE OFF");
            }
            printStatus();
            return;
        }

        // Скорость — работает и в ручном, и в круизе
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

        // Острота поворота
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

        // Залипающие команды движения
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

    void resetMotion() {
        linear_cmd_ = angular_cmd_ = 0.0;
        cruise_active_ = auto_mode_ = false;
        cruise_speed_ = 0.0;
    }

    // ================= СМЕШИВАНИЕ =================
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
        // Если были в автономе — снимаем эталон курса
        if (auto_mode_ && clear_target_client_->service_is_ready()) {
            auto req = std::make_shared<std_srvs::srv::Empty::Request>();
            clear_target_client_->async_send_request(req);
        }
        resetMotion();
        is_armed_ = false;
        smooth_left_ = smooth_right_ = 0.0;
        RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP");
        publishMotors(0.0, 0.0);
        printStatus();
    }

    // ================= TCP =================
    void cmdCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;
        if (cmd.size() == 1) { handleKey(cmd[0]); return; }

        std::lock_guard<std::mutex> lk(mtx_);
        if (cmd == "arm" || cmd == "ARM") {
            is_armed_ = true;
            RCLCPP_WARN(this->get_logger(), "ARMED (TCP)");
            printStatus(); return;
        }
        if (cmd == "disarm" || cmd == "DISARM" || cmd == "stop" || cmd == "STOP") {
            if (auto_mode_ && clear_target_client_->service_is_ready()) {
                auto req = std::make_shared<std_srvs::srv::Empty::Request>();
                clear_target_client_->async_send_request(req);
            }
            resetMotion(); is_armed_ = false;
            publishMotors(0.0, 0.0);
            RCLCPP_WARN(this->get_logger(), "DISARMED (TCP)");
            printStatus(); return;
        }
        RCLCPP_WARN(this->get_logger(), "Неизвестная команда: '%s'", cmd.c_str());
    }

    // ================= ГЛАВНЫЙ ЦИКЛ =================
    void controlLoop() {
        double v = 0.0, w = 0.0;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (is_armed_) {
                if (auto_mode_) {
                    v = linear_cmd_;
                    double w_corr = correction_ * correction_scale_;
                    if (invert_correction_) w_corr = -w_corr;
                    w_corr = std::clamp(w_corr, -max_erpm_ * 0.5, max_erpm_ * 0.5);
                    w = w_corr;
                } else if (cruise_active_) {
                    v = linear_cmd_;   // подруливание работает
                    w = angular_cmd_;
                } else {
                    v = linear_cmd_;
                    w = angular_cmd_;
                }
            }
        }

        auto [tl, tr] = mixMotors(v, w);

        // === Slew-rate limiter в узле: не даём заданию прыгать ===
        const double dt = 0.02;
        const double max_delta = erpm_slew_rate_ * dt;

        smooth_left_  += std::clamp(tl - smooth_left_,  -max_delta, max_delta);
        smooth_right_ += std::clamp(tr - smooth_right_, -max_delta, max_delta);

        // Мёртвая зона: ниже min_erpm — в ноль, чтобы VESC не дёргался
        if (std::abs(tl) < min_erpm_ && std::abs(smooth_left_)  < min_erpm_) smooth_left_  = 0.0;
        if (std::abs(tr) < min_erpm_ && std::abs(smooth_right_) < min_erpm_) smooth_right_ = 0.0;

        publishMotors(smooth_left_, smooth_right_);

        static int cnt = 0;
        if (++cnt >= 25) {
            cnt = 0;
            const char* mode = auto_mode_ ? "AUTO" : (cruise_active_ ? "CRUISE" : "MANUAL");
            RCLCPP_INFO(this->get_logger(),
                        "%s | L=%.0f R=%.0f ERPM | v=%.0f w=%.0f step=%.0f turn=%.2f",
                        mode, smooth_left_, smooth_right_, v, w, speed_step_, turn_factor_);
        }
    }

    void printHelp() {
        std::cout << "\n=======================================================\n"
                  << "  VESC Teleop — 2 мотора, режим СКОРОСТИ (ERPM)\n"
                  << "-------------------------------------------------------\n"
                  << "  [E]        — ARM / DISARM\n"
                  << "  [M]        — АВТОНОМНЫЙ РЕЖИМ\n"
                  << "  [R] / [F]  — Больше/меньше скорость (работает и в круизе)\n"
                  << "  [Q]        — Круиз (база = текущая скорость)\n"
                  << "  [W] [S]    — Вперёд / Назад (залипание, повторно — стоп)\n"
                  << "  [A] [D]    — Поворот влево/вправо (залипание)\n"
                  << "  [,] [.]    — Меньше/больше острота поворота\n"
                  << "  [Пробел]   — АВАРИЙНЫЙ СТОП\n"
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

    // === Поля ===
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_, right_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr correction_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex mtx_;

    // === НОВЫЕ поля ===
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr set_target_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_target_client_;
    double auto_speed_erpm_;

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