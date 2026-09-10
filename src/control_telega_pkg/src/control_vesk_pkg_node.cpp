#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>

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
        // === Параметры ===
        this->declare_parameter<double>("correction_scale", 500.0);
        this->declare_parameter<bool>("invert_correction", false);

        this->get_parameter("correction_scale", correction_scale_);
        this->get_parameter("invert_correction", invert_correction_);

        RCLCPP_INFO(this->get_logger(), "correction_scale = %.1f", correction_scale_);
        RCLCPP_INFO(this->get_logger(), "invert_correction = %s", invert_correction_ ? "true" : "false");

        is_armed_ = false;
        linear_cmd_ = 0.0;
        angular_cmd_ = 0.0;
        speed_step_ = 3000.0;
        max_speed_ = 15000.0;
        turn_factor_ = 0.6;

        cruise_active_ = false;
        cruise_speed_ = 3000.0;

        auto_mode_ = false;
        auto_linear_speed_ = 3000.0;
        correction_ = 0.0;

        left_pub_  = this->create_publisher<std_msgs::msg::Float64>("/left/commands/motor/speed", 10);
        right_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right/commands/motor/speed", 10);

        // === ОТПРАВКА НУЛЕЙ ПРИ СТАРТЕ ===
        std_msgs::msg::Float64 zero_msg;
        zero_msg.data = 0.0;
        left_pub_->publish(zero_msg);
        right_pub_->publish(zero_msg);
        RCLCPP_INFO(this->get_logger(), "Отправлены нулевые команды на VESC при старте");

        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/telega_commands", 10,
            std::bind(&VescTeleopNode::cmdCallback, this, std::placeholders::_1));

        correction_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/correction", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                correction_ = msg->data;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "Получена коррекция: %.3f", correction_);
            });

        timer_ = this->create_wall_timer(50ms, std::bind(&VescTeleopNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "==============================================");
        RCLCPP_INFO(this->get_logger(), "  VESC Teleop — 2 мотора (left / right)");
        RCLCPP_INFO(this->get_logger(), "  РЕЖИМ: Скорость (RPM)");
        RCLCPP_INFO(this->get_logger(), "  Топик команд: /telega_commands");
        RCLCPP_INFO(this->get_logger(), "==============================================");
        printHelp();
        printStatus();
    }

    void readKeyboard() {
        while (rclcpp::ok()) {
            int ch = getch();

            // ARM / DISARM
            if (ch == 'e' || ch == 'E') {
                is_armed_ = !is_armed_;
                if (is_armed_) {
                    std::cout << "\n[СТАТУС] >>> ARMED <<<\n";
                    RCLCPP_WARN(this->get_logger(), "ARMED");
                } else {
                    linear_cmd_ = 0.0;
                    angular_cmd_ = 0.0;
                    cruise_active_ = false;
                    auto_mode_ = false;
                    std::cout << "\n[СТАТУС] >>> DISARMED <<<\n";
                    RCLCPP_WARN(this->get_logger(), "DISARMED");
                    // При DISARM сразу отправляем нули
                    std_msgs::msg::Float64 zero_msg;
                    zero_msg.data = 0.0;
                    left_pub_->publish(zero_msg);
                    right_pub_->publish(zero_msg);
                }
                printStatus();
                continue;
            }

            // Круиз-контроль (ручной режим)
            if (ch == 'q' || ch == 'Q') {
                if (is_armed_ && !auto_mode_) {
                    cruise_active_ = !cruise_active_;
                    if (cruise_active_) {
                        linear_cmd_ = 0.0;
                        angular_cmd_ = 0.0;
                        std::cout << "\n[КРУИЗ-КОНТРОЛЬ] АКТИВЕН (3000 RPM)\n";
                        RCLCPP_WARN(this->get_logger(), "CRUISE CONTROL: ON");
                    } else {
                        linear_cmd_ = 0.0;
                        angular_cmd_ = 0.0;
                        std::cout << "\n[КРУИЗ-КОНТРОЛЬ] ВЫКЛЮЧЕН\n";
                        RCLCPP_WARN(this->get_logger(), "CRUISE CONTROL: OFF");
                    }
                    printStatus();
                    continue;
                } else if (auto_mode_) {
                    std::cout << "\n[ПРЕДУПРЕЖДЕНИЕ] Круиз-контроль недоступен в автономном режиме\n";
                }
                continue;
            }

            // === АВТОНОМНЫЙ РЕЖИМ (M) ===
            if (ch == 'm' || ch == 'M') {
                if (is_armed_) {
                    auto_mode_ = !auto_mode_;
                    if (auto_mode_) {
                        linear_cmd_ = speed_step_;
                        angular_cmd_ = 0.0;
                        auto_linear_speed_ = speed_step_;
                        std::cout << "\n[АВТОНОМНЫЙ РЕЖИМ] ВКЛЮЧЕН (движение вперёд с коррекцией курса)\n";
                        RCLCPP_WARN(this->get_logger(), "AUTO MODE: ON");
                    } else {
                        linear_cmd_ = 0.0;
                        angular_cmd_ = 0.0;
                        std::cout << "\n[АВТОНОМНЫЙ РЕЖИМ] ВЫКЛЮЧЕН\n";
                        RCLCPP_WARN(this->get_logger(), "AUTO MODE: OFF");
                        // Отправляем нули при выключении автонома
                        std_msgs::msg::Float64 zero_msg;
                        zero_msg.data = 0.0;
                        left_pub_->publish(zero_msg);
                        right_pub_->publish(zero_msg);
                    }
                    printStatus();
                    continue;
                } else {
                    std::cout << "\n[ПРЕДУПРЕЖДЕНИЕ] Сначала заармите робота (E)\n";
                }
                continue;
            }

            // Блокировка ручных команд в автономе (кроме R/F и пробела)
            if (!is_armed_ || auto_mode_) {
                if (!is_armed_) {
                    // игнорируем
                } else if (auto_mode_) {
                    if (ch == 'r' || ch == 'R') {
                        speed_step_ = std::min(speed_step_ + 100.0, max_speed_);
                        if (auto_mode_) linear_cmd_ = speed_step_;
                        printStatus();
                    }
                    else if (ch == 'f' || ch == 'F') {
                        speed_step_ = std::max(100.0, speed_step_ - 100.0);
                        if (auto_mode_) linear_cmd_ = speed_step_;
                        printStatus();
                    }
                    else if (ch == ' ') {
                        linear_cmd_ = 0.0;
                        angular_cmd_ = 0.0;
                        is_armed_ = false;
                        cruise_active_ = false;
                        auto_mode_ = false;
                        std::cout << "\n[АВАРИЙНЫЙ СТОП] DISARMED\n";
                        RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP");
                        // Отправляем нули
                        std_msgs::msg::Float64 zero_msg;
                        zero_msg.data = 0.0;
                        left_pub_->publish(zero_msg);
                        right_pub_->publish(zero_msg);
                        printStatus();
                    }
                }
                continue;
            }

            // РУЧНОЙ РЕЖИМ
            if (ch == 'r' || ch == 'R') {
                speed_step_ = std::min(speed_step_ + 100.0, max_speed_);
                printStatus();
            }
            else if (ch == 'f' || ch == 'F') {
                speed_step_ = std::max(100.0, speed_step_ - 100.0);
                printStatus();
            }
            else if (ch == 'w' || ch == 'W') {
                linear_cmd_ = speed_step_;
                last_linear_time_ = this->now();
            }
            else if (ch == 's' || ch == 'S') {
                linear_cmd_ = -speed_step_;
                last_linear_time_ = this->now();
            }
            else if (ch == 'a' || ch == 'A') {
                angular_cmd_ = speed_step_ * turn_factor_;
                last_angular_time_ = this->now();
            }
            else if (ch == 'd' || ch == 'D') {
                angular_cmd_ = -speed_step_ * turn_factor_;
                last_angular_time_ = this->now();
            }
            else if (ch == ' ') {
                linear_cmd_ = 0.0;
                angular_cmd_ = 0.0;
                is_armed_ = false;
                cruise_active_ = false;
                auto_mode_ = false;
                std::cout << "\n[АВАРИЙНЫЙ СТОП] DISARMED\n";
                RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP");
                std_msgs::msg::Float64 zero_msg;
                zero_msg.data = 0.0;
                left_pub_->publish(zero_msg);
                right_pub_->publish(zero_msg);
                printStatus();
            }
        }
    }

private:
    void cmdCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;
        RCLCPP_INFO(this->get_logger(), "📥 Получена команда: '%s'", cmd.c_str());

        // ARM/DISARM
        if (cmd == "e" || cmd == "E") {
            is_armed_ = !is_armed_;
            if (!is_armed_) {
                linear_cmd_ = 0.0;
                angular_cmd_ = 0.0;
                cruise_active_ = false;
                auto_mode_ = false;
                RCLCPP_WARN(this->get_logger(), "DISARMED");
                std_msgs::msg::Float64 zero_msg;
                zero_msg.data = 0.0;
                left_pub_->publish(zero_msg);
                right_pub_->publish(zero_msg);
            } else {
                RCLCPP_WARN(this->get_logger(), "ARMED");
            }
            printStatus();
            return;
        }
        if (cmd == "arm" || cmd == "ARM") {
            is_armed_ = true;
            RCLCPP_WARN(this->get_logger(), "ARMED");
            printStatus();
            return;
        }
        if (cmd == "disarm" || cmd == "DISARM" || cmd == "stop" || cmd == "STOP") {
            is_armed_ = false;
            linear_cmd_ = 0.0;
            angular_cmd_ = 0.0;
            cruise_active_ = false;
            auto_mode_ = false;
            RCLCPP_WARN(this->get_logger(), "DISARMED");
            std_msgs::msg::Float64 zero_msg;
            zero_msg.data = 0.0;
            left_pub_->publish(zero_msg);
            right_pub_->publish(zero_msg);
            printStatus();
            return;
        }

        if (!is_armed_) return;

        // Автономный режим (M)
        if (cmd == "m" || cmd == "M") {
            auto_mode_ = !auto_mode_;
            if (auto_mode_) {
                linear_cmd_ = speed_step_;
                angular_cmd_ = 0.0;
                auto_linear_speed_ = speed_step_;
                RCLCPP_WARN(this->get_logger(), "AUTO MODE ON (TCP)");
            } else {
                linear_cmd_ = 0.0;
                angular_cmd_ = 0.0;
                RCLCPP_WARN(this->get_logger(), "AUTO MODE OFF (TCP)");
                std_msgs::msg::Float64 zero_msg;
                zero_msg.data = 0.0;
                left_pub_->publish(zero_msg);
                right_pub_->publish(zero_msg);
            }
            printStatus();
            return;
        }

        // В автономном режиме игнорируем почти всё
        if (auto_mode_) {
            if (cmd == "r" || cmd == "R") {
                speed_step_ = std::min(speed_step_ + 100.0, max_speed_);
                if (auto_mode_) linear_cmd_ = speed_step_;
                printStatus();
                return;
            }
            if (cmd == "f" || cmd == "F") {
                speed_step_ = std::max(100.0, speed_step_ - 100.0);
                if (auto_mode_) linear_cmd_ = speed_step_;
                printStatus();
                return;
            }
            if (cmd == " " || cmd == "space") {
                linear_cmd_ = 0.0;
                angular_cmd_ = 0.0;
                is_armed_ = false;
                auto_mode_ = false;
                cruise_active_ = false;
                RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP (TCP)");
                std_msgs::msg::Float64 zero_msg;
                zero_msg.data = 0.0;
                left_pub_->publish(zero_msg);
                right_pub_->publish(zero_msg);
                printStatus();
                return;
            }
            RCLCPP_WARN(this->get_logger(), "Команда '%s' игнорируется в автономном режиме", cmd.c_str());
            return;
        }

        // РУЧНОЙ РЕЖИМ
        if (cmd == "q" || cmd == "Q") {
            cruise_active_ = !cruise_active_;
            if (cruise_active_) {
                linear_cmd_ = 0.0;
                angular_cmd_ = 0.0;
                RCLCPP_WARN(this->get_logger(), "CRUISE ON");
            } else {
                linear_cmd_ = 0.0;
                angular_cmd_ = 0.0;
                RCLCPP_WARN(this->get_logger(), "CRUISE OFF");
            }
            printStatus();
            return;
        }

        if (cmd == "r" || cmd == "R") {
            speed_step_ = std::min(speed_step_ + 100.0, max_speed_);
            printStatus();
            return;
        }
        if (cmd == "f" || cmd == "F") {
            speed_step_ = std::max(100.0, speed_step_ - 100.0);
            printStatus();
            return;
        }

        if (cmd == "w" || cmd == "W") {
            linear_cmd_ = speed_step_;
            last_linear_time_ = this->now();
        }
        else if (cmd == "s" || cmd == "S") {
            linear_cmd_ = -speed_step_;
            last_linear_time_ = this->now();
        }
        else if (cmd == "a" || cmd == "A") {
            angular_cmd_ = speed_step_ * turn_factor_;
            last_angular_time_ = this->now();
        }
        else if (cmd == "d" || cmd == "D") {
            angular_cmd_ = -speed_step_ * turn_factor_;
            last_angular_time_ = this->now();
        }
        else if (cmd == " " || cmd == "space") {
            linear_cmd_ = 0.0;
            angular_cmd_ = 0.0;
            is_armed_ = false;
            cruise_active_ = false;
            auto_mode_ = false;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP");
            std_msgs::msg::Float64 zero_msg;
            zero_msg.data = 0.0;
            left_pub_->publish(zero_msg);
            right_pub_->publish(zero_msg);
            printStatus();
        }
    }

    void controlLoop() {
        auto now = this->now();

        // === Если не заармлен, всегда отправляем нули ===
        if (!is_armed_) {
            std_msgs::msg::Float64 zero_msg;
            zero_msg.data = 0.0;
            left_pub_->publish(zero_msg);
            right_pub_->publish(zero_msg);
            // Логировать не будем, чтобы не засорять
            return;
        }

        // АВТОНОМНЫЙ РЕЖИМ
        if (auto_mode_) {
            // используем correction_ как угловую команду
            double effective_angular = correction_ * correction_scale_;
            if (invert_correction_) effective_angular = -effective_angular;
            effective_angular = std::clamp(effective_angular, -max_speed_ * 0.5, max_speed_ * 0.5);

            double left  = -linear_cmd_ - effective_angular;
            double right =  linear_cmd_ - effective_angular;

            left  = std::clamp(left,  -max_speed_, max_speed_);
            right = std::clamp(right, -max_speed_, max_speed_);

            std_msgs::msg::Float64 msg_l, msg_r;
            msg_l.data = left;
            msg_r.data = right;
            left_pub_->publish(msg_l);
            right_pub_->publish(msg_r);

            static int cnt = 0;
            if (++cnt >= 20) {
                cnt = 0;
                RCLCPP_INFO(this->get_logger(), "AUTO | L=%.1f RPM | R=%.1f RPM | corr=%.3f (scaled=%.1f)",
                            left, right, correction_, effective_angular);
            }
            return;
        }

        // РУЧНОЙ РЕЖИМ
        if (cruise_active_) {
            linear_cmd_ = cruise_speed_;
            angular_cmd_ = 0.0;
        } else {
            if ((now - last_linear_time_).seconds() > 0.20) {
                linear_cmd_ = 0.0;
            }
            if ((now - last_angular_time_).seconds() > 0.20) {
                angular_cmd_ = 0.0;
            }
        }

        double left  = 0.0;
        double right = 0.0;
        if (angular_cmd_ == 0.0) {
            left  = -linear_cmd_;
            right =  linear_cmd_;
        } else {
            left  = -linear_cmd_ - angular_cmd_;
            right =  linear_cmd_ - angular_cmd_;
        }
        left  = std::clamp(left,  -max_speed_, max_speed_);
        right = std::clamp(right, -max_speed_, max_speed_);

        std_msgs::msg::Float64 msg_l, msg_r;
        msg_l.data = left;
        msg_r.data = right;
        left_pub_->publish(msg_l);
        right_pub_->publish(msg_r);

        static int cnt = 0;
        if (++cnt >= 20) {
            cnt = 0;
            if (cruise_active_) {
                RCLCPP_INFO(this->get_logger(), "CRUISE | L=%.1f RPM | R=%.1f RPM", left, right);
            } else {
                RCLCPP_INFO(this->get_logger(), "MANUAL | L=%.1f RPM | R=%.1f RPM | step=%.1f",
                            left, right, speed_step_);
            }
        }
    }

    void printHelp() {
        std::cout << "\n=======================================================\n"
                  << "  Управление двумя VESC (дифференциал)\n"
                  << "  РЕЖИМ: Скорость (RPM)\n"
                  << "-------------------------------------------------------\n"
                  << "  [E]        — ARM / DISARM\n"
                  << "  [M]        — АВТОНОМНЫЙ РЕЖИМ (вкл/выкл)\n"
                  << "               В автономе робот едет прямо и корректирует курс\n"
                  << "  [R] / [F]  — Увеличить/уменьшить скорость (в автономе тоже работает)\n"
                  << "  [Q]        — Круиз-контроль (только в ручном режиме)\n"
                  << "  [W] [S]    — Вперёд / Назад (ручной режим)\n"
                  << "  [A] [D]    — Повороты (ручной режим)\n"
                  << "  [Пробел]   — Аварийный стоп + DISARM\n"
                  << "=======================================================\n\n";
    }

    void printStatus() {
        std::cout << "\r[Статус] "
                  << (is_armed_ ? "ARMED   " : "DISARMED")
                  << " | " << (auto_mode_ ? "AUTO ON " : "        ")
                  << " | " << (cruise_active_ ? "CRUISE  " : "        ")
                  << " | Скорость = " << speed_step_ << " RPM"
                  << "          " << std::flush;
    }

    // === Поля ===
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr correction_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_armed_;
    double linear_cmd_;
    double angular_cmd_;
    double speed_step_;
    double max_speed_;
    double turn_factor_;

    bool cruise_active_;
    double cruise_speed_;

    bool auto_mode_;
    double auto_linear_speed_;
    double correction_;

    double correction_scale_;
    bool invert_correction_;

    rclcpp::Time last_linear_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_angular_time_{0, 0, RCL_ROS_TIME};
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
