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
        is_armed_ = false;
        linear_cmd_ = 0.0;
        angular_cmd_ = 0.0;
        speed_step_ = 3000.0;
        max_speed_ = 15000.0;
        turn_factor_ = 0.6;

        // ===== Круиз-контроль =====
        cruise_active_ = false;
        cruise_speed_ = 3000.0;

        // Публикуем в топики СКОРОСТИ (RPM)
        left_pub_  = this->create_publisher<std_msgs::msg::Float64>("/left/commands/motor/speed", 10);
        right_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right/commands/motor/speed", 10);

        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/telega_commands", 10,
            std::bind(&VescTeleopNode::cmdCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(50ms, std::bind(&VescTeleopNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "==============================================");
        RCLCPP_INFO(this->get_logger(), "  VESC Teleop — 2 мотора (left / right)");
        RCLCPP_INFO(this->get_logger(), "  РЕЖИМ: Скорость (RPM)");
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
                    std::cout << "\n[СТАТУС] >>> DISARMED <<<\n";
                    RCLCPP_WARN(this->get_logger(), "DISARMED");
                }
                printStatus();
                continue;
            }

            // ===== Круиз-контроль на Q =====
            if (ch == 'q' || ch == 'Q') {
                if (is_armed_) {
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
                }
            }

            if (!is_armed_ || cruise_active_) continue;

            // Регулировка скорости
            if (ch == 'r' || ch == 'R') {
                speed_step_ = std::min(speed_step_ + 100.0, max_speed_);
                printStatus();
            }
            else if (ch == 'f' || ch == 'F') {
                speed_step_ = std::max(100.0, speed_step_ - 100.0);
                printStatus();
            }
            // Вперёд (W)
            else if (ch == 'w' || ch == 'W') {
                linear_cmd_ = speed_step_;
                last_linear_time_ = this->now();
            }
            // Назад (S)
            else if (ch == 's' || ch == 'S') {
                linear_cmd_ = -speed_step_;
                last_linear_time_ = this->now();
            }
            // Влево (A)
            else if (ch == 'd' || ch == 'D') {
                angular_cmd_ = -speed_step_ * turn_factor_;
                last_angular_time_ = this->now();
            }
            // Вправо (D)
            else if (ch == 'a' || ch == 'A') {
                angular_cmd_ = speed_step_ * turn_factor_;
                last_angular_time_ = this->now();
            }
            // Аварийный стоп
            else if (ch == ' ') {
                linear_cmd_ = 0.0;
                angular_cmd_ = 0.0;
                is_armed_ = false;
                cruise_active_ = false;
                std::cout << "\n[АВАРИЙНЫЙ СТОП] DISARMED\n";
                RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP");
                printStatus();
            }
        }
    }

private:
    void cmdCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;

        if (cmd == "arm" || cmd == "ARM") {
            is_armed_ = true;
            RCLCPP_WARN(this->get_logger(), "TCP: ARMED");
        }
        else if (cmd == "disarm" || cmd == "DISARM" || cmd == "stop" || cmd == "STOP") {
            is_armed_ = false;
            linear_cmd_ = 0.0;
            angular_cmd_ = 0.0;
            cruise_active_ = false;
            RCLCPP_WARN(this->get_logger(), "TCP: DISARMED");
        }
        else if (!is_armed_) return;
        else if (cmd == "w" || cmd == "W") {
            linear_cmd_ = speed_step_;
            last_linear_time_ = this->now();
        }
        else if (cmd == "s" || cmd == "S") {
            linear_cmd_ = -speed_step_;
            last_linear_time_ = this->now();
        }
        else if (cmd == "d" || cmd == "D") {
            angular_cmd_ = -speed_step_ * turn_factor_;
            last_angular_time_ = this->now();
        }
        else if (cmd == "a" || cmd == "A") {
            angular_cmd_ = speed_step_ * turn_factor_;
            last_angular_time_ = this->now();
        }
        else if (cmd == " " || cmd == "space") {
            linear_cmd_ = 0.0;
            angular_cmd_ = 0.0;
            is_armed_ = false;
            cruise_active_ = false;
        }
        else if (cmd == "cruise_on" || cmd == "CRUISE_ON") {
            if (is_armed_) cruise_active_ = true;
        }
        else if (cmd == "cruise_off" || cmd == "CRUISE_OFF") {
            cruise_active_ = false;
        }
    }

    void controlLoop() {
        auto now = this->now();

        // ===== Круиз-контроль =====
        if (cruise_active_ && is_armed_) {
            linear_cmd_ = cruise_speed_;
            angular_cmd_ = 0.0;
        } else {
            // Таймаут отпускания клавиш (0.2 сек)
            if ((now - last_linear_time_).seconds() > 0.20) {
                linear_cmd_ = 0.0;
            }
            if ((now - last_angular_time_).seconds() > 0.20) {
                angular_cmd_ = 0.0;
            }
        }

        double left  = 0.0;
        double right = 0.0;

        if (is_armed_) {
            // ===== ЗЕРКАЛЬНАЯ ЛОГИКА =====
            // Если едем прямо (или круиз), моторы вращаются в РАЗНЫЕ стороны.
            if (angular_cmd_ == 0.0) {
                left  = -linear_cmd_;
                right =  linear_cmd_;
            }
            // Если поворачиваем, моторы вращаются в ОДНУ сторону.
            else {
                left  = -linear_cmd_ - angular_cmd_;
                right = linear_cmd_ - angular_cmd_;
            }

            left  = std::clamp(left,  -max_speed_, max_speed_);
            right = std::clamp(right, -max_speed_, max_speed_);
        }

        // Публикуем команды
        std_msgs::msg::Float64 msg_l, msg_r;
        msg_l.data = left;
        msg_r.data = right;

        left_pub_->publish(msg_l);
        right_pub_->publish(msg_r);

        // Лог раз в секунду
        static int cnt = 0;
        if (++cnt >= 20) {
            cnt = 0;
            if (is_armed_ && cruise_active_) {
                RCLCPP_INFO(this->get_logger(), "CRUISE ACTIVE | L=%.1f RPM | R=%.1f RPM", left, right);
            } else if (is_armed_) {
                RCLCPP_INFO(this->get_logger(), "ARMED | L=%.1f RPM | R=%.1f RPM | step=%.1f",
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
                  << "  [Q]        — Круиз-контроль (3000 RPM, вкл/выкл)\n"
                  << "  [W]        — Вперёд\n"
                  << "  [S]        — Назад\n"
                  << "  [A]        — Влево\n"
                  << "  [D]        — Вправо\n"
                  << "  [R] / [F]  — Быстрее / Медленнее (RPM)\n"
                  << "  [Пробел]   — Аварийный стоп + DISARM\n"
                  << "=======================================================\n\n";
    }

    void printStatus() {
        std::cout << "\r[Статус] "
                  << (is_armed_ ? "ARMED   " : "DISARMED")
                  << " | " << (cruise_active_ ? "CRUISE ON " : "          ")
                  << " | Скорость = " << speed_step_ << " RPM"
                  << "          " << std::flush;
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_armed_;
    double linear_cmd_;
    double angular_cmd_;
    double speed_step_;
    double max_speed_;
    double turn_factor_;

    // ===== Круиз-контроль =====
    bool cruise_active_;
    double cruise_speed_;

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
