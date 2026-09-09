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
        speed_step_ = 3.0;
        max_current_ = 15.0;
        turn_factor_ = 0.6;

        left_pub_  = this->create_publisher<std_msgs::msg::Float64>("/left/commands/motor/current", 10);
        right_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right/commands/motor/current", 10);

        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/telega_commands", 10,
            std::bind(&VescTeleopNode::cmdCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(50ms, std::bind(&VescTeleopNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "==============================================");
        RCLCPP_INFO(this->get_logger(), "  VESC Teleop — 2 мотора (left / right)");
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
                    std::cout << "\n[СТАТУС] >>> DISARMED <<<\n";
                    RCLCPP_WARN(this->get_logger(), "DISARMED");
                }
                printStatus();
                continue;
            }

            if (!is_armed_) continue;

            // Регулировка скорости
            if (ch == 'r' || ch == 'R') {
                speed_step_ = std::min(speed_step_ + 1.0, max_current_);
                printStatus();
            }
            else if (ch == 'f' || ch == 'F') {
                speed_step_ = std::max(1.0, speed_step_ - 1.0);
                printStatus();
            }
            // Вперёд
            else if (ch == 'd' || ch == 'D') {
                linear_cmd_ = speed_step_;
                last_linear_time_ = this->now();
            }
            // Назад
            else if (ch == 'a' || ch == 'A') {
                linear_cmd_ = -speed_step_;
                last_linear_time_ = this->now();
            }
            // Влево
            else if (ch == 'w' || ch == 'W') {
                angular_cmd_ = -speed_step_ * turn_factor_;
                last_angular_time_ = this->now();
            }
            // Вправо
            else if (ch == 's' || ch == 'S') {
                angular_cmd_ = speed_step_ * turn_factor_;
                last_angular_time_ = this->now();
            }
            // Аварийный стоп
            else if (ch == ' ') {
                linear_cmd_ = 0.0;
                angular_cmd_ = 0.0;
                is_armed_ = false;
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
        else if (cmd == "a" || cmd == "A") {
            angular_cmd_ = -speed_step_ * turn_factor_;
            last_angular_time_ = this->now();
        }
        else if (cmd == "d" || cmd == "D") {
            angular_cmd_ = speed_step_ * turn_factor_;
            last_angular_time_ = this->now();
        }
        else if (cmd == " " || cmd == "space") {
            linear_cmd_ = 0.0;
            angular_cmd_ = 0.0;
            is_armed_ = false;
        }
    }

    void controlLoop() {
        auto now = this->now();

        // Таймаут отпускания клавиш (0.2 сек)
        if ((now - last_linear_time_).seconds() > 0.20) {
            linear_cmd_ = 0.0;
        }
        if ((now - last_angular_time_).seconds() > 0.20) {
            angular_cmd_ = 0.0;
        }

        double left  = 0.0;
        double right = 0.0;

        if (is_armed_) {
            left  = linear_cmd_ + angular_cmd_;
            right = linear_cmd_ - angular_cmd_;

            left  = std::clamp(left,  -max_current_, max_current_);
            right = std::clamp(right, -max_current_, max_current_);
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
            if (is_armed_) {
                RCLCPP_INFO(this->get_logger(), "ARMED | L=%.1f A | R=%.1f A | step=%.1f",
                            left, right, speed_step_);
            }
        }
    }

    void printHelp() {
        std::cout << "\n=======================================================\n"
                  << "  Управление двумя VESC (дифференциал)\n"
                  << "-------------------------------------------------------\n"
                  << "  [E]        — ARM / DISARM\n"
                  << "  [W]        — Вперёд\n"
                  << "  [S]        — Назад\n"
                  << "  [A]        — Влево\n"
                  << "  [D]        — Вправо\n"
                  << "  [R] / [F]  — Быстрее / Медленнее\n"
                  << "  [Пробел]   — Аварийный стоп + DISARM\n"
                  << "=======================================================\n\n";
    }

    void printStatus() {
        std::cout << "\r[Статус] "
                  << (is_armed_ ? "ARMED   " : "DISARMED")
                  << " | Скорость = " << speed_step_ << " A"
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
    double max_current_;
    double turn_factor_;

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
