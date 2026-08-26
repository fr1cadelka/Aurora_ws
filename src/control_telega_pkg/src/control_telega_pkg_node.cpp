#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <std_msgs/msg/string.hpp>  // Добавлено для подписчика
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
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

class MotorTeleopNode : public rclcpp::Node {
public:
    MotorTeleopNode() : Node("control_telega_pkg_node") {
        is_connected_ = false;
        is_armed_ = false;

        linear_throttle_ = 1500;
        angular_steering_ = 1500;

        // Настройки ручного шага скорости
        manual_speed_step_ = 50;  // Изначально маленькая скорость (смещение от нейтрали 1500)

        // Настройки круиз-контроля (строго фиксированная маленькая скорость)
        cruise_control_active_ = false;
        cruise_speed_pwm_ = 1580; // Маленькая фиксированная скорость для круиза вперед

        last_throttle_press_time_ = this->now();
        last_steering_press_time_ = this->now();

        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&MotorTeleopNode::stateCallback, this, std::placeholders::_1));

        // ДОБАВЛЕН ПОДПИСЧИК НА КОМАНДЫ ОТ TCP КЛИЕНТА
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/telega_commands", 10,  // Топик для команд тележки
            std::bind(&MotorTeleopNode::cmdCallback, this, std::placeholders::_1));

        command_client_ = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

        timer_ = this->create_wall_timer(50ms, std::bind(&MotorTeleopNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Узел тележки (динамическая скорость + круиз + TCP команды) запущен.");
        printHelp();
        printStatus();
    }

    // ДОБАВЛЕН МЕТОД ОБРАБОТКИ КОМАНД ОТ TCP
    void cmdCallback(const std_msgs::msg::String::SharedPtr msg) {
        // Не обрабатываем, если узел не активен
        if (!rclcpp::ok()) return;

        std::string command = msg->data;

        // Логируем полученную команду
        RCLCPP_DEBUG(this->get_logger(), "Получена TCP команда: '%s'", command.c_str());

        // Отключаем круиз-контроль при любой команде движения
        if (cruise_control_active_ &&
            (command == "w" || command == "W" ||
             command == "s" || command == "S")) {
            cruise_control_active_ = false;
            std::cout << "\n[Круиз] Отключен через TCP команду.\n";
        }

        // --- Обработка изменения скорости ---
        if (command == "r" || command == "R") {
            manual_speed_step_ = std::min(450, manual_speed_step_ + 25);
            printStatus();
        }
        else if (command == "f" || command == "F") {
            manual_speed_step_ = std::max(25, manual_speed_step_ - 25);
            printStatus();
        }

        // --- Линейное управление ---
        else if (command == "w" || command == "W") {
            linear_throttle_ = 1500 + manual_speed_step_;
            last_throttle_press_time_ = this->now();
        }
        else if (command == "s" || command == "S") {
            linear_throttle_ = 1500 - manual_speed_step_;
            last_throttle_press_time_ = this->now();
        }

        // --- Рулевое управление ---
        else if (command == "a" || command == "A") {
            angular_steering_ = 1350; // Поворот влево
            last_steering_press_time_ = this->now();
        }
        else if (command == "d" || command == "D") {
            angular_steering_ = 1650; // Поворот вправо
            last_steering_press_time_ = this->now();
        }

        // --- Круиз-контроль ---
        else if (command == "q" || command == "Q") {
            cruise_control_active_ = !cruise_control_active_;
            linear_throttle_ = 1500;
            angular_steering_ = 1500;
            if (cruise_control_active_) {
                std::cout << "\n[КРУИЗ] АКТИВИРОВАН через TCP! Скорость: " << cruise_speed_pwm_ << " мкс.\n";
            } else {
                std::cout << "\n[КРУИЗ] ДЕАКТИВИРОВАН через TCP. Моторы остановлены.\n";
            }
        }

        // --- Тормоз ---
        else if (command == "space" || command == " ") {
            cruise_control_active_ = false;
            linear_throttle_ = 1500;
            angular_steering_ = 1500;
            std::cout << "\r[TCP Управление] ТОРМОЗ                         " << std::flush;
        }

        // --- Остановка (команда stop) ---
        else if (command == "stop" || command == "STOP") {
            cruise_control_active_ = false;
            linear_throttle_ = 1500;
            angular_steering_ = 1500;
            std::cout << "\r[TCP Управление] СТОП                          " << std::flush;
        }
    }

    void readKeyboard() {
        while (rclcpp::ok()) {
            int ch = getch();

            if (cruise_control_active_ && (ch == 'w' || ch == 'W' || ch == 's' || ch == 'S')) {
                cruise_control_active_ = false;
                std::cout << "\n[Круиз] Отключен пользователем через изменение скорости (W/S).\n";
            }

            // --- Изменение чувствительности ручной скорости (Инкремент/Декремент) ---
            if (ch == 'r' || ch == 'R') {
                manual_speed_step_ = std::min(450, manual_speed_step_ + 25); // Максимум смещение 450 (до 1950 мкс)
                printStatus();
            }
            else if (ch == 'f' || ch == 'F') {
                manual_speed_step_ = std::max(25, manual_speed_step_ - 25);  // Минимум смещение 25 (до 1525 мкс)
                printStatus();
            }

            // --- Линейное ручное управление (динамическая скорость) ---
            else if (ch == 'w' || ch == 'W') {
                linear_throttle_ = 1500 + manual_speed_step_;
                last_throttle_press_time_ = this->now();
            }
            else if (ch == 's' || ch == 'S') {
                linear_throttle_ = 1500 - manual_speed_step_;
                last_throttle_press_time_ = this->now();
            }
            // --- Рулевое управление ---
            else if (ch == 'a' || ch == 'A') {
                angular_steering_ = 1350; // Поворот влево
                last_steering_press_time_ = this->now();
            }
            else if (ch == 'd' || ch == 'D') {
                angular_steering_ = 1650; // Поворот вправо
                last_steering_press_time_ = this->now();
            }
            // --- Круиз-контроль ---
            else if (ch == 'q' || ch == 'Q') {
                cruise_control_active_ = !cruise_control_active_;
                linear_throttle_ = 1500;
                angular_steering_ = 1500;
                if (cruise_control_active_) {
                    std::cout << "\n[КРУИЗ] АКТИВИРОВАН! Постоянное движение на маленькой скорости: " << cruise_speed_pwm_ << " мкс.\n";
                } else {
                    std::cout << "\n[КРУИЗ] ДЕАКТИВИРОВАН. Моторы остановлены.\n";
                }
            }
            // --- Тормоз ---
            else if (ch == ' ') {
                cruise_control_active_ = false;
                linear_throttle_ = 1500;
                angular_steering_ = 1500;
                std::cout << "\r[Управление] МГНОВЕННЫЙ ТОРМОЗ                         " << std::flush;
            }
        }
    }

private:
    void printHelp() {
        std::cout << "\n=================================================\n"
                  << "Управление с динамической скоростью и круизом:\n"
                  << "  Удерживайте [W] / [S] : Вперед / Назад (на ручной скорости)\n"
                  << "  Удерживайте [A] / [D] : Поворот Влево / Вправо\n"
                  << "  Нажмите [R] / [F]     : Повысить / Снизить ручную скорость\n"
                  << "  Нажмите [Q]           : Вкл/Выкл Круиз-контроль (всегда малая скорость)\n"
                  << "  Нажмите [Пробел]      : Экстренный тормоз\n"
                  << "  TCP команды на топике : /telega_commands\n"
                  << "  Выход из программы    : [Ctrl + C]\n"
                  << "=================================================\n\n";
    }

    void printStatus() {
        std::cout << "\r[Настройка] Выставленный ручной ШИМ: вперед="
                  << (1500 + manual_speed_step_) << " мкс | назад="
                  << (1500 - manual_speed_step_) << " мкс        " << std::flush;
    }

    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
        is_connected_ = msg->connected;
        if (is_armed_ != msg->armed) {
            is_armed_ = msg->armed;
            if (is_armed_) {
                std::cout << "\n[СТАТУС] ArduPilot в режиме ARMED! Тележка готова.\n";
            } else {
                std::cout << "\n[СТАТУС] ArduPilot в режиме DISARMED! Моторы заблокированы.\n";
            }
        }
    }

    void controlLoop() {
        if (!is_connected_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Ожидание подключения к ArduPilot...");
            return;
        }

        auto current_time = this->now();

        // Проверка таймаута руля (A/D)
        auto elapsed_steering = current_time - last_steering_press_time_;
        if (elapsed_steering.nanoseconds() > 200000000) {
            angular_steering_ = 1500;
        }

        // Проверка таймаута газа (W/S) — только если круиз выключен
        if (!cruise_control_active_) {
            auto elapsed_throttle = current_time - last_throttle_press_time_;
            if (elapsed_throttle.nanoseconds() > 200000000) {
                linear_throttle_ = 1500;
            }
        }

        int pwm_motor1 = 1500;
        int pwm_motor2 = 1500;

        if (is_armed_) {
            // Круиз всегда едет на фиксированной малой скорости cruise_speed_pwm_, игнорируя шаги
            int current_linear = cruise_control_active_ ? cruise_speed_pwm_ : linear_throttle_;

            int steering_offset = angular_steering_ - 1500;

            pwm_motor1 = current_linear + steering_offset;
            pwm_motor2 = current_linear - steering_offset;

            pwm_motor1 = std::clamp(pwm_motor1, 1000, 2000);
            pwm_motor2 = std::clamp(pwm_motor2, 1000, 2000);
        } else {
            linear_throttle_ = 1500;
            angular_steering_ = 1500;
            cruise_control_active_ = false;
        }

        //Фргамент кода котоырй отвечает за выбор выода шим сигнала
        sendServoCommand(1, pwm_motor1);
        sendServoCommand(2, pwm_motor2);
    }

    void sendServoCommand(int channel, int pwm_value) {
        if (!command_client_->service_is_ready()) {
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        request->broadcast = false;
        request->command = 183;
        request->param1 = static_cast<float>(channel);
        request->param2 = static_cast<float>(pwm_value);
        request->param3 = 0.0;
        request->param4 = 0.0;
        request->param5 = 0.0;
        request->param6 = 0.0;
        request->param7 = 0.0;

        auto result = command_client_->async_send_request(request);
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;  // ДОБАВЛЕН ПОДПИСЧИК
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_connected_;
    bool is_armed_;
    int linear_throttle_;
    int angular_steering_;

    // Переменные динамической ручной скорости и круиза
    int manual_speed_step_;
    bool cruise_control_active_;
    int cruise_speed_pwm_;

    rclcpp::Time last_throttle_press_time_;
    rclcpp::Time last_steering_press_time_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorTeleopNode>();

    std::thread keyboard_thread(&MotorTeleopNode::readKeyboard, node);

    rclcpp::spin(node);

    if (keyboard_thread.joinable()) {
        keyboard_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}
