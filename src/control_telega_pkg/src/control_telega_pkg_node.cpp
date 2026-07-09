#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>

using namespace std::chrono_literals;

// Функция для неблокирующего чтения одиночных символов с клавиатуры
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

        // Базовые значения (1500 — стоп для ArduRover)
        linear_throttle_ = 1500;
        angular_steering_ = 1500;

        // Подписка на состояние MAVROS
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&MotorTeleopNode::stateCallback, this, std::placeholders::_1));

        // Клиент сервиса для отправки DO_SET_SERVO
        command_client_ = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

        // Таймер отправки команд (10 Гц)
        timer_ = this->create_wall_timer(100ms, std::bind(&MotorTeleopNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Узел управления двухмоторной тележкой запущен.");
        printHelp();
    }

    void readKeyboard() {
        while (rclcpp::ok()) {
            int ch = getch();
            if (ch == 'w' || ch == 'W') {
                linear_throttle_ = std::min(2000, linear_throttle_ + 25);
                printStatus();
            }
            else if (ch == 's' || ch == 'S') {
                linear_throttle_ = std::max(1000, linear_throttle_ - 25);
                printStatus();
            }
            else if (ch == 'a' || ch == 'A') {
                angular_steering_ = std::max(1000, angular_steering_ - 25); // Поворот влево
                printStatus();
            }
            else if (ch == 'd' || ch == 'D') {
                angular_steering_ = std::min(2000, angular_steering_ + 25); // Поворот вправо
                printStatus();
            }
            else if (ch == ' ') { // Пробел — полная остановка и сброс
                linear_throttle_ = 1500;
                angular_steering_ = 1500;
                std::cout << "\r[Управление] МГНОВЕННЫЙ ТОРМОЗ (1500 мкс)             " << std::flush;
            }
            else if (ch == 'q' || ch == 'Q') {
                RCLCPP_INFO(this->get_logger(), "Выход из программы.");
                sendServoCommand(1, 1500); // Глушим мотор 1
                sendServoCommand(2, 1500); // Глушим мотор 2
                rclcpp::shutdown();
                break;
            }
        }
    }

private:
    void printHelp() {
        std::cout << "\n=================================================\n"
                  << "Управление двухмоторной тележкой (Servo 1 и 2):\n"
                  << "  [W] / [S] : Вперед / Назад (Изменение базы газа)\n"
                  << "  [A] / [D] : Поворот Влево / Вправо\n"
                  << "  [Пробел]  : Мгновенно остановить оба мотора\n"
                  << "  [Q]       : Выйти из узла\n"
                  << "=================================================\n\n";
    }

    void printStatus() {
        std::cout << "\r[База] Газ: " << linear_throttle_
                  << " | Руль: " << angular_steering_ << "      " << std::flush;
    }

    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
        is_connected_ = msg->connected;
        if (is_armed_ != msg->armed) {
            is_armed_ = msg->armed;
            if (is_armed_) {
                std::cout << "\n[СТАТУС] ArduPilot в режиме ARMED! Моторы готовы.\n";
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

        int pwm_motor1 = 1500;
        int pwm_motor2 = 1500;

        if (is_armed_) {
            // Дифференциальное микширование (Skid Steering)
            // Отклонение руля от нейтрали (1500)
            int steering_offset = angular_steering_ - 1500;

            // Расчет для двух бортов
            pwm_motor1 = linear_throttle_ + steering_offset; // Левый борт (Servo 1)
            pwm_motor2 = linear_throttle_ - steering_offset; // Правый борт (Servo 2)

            // Ограничиваем ШИМ жесткими рамками [1000, 2000]
            pwm_motor1 = std::clamp(pwm_motor1, 1000, 2000);
            pwm_motor2 = std::clamp(pwm_motor2, 1000, 2000);
        } else {
            // Если Disarmed — жесткий сброс всех переменных
            linear_throttle_ = 1500;
            angular_steering_ = 1500;
        }

        // Отправляем команды на оба физических пина последовательно
        sendServoCommand(1, pwm_motor1); // Первому мотору на Servo 1
        sendServoCommand(2, pwm_motor2); // Второму мотору на Servo 2
    }

    void sendServoCommand(int channel, int pwm_value) {
        if (!command_client_->service_is_ready()) {
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        request->broadcast = false;
        request->command = 183; // MAV_CMD_DO_SET_SERVO
        request->param1 = static_cast<float>(channel);   // Динамический выбор пина (1.0 или 2.0)
        request->param2 = static_cast<float>(pwm_value); // Рассчитанный ШИМ
        request->param3 = 0.0;
        request->param4 = 0.0;
        request->param5 = 0.0;
        request->param6 = 0.0;
        request->param7 = 0.0;

        auto result = command_client_->async_send_request(request);
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool is_connected_;
    bool is_armed_;
    int linear_throttle_;
    int angular_steering_;
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


// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <std_msgs/msg/float32_multi_array.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <mavros_msgs/msg/state.hpp>
// #include <mavros_msgs/msg/override_rc_in.hpp>
// #include <mavros_msgs/srv/command_bool.hpp>
// #include <mavros_msgs/srv/set_mode.hpp>
// #include <termios.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <iostream>
// #include <thread>
// #include <atomic>
// #include <sstream>
// #include <iomanip>
// #include <chrono>
// #include <algorithm>
// #include <array>

// class KeyboardTeleop : public rclcpp::Node {
// public:
//     KeyboardTeleop() : Node("keyboard_teleop"), running_(true), pixhawk_connected_(false), armed_(false) {
//         // ============================================
//         // 1. ПАРАМЕТРЫ
//         // ============================================
//         this->declare_parameter("rc_topic", "/mavros/rc/override");
//         this->declare_parameter("data_topic", "/teleop/data");
//         this->declare_parameter("key_topic", "/teleop/key");
//         this->declare_parameter("max_speed", 0.3);
//         this->declare_parameter("turn_speed", 0.5);
//         this->declare_parameter("speed_step", 0.05);
//         this->declare_parameter("log_level", "info");

//         rc_topic_ = this->get_parameter("rc_topic").as_string();
//         data_topic_ = this->get_parameter("data_topic").as_string();
//         key_topic_ = this->get_parameter("key_topic").as_string();
//         max_speed_ = this->get_parameter("max_speed").as_double();
//         turn_speed_ = this->get_parameter("turn_speed").as_double();
//         speed_step_ = this->get_parameter("speed_step").as_double();
//         log_level_ = this->get_parameter("log_level").as_string();

//         // ============================================
//         // 2. ПУБЛИКАТОРЫ
//         // ============================================
//         // Главный публикатор для RC Override (управление моторами!)
//         rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(rc_topic_, 10);

//         // Для TCP/моста (скорости)
//         data_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(data_topic_, 10);

//         // Для отладки (нажатые клавиши)
//         key_pub_ = this->create_publisher<std_msgs::msg::String>(key_topic_, 10);

//         // ============================================
//         // 3. СЕРВИСЫ ДЛЯ ВЗВЕДЕНИЯ
//         // ============================================
//         arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
//         mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

//         // ============================================
//         // 4. ПОДПИСКА НА СТАТУС MAVROS
//         // ============================================
//         state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
//             "/mavros/state", 10,
//             std::bind(&KeyboardTeleop::state_callback, this, std::placeholders::_1));

//         // ============================================
//         // 5. ТАЙМЕРЫ
//         // ============================================
//         check_connection_timer_ = this->create_wall_timer(
//             std::chrono::seconds(2),
//             std::bind(&KeyboardTeleop::check_connection, this));

//         // Автоматическое взведение после подключения
//         auto arm_timer = this->create_wall_timer(
//             std::chrono::seconds(3),
//             [this]() {
//                 if (pixhawk_connected_ && !armed_) {
//                     RCLCPP_INFO(this->get_logger(), "🔄 Пытаюсь взвести Pixhawk...");
//                     set_guided_mode();
//                     arm_rover();
//                 }
//             });
//         arm_timer_ = arm_timer;

//         // ============================================
//         // 6. СТАТИСТИКА
//         // ============================================
//         last_publish_time_ = this->now();
//         command_count_ = 0;

//         // ============================================
//         // 7. ВЫВОД СПРАВКИ
//         // ============================================
//         print_help();

//         // ============================================
//         // 8. ТАЙМЕР ДЛЯ СТАТИСТИКИ
//         // ============================================
//         stats_timer_ = this->create_wall_timer(
//             std::chrono::seconds(10),
//             std::bind(&KeyboardTeleop::print_stats, this));

//         // ============================================
//         // 9. ЗАПУСК ПОТОКА КЛАВИАТУРЫ
//         // ============================================
//         input_thread_ = std::thread(&KeyboardTeleop::read_keys, this);

//         RCLCPP_INFO(this->get_logger(), "✅ Узел управления запущен");
//         RCLCPP_INFO(this->get_logger(), "📤 Публикация в:");
//         RCLCPP_INFO(this->get_logger(), "   - %s (RC Override)", rc_topic_.c_str());
//         RCLCPP_INFO(this->get_logger(), "   - %s (TCP/мост)", data_topic_.c_str());
//         RCLCPP_INFO(this->get_logger(), "   - %s (клавиши)", key_topic_.c_str());
//     }

//     ~KeyboardTeleop() {
//         running_ = false;
//         if (input_thread_.joinable()) {
//             input_thread_.join();
//         }
//         RCLCPP_INFO(this->get_logger(), "🛑 Узел остановлен");
//     }

// private:
//     // ============================================
//     // КОЛБЭК СТАТУСА MAVROS
//     // ============================================
//     void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
//         current_state_ = *msg;
//         pixhawk_connected_ = msg->connected;
//         armed_ = msg->armed;

//         if (msg->connected) {
//             RCLCPP_DEBUG(this->get_logger(), "✅ Pixhawk подключен");
//         } else {
//             RCLCPP_WARN(this->get_logger(), "❌ Pixhawk НЕ подключен");
//         }
//     }

//     // ============================================
//     // ПРОВЕРКА ПОДКЛЮЧЕНИЯ
//     // ============================================
//     void check_connection() {
//         if (!pixhawk_connected_) {
//             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
//                                  "⚠️ Ожидание подключения к Pixhawk...");
//         }
//     }

//     // ============================================
//     // УСТАНОВКА РЕЖИМА GUIDED
//     // ============================================
//     void set_guided_mode() {
//         if (!mode_client_->wait_for_service(std::chrono::seconds(2))) {
//             RCLCPP_WARN(this->get_logger(), "⚠️ Сервис set_mode не доступен");
//             return;
//         }
//         auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
//         request->custom_mode = "GUIDED";
//         mode_client_->async_send_request(request);
//     }

//     // ============================================
//     // ВЗВЕДЕНИЕ PIXHAWK
//     // ============================================
//     void arm_rover() {
//         if (!arm_client_->wait_for_service(std::chrono::seconds(2))) {
//             RCLCPP_WARN(this->get_logger(), "⚠️ Сервис arming не доступен");
//             return;
//         }
//         auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
//         request->value = true;
//         arm_client_->async_send_request(request);
//         RCLCPP_INFO(this->get_logger(), "✅ Команда на взведение отправлена");
//     }

//     // ============================================
//     // СТАТУС PIXHAWK
//     // ============================================
//     std::string get_pixhawk_status() {
//         std::stringstream ss;
//         if (pixhawk_connected_) {
//             ss << "✅ ПОДКЛЮЧЕН";
//             ss << (armed_ ? " [ВЗВЕДЕН]" : " [НЕ ВЗВЕДЕН]");
//             if (!current_state_.mode.empty()) {
//                 ss << " [" << current_state_.mode << "]";
//             }
//         } else {
//             ss << "❌ НЕ ПОДКЛЮЧЕН";
//         }
//         return ss.str();
//     }

//     // ============================================
//     // ВЫВОД СПРАВКИ
//     // ============================================
//     void print_help() {
//         std::stringstream ss;
//         std::string status = get_pixhawk_status();

//         ss << "\n";
//         ss << "╔══════════════════════════════════════════════════════════════════════════╗\n";
//         ss << "║                    🚜 УПРАВЛЕНИЕ ГУСЕНИЧНОЙ ТЕЛЕЖКОЙ                   ║\n";
//         ss << "╠══════════════════════════════════════════════════════════════════════════╣\n";
//         ss << "║                                                                          ║\n";
//         ss << "║  ╔══════════════════════════════════════════════════════════════════╗     ║\n";
//         ss << "║  ║   🔌 СТАТУС PIXHAWK: " << std::left << std::setw(35) << status << "║     ║\n";
//         ss << "║  ╚══════════════════════════════════════════════════════════════════╝     ║\n";
//         ss << "║                                                                          ║\n";
//         ss << "║  ╔══════════════════════════════════════════════════════════════════╗     ║\n";
//         ss << "║  ║                     🎮 УПРАВЛЕНИЕ ДВИЖЕНИЕМ                    ║     ║\n";
//         ss << "║  ╠══════════════════════════════════════════════════════════════════╣     ║\n";
//         ss << "║  ║                                                                  ║     ║\n";
//         ss << "║  ║                      [  ↑  ]                                    ║     ║\n";
//         ss << "║  ║                   W - Вперёд                                    ║     ║\n";
//         ss << "║  ║                                                                  ║     ║\n";
//         ss << "║  ║            [  ←  ]        [  →  ]                              ║     ║\n";
//         ss << "║  ║         A - Налево    D - Направо                              ║     ║\n";
//         ss << "║  ║                                                                  ║     ║\n";
//         ss << "║  ║   [  ↺  ]              [  ↻  ]                                 ║     ║\n";
//         ss << "║  ║ Q - Разворот налево  E - Разворот направо                     ║     ║\n";
//         ss << "║  ║   (на месте)           (на месте)                               ║     ║\n";
//         ss << "║  ║                                                                  ║     ║\n";
//         ss << "║  ║                      [  ↓  ]                                    ║     ║\n";
//         ss << "║  ║                   S - Назад                                     ║     ║\n";
//         ss << "║  ║                                                                  ║     ║\n";
//         ss << "║  ║   ╔═══════════════════════════════════════════════════════════╗  ║     ║\n";
//         ss << "║  ║   ║   [ SPACE ] - ЭКСТРЕННАЯ ОСТАНОВКА                      ║  ║     ║\n";
//         ss << "║  ║   ╚═══════════════════════════════════════════════════════════╝  ║     ║\n";
//         ss << "║  ╚══════════════════════════════════════════════════════════════════╝     ║\n";
//         ss << "║                                                                          ║\n";
//         ss << "║  ╔══════════════════════════════════════════════════════════════════╗     ║\n";
//         ss << "║  ║                    ⚡ НАСТРОЙКА СКОРОСТИ                      ║     ║\n";
//         ss << "║  ╠══════════════════════════════════════════════════════════════════╣     ║\n";
//         ss << "║  ║   [ R ] - Увеличить скорость    [ F ] - Уменьшить              ║     ║\n";
//         ss << "║  ║   [ T ] - Увеличить поворот     [ G ] - Уменьшить              ║     ║\n";
//         ss << "║  ║                                                                  ║     ║\n";
//         ss << "║  ║   ТЕКУЩАЯ СКОРОСТЬ:                                             ║     ║\n";
//         ss << "║  ║   ┌─────────────────────────────────────────────────────────┐    ║     ║\n";
//         ss << "║  ║   │  Линейная: " << std::setw(5) << std::fixed << std::setprecision(2) << max_speed_ << " м/с    │    ║     ║\n";
//         ss << "║  ║   │  Поворот:  " << std::setw(5) << std::fixed << std::setprecision(2) << turn_speed_ << " рад/с   │    ║     ║\n";
//         ss << "║  ║   └─────────────────────────────────────────────────────────┘    ║     ║\n";
//         ss << "║  ╚══════════════════════════════════════════════════════════════════╝     ║\n";
//         ss << "║                                                                          ║\n";
//         ss << "║  ╔══════════════════════════════════════════════════════════════════╗     ║\n";
//         ss << "║  ║   [ H ] - Справка    [ P ] - Параметры    [ Ctrl+C ] - Выход   ║     ║\n";
//         ss << "║  ╚══════════════════════════════════════════════════════════════════╝     ║\n";
//         ss << "╚══════════════════════════════════════════════════════════════════════════╝\n";

//         RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
//     }

//     // ============================================
//     // ПОКАЗАТЬ ПАРАМЕТРЫ
//     // ============================================
//     void print_params() {
//         std::stringstream ss;
//         ss << "\n";
//         ss << "╔══════════════════════════════════════════════════════════════╗\n";
//         ss << "║              📊 ТЕКУЩИЕ ПАРАМЕТРЫ                          ║\n";
//         ss << "╠══════════════════════════════════════════════════════════════╣\n";
//         ss << "║  🔌 " << get_pixhawk_status() << "\n";
//         ss << "║                                                              ║\n";
//         ss << "║  Линейная скорость:  " << std::fixed << std::setprecision(2) << max_speed_ << " м/с\n";
//         ss << "║  Поворот:            " << std::fixed << std::setprecision(2) << turn_speed_ << " рад/с\n";
//         ss << "║  Шаг:                " << std::fixed << std::setprecision(2) << speed_step_ << "\n";
//         ss << "║  Команд:             " << command_count_ << "\n";
//         ss << "╚══════════════════════════════════════════════════════════════╝\n";
//         RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
//     }

//     // ============================================
//     // СТАТИСТИКА
//     // ============================================
//     void print_stats() {
//         if (command_count_ > 0) {
//             RCLCPP_INFO(this->get_logger(), "📊 %d команд, статус: %s",
//                         command_count_, pixhawk_connected_ ? "✅" : "❌");
//             command_count_ = 0;
//         }
//     }

//     // ============================================
//     // ПУБЛИКАЦИЯ RC КОМАНДЫ (ГЛАВНОЕ!)
//     // ============================================
//     void publish_rc(double linear, double angular) {
//         auto msg = mavros_msgs::msg::OverrideRCIn();

//         // Инициализируем все 18 каналов нулями
//         msg.channels = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//         // 1500 = стоп, 1900 = вперёд, 1100 = назад
//         // Канал 1 = левый мотор, Канал 3 = правый мотор
//         int left = 1500 + static_cast<int>(linear * 400) - static_cast<int>(angular * 200);
//         int right = 1500 + static_cast<int>(linear * 400) + static_cast<int>(angular * 200);

//         msg.channels[0] = std::clamp(left, 1000, 2000);   // Левый мотор (Канал 1)
//         msg.channels[2] = std::clamp(right, 1000, 2000);  // Правый мотор (Канал 3)

//         rc_pub_->publish(msg);

//         // Логирование
//         if (log_level_ == "debug") {
//             RCLCPP_DEBUG(this->get_logger(), "📤 RC: L=%d, R=%d", msg.channels[0], msg.channels[2]);
//         }
//     }

//     // ============================================
//     // ПУБЛИКАЦИЯ ДАННЫХ
//     // ============================================
//     void publish_data(const std::string& key_name, double linear, double angular) {
//         // 1. Клавиша
//         auto key_msg = std_msgs::msg::String();
//         key_msg.data = key_name;
//         key_pub_->publish(key_msg);

//         // 2. RC команда (управление моторами!)
//         publish_rc(linear, angular);

//         // 3. Данные для TCP/моста
//         auto data_msg = std_msgs::msg::Float32MultiArray();
//         data_msg.data.clear();
//         data_msg.data.push_back(static_cast<float>(linear));
//         data_msg.data.push_back(static_cast<float>(angular));
//         data_pub_->publish(data_msg);

//         command_count_++;
//     }

//     // ============================================
//     // НАСТРОЙКА СКОРОСТИ
//     // ============================================
//     void set_max_speed(double new_speed) {
//         if (new_speed < 0.05) {
//             RCLCPP_WARN(this->get_logger(), "⚠️ Минимальная скорость: 0.05 м/с");
//             max_speed_ = 0.05;
//         } else if (new_speed > 1.0) {
//             RCLCPP_WARN(this->get_logger(), "⚠️ Максимальная скорость: 1.0 м/с");
//             max_speed_ = 1.0;
//         } else {
//             max_speed_ = new_speed;
//             RCLCPP_INFO(this->get_logger(), "✅ Скорость: %.2f м/с", max_speed_);
//         }
//     }

//     void set_turn_speed(double new_speed) {
//         if (new_speed < 0.1) {
//             RCLCPP_WARN(this->get_logger(), "⚠️ Минимальный поворот: 0.1 рад/с");
//             turn_speed_ = 0.1;
//         } else if (new_speed > 1.5) {
//             RCLCPP_WARN(this->get_logger(), "⚠️ Максимальный поворот: 1.5 рад/с");
//             turn_speed_ = 1.5;
//         } else {
//             turn_speed_ = new_speed;
//             RCLCPP_INFO(this->get_logger(), "✅ Поворот: %.2f рад/с", turn_speed_);
//         }
//     }

//     // ============================================
//     // ЧТЕНИЕ КЛАВИАТУРЫ
//     // ============================================
//     void read_keys() {
//         struct termios oldt, newt;
//         if (tcgetattr(STDIN_FILENO, &oldt) != 0) {
//             RCLCPP_ERROR(this->get_logger(), "❌ Не удалось получить настройки терминала");
//             return;
//         }
//         newt = oldt;
//         newt.c_lflag &= ~(ICANON | ECHO);
//         newt.c_cc[VMIN] = 0;
//         newt.c_cc[VTIME] = 0;
//         if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) != 0) {
//             RCLCPP_ERROR(this->get_logger(), "❌ Не удалось установить настройки терминала");
//             return;
//         }

//         int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
//         if (flags == -1 || fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1) {
//             RCLCPP_ERROR(this->get_logger(), "❌ Не удалось сделать STDIN неблокирующим");
//             tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
//             return;
//         }

//         char key;
//         double linear = 0.0;
//         double angular = 0.0;
//         std::string key_name;
//         bool key_pressed = false;

//         while (running_ && rclcpp::ok()) {
//             if (read(STDIN_FILENO, &key, 1) > 0) {
//                 key_pressed = true;
//                 linear = 0.0;
//                 angular = 0.0;

//                 switch (key) {
//                 case 'w': case 'W':
//                     linear = max_speed_;
//                     key_name = "W";
//                     RCLCPP_INFO(this->get_logger(), "⬆️ Вперёд [%.2f м/с]", linear);
//                     break;

//                 case 's': case 'S':
//                     linear = -max_speed_;
//                     key_name = "S";
//                     RCLCPP_INFO(this->get_logger(), "⬇️ Назад [%.2f м/с]", linear);
//                     break;

//                 case 'a': case 'A':
//                     linear = max_speed_ * 0.5;
//                     angular = turn_speed_;
//                     key_name = "A";
//                     RCLCPP_INFO(this->get_logger(), "↰ Поворот налево [lin=%.2f, ang=%.2f]", linear, angular);
//                     break;

//                 case 'd': case 'D':
//                     linear = max_speed_ * 0.5;
//                     angular = -turn_speed_;
//                     key_name = "D";
//                     RCLCPP_INFO(this->get_logger(), "↱ Поворот направо [lin=%.2f, ang=%.2f]", linear, angular);
//                     break;

//                 case 'q': case 'Q':
//                     angular = turn_speed_;
//                     key_name = "Q";
//                     RCLCPP_INFO(this->get_logger(), "🔄 Разворот налево [ang=%.2f]", angular);
//                     break;

//                 case 'e': case 'E':
//                     angular = -turn_speed_;
//                     key_name = "E";
//                     RCLCPP_INFO(this->get_logger(), "🔄 Разворот направо [ang=%.2f]", angular);
//                     break;

//                 case ' ':
//                     linear = 0.0;
//                     angular = 0.0;
//                     key_name = "SPACE";
//                     RCLCPP_INFO(this->get_logger(), "🛑 СТОП!");
//                     break;

//                 case 'r': case 'R':
//                     set_max_speed(max_speed_ + speed_step_);
//                     continue;

//                 case 'f': case 'F':
//                     set_max_speed(max_speed_ - speed_step_);
//                     continue;

//                 case 't': case 'T':
//                     set_turn_speed(turn_speed_ + speed_step_);
//                     continue;

//                 case 'g': case 'G':
//                     set_turn_speed(turn_speed_ - speed_step_);
//                     continue;

//                 case 'h': case 'H':
//                     print_help();
//                     continue;

//                 case 'p': case 'P':
//                     print_params();
//                     continue;

//                 default:
//                     key_pressed = false;
//                     continue;
//                 }

//                 publish_data(key_name, linear, angular);

//             } else {
//                 if (key_pressed) {
//                     key_pressed = false;
//                     key_name = "RELEASE";
//                     publish_data(key_name, 0.0, 0.0);
//                     RCLCPP_DEBUG(this->get_logger(), "⏹️ Остановка (клавиша отпущена)");
//                 }
//             }

//             std::this_thread::sleep_for(std::chrono::milliseconds(50));
//         }

//         tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
//     }

//     // ============================================
//     // ПЕРЕМЕННЫЕ
//     // ============================================
//     std::string rc_topic_;
//     std::string data_topic_;
//     std::string key_topic_;
//     std::string log_level_;
//     double max_speed_;
//     double turn_speed_;
//     double speed_step_;

//     rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
//     rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr data_pub_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr key_pub_;
//     rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
//     rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
//     rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
//     rclcpp::TimerBase::SharedPtr stats_timer_;
//     rclcpp::TimerBase::SharedPtr check_connection_timer_;
//     rclcpp::TimerBase::SharedPtr arm_timer_;
//     std::thread input_thread_;
//     std::atomic<bool> running_;

//     mavros_msgs::msg::State current_state_;
//     bool pixhawk_connected_;
//     bool armed_;

//     rclcpp::Time last_publish_time_;
//     int command_count_;
// };

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);

//     try {
//         auto node = std::make_shared<KeyboardTeleop>();
//         rclcpp::spin(node);
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(rclcpp::get_logger("main"), "❌ Критическая ошибка: %s", e.what());
//         return 1;
//     }

//     rclcpp::shutdown();
//     return 0;
// }
