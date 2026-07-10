#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <fcntl.h>
#include <errno.h>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <map>

// ============================================
// НАСТРОЙКИ ПОДКЛЮЧЕНИЯ
// ============================================
const std::string SERVER_IP = "192.168.31.64";  // IP сервера (2-й ПК)
const int SERVER_PORT = 6000;                    // Порт сервера

std::atomic<bool> running(true);
int sockfd = -1;

class TcpClientNode : public rclcpp::Node {
public:
    TcpClientNode() : Node("tcp_client_node") {
        // Публикаторы для отправки команд в ROS 2
        telega_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/telega_commands", 10);
        camera_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/camera_control", 10);

        RCLCPP_INFO(this->get_logger(), "📤 Публикация команд тележки в: /telega_commands");
        RCLCPP_INFO(this->get_logger(), "📤 Публикация команд камеры в: /camera_control");
    }

    void process_server_message(const std::string& line) {
        auto msg = std_msgs::msg::String();

        // Обработка команд от сервера
        if (line.find("KEY:") == 0) {
            std::string key_info = line.substr(4);

            // Маппинг клавиш в команды
            std::map<std::string, std::string> key_mapping;
            // Тележка
            key_mapping["W:press"] = "w";
            key_mapping["S:press"] = "s";
            key_mapping["A:press"] = "a";
            key_mapping["D:press"] = "d";
            key_mapping["Q:press"] = "q";
            key_mapping["R:press"] = "r";
            key_mapping["F:press"] = "f";
            key_mapping["SPACE:press"] = " ";
            // Камера
            key_mapping["UP:press"] = "UP";
            key_mapping["DOWN:press"] = "DOWN";
            key_mapping["LEFT:press"] = "LEFT";
            key_mapping["RIGHT:press"] = "RIGHT";
            key_mapping["PLUS:press"] = "PLUS";
            key_mapping["MINUS:press"] = "MINUS";
            key_mapping["Z:press"] = "Z";
            key_mapping["X:press"] = "X";
            key_mapping["C:press"] = "C";

            auto it = key_mapping.find(key_info);
            if (it != key_mapping.end()) {
                std::string command = it->second;
                msg.data = command;

                // Определяем тип команды и публикуем в нужный топик
                if (command == "UP" || command == "DOWN" ||
                    command == "LEFT" || command == "RIGHT" ||
                    command == "PLUS" || command == "MINUS" ||
                    command == "Z" || command == "X" || command == "C") {
                    // Команда камеры
                    camera_cmd_pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "📷 Отправлена команда камере: '%s'", command.c_str());
                } else {
                    // Команда тележки
                    telega_cmd_pub_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "🚜 Отправлена команда тележке: '%s'", command.c_str());
                }

                // Вывод на экран
                std::map<std::string, std::string> display;
                display["w"] = "🚜 ВПЕРЁД";
                display["s"] = "🚜 НАЗАД";
                display["a"] = "🚜 НАЛЕВО";
                display["d"] = "🚜 НАПРАВО";
                display["q"] = "🚜 КРУИЗ-КОНТРОЛЬ";
                display["r"] = "🚜 СКОРОСТЬ +";
                display["f"] = "🚜 СКОРОСТЬ -";
                display[" "] = "🚜 СТОП";
                display["UP"] = "📷 НАКЛОН ВВЕРХ";
                display["DOWN"] = "📷 НАКЛОН ВНИЗ";
                display["LEFT"] = "📷 ПОВОРОТ НАЛЕВО";
                display["RIGHT"] = "📷 ПОВОРОТ НАПРАВО";
                display["PLUS"] = "📷 ПРИБЛИЖЕНИЕ";
                display["MINUS"] = "📷 ОТДАЛЕНИЕ";
                display["Z"] = "📷 ЗУМ +";
                display["X"] = "📷 ЗУМ -";
                display["C"] = "📷 СТОП ЗУМ";

                auto disp_it = display.find(command);
                if (disp_it != display.end()) {
                    std::cout << "⌨️  " << disp_it->second << std::endl;
                }
            }
        }
        else if (line.find("TELEGA:") == 0) {
            std::string cmd = line.substr(7);
            msg.data = cmd;
            telega_cmd_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "🚜 TCP команда тележке: '%s'", cmd.c_str());
        }
        else if (line.find("CAMERA:") == 0) {
            std::string cmd = line.substr(7);
            msg.data = cmd;
            camera_cmd_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "📷 TCP команда камере: '%s'", cmd.c_str());
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telega_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_cmd_pub_;
};

// ============================================
// ПОТОК ЧТЕНИЯ ОТ СЕРВЕРА
// ============================================
void server_read_thread(std::shared_ptr<TcpClientNode> node) {
    char buffer[4096];
    std::string partial_line;

    std::cout << "\n=================================================" << std::endl;
    std::cout << "ОЖИДАНИЕ ДАННЫХ ОТ СЕРВЕРА" << std::endl;
    std::cout << "=================================================" << std::endl;
    std::cout << "📡 Клиент подключен и слушает сервер..." << std::endl;
    std::cout << "   Команды с сервера передаются в ROS 2 топики\n" << std::endl;

    while (running && sockfd != -1 && rclcpp::ok()) {
        memset(buffer, 0, sizeof(buffer));
        ssize_t bytes_read = recv(sockfd, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            std::string data = partial_line + std::string(buffer);
            partial_line.clear();

            std::istringstream stream(data);
            std::string line;

            while (std::getline(stream, line)) {
                line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());

                if (!line.empty()) {
                    // Отправляем в ROS 2
                    node->process_server_message(line);

                    // Дополнительный вывод для данных камеры и тележки
                    if (line.find("CAMERA:") == 0 && line.find(",") != std::string::npos) {
                        std::string values = line.substr(7);
                        size_t comma = values.find(',');
                        if (comma != std::string::npos) {
                            std::string pitch_str = values.substr(0, comma);
                            std::string yaw_str = values.substr(comma + 1);
                            std::cout << "📷 КАМЕРА | Наклон: " << std::setw(7) << pitch_str
                                      << "° | Поворот: " << std::setw(7) << yaw_str << "°" << std::endl;
                        }
                    }
                    else if (line.find("TELEOP:") == 0) {
                        std::string values = line.substr(7);
                        size_t comma = values.find(',');
                        if (comma != std::string::npos) {
                            std::string linear_str = values.substr(0, comma);
                            std::string angular_str = values.substr(comma + 1);
                            std::cout << "🚜 ТЕЛЕЖКА | Скорость: " << std::setw(7) << linear_str
                                      << " | Поворот: " << std::setw(7) << angular_str << std::endl;
                        }
                    }
                }
            }
        }
        else if (bytes_read == 0) {
            std::cout << "\n❌ Сервер отключился" << std::endl;
            running = false;
            break;
        }
        else {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "❌ Ошибка чтения: " << strerror(errno) << std::endl;
                running = false;
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ============================================
// ГЛАВНАЯ ФУНКЦИЯ
// ============================================
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    std::cout << "========================================" << std::endl;
    std::cout << "   TCP КЛИЕНТ ДЛЯ РОБОТА (ROS 2)" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Подключение к серверу " << SERVER_IP << ":" << SERVER_PORT << "..." << std::endl;

    // Создаём ROS 2 узел
    auto node = std::make_shared<TcpClientNode>();

    // Создаём сокет
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        std::cerr << "❌ Не удалось создать сокет: " << strerror(errno) << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    // Настраиваем адрес сервера
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);

    if (inet_pton(AF_INET, SERVER_IP.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "❌ Неверный IP-адрес: " << SERVER_IP << std::endl;
        close(sockfd);
        rclcpp::shutdown();
        return 1;
    }

    // Подключаемся
    if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "❌ Не удалось подключиться: " << strerror(errno) << std::endl;
        close(sockfd);
        rclcpp::shutdown();
        return 1;
    }

    // Устанавливаем неблокирующий режим
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags != -1) {
        fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
    }

    std::cout << "✅ Подключено к серверу!" << std::endl;
    std::cout << "💡 Нажмите Ctrl+C для выхода\n" << std::endl;

    // Запускаем поток чтения от сервера
    std::thread read_thread(server_read_thread, node);

    // Запускаем ROS 2 спин
    rclcpp::spin(node);

    running = false;

    if (read_thread.joinable()) {
        read_thread.join();
    }

    if (sockfd != -1) {
        close(sockfd);
        std::cout << "\n🔌 Отключено от сервера" << std::endl;
    }

    rclcpp::shutdown();
    std::cout << "👋 Клиент завершил работу" << std::endl;

    return 0;
}
