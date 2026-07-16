#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/gimbal_manager_set_pitchyaw.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <vector>
#include <mutex>
#include <cstring>
#include <string>
#include <errno.h>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <map>
#include <chrono>
#include <regex>
#include <algorithm>
#include <termios.h>
#include <ifaddrs.h>
#include <netdb.h>

// ============================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ (ЦВЕТА, ЛОГИ)
// ============================================
#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_BOLD    "\033[1m"

std::string get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) % 1000;
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S")
       << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

void log_info(const std::string& msg) {
    std::cout << COLOR_GREEN << "[" << get_timestamp() << "] [INFO] "
              << COLOR_RESET << msg << std::endl;
}

void log_warn(const std::string& msg) {
    std::cout << COLOR_YELLOW << "[" << get_timestamp() << "] [WARN] "
              << COLOR_RESET << msg << std::endl;
}

void log_error(const std::string& msg) {
    std::cout << COLOR_RED << "[" << get_timestamp() << "] [ERROR] "
              << COLOR_RESET << msg << std::endl;
}

void log_debug(const std::string& msg) {
    std::cout << COLOR_CYAN << "[" << get_timestamp() << "] [DEBUG] "
              << COLOR_RESET << msg << std::endl;
}

// ============================================
// ФУНКЦИЯ ДЛЯ ЧТЕНИЯ КЛАВИШ (НЕБЛОКИРУЮЩАЯ)
// ============================================
int getch_nonblock() {
    static struct termios oldt, newt;
    static bool initialized = false;

    if (!initialized) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        initialized = true;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    int ch = getchar();

    fcntl(STDIN_FILENO, F_SETFL, flags);

    if (ch == EOF) {
        return -1;
    }
    return ch;
}

class TcpServerNode : public rclcpp::Node {
public:
    TcpServerNode() : Node("tcp_server_node"), server_fd_(-1), running_(true), command_count_(0) {
        log_info("========================================");
        log_info("🌐 TCP СЕРВЕР ДЛЯ РОВЕРА");
        log_info("========================================");

        // ============================================
        // 1. ПАРАМЕТРЫ
        // ============================================
        this->declare_parameter("server_ip", "0.0.0.0");
        this->declare_parameter("server_port", 6000);
        this->declare_parameter("max_clients", 10);
        this->declare_parameter("gimbal_topic", "/gimbal/commands");
        this->declare_parameter("teleop_topic", "/teleop/data");
        this->declare_parameter("key_topic", "/teleop/key");
        this->declare_parameter("keyboard_commands_topic", "/keyboard_commands");
        this->declare_parameter("camera_cmd_topic", "/camera_control");
        this->declare_parameter("telega_cmd_topic", "/telega_commands");

        server_ip_ = this->get_parameter("server_ip").as_string();
        server_port_ = this->get_parameter("server_port").as_int();
        max_clients_ = this->get_parameter("max_clients").as_int();
        gimbal_topic_ = this->get_parameter("gimbal_topic").as_string();
        teleop_topic_ = this->get_parameter("teleop_topic").as_string();
        key_topic_ = this->get_parameter("key_topic").as_string();
        keyboard_commands_topic_ = this->get_parameter("keyboard_commands_topic").as_string();
        camera_cmd_topic_ = this->get_parameter("camera_cmd_topic").as_string();
        telega_cmd_topic_ = this->get_parameter("telega_cmd_topic").as_string();

        log_info("📋 ПАРАМЕТРЫ:");
        log_info("  server_ip: " + server_ip_);
        log_info("  server_port: " + std::to_string(server_port_));
        log_info("  max_clients: " + std::to_string(max_clients_));
        log_info("  gimbal_topic: " + gimbal_topic_);
        log_info("  teleop_topic: " + teleop_topic_);
        log_info("  key_topic: " + key_topic_);
        log_info("  camera_cmd_topic: " + camera_cmd_topic_);
        log_info("  telega_cmd_topic: " + telega_cmd_topic_);

        // ============================================
        // 2. ПОДПИСЧИКИ
        // ============================================
        log_info("========================================");
        log_info("📋 НАСТРОЙКА ПОДПИСЧИКОВ");
        log_info("========================================");

        gimbal_sub_ = this->create_subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>(
            gimbal_topic_, 10,
            std::bind(&TcpServerNode::gimbal_callback, this, std::placeholders::_1));
        log_info("  ✅ Подписка на камеру: " + gimbal_topic_);

        teleop_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            teleop_topic_, 10,
            std::bind(&TcpServerNode::teleop_callback, this, std::placeholders::_1));
        log_info("  ✅ Подписка на тележку: " + teleop_topic_);

        key_sub_ = this->create_subscription<std_msgs::msg::String>(
            key_topic_, 10,
            std::bind(&TcpServerNode::key_callback, this, std::placeholders::_1));
        log_info("  ✅ Подписка на клавиши: " + key_topic_);

        keyboard_commands_sub_ = this->create_subscription<std_msgs::msg::String>(
            keyboard_commands_topic_, 10,
            std::bind(&TcpServerNode::keyboard_commands_callback, this, std::placeholders::_1));
        log_info("  ✅ Подписка на команды клавиатуры: " + keyboard_commands_topic_);

        // ============================================
        // 3. ПУБЛИКАТОРЫ
        // ============================================
        log_info("========================================");
        log_info("📤 НАСТРОЙКА ПУБЛИКАТОРОВ");
        log_info("========================================");

        angle_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/server_received/angles", 10);
        log_info("  ✅ Публикация углов: /server_received/angles");

        camera_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            camera_cmd_topic_, 10);
        log_info("  ✅ Публикация команд камеры: " + camera_cmd_topic_);

        telega_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            telega_cmd_topic_, 10);
        log_info("  ✅ Публикация команд тележки: " + telega_cmd_topic_);

        // ============================================
        // 4. ЗАПУСК TCP-СЕРВЕРА
        // ============================================
        log_info("========================================");
        log_info("🌐 ЗАПУСК TCP-СЕРВЕРА");
        log_info("========================================");

        if (!start_server(server_ip_, server_port_)) {
            log_error("❌ Не удалось запустить сервер");
            return;
        }

        // ============================================
        // 5. ЗАПУСК ПОТОКОВ
        // ============================================
        log_info("📋 ЗАПУСК ПОТОКОВ");
        log_info("  ✅ Поток чтения клиентов");
        read_thread_ = std::thread(&TcpServerNode::read_from_clients, this);

        log_info("  ✅ Поток клавиатуры");
        keyboard_thread_ = std::thread(&TcpServerNode::read_keyboard, this);

        // ============================================
        // 6. ТАЙМЕР ДЛЯ СТАТИСТИКИ (каждые 15 секунд)
        // ============================================
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(15),
            std::bind(&TcpServerNode::print_stats, this));

        last_activity_time_ = std::chrono::steady_clock::now();

        log_info("========================================");
        log_info(COLOR_GREEN "✅ TCP-СЕРВЕР УСПЕШНО ЗАПУЩЕН" COLOR_RESET);
        log_info("  🌐 IP: " + server_ip_);
        log_info("  🔌 Порт: " + std::to_string(server_port_));
        log_info("  🖥️  Макс. клиентов: " + std::to_string(max_clients_));
        log_info("  ⌨️  Управление с клавиатуры сервера");
        log_info("========================================");
        log_info("");
        log_info("💡 Нажмите Ctrl+C для остановки сервера");
        log_info("");
    }

    ~TcpServerNode() {
        log_warn("🛑 ОСТАНОВКА СЕРВЕРА...");
        running_ = false;

        if (read_thread_.joinable()) {
            read_thread_.join();
        }

        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }

        if (accept_thread_.joinable()) {
            accept_thread_.join();
        }

        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            for (int client : clients_) {
                if (client != -1) {
                    close(client);
                }
            }
            clients_.clear();
        }

        if (server_fd_ != -1) {
            close(server_fd_);
        }

        tcgetattr(STDIN_FILENO, &old_termios_);

        log_info("📊 ВСЕГО КОМАНД: " + std::to_string(command_count_));
        log_info("✅ Сервер остановлен");
    }

private:
    // ============================================
    // ЧТЕНИЕ КЛАВИАТУРЫ НА СЕРВЕРЕ
    // ============================================
    void read_keyboard() {
        std::cout << COLOR_CYAN << "\n========================================" << std::endl;
        std::cout << "УПРАВЛЕНИЕ С КЛАВИАТУРЫ СЕРВЕРА:" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "🚜 ТЕЛЕЖКА:" << std::endl;
        std::cout << "  W/S - Вперёд/Назад" << std::endl;
        std::cout << "  A/D - Налево/Направо" << std::endl;
        std::cout << "  Q   - Круиз-контроль" << std::endl;
        std::cout << "  R/F - Увеличить/Уменьшить скорость" << std::endl;
        std::cout << "  ПРОБЕЛ - Стоп" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "📷 КАМЕРА:" << std::endl;
        std::cout << "  СТРЕЛКИ - Наклон/Поворот" << std::endl;
        std::cout << "  +/- - Приближение/Отдаление" << std::endl;
        std::cout << "  Z/X/C - Зум +/-/Стоп" << std::endl;
        std::cout << "========================================" << COLOR_RESET << std::endl;

        while (running_ && rclcpp::ok()) {
            int ch = getch_nonblock();

            if (ch == -1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            std::string ros_command;
            std::string tcp_command;
            bool send_to_tcp = true;

            switch (ch) {
            case 'w': case 'W':
                ros_command = "w";
                tcp_command = "KEY:W:press\n";
                std::cout << COLOR_GREEN << "🚜 Вперёд" << COLOR_RESET << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 's': case 'S':
                ros_command = "s";
                tcp_command = "KEY:S:press\n";
                std::cout << COLOR_GREEN << "🚜 Назад" << COLOR_RESET << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'a': case 'A':
                ros_command = "a";
                tcp_command = "KEY:A:press\n";
                std::cout << COLOR_GREEN << "🚜 Налево" << COLOR_RESET << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'd': case 'D':
                ros_command = "d";
                tcp_command = "KEY:D:press\n";
                std::cout << COLOR_GREEN << "🚜 Направо" << COLOR_RESET << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'q': case 'Q':
                ros_command = "q";
                tcp_command = "KEY:Q:press\n";
                std::cout << COLOR_YELLOW << "🚜 Круиз-контроль" << COLOR_RESET << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'r': case 'R':
                ros_command = "r";
                tcp_command = "KEY:R:press\n";
                std::cout << COLOR_YELLOW << "🚜 Скорость +" << COLOR_RESET << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'f': case 'F':
                ros_command = "f";
                tcp_command = "KEY:F:press\n";
                std::cout << COLOR_YELLOW << "🚜 Скорость -" << COLOR_RESET << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case ' ':
                ros_command = " ";
                tcp_command = "KEY:SPACE:press\n";
                std::cout << COLOR_RED << "🚜 СТОП" << COLOR_RESET << std::endl;
                telega_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 27: {
                int next = getch_nonblock();
                if (next == 91) {
                    int arrow = getch_nonblock();
                    switch (arrow) {
                    case 65:
                        ros_command = "UP";
                        tcp_command = "KEY:UP:press\n";
                        std::cout << COLOR_BLUE << "📷 Наклон вверх" << COLOR_RESET << std::endl;
                        camera_cmd_pub_->publish(create_string_msg(ros_command));
                        break;
                    case 66:
                        ros_command = "DOWN";
                        tcp_command = "KEY:DOWN:press\n";
                        std::cout << COLOR_BLUE << "📷 Наклон вниз" << COLOR_RESET << std::endl;
                        camera_cmd_pub_->publish(create_string_msg(ros_command));
                        break;
                    case 68:
                        ros_command = "LEFT";
                        tcp_command = "KEY:LEFT:press\n";
                        std::cout << COLOR_BLUE << "📷 Поворот налево" << COLOR_RESET << std::endl;
                        camera_cmd_pub_->publish(create_string_msg(ros_command));
                        break;
                    case 67:
                        ros_command = "RIGHT";
                        tcp_command = "KEY:RIGHT:press\n";
                        std::cout << COLOR_BLUE << "📷 Поворот направо" << COLOR_RESET << std::endl;
                        camera_cmd_pub_->publish(create_string_msg(ros_command));
                        break;
                    }
                }
            }
            break;
            case '+': case '=':
                ros_command = "PLUS";
                tcp_command = "KEY:PLUS:press\n";
                std::cout << COLOR_BLUE << "📷 Приближение" << COLOR_RESET << std::endl;
                camera_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case '-': case '_':
                ros_command = "MINUS";
                tcp_command = "KEY:MINUS:press\n";
                std::cout << COLOR_BLUE << "📷 Отдаление" << COLOR_RESET << std::endl;
                camera_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'z': case 'Z':
                ros_command = "Z";
                tcp_command = "KEY:Z:press\n";
                std::cout << COLOR_BLUE << "📷 Зум +" << COLOR_RESET << std::endl;
                camera_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'x': case 'X':
                ros_command = "X";
                tcp_command = "KEY:X:press\n";
                std::cout << COLOR_BLUE << "📷 Зум -" << COLOR_RESET << std::endl;
                camera_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            case 'c': case 'C':
                ros_command = "C";
                tcp_command = "KEY:C:press\n";
                std::cout << COLOR_BLUE << "📷 Стоп зум" << COLOR_RESET << std::endl;
                camera_cmd_pub_->publish(create_string_msg(ros_command));
                break;
            default:
                send_to_tcp = false;
                break;
            }

            if (send_to_tcp && !tcp_command.empty()) {
                send_to_clients(tcp_command.c_str(), tcp_command.length());
            }

            command_count_++;
            last_activity_time_ = std::chrono::steady_clock::now();
        }
    }

    std_msgs::msg::String create_string_msg(const std::string& data) {
        auto msg = std_msgs::msg::String();
        msg.data = data;
        return msg;
    }

    // ============================================
    // TCP-СЕРВЕР: ЗАПУСК
    // ============================================
    bool start_server(const std::string& ip, int port) {
        log_info("📋 Создание сокета...");
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ == -1) {
            log_error("❌ Не удалось создать сокет: " + std::string(strerror(errno)));
            return false;
        }
        log_info("  ✅ Сокет создан (FD: " + std::to_string(server_fd_) + ")");

        int opt = 1;
        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            log_warn("  ⚠️ Не удалось установить SO_REUSEADDR: " + std::string(strerror(errno)));
        } else {
            log_info("  ✅ SO_REUSEADDR установлен");
        }

        struct sockaddr_in address;
        memset(&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_port = htons(port);

        if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) <= 0) {
            log_error("❌ Неверный IP-адрес: " + ip);
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        log_info("📋 Привязка к " + ip + ":" + std::to_string(port) + "...");
        if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
            log_error("❌ Ошибка привязки: " + std::string(strerror(errno)));
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }
        log_info("  ✅ Привязка успешна");

        log_info("📋 Настройка прослушивания (макс. " + std::to_string(max_clients_) + " клиентов)...");
        if (listen(server_fd_, max_clients_) < 0) {
            log_error("❌ Ошибка прослушивания: " + std::string(strerror(errno)));
            close(server_fd_);
            server_fd_ = -1;
            return false;
        }
        log_info("  ✅ Прослушивание настроено");

        int flags = fcntl(server_fd_, F_GETFL, 0);
        if (flags == -1 || fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK) == -1) {
            log_warn("  ⚠️ Не удалось установить неблокирующий режим");
        } else {
            log_info("  ✅ Неблокирующий режим установлен");
        }

        log_info("📋 Запуск потока приёма клиентов...");
        accept_thread_ = std::thread(&TcpServerNode::accept_clients, this);
        log_info("  ✅ Поток приёма клиентов запущен");

        return true;
    }

    // ============================================
    // TCP-СЕРВЕР: ПРИЁМ КЛИЕНТОВ
    // ============================================
    void accept_clients() {
        log_info("🔄 Поток приёма клиентов запущен");

        while (running_ && rclcpp::ok()) {
            struct sockaddr_in client_addr;
            socklen_t addrlen = sizeof(client_addr);

            int client_socket = accept(server_fd_, (struct sockaddr*)&client_addr, &addrlen);

            if (client_socket < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                if (errno != EINTR) {
                    log_warn("Ошибка accept(): " + std::string(strerror(errno)));
                }
                continue;
            }

            int flags = fcntl(client_socket, F_GETFL, 0);
            if (flags != -1) {
                fcntl(client_socket, F_SETFL, flags | O_NONBLOCK);
            }

            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                if (clients_.size() >= static_cast<size_t>(max_clients_)) {
                    log_warn("⛔ Достигнут лимит клиентов (" + std::to_string(max_clients_) + "). Отклоняем.");
                    close(client_socket);
                    continue;
                }
            }

            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
            int client_port = ntohs(client_addr.sin_port);

            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                clients_.push_back(client_socket);
                log_info(COLOR_GREEN "✅ НОВЫЙ КЛИЕНТ: " + std::string(client_ip) + ":" + std::to_string(client_port) +
                         " (всего: " + std::to_string(clients_.size()) + ")" COLOR_RESET);
            }
        }
        log_info("🔄 Поток приёма клиентов остановлен");
    }

    // ============================================
    // ЧТЕНИЕ КОМАНД ОТ КЛИЕНТОВ
    // ============================================
    void read_from_clients() {
        char buffer[1024];
        log_info("🔄 Поток чтения клиентов запущен");

        while (running_ && rclcpp::ok()) {
            std::vector<int> clients_copy;
            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                clients_copy = clients_;
            }

            for (int client : clients_copy) {
                memset(buffer, 0, sizeof(buffer));
                ssize_t bytes_read = recv(client, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);

                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';
                    std::string data(buffer);

                    std::istringstream stream(data);
                    std::string line;

                    while (std::getline(stream, line)) {
                        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());

                        if (!line.empty()) {
                            log_info(COLOR_CYAN "📨 ОТ КЛИЕНТА: '" + line + "'" COLOR_RESET);
                            process_client_command(line);
                        }
                    }
                }
                else if (bytes_read == 0) {
                    log_warn("🔌 Клиент отключился (recv=0). Удаляем.");
                    std::lock_guard<std::mutex> lock(clients_mutex_);
                    auto it = std::find(clients_.begin(), clients_.end(), client);
                    if (it != clients_.end()) {
                        close(*it);
                        clients_.erase(it);
                        log_info("  Клиентов осталось: " + std::to_string(clients_.size()));
                    }
                }
                else {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        log_warn("Ошибка recv(): " + std::string(strerror(errno)) + ". Удаляем клиента.");
                        std::lock_guard<std::mutex> lock(clients_mutex_);
                        auto it = std::find(clients_.begin(), clients_.end(), client);
                        if (it != clients_.end()) {
                            close(*it);
                            clients_.erase(it);
                            log_info("  Клиентов осталось: " + std::to_string(clients_.size()));
                        }
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        log_info("🔄 Поток чтения клиентов остановлен");
    }

    // ============================================
    // ОБРАБОТКА КОМАНД ОТ КЛИЕНТОВ
    // ============================================
    void process_client_command(const std::string& command) {
        auto msg = std_msgs::msg::String();

        if (command.find("TELEGA:") == 0) {
            std::string telega_cmd = command.substr(7);
            std::map<std::string, std::string> telega_mapping = {
                {"W", "w"}, {"S", "s"}, {"A", "a"}, {"D", "d"},
                {"Q", "q"}, {"R", "r"}, {"F", "f"},
                {"UP", "w"}, {"DOWN", "s"}, {"LEFT", "a"}, {"RIGHT", "d"},
                {"SPACE", " "}, {"STOP", "stop"}
            };

            auto it = telega_mapping.find(telega_cmd);
            msg.data = (it != telega_mapping.end()) ? it->second : telega_cmd;
            telega_cmd_pub_->publish(msg);
            log_info("🚜 Команда тележке: '" + telega_cmd + "' -> '" + msg.data + "'");
        }
        else if (command.find("CAMERA:") == 0) {
            std::string camera_cmd = command.substr(7);
            std::map<std::string, std::string> camera_mapping = {
                {"UP", "UP"}, {"DOWN", "DOWN"}, {"LEFT", "LEFT"}, {"RIGHT", "RIGHT"},
                {"PLUS", "PLUS"}, {"MINUS", "MINUS"},
                {"Z", "Z"}, {"X", "X"}, {"C", "C"}, {"STOP", "STOP"},
                {"ZOOM_IN", "Z"}, {"ZOOM_OUT", "X"}, {"ZOOM_STOP", "C"}
            };

            auto it = camera_mapping.find(camera_cmd);
            msg.data = (it != camera_mapping.end()) ? it->second : camera_cmd;
            camera_cmd_pub_->publish(msg);
            log_info("📷 Команда камере: '" + camera_cmd + "' -> '" + msg.data + "'");
        }
        else {
            std::map<std::string, std::string> direct_mapping = {
                {"w", "w"}, {"W", "w"}, {"s", "s"}, {"S", "s"},
                {"a", "a"}, {"A", "a"}, {"d", "d"}, {"D", "d"},
                {"q", "q"}, {"Q", "q"}, {"r", "r"}, {"R", "r"},
                {"f", "f"}, {"F", "f"}, {" ", " "}, {"SPACE", " "},
                {"UP", "UP"}, {"DOWN", "DOWN"}, {"LEFT", "LEFT"}, {"RIGHT", "RIGHT"},
                {"PLUS", "PLUS"}, {"MINUS", "MINUS"},
                {"Z", "Z"}, {"X", "X"}, {"C", "C"}, {"STOP", "STOP"}
            };

            auto it = direct_mapping.find(command);
            if (it != direct_mapping.end()) {
                std::string mapped_cmd = it->second;

                if (mapped_cmd == "UP" || mapped_cmd == "DOWN" ||
                    mapped_cmd == "LEFT" || mapped_cmd == "RIGHT" ||
                    mapped_cmd == "PLUS" || mapped_cmd == "MINUS" ||
                    mapped_cmd == "Z" || mapped_cmd == "X" || mapped_cmd == "C" ||
                    mapped_cmd == "STOP") {
                    msg.data = mapped_cmd;
                    camera_cmd_pub_->publish(msg);
                    log_info("📷 Прямая команда камере: '" + mapped_cmd + "'");
                } else {
                    msg.data = mapped_cmd;
                    telega_cmd_pub_->publish(msg);
                    log_info("🚜 Прямая команда тележке: '" + mapped_cmd + "'");
                }
            } else {
                msg.data = command;
                telega_cmd_pub_->publish(msg);
                log_warn("🔄 Неизвестная команда: '" + command + "' отправлена тележке");
            }
        }

        command_count_++;
        last_activity_time_ = std::chrono::steady_clock::now();
    }

    // ============================================
    // КОЛБЭКИ
    // ============================================
    void gimbal_callback(const mavros_msgs::msg::GimbalManagerSetPitchyaw::SharedPtr msg) {
        command_count_++;
        last_activity_time_ = std::chrono::steady_clock::now();

        auto angle_msg = std_msgs::msg::Float32MultiArray();
        angle_msg.data.push_back(msg->pitch);
        angle_msg.data.push_back(msg->yaw);
        angle_publisher_->publish(angle_msg);

        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "CAMERA:%.2f,%.2f\n", msg->pitch, msg->yaw);
        send_to_clients(buffer, len);
    }

    void teleop_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;

        float linear = msg->data[0];
        float angular = msg->data[1];
        command_count_++;
        last_activity_time_ = std::chrono::steady_clock::now();

        char buffer[128];
        int len = snprintf(buffer, sizeof(buffer), "TELEOP:%.2f,%.2f\n", linear, angular);
        send_to_clients(buffer, len);
    }

    void key_callback(const std_msgs::msg::String::SharedPtr msg) {
        command_count_++;
        last_activity_time_ = std::chrono::steady_clock::now();
        char buffer[256];
        int len = snprintf(buffer, sizeof(buffer), "KEY:%s\n", msg->data.c_str());
        send_to_clients(buffer, len);
    }

    void keyboard_commands_callback(const std_msgs::msg::String::SharedPtr msg) {
        command_count_++;
        last_activity_time_ = std::chrono::steady_clock::now();
        std::string tcp_message = "KEYBOARD:" + msg->data + "\n";
        send_to_clients(tcp_message.c_str(), tcp_message.length());
    }

    // ============================================
    // ОТПРАВКА ВСЕМ КЛИЕНТАМ
    // ============================================
    void send_to_clients(const char* buffer, int len) {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        if (clients_.empty()) return;

        for (auto it = clients_.begin(); it != clients_.end();) {
            int client = *it;
            ssize_t bytes_sent = send(client, buffer, len, MSG_NOSIGNAL | MSG_DONTWAIT);

            if (bytes_sent < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    close(client);
                    it = clients_.erase(it);
                } else {
                    ++it;
                }
            } else {
                ++it;
            }
        }
    }

    // ============================================
    // СТАТИСТИКА
    // ============================================
    void print_stats() {
        auto now = std::chrono::steady_clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::seconds>(now - last_activity_time_).count();

        std::cout << COLOR_MAGENTA << "\n========================================" << std::endl;
        std::cout << "📊 СТАТИСТИКА СЕРВЕРА" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "  👥 Клиентов: " << clients_.size() << " / " << max_clients_ << std::endl;
        std::cout << "  📝 Команд: " << command_count_ << std::endl;
        std::cout << "  ⏱️  Активность: " << dt << " сек назад" << std::endl;

        if (clients_.empty()) {
            std::cout << "  ⚠️  Нет подключенных клиентов" << std::endl;
            std::cout << "  💡 Ожидание подключения..." << std::endl;
        } else {
            std::cout << "  ✅ Подключены клиенты" << std::endl;
        }
        std::cout << "========================================\n" << COLOR_RESET << std::endl;
    }

    // ============================================
    // ПЕРЕМЕННЫЕ
    // ============================================
    std::string server_ip_;
    int server_port_;
    int max_clients_;
    std::string gimbal_topic_, teleop_topic_, key_topic_, keyboard_commands_topic_;
    std::string camera_cmd_topic_, telega_cmd_topic_;

    int server_fd_;
    std::vector<int> clients_;
    std::mutex clients_mutex_;
    std::thread accept_thread_, read_thread_, keyboard_thread_;
    std::atomic<bool> running_;
    struct termios old_termios_;

    rclcpp::Subscription<mavros_msgs::msg::GimbalManagerSetPitchyaw>::SharedPtr gimbal_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr teleop_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_commands_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telega_cmd_pub_;

    rclcpp::TimerBase::SharedPtr stats_timer_;
    std::chrono::steady_clock::time_point last_activity_time_;
    int command_count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}