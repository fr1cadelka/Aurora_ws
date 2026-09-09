#include <rclcpp/rclcpp.hpp>
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
#include <algorithm>
#include <termios.h>
#include <ifaddrs.h>
#include <netdb.h>

// ============================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ (ЦВЕТА, ЛОГИ)
// ============================================
#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_RED     "\033[31m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_BLUE    "\033[34m"

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

// ============================================
// НЕБЛОКИРУЮЩЕЕ ЧТЕНИЕ КЛАВИШ
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

// ============================================
// TCP СЕРВЕР
// ============================================
class TcpServerNode : public rclcpp::Node {
public:
    TcpServerNode() : Node("tcp_server_node"),
        server_fd_(-1),
        running_(true),
        command_count_(0)
    {
        log_info("========================================");
        log_info("🌐 TCP СЕРВЕР (VESC)");
        log_info("========================================");

        // Параметры
        this->declare_parameter("server_ip", "127.0.0.1");
        this->declare_parameter("server_port", 6000);
        this->declare_parameter("max_clients", 10);

        server_ip_ = this->get_parameter("server_ip").as_string();
        server_port_ = this->get_parameter("server_port").as_int();
        max_clients_ = this->get_parameter("max_clients").as_int();

        log_info("📋 ПАРАМЕТРЫ:");
        log_info("  server_ip: " + server_ip_);
        log_info("  server_port: " + std::to_string(server_port_));
        log_info("  max_clients: " + std::to_string(max_clients_));

        // Запуск сервера
        if (!start_server(server_ip_, server_port_)) {
            log_error("❌ Не удалось запустить сервер");
            return;
        }

        // Потоки
        read_thread_ = std::thread(&TcpServerNode::read_from_clients, this);
        keyboard_thread_ = std::thread(&TcpServerNode::read_keyboard, this);

        // Таймер статистики
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(15),
            std::bind(&TcpServerNode::print_stats, this));

        last_activity_time_ = std::chrono::steady_clock::now();

        log_info("========================================");
        log_info(COLOR_GREEN "✅ TCP-СЕРВЕР ЗАПУЩЕН" COLOR_RESET);
        log_info("  🌐 IP: " + server_ip_);
        log_info("  🔌 Порт: " + std::to_string(server_port_));
        log_info("  ⌨️  Управление: WASD, Q(круиз), E(ARM), R/F(скорость), ПРОБЕЛ(стоп)");
        log_info("========================================");
        log_info("");
        log_info("💡 Нажмите Ctrl+C для остановки");
        log_info("");
    }

    ~TcpServerNode() {
        log_warn("🛑 ОСТАНОВКА СЕРВЕРА...");
        running_ = false;

        if (read_thread_.joinable()) read_thread_.join();
        if (keyboard_thread_.joinable()) keyboard_thread_.join();
        if (accept_thread_.joinable()) accept_thread_.join();

        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            for (int client : clients_) {
                if (client != -1) close(client);
            }
            clients_.clear();
        }

        if (server_fd_ != -1) close(server_fd_);
        log_info("📊 ВСЕГО КОМАНД: " + std::to_string(command_count_));
        log_info("✅ Сервер остановлен");
    }

private:
    // ============================================
    // ЧТЕНИЕ КЛАВИАТУРЫ
    // ============================================
    void read_keyboard() {
        std::cout << COLOR_CYAN << "\n========================================" << std::endl;
        std::cout << "УПРАВЛЕНИЕ ТЕЛЕЖКОЙ (VESC):" << std::endl;
        std::cout << "  W/S - Вперёд/Назад" << std::endl;
        std::cout << "  A/D - Налево/Направо" << std::endl;
        std::cout << "  Q   - Круиз-контроль (вкл/выкл)" << std::endl;
        std::cout << "  E   - ARM / DISARM" << std::endl;
        std::cout << "  R/F - Увеличить/Уменьшить скорость" << std::endl;
        std::cout << "  ПРОБЕЛ - Аварийный стоп (DISARM)" << std::endl;
        std::cout << "========================================" << COLOR_RESET << std::endl;

        while (running_ && rclcpp::ok()) {
            int ch = getch_nonblock();

            if (ch == -1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            std::string command;
            bool send = false;

            switch (ch) {
            case 'w': case 'W': command = "w"; send = true; break;
            case 's': case 'S': command = "s"; send = true; break;
            case 'a': case 'A': command = "a"; send = true; break;
            case 'd': case 'D': command = "d"; send = true; break;
            case 'q': case 'Q': command = "q"; send = true; break;
            case 'e': case 'E': command = "e"; send = true; break;
            case 'r': case 'R': command = "r"; send = true; break;
            case 'f': case 'F': command = "f"; send = true; break;
            case ' ': command = " "; send = true; break;
            default: break;
            }

            if (send) {
                // Отображаем нажатую клавишу
                std::cout << COLOR_GREEN << "📤 " << command << COLOR_RESET << std::endl;

                // Отправляем всем клиентам
                command += "\n";
                send_to_clients(command.c_str(), command.length());

                command_count_++;
                last_activity_time_ = std::chrono::steady_clock::now();
            }
        }
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
    // ПРИЁМ КЛИЕНТОВ
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
    // ЧТЕНИЕ ДАННЫХ ОТ КЛИЕНТОВ (не используется)
    // ============================================
    void read_from_clients() {
        char buffer[1024];
        log_info("🔄 Поток чтения клиентов запущен (только для поддержки соединения)");

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
                    // Игнорируем входящие данные (сервер только отправляет)
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
        std::cout << "========================================\n" << COLOR_RESET << std::endl;
    }

    // ============================================
    // ПЕРЕМЕННЫЕ
    // ============================================
    std::string server_ip_;
    int server_port_;
    int max_clients_;

    int server_fd_;
    std::vector<int> clients_;
    std::mutex clients_mutex_;
    std::thread accept_thread_, read_thread_, keyboard_thread_;
    std::atomic<bool> running_;

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