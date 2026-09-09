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
#include <chrono>

class TcpClientNode : public rclcpp::Node {
public:
    TcpClientNode() : Node("tcp_client_node"), sockfd_(-1), running_(true) {
        this->declare_parameter("server_ip", "127.0.0.1");
        this->declare_parameter("server_port", 6000);
        this->declare_parameter("reconnect_delay_sec", 3.0);
        this->declare_parameter("max_reconnect_attempts", 300);

        server_ip_ = this->get_parameter("server_ip").as_string();
        server_port_ = this->get_parameter("server_port").as_int();
        reconnect_delay_ = this->get_parameter("reconnect_delay_sec").as_double();
        max_reconnect_attempts_ = this->get_parameter("max_reconnect_attempts").as_int();

        RCLCPP_INFO(this->get_logger(), "=== TCP-клиент (VESC) запущен ===");
        RCLCPP_INFO(this->get_logger(), "  server_ip: %s", server_ip_.c_str());
        RCLCPP_INFO(this->get_logger(), "  server_port: %d", server_port_);
        RCLCPP_INFO(this->get_logger(), "  reconnect_delay: %.1f сек", reconnect_delay_);
        RCLCPP_INFO(this->get_logger(), "  max_reconnect_attempts: %d", max_reconnect_attempts_);

        // Публикатор команд для VESC
        telega_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/telega_commands", 10);

        reconnect_thread_ = std::thread(&TcpClientNode::reconnect_loop, this);
    }

    ~TcpClientNode() {
        running_ = false;
        if (sockfd_ != -1) {
            close(sockfd_);
            sockfd_ = -1;
        }
        if (reconnect_thread_.joinable())
            reconnect_thread_.join();
        if (read_thread_.joinable())
            read_thread_.join();
        RCLCPP_INFO(this->get_logger(), "🛑 TCP-клиент остановлен");
    }

private:
    void reconnect_loop() {
        int attempt = 0;
        while (running_ && rclcpp::ok()) {
            if (sockfd_ == -1) {
                attempt++;
                RCLCPP_INFO(this->get_logger(), "Попытка подключения %d к %s:%d...",
                            attempt, server_ip_.c_str(), server_port_);
                if (connect_to_server()) {
                    RCLCPP_INFO(this->get_logger(), "✅ Подключено к серверу %s:%d",
                                server_ip_.c_str(), server_port_);
                    attempt = 0;
                    if (read_thread_.joinable())
                        read_thread_.join();
                    read_thread_ = std::thread(&TcpClientNode::read_from_server, this);
                } else {
                    if (max_reconnect_attempts_ > 0 && attempt >= max_reconnect_attempts_) {
                        RCLCPP_ERROR(this->get_logger(), "❌ Достигнут лимит попыток. Завершение.");
                        rclcpp::shutdown();
                        return;
                    }
                    RCLCPP_WARN(this->get_logger(), "Не удалось подключиться, повтор через %.1f сек...",
                                reconnect_delay_);
                    std::this_thread::sleep_for(std::chrono::milliseconds(
                        static_cast<int>(reconnect_delay_ * 1000)));
                }
            } else {
                if (!is_connected()) {
                    RCLCPP_WARN(this->get_logger(), "Соединение потеряно. Переподключение...");
                    close(sockfd_);
                    sockfd_ = -1;
                    if (read_thread_.joinable())
                        read_thread_.join();
                } else {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }
    }

    bool connect_to_server() {
        sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Не удалось создать сокет: %s", strerror(errno));
            return false;
        }

        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port_);

        if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Неверный IP-адрес: %s", server_ip_.c_str());
            close(sockfd_);
            sockfd_ = -1;
            return false;
        }

        if (connect(sockfd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка подключения: %s", strerror(errno));
            close(sockfd_);
            sockfd_ = -1;
            return false;
        }

        int flags = fcntl(sockfd_, F_GETFL, 0);
        if (flags != -1) {
            fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);
        }

        // Приветствие (необязательно)
        std::string hello = "HELLO\n";
        send(sockfd_, hello.c_str(), hello.length(), 0);

        return true;
    }

    bool is_connected() {
        if (sockfd_ == -1) return false;
        char buffer[1];
        ssize_t ret = recv(sockfd_, buffer, 0, MSG_DONTWAIT | MSG_PEEK);
        if (ret == 0) return false;
        if (ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return false;
        return true;
    }

    void read_from_server() {
        char buffer[4096];
        std::string partial_line;

        RCLCPP_INFO(this->get_logger(), "📡 Начало чтения данных от сервера");

        while (running_ && sockfd_ != -1 && rclcpp::ok()) {
            memset(buffer, 0, sizeof(buffer));
            ssize_t bytes_read = recv(sockfd_, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);

            if (bytes_read > 0) {
                buffer[bytes_read] = '\0';
                std::string data = partial_line + std::string(buffer);
                partial_line.clear();

                std::istringstream stream(data);
                std::string line;

                while (std::getline(stream, line)) {
                    // Удаляем \r, если есть
                    line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());

                    if (!line.empty()) {
                        // Игнорируем "HELLO" если оно пришло
                        if (line == "HELLO") continue;

                        // Публикуем команду в топик /telega_commands
                        auto msg = std_msgs::msg::String();
                        msg.data = line;
                        telega_cmd_pub_->publish(msg);

                        RCLCPP_INFO(this->get_logger(), "📨 Получена команда: '%s'", line.c_str());
                    }
                }
            }
            else if (bytes_read == 0) {
                RCLCPP_WARN(this->get_logger(), "Сервер закрыл соединение");
                break;
            }
            else {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    RCLCPP_WARN(this->get_logger(), "Ошибка чтения: %s", strerror(errno));
                    break;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (sockfd_ != -1) {
            close(sockfd_);
            sockfd_ = -1;
        }
        RCLCPP_INFO(this->get_logger(), "Поток чтения завершён");
    }

    std::string server_ip_;
    int server_port_;
    double reconnect_delay_;
    int max_reconnect_attempts_;

    int sockfd_;
    std::atomic<bool> running_;
    std::thread reconnect_thread_;
    std::thread read_thread_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telega_cmd_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}