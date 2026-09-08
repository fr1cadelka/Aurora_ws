#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <vector>
#include <cstring>

class UdpVideoPublisher : public rclcpp::Node
{
public:
    UdpVideoPublisher() : Node("udp_video_publisher"), running_(true), sock_(-1),
        frame_count_(0), bytes_received_(0), bad_frames_(0)
    {
        RCLCPP_INFO(this->get_logger(), "=== UDP VIDEO PUBLISHER ===");

        int port = this->declare_parameter("port", 12346);
        std::string topic = this->declare_parameter("topic", "/camera_image/compressed");

        RCLCPP_INFO(this->get_logger(), "Listening on port: %d", port);
        RCLCPP_INFO(this->get_logger(), "Publishing to topic: %s", topic.c_str());

        // ===== ПУБЛИКАТОР ДЛЯ LINE FOLLOWER =====
        image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            topic,
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

        // Создаём UDP сокет
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
            throw std::runtime_error("Socket failed");
        }

        // Большой буфер приёма
        int rcvbuf = 8 * 1024 * 1024;
        setsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

        // Таймаут
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 5000;
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // Настройка адреса
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Bind failed on port %d", port);
            close(sock_);
            throw std::runtime_error("Bind failed");
        }

        // Запускаем поток приёма
        recv_thread_ = std::thread(&UdpVideoPublisher::receive_loop, this);

        // Статистика
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&UdpVideoPublisher::log_stats, this)
            );

        RCLCPP_INFO(this->get_logger(), "UDP Video Publisher ready");
        RCLCPP_INFO(this->get_logger(), "Waiting for video stream on port %d...", port);
    }

    ~UdpVideoPublisher()
    {
        running_ = false;
        if (recv_thread_.joinable())
            recv_thread_.join();
        if (sock_ >= 0)
            close(sock_);
        RCLCPP_INFO(this->get_logger(), "UDP Video Publisher stopped");
    }

private:
    int sock_;
    std::thread recv_thread_;
    std::atomic<bool> running_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::atomic<int> frame_count_;
    std::atomic<int> bytes_received_;
    std::atomic<int> bad_frames_;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;

    void receive_loop()
    {
        const size_t MAX_SIZE = 4 * 1024 * 1024;
        std::vector<uint8_t> buffer(MAX_SIZE);

        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);

        RCLCPP_INFO(this->get_logger(), "Receive thread started");

        while (running_ && rclcpp::ok()) {
            int n = recvfrom(sock_, buffer.data(), MAX_SIZE, 0,
                             reinterpret_cast<struct sockaddr*>(&from), &from_len);

            if (n <= 0) continue;

            bytes_received_ += n;

            // Проверка JPEG
            if (n < 4 || buffer[0] != 0xFF || buffer[1] != 0xD8) {
                bad_frames_++;
                continue;
            }

            try {
                // ===== ПУБЛИКУЕМ В ROS ДЛЯ LINE FOLLOWER =====
                auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
                msg->header.stamp = this->now();
                msg->header.frame_id = "camera";
                msg->format = "jpeg";
                msg->data.assign(buffer.data(), buffer.data() + n);

                image_pub_->publish(std::move(msg));
                frame_count_++;
            }
            catch (const std::exception& e) {
                bad_frames_++;
            }
        }
    }

    void log_stats()
    {
        int fps = frame_count_.exchange(0);
        int bytes = bytes_received_.exchange(0);
        int bad = bad_frames_.exchange(0);

        if (fps > 0) {
            RCLCPP_INFO(this->get_logger(),
                        "[UDP] FPS: %d | %.2f MB/s | Bad: %d | Published: %d",
                        fps, bytes / (1024.0 * 1024.0), bad, fps);
        } else {
            RCLCPP_WARN(this->get_logger(), "[UDP] No data received");
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<UdpVideoPublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("UdpVideoPublisher"), "Fatal error: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
