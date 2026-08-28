#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <chrono>

class FastUdpServer : public rclcpp::Node
{
public:
    FastUdpServer() : Node("fast_udp_server"), sock_(-1), running_(true),
        frame_count_(0), data_id_(0)
    {
        RCLCPP_INFO(this->get_logger(), "=== FAST UDP SERVER (forward JPEG) ===");

        std::string client_ip = this->declare_parameter("server_ip", "127.0.0.1");
        int port              = this->declare_parameter("server_port", 12346);
        int target_fps        = this->declare_parameter("target_fps", 25);

        RCLCPP_INFO(this->get_logger(), "Target: %s:%d | fps=%d",
                    client_ip.c_str(), port, target_fps);

        target_fps_ = std::max(1, target_fps);

        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) throw std::runtime_error("Socket failed");

        int sndbuf = 8 * 1024 * 1024;
        setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

        int flags = fcntl(sock_, F_GETFL, 0);
        fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

        memset(&addr_, 0, sizeof(addr_));
        addr_.sin_family = AF_INET;
        addr_.sin_port   = htons(port);
        if (inet_pton(AF_INET, client_ip.c_str(), &addr_.sin_addr) <= 0) {
            close(sock_);
            throw std::runtime_error("Invalid IP");
        }

        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_image/compressed",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(&FastUdpServer::image_callback, this, std::placeholders::_1)
            );

        send_thread_ = std::thread(&FastUdpServer::send_loop, this);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&FastUdpServer::log_fps, this)
            );

        RCLCPP_INFO(this->get_logger(), "Server ready (no re-encode)");
    }

    ~FastUdpServer()
    {
        running_ = false;
        cv_.notify_all();
        if (send_thread_.joinable()) send_thread_.join();
        if (sock_ >= 0) close(sock_);
    }

private:
    int sock_;
    struct sockaddr_in addr_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::thread send_thread_;
    std::atomic<bool> running_;
    std::atomic<int> frame_count_;

    std::mutex data_mutex_;
    std::condition_variable cv_;
    std::vector<uint8_t> latest_data_;
    uint64_t data_id_;

    int target_fps_;

    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        if (msg->data.empty()) return;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_data_ = msg->data;   // уже JPEG
            ++data_id_;
        }
        cv_.notify_one();
    }

    void send_loop()
    {
        using clock = std::chrono::steady_clock;
        const auto period = std::chrono::microseconds(1000000 / target_fps_);

        uint64_t last_sent_id = 0;

        while (running_ && rclcpp::ok()) {
            std::vector<uint8_t> data;
            uint64_t current_id = 0;

            {
                std::unique_lock<std::mutex> lock(data_mutex_);
                cv_.wait_for(lock, period, [&] {
                    return !running_ || data_id_ != last_sent_id;
                });

                if (!running_) break;
                if (latest_data_.empty() || data_id_ == last_sent_id)
                    continue;

                data = latest_data_;
                current_id = data_id_;
            }

            ssize_t sent = sendto(sock_, data.data(), data.size(), 0,
                                  reinterpret_cast<struct sockaddr*>(&addr_),
                                  sizeof(addr_));
            if (sent > 0) {
                ++frame_count_;
                last_sent_id = current_id;
            }
        }
    }

    void log_fps()
    {
        int fps = frame_count_.exchange(0);
        RCLCPP_INFO(this->get_logger(), "[Server] FPS: %d", fps);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<FastUdpServer>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("FastUdpServer"), "Fatal: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
