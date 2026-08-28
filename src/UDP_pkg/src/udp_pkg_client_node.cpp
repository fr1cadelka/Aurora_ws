#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <vector>
#include <cstring>

class FastUdpClient : public rclcpp::Node
{
public:
    FastUdpClient() : Node("fast_udp_client"), running_(true), sock_(-1),
        frame_count_(0), bytes_received_(0)
    {
        RCLCPP_INFO(this->get_logger(), "=== FAST UDP CLIENT (Low Latency) ===");

        int port = this->declare_parameter("port", 12346);
        RCLCPP_INFO(this->get_logger(), "Listening on port %d", port);

        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
            throw std::runtime_error("Socket failed");
        }

        // Большой буфер приёма
        int rcvbuf = 8 * 1024 * 1024;
        setsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

        // Короткий таймаут
        struct timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = 5000;   // 5 мс
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family      = AF_INET;
        addr.sin_port        = htons(port);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Bind failed on port %d", port);
            close(sock_);
            throw std::runtime_error("Bind failed");
        }

        cv::namedWindow("Video", cv::WINDOW_NORMAL);
        cv::resizeWindow("Video", 800, 450);

        recv_thread_ = std::thread(&FastUdpClient::receive_loop, this);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&FastUdpClient::log_fps, this)
            );

        RCLCPP_INFO(this->get_logger(), "Client ready");
    }

    ~FastUdpClient()
    {
        running_ = false;
        if (recv_thread_.joinable())
            recv_thread_.join();
        if (sock_ >= 0)
            close(sock_);
        cv::destroyAllWindows();
        RCLCPP_INFO(this->get_logger(), "Client stopped");
    }

private:
    int sock_;
    std::thread recv_thread_;
    std::atomic<bool> running_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<int> frame_count_;
    std::atomic<int> bytes_received_;

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

            if (n <= 0)
                continue;

            bytes_received_ += n;

            // Быстрая проверка JPEG
            if (n < 2 || buffer[0] != 0xFF || buffer[1] != 0xD8)
                continue;

            try {
                std::vector<uchar> data(buffer.data(), buffer.data() + n);
                cv::Mat img = cv::imdecode(data, cv::IMREAD_COLOR);

                if (!img.empty()) {
                    cv::imshow("Video", img);
                    cv::waitKey(1);
                    ++frame_count_;
                }
            }
            catch (...) {
                // игнорируем битые кадры
            }
        }
    }

    void log_fps()
    {
        int fps   = frame_count_.exchange(0);
        int bytes = bytes_received_.exchange(0);
        RCLCPP_INFO(this->get_logger(),
                    "[Client] FPS: %d | %.2f MB/s",
                    fps, bytes / (1024.0 * 1024.0));
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<FastUdpClient>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("FastUdpClient"), "Fatal: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
