#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

class FastUdpServer : public rclcpp::Node
{
public:
    FastUdpServer() : Node("fast_udp_server"), sock_(-1), running_(true),
        frame_count_(0), has_new_frame_(false)
    {
        RCLCPP_INFO(this->get_logger(), "=== FAST UDP SERVER (Threaded + Stable FPS) ===");

        std::string client_ip = this->declare_parameter("server_ip", "127.0.0.1");
        int port              = this->declare_parameter("server_port", 12346);
        int quality           = this->declare_parameter("jpeg_quality", 55);
        int target_fps        = this->declare_parameter("target_fps", 25);
        bool do_resize        = this->declare_parameter("resize", false);
        int resize_width      = this->declare_parameter("resize_width", 960);
        int resize_height     = this->declare_parameter("resize_height", 432);

        RCLCPP_INFO(this->get_logger(), "Target: %s:%d | quality=%d | fps=%d",
                    client_ip.c_str(), port, quality, target_fps);

        jpeg_quality_  = quality;
        target_fps_    = target_fps;
        resize_        = do_resize;
        resize_width_  = resize_width;
        resize_height_ = resize_height;

        // --- UDP socket ---
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) throw std::runtime_error("Socket failed");

        int sndbuf = 8 * 1024 * 1024;
        setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

        memset(&addr_, 0, sizeof(addr_));
        addr_.sin_family = AF_INET;
        addr_.sin_port   = htons(port);
        if (inet_pton(AF_INET, client_ip.c_str(), &addr_.sin_addr) <= 0) {
            close(sock_);
            throw std::runtime_error("Invalid IP");
        }

        // Подписка — только сохраняем последний кадр (очень быстро)
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_image",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(&FastUdpServer::image_callback, this, std::placeholders::_1)
            );

        // Поток кодирования + отправки
        send_thread_ = std::thread(&FastUdpServer::send_loop, this);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&FastUdpServer::log_fps, this)
            );

        RCLCPP_INFO(this->get_logger(), "Server ready (threaded)");
    }

    ~FastUdpServer()
    {
        running_ = false;
        if (send_thread_.joinable())
            send_thread_.join();
        if (sock_ >= 0)
            close(sock_);
        RCLCPP_INFO(this->get_logger(), "Server stopped");
    }

private:
    int sock_;
    struct sockaddr_in addr_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::thread send_thread_;
    std::atomic<bool> running_;
    std::atomic<int> frame_count_;

    // Последний кадр
    std::mutex frame_mutex_;
    cv::Mat latest_frame_;
    std::atomic<bool> has_new_frame_;

    int jpeg_quality_;
    int target_fps_;
    bool resize_;
    int resize_width_;
    int resize_height_;

    // ============================================================
    // Callback — максимально быстрый (только копируем кадр)
    // ============================================================
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (msg->data.empty() || msg->height == 0 || msg->width == 0)
            return;

        try {
            cv::Mat img(msg->height, msg->width, CV_8UC3,
                        const_cast<uint8_t*>(msg->data.data()), msg->step);

            if (msg->encoding == "rgb8") {
                cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
            }

            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                img.copyTo(latest_frame_);
                has_new_frame_ = true;
            }
        }
        catch (...) {}
    }

    // ============================================================
    // Фоновый поток — кодирование + отправка с фиксированным FPS
    // ============================================================
    void send_loop()
    {
        const auto period = std::chrono::microseconds(1000000 / std::max(1, target_fps_));
        auto next_time = std::chrono::steady_clock::now();

        std::vector<uchar> buf;
        std::vector<int> params = {
            cv::IMWRITE_JPEG_QUALITY, jpeg_quality_,
            cv::IMWRITE_JPEG_OPTIMIZE, 0,
            cv::IMWRITE_JPEG_PROGRESSIVE, 0
        };

        while (running_ && rclcpp::ok()) {
            next_time += period;
            std::this_thread::sleep_until(next_time);

            cv::Mat frame;
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                if (!has_new_frame_ || latest_frame_.empty())
                    continue;
                latest_frame_.copyTo(frame);
                has_new_frame_ = false;   // забрали
            }

            try {
                if (resize_ && (frame.cols != resize_width_ || frame.rows != resize_height_)) {
                    cv::resize(frame, frame,
                               cv::Size(resize_width_, resize_height_),
                               0, 0, cv::INTER_NEAREST);
                }

                buf.clear();
                if (!cv::imencode(".jpg", frame, buf, params) || buf.empty())
                    continue;

                ssize_t sent = sendto(sock_, buf.data(), buf.size(), 0,
                                      reinterpret_cast<struct sockaddr*>(&addr_),
                                      sizeof(addr_));
                if (sent > 0)
                    ++frame_count_;
            }
            catch (...) {}
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
