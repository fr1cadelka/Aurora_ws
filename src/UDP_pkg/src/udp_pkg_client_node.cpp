#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <map>
#include <chrono>

#pragma pack(push, 1)
struct FrameHeader {
    uint32_t frame_id;
    uint16_t chunk_idx;
    uint16_t total_chunks;
    uint32_t chunk_size;
    uint32_t total_size;
    uint64_t timestamp_us;
};
#pragma pack(pop)

static constexpr size_t MAX_PACKET_SIZE   = 1500;
static constexpr size_t HEADER_SIZE       = sizeof(FrameHeader);
static constexpr size_t MAX_CHUNK_PAYLOAD = 1400 - HEADER_SIZE;  // должно совпадать с сервером

struct PartialFrame {
    uint32_t total_size = 0;
    uint16_t total_chunks = 0;
    uint16_t received = 0;
    std::vector<uint8_t> data;
    std::vector<bool> mask;
    std::chrono::steady_clock::time_point first_seen;
};

class UdpClientNode : public rclcpp::Node
{
public:
    UdpClientNode() : Node("udp_client_node"),
        sock_(-1), running_(true),
        fps_(0), bad_(0), dropped_(0), bytes_(0),
        display_ready_(false)
    {
        int port = declare_parameter("port", 12346);
        std::string topic = declare_parameter("topic", "/camera_image/compressed");
        show_window_ = declare_parameter("show_window", true);
        stale_ms_    = declare_parameter("stale_timeout_ms", 200);

        RCLCPP_INFO(get_logger(), "UDP client listening on :%d", port);
        RCLCPP_INFO(get_logger(), "Publish topic: %s | show_window=%d",
                    topic.c_str(), show_window_);

        pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

        sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) throw std::runtime_error("socket() failed");

        int rcvbuf = 32 * 1024 * 1024;
        setsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
        int reuse = 1;
        setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        timeval tv{0, 100000};  // 100 ms
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port);
        addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            close(sock_);
            throw std::runtime_error("bind() failed");
        }

        recv_thread_ = std::thread(&UdpClientNode::recv_loop, this);

        stats_timer_ = create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&UdpClientNode::log_stats, this));
    }

    ~UdpClientNode() {
        running_ = false;
        display_cv_.notify_all();
        if (recv_thread_.joinable()) recv_thread_.join();
        if (sock_ >= 0) close(sock_);
        if (show_window_) cv::destroyAllWindows();
    }

    // Вызывается из главного потока (main), блокирующая
    void run_display() {
        if (!show_window_) return;
        while (running_ && rclcpp::ok()) {
            std::vector<uint8_t> data;
            {
                std::unique_lock<std::mutex> lk(display_mtx_);
                display_cv_.wait_for(lk, std::chrono::milliseconds(200),
                                     [&] { return !running_ || display_ready_; });
                if (!running_) break;
                if (!display_ready_) continue;
                data = display_data_;
                display_ready_ = false;
            }
            if (data.empty()) continue;
            cv::Mat raw(1, static_cast<int>(data.size()), CV_8UC1, data.data());
            cv::Mat img = cv::imdecode(raw, cv::IMREAD_COLOR);
            if (img.empty()) continue;
            cv::imshow("Aurora Rover Camera", img);
            int k = cv::waitKey(1);
            if (k == 27) { running_ = false; rclcpp::shutdown(); }
        }
    }

private:
    int sock_;
    std::thread recv_thread_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;

    std::atomic<bool> running_;
    std::atomic<int> fps_, bad_, dropped_;
    std::atomic<uint64_t> bytes_;
    bool show_window_;
    int stale_ms_;

    std::map<uint32_t, PartialFrame> partial_;
    uint32_t last_published_id_ = 0;
    bool have_last_ = false;

    std::mutex display_mtx_;
    std::condition_variable display_cv_;
    std::vector<uint8_t> display_data_;
    bool display_ready_;

    void recv_loop() {
        std::vector<uint8_t> buf(MAX_PACKET_SIZE);

        while (running_ && rclcpp::ok()) {
            sockaddr_in from{};
            socklen_t from_len = sizeof(from);
            ssize_t n = ::recvfrom(sock_, buf.data(), buf.size(), 0,
                                   reinterpret_cast<sockaddr*>(&from), &from_len);
            if (n < 0) { cleanup_stale(); continue; }
            if (static_cast<size_t>(n) < HEADER_SIZE) { ++bad_; continue; }

            bytes_ += static_cast<uint64_t>(n);

            FrameHeader hdr;
            std::memcpy(&hdr, buf.data(), HEADER_SIZE);

            if (hdr.total_chunks == 0 ||
                hdr.chunk_idx >= hdr.total_chunks ||
                hdr.chunk_size == 0 ||
                hdr.chunk_size > MAX_CHUNK_PAYLOAD ||
                HEADER_SIZE + hdr.chunk_size > static_cast<size_t>(n)) {
                ++bad_; continue;
            }

            // Устаревшие кадры — в топку
            if (have_last_ && hdr.frame_id <= last_published_id_) continue;

            auto& pf = partial_[hdr.frame_id];
            if (pf.total_size == 0) {
                pf.total_size     = hdr.total_size;
                pf.total_chunks   = hdr.total_chunks;
                pf.received       = 0;
                pf.data.resize(hdr.total_size);
                pf.mask.assign(hdr.total_chunks, false);
                pf.first_seen     = std::chrono::steady_clock::now();
            }

            size_t offset = static_cast<size_t>(hdr.chunk_idx) * MAX_CHUNK_PAYLOAD;
            if (offset + hdr.chunk_size > pf.data.size()) {
                ++bad_;
                partial_.erase(hdr.frame_id);
                continue;
            }

            if (!pf.mask[hdr.chunk_idx]) {
                std::memcpy(pf.data.data() + offset,
                            buf.data() + HEADER_SIZE, hdr.chunk_size);
                pf.mask[hdr.chunk_idx] = true;
                pf.received++;
            }

            if (pf.received == pf.total_chunks) {
                publish_frame(pf.data);
                last_published_id_ = hdr.frame_id;
                have_last_ = true;
                partial_.erase(hdr.frame_id);
            }
        }
    }

    void cleanup_stale() {
        auto now = std::chrono::steady_clock::now();
        auto limit = std::chrono::milliseconds(stale_ms_);
        for (auto it = partial_.begin(); it != partial_.end(); ) {
            if (now - it->second.first_seen > limit) {
                ++dropped_;
                it = partial_.erase(it);
            } else ++it;
        }
    }

    void publish_frame(const std::vector<uint8_t>& data) {
        if (data.size() < 4 || data[0] != 0xFF || data[1] != 0xD8) {
            ++bad_;
            return;
        }

        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp    = now();
        msg->header.frame_id = "camera";
        msg->format          = "jpeg";
        msg->data            = data;
        pub_->publish(std::move(msg));
        ++fps_;

        if (show_window_) {
            std::lock_guard<std::mutex> lk(display_mtx_);
            display_data_ = data;
            display_ready_ = true;
            display_cv_.notify_one();
        }
    }

    void log_stats() {
        int f = fps_.exchange(0);
        int b = bad_.exchange(0);
        int d = dropped_.exchange(0);
        uint64_t by = bytes_.exchange(0);
        if (f > 0)
            RCLCPP_INFO(get_logger(), "[UDP Client] FPS: %d | %.2f MB/s | bad: %d | dropped: %d",
                        f, by / 1024.0 / 1024.0, b, d);
        else
            RCLCPP_WARN(get_logger(), "[UDP Client] no frames | bad: %d | dropped: %d", b, d);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<UdpClientNode>();
        // ROS-спин в фоне, дисплей — в главном потоке
        std::thread spin_thread([node] { rclcpp::spin(node); });
        node->run_display();
        rclcpp::shutdown();
        if (spin_thread.joinable()) spin_thread.join();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("udp_client"), "Fatal: %s", e.what());
        return 1;
    }
    return 0;
}