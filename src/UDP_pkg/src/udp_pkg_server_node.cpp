#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cerrno>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>
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

static constexpr size_t MAX_PACKET_SIZE  = 1400;
static constexpr size_t HEADER_SIZE      = sizeof(FrameHeader);
static constexpr size_t MAX_CHUNK_PAYLOAD = MAX_PACKET_SIZE - HEADER_SIZE;

class UdpServerNode : public rclcpp::Node
{
public:
    UdpServerNode() : Node("udp_server_node"),
        sock_(-1), running_(true),
        rx_frames_(0), tx_frames_(0), tx_bytes_(0)
    {
        std::string client_ip = declare_parameter("server_ip", "192.168.68.98");
        int port              = declare_parameter("server_port", 12346);
        target_fps_           = declare_parameter("target_fps", 25);

        RCLCPP_INFO(get_logger(), "UDP server -> %s:%d @ %d fps",
                    client_ip.c_str(), port, target_fps_);

        sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) throw std::runtime_error("socket() failed");

        int sndbuf = 16 * 1024 * 1024;
        setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

        int flags = fcntl(sock_, F_GETFL, 0);
        fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

        std::memset(&addr_, 0, sizeof(addr_));
        addr_.sin_family = AF_INET;
        addr_.sin_port   = htons(port);
        if (inet_pton(AF_INET, client_ip.c_str(), &addr_.sin_addr) <= 0) {
            close(sock_);
            throw std::runtime_error("Invalid client IP");
        }

        subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_image/compressed",
            rclcpp::QoS(rclcpp::KeepLast(2)).best_effort(),
            std::bind(&UdpServerNode::on_image, this, std::placeholders::_1));

        send_thread_ = std::thread(&UdpServerNode::send_loop, this);

        stats_timer_ = create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&UdpServerNode::log_stats, this));
    }

    ~UdpServerNode() {
        running_ = false;
        cv_.notify_all();
        if (send_thread_.joinable()) send_thread_.join();
        if (sock_ >= 0) close(sock_);
    }

private:
    int sock_;
    sockaddr_in addr_;
    std::thread send_thread_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    std::atomic<bool> running_;
    std::atomic<int> rx_frames_, tx_frames_;
    std::atomic<uint64_t> tx_bytes_;
    int target_fps_;

    std::mutex mtx_;
    std::condition_variable cv_;
    std::vector<uint8_t> pending_data_;
    uint64_t pending_id_ = 0;

    void on_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        if (msg->data.empty()) return;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            pending_data_ = msg->data;   // всегда держим только свежий кадр
            ++pending_id_;
        }
        rx_frames_++;
        cv_.notify_one();
    }

    void send_loop() {
        using clock = std::chrono::steady_clock;
        auto next = clock::now();
        const auto period = std::chrono::microseconds(1000000 / std::max(1, target_fps_));

        uint64_t last_sent_id = 0;
        uint32_t frame_id = 0;
        std::vector<uint8_t> packet(MAX_PACKET_SIZE);

        while (running_ && rclcpp::ok()) {
            std::vector<uint8_t> data;
            uint64_t cur_id = 0;

            {
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.wait_for(lk, period, [&] {
                    return !running_ || pending_id_ != last_sent_id;
                });
                if (!running_) break;
                if (pending_data_.empty() || pending_id_ == last_sent_id)
                    continue;
                data = pending_data_;
                cur_id = pending_id_;
            }
            last_sent_id = cur_id;

            // Ограничение FPS
            next += period;
            auto now_c = clock::now();
            if (next > now_c) std::this_thread::sleep_until(next);
            else              next = now_c;

            // Разбиение и отправка
            uint32_t total_size = static_cast<uint32_t>(data.size());
            uint16_t total_chunks = static_cast<uint16_t>(
                (total_size + MAX_CHUNK_PAYLOAD - 1) / MAX_CHUNK_PAYLOAD);
            uint64_t ts = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::system_clock::now().time_since_epoch()).count();

            auto* hdr = reinterpret_cast<FrameHeader*>(packet.data());
            uint64_t sent_bytes = 0;
            bool ok = true;

            for (uint16_t i = 0; i < total_chunks; ++i) {
                size_t offset = static_cast<size_t>(i) * MAX_CHUNK_PAYLOAD;
                size_t chunk  = std::min(MAX_CHUNK_PAYLOAD, data.size() - offset);

                hdr->frame_id     = frame_id;
                hdr->chunk_idx    = i;
                hdr->total_chunks = total_chunks;
                hdr->chunk_size   = static_cast<uint32_t>(chunk);
                hdr->total_size   = total_size;
                hdr->timestamp_us = ts;

                std::memcpy(packet.data() + HEADER_SIZE, data.data() + offset, chunk);

                ssize_t s = ::sendto(sock_, packet.data(), HEADER_SIZE + chunk, 0,
                                     reinterpret_cast<sockaddr*>(&addr_), sizeof(addr_));
                if (s < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        // Буфер полон — дропаем весь кадр, чтобы не отставать
                        ok = false;
                        break;
                    }
                } else {
                    sent_bytes += static_cast<uint64_t>(s);
                }
            }

            if (ok) {
                ++frame_id;
                tx_frames_++;
                tx_bytes_ += sent_bytes;
            }
        }
    }

    void log_stats() {
        int rx = rx_frames_.exchange(0);
        int tx = tx_frames_.exchange(0);
        uint64_t bytes = tx_bytes_.exchange(0);
        RCLCPP_INFO(get_logger(), "[UDP Server] RX: %d fps | TX: %d fps | %.2f MB/s",
                    rx, tx, bytes / 1024.0 / 1024.0);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<UdpServerNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("udp_server"), "Fatal: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}