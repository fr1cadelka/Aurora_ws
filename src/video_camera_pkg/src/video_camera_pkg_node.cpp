#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <cstdio>
#include <array>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <memory>

class VideoCamera : public rclcpp::Node
{
public:
    VideoCamera() : Node("video_camera_node"), running_(true)
    {
        RCLCPP_INFO(this->get_logger(), "=== VideoCamera Node (ffmpeg) ===");

        this->declare_parameter<std::string>("rtsp_url_0", "rtsp://admin:aurora_2050@192.168.31.166:554/ch1");
        this->declare_parameter<std::string>("rtsp_url_1", "rtsp://admin:aurora_2050@192.168.31.166:554/ch2");
        rtsp_urls_[0] = this->get_parameter("rtsp_url_0").as_string();
        rtsp_urls_[1] = this->get_parameter("rtsp_url_1").as_string();

        RCLCPP_INFO(this->get_logger(), "RTSP URLs:");
        for (int i = 0; i < 2; ++i) {
            RCLCPP_INFO(this->get_logger(), "  [%d] %s", i, rtsp_urls_[i].c_str());
        }

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera_image", qos);

        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/set_rtsp_stream", 10,
            std::bind(&VideoCamera::on_set_stream, this, std::placeholders::_1));

        current_stream_id_ = 0;
        start_ffmpeg(rtsp_urls_[0]);

        RCLCPP_INFO(this->get_logger(), "VideoCamera Node ready!");
    }

    ~VideoCamera()
    {
        running_ = false;
        if (ffmpeg_thread_.joinable()) ffmpeg_thread_.join();
        if (pipe_) pclose(pipe_);
        RCLCPP_INFO(this->get_logger(), "VideoCamera Node shutdown");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    std::string rtsp_urls_[2];
    int current_stream_id_;
    std::atomic<bool> running_;
    std::thread ffmpeg_thread_;
    FILE* pipe_ = nullptr;

    void start_ffmpeg(const std::string& url)
    {
        if (ffmpeg_thread_.joinable()) {
            running_ = false;
            ffmpeg_thread_.join();
            running_ = true;
        }
        if (pipe_) {
            pclose(pipe_);
            pipe_ = nullptr;
        }

        // Команда ffmpeg: выводим сырой H.264 в stdout
        std::string cmd = "ffmpeg -rtsp_transport tcp -i \"" + url + "\" -c copy -f h264 - 2>/dev/null";
        pipe_ = popen(cmd.c_str(), "r");
        if (!pipe_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start ffmpeg for URL %s", url.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "ffmpeg started for URL: %s", url.c_str());

        ffmpeg_thread_ = std::thread([this]() {
            const size_t BUFSIZE = 65536;
            std::array<char, BUFSIZE> buffer;
            std::vector<uint8_t> frame_data;
            bool in_frame = false;

            while (running_ && !feof(pipe_)) {
                size_t bytes = fread(buffer.data(), 1, BUFSIZE, pipe_);
                if (bytes == 0) {
                    if (feof(pipe_)) break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }

                // Поиск стартовых кодов
                for (size_t i = 0; i < bytes; ) {
                    if (i + 3 < bytes && buffer[i] == 0x00 && buffer[i+1] == 0x00 && buffer[i+2] == 0x00 && buffer[i+3] == 0x01) {
                        // 4-байтовый стартовый код
                        if (!frame_data.empty() && in_frame) {
                            publish_frame(frame_data);
                            frame_data.clear();
                        }
                        frame_data.insert(frame_data.end(), &buffer[i], &buffer[i] + 4);
                        i += 4;
                        in_frame = true;
                        continue;
                    }
                    if (i + 2 < bytes && buffer[i] == 0x00 && buffer[i+1] == 0x00 && buffer[i+2] == 0x01) {
                        // 3-байтовый стартовый код
                        if (!frame_data.empty() && in_frame) {
                            publish_frame(frame_data);
                            frame_data.clear();
                        }
                        frame_data.insert(frame_data.end(), &buffer[i], &buffer[i] + 3);
                        i += 3;
                        in_frame = true;
                        continue;
                    }
                    if (in_frame) {
                        frame_data.push_back(buffer[i]);
                    }
                    i++;
                }
            }
            if (!frame_data.empty()) {
                publish_frame(frame_data);
            }
            RCLCPP_INFO(this->get_logger(), "ffmpeg thread finished");
        });
    }

    void publish_frame(const std::vector<uint8_t>& data)
    {
        if (data.empty()) return;
        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";
        msg->encoding = "h264";
        msg->data.assign(data.begin(), data.end());
        publisher_->publish(*msg);
        static unsigned int frame_count = 0;
        static rclcpp::Time last_log = this->now();
        frame_count++;
        auto now = this->now();
        if ((now - last_log).seconds() >= 1.0) {
            RCLCPP_INFO(this->get_logger(), "[VideoCamera] FPS: %d, Frame size: %.2f KB",
                        frame_count, data.size() / 1024.0);
            frame_count = 0;
            last_log = now;
        }
    }

    void on_set_stream(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int new_id = msg->data;
        if (new_id < 0 || new_id > 1) {
            RCLCPP_WARN(this->get_logger(), "Invalid stream id: %d (must be 0,1)", new_id);
            return;
        }
        if (new_id == current_stream_id_) return;

        RCLCPP_INFO(this->get_logger(), "Switching to stream %d: %s", new_id, rtsp_urls_[new_id].c_str());
        current_stream_id_ = new_id;
        start_ffmpeg(rtsp_urls_[new_id]);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoCamera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}