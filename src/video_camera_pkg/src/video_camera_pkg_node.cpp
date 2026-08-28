#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <cstdlib>

class VideoCamera : public rclcpp::Node
{
public:
    VideoCamera() : Node("video_camera_node"), running_(true), frame_ready_(false)
    {
        RCLCPP_INFO(this->get_logger(), "=== VideoCamera (MJPEG → JPEG once) ===");

        this->declare_parameter<std::string>("stream_url",
                                             "http://admin:aurora_2050@192.168.31.166/ISAPI/Streaming/channels/102/httpPreview");
        this->declare_parameter<int>("target_fps", 25);
        this->declare_parameter<int>("jpeg_quality", 50);

        std::string url = this->get_parameter("stream_url").as_string();
        target_fps_   = this->get_parameter("target_fps").as_int();
        jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();

        RCLCPP_INFO(this->get_logger(), "URL: %s", url.c_str());
        RCLCPP_INFO(this->get_logger(), "FPS: %d | JPEG quality: %d", target_fps_, jpeg_quality_);

        setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS",
               "fflags;nobuffer+discardcorrupt|"
               "flags;low_delay|"
               "max_delay;0|"
               "analyzeduration;0|"
               "probesize;32|"
               "buffer_size;1024|"
               "framedrop;1",
               1);

        cap_.open(url, cv::CAP_FFMPEG);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open stream");
            throw std::runtime_error("Stream open failed");
        }
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera_image/compressed",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()
            );

        capture_thread_ = std::thread(&VideoCamera::capture_loop, this);
        publish_thread_ = std::thread(&VideoCamera::publish_loop, this);

        RCLCPP_INFO(this->get_logger(), "Camera started");
    }

    ~VideoCamera()
    {
        running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
        if (publish_thread_.joinable()) publish_thread_.join();
        cap_.release();
    }

private:
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    std::thread capture_thread_;
    std::thread publish_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> frame_ready_;
    std::mutex frame_mutex_;
    cv::Mat latest_frame_;
    int target_fps_;
    int jpeg_quality_;

    void capture_loop()
    {
        cv::Mat frame;
        while (running_ && rclcpp::ok()) {
            if (!cap_.read(frame) || frame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                frame.copyTo(latest_frame_);
                frame_ready_ = true;
            }
        }
    }

    void publish_loop()
    {
        const auto period = std::chrono::microseconds(1000000 / std::max(1, target_fps_));
        auto next_time = std::chrono::steady_clock::now();

        std::vector<uchar> buf;
        std::vector<int> params = {
            cv::IMWRITE_JPEG_QUALITY, jpeg_quality_,
            cv::IMWRITE_JPEG_OPTIMIZE, 0,
            cv::IMWRITE_JPEG_PROGRESSIVE, 0
        };

        sensor_msgs::msg::CompressedImage msg;
        msg.format = "jpeg";
        msg.header.frame_id = "camera";

        int count = 0;
        auto last_log = this->now();

        while (running_ && rclcpp::ok()) {
            next_time += period;
            std::this_thread::sleep_until(next_time);

            cv::Mat frame;
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                if (!frame_ready_ || latest_frame_.empty())
                    continue;
                latest_frame_.copyTo(frame);
            }

            buf.clear();
            if (!cv::imencode(".jpg", frame, buf, params) || buf.empty())
                continue;

            msg.header.stamp = this->now();
            msg.data = buf;   // уже готовый JPEG

            publisher_->publish(msg);

            ++count;
            auto now = this->now();
            if ((now - last_log).seconds() >= 1.0) {
                RCLCPP_INFO(this->get_logger(), "Camera FPS: %d | JPEG ~%zu bytes",
                            count, buf.size());
                count = 0;
                last_log = now;
            }
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<VideoCamera>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("VideoCamera"), "Fatal: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
