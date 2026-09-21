#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

class VideoCamera : public rclcpp::Node
{
public:
    VideoCamera() : Node("video_camera_node"),
        running_(true), frame_ready_(false), connected_(false)
    {
        RCLCPP_INFO(get_logger(), "=== VideoCamera (MJPEG -> JPEG) ===");

        stream_url_ = declare_parameter<std::string>(
            "stream_url",
            "http://admin:aurora_2050@192.168.31.166/ISAPI/Streaming/channels/102/httpPreview");
        target_fps_         = declare_parameter("target_fps", 25);
        jpeg_quality_       = declare_parameter("jpeg_quality", 60);
        reconnect_delay_ms_ = declare_parameter("reconnect_delay_ms", 1000);

        RCLCPP_INFO(get_logger(), "URL: %s", stream_url_.c_str());
        RCLCPP_INFO(get_logger(), "fps=%d quality=%d", target_fps_, jpeg_quality_);

        // Более консервативные опции — HikVision нормально открывается
        setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS",
               "fflags;nobuffer|"
               "flags;low_delay|"
               "max_delay;500000|"
               "reorder_queue_size;0",
               1);

        publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera_image/compressed",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

        capture_thread_ = std::thread(&VideoCamera::capture_loop, this);
        publish_thread_ = std::thread(&VideoCamera::publish_loop, this);
    }

    ~VideoCamera() {
        running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
        if (publish_thread_.joinable()) publish_thread_.join();
        std::lock_guard<std::mutex> lk(cap_mutex_);
        if (cap_.isOpened()) cap_.release();
    }

private:
    cv::VideoCapture cap_;
    std::mutex cap_mutex_;
    std::string stream_url_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    std::thread capture_thread_;
    std::thread publish_thread_;
    std::atomic<bool> running_, frame_ready_, connected_;
    std::mutex frame_mutex_;
    cv::Mat latest_frame_;
    int target_fps_, jpeg_quality_, reconnect_delay_ms_;

    bool open_camera() {
        std::lock_guard<std::mutex> lk(cap_mutex_);
        if (cap_.isOpened()) cap_.release();
        bool ok = cap_.open(stream_url_, cv::CAP_FFMPEG);
        if (ok) cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        return ok;
    }

    void capture_loop() {
        cv::Mat frame;
        int failures = 0;

        while (running_ && rclcpp::ok()) {
            if (!connected_) {
                RCLCPP_WARN(get_logger(), "Connecting to camera...");
                if (open_camera()) {
                    connected_ = true;
                    failures = 0;
                    RCLCPP_INFO(get_logger(), "Camera connected");
                } else {
                    RCLCPP_ERROR(get_logger(), "Open failed, retry in %d ms", reconnect_delay_ms_);
                    std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms_));
                    continue;
                }
            }

            bool ok;
            {
                std::lock_guard<std::mutex> lk(cap_mutex_);
                ok = cap_.read(frame);
            }
            if (!ok || frame.empty()) {
                if (++failures > 30) {
                    RCLCPP_WARN(get_logger(), "Read failures > 30, reconnecting");
                    connected_ = false;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            failures = 0;

            {
                std::lock_guard<std::mutex> lk(frame_mutex_);
                frame.copyTo(latest_frame_);
                frame_ready_ = true;
            }
        }
    }

    void publish_loop() {
        const auto period = std::chrono::microseconds(1000000 / std::max(1, target_fps_));
        auto next = std::chrono::steady_clock::now();

        std::vector<uchar> buf;
        std::vector<int> params = {
            cv::IMWRITE_JPEG_QUALITY, jpeg_quality_,
            cv::IMWRITE_JPEG_OPTIMIZE, 0,
            cv::IMWRITE_JPEG_PROGRESSIVE, 0
        };

        int count = 0;
        auto last_log = std::chrono::steady_clock::now();

        while (running_ && rclcpp::ok()) {
            next += period;
            std::this_thread::sleep_until(next);

            cv::Mat frame;
            {
                std::lock_guard<std::mutex> lk(frame_mutex_);
                if (!frame_ready_ || latest_frame_.empty()) continue;
                latest_frame_.copyTo(frame);
            }

            buf.clear();
            if (!cv::imencode(".jpg", frame, buf, params) || buf.empty()) continue;

            sensor_msgs::msg::CompressedImage msg;
            msg.header.stamp = now();
            msg.header.frame_id = "camera";
            msg.format = "jpeg";
            msg.data = buf;
            publisher_->publish(msg);

            ++count;
            auto t = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(t - last_log).count() >= 1) {
                RCLCPP_INFO(get_logger(), "Camera FPS: %d | JPEG ~%zu bytes", count, buf.size());
                count = 0;
                last_log = t;
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<VideoCamera>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("video_camera"), "Fatal: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}