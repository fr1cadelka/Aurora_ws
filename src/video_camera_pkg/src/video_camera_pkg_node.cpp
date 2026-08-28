#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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
        RCLCPP_INFO(this->get_logger(), "=== VideoCamera Low-Latency Stable FPS ===");

        this->declare_parameter<std::string>("rtsp_url",
                                             "rtsp://admin:aurora_2050@192.168.31.166:554/Streaming/Channels/102"); // SUB-STREAM!
        this->declare_parameter<int>("target_fps", 25);
        this->declare_parameter<int>("width", 960);
        this->declare_parameter<int>("height", 432);

        std::string url = this->get_parameter("rtsp_url").as_string();
        target_fps_ = this->get_parameter("target_fps").as_int();
        int width  = this->get_parameter("width").as_int();
        int height = this->get_parameter("height").as_int();

        RCLCPP_INFO(this->get_logger(), "RTSP: %s", url.c_str());
        RCLCPP_INFO(this->get_logger(), "Target FPS: %d", target_fps_);

        // Максимально агрессивные настройки низкой задержки
        setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS",
               "rtsp_transport;tcp|"
               "fflags;nobuffer+discardcorrupt+genpts|"
               "flags;low_delay|"
               "max_delay;0|"
               "analyzeduration;0|"
               "probesize;32|"
               "buffer_size;1024|"
               "framedrop;1|"
               "reorder_queue_size;0",
               1);

        cap_.open(url, cv::CAP_FFMPEG);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
            throw std::runtime_error("Camera open failed");
        }

        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        // Не ставим WIDTH/HEIGHT — камера сама отдаёт то, что настроено в sub-stream

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera_image",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()
            );

        // Поток захвата всегда крутится и хранит только последний кадр
        capture_thread_ = std::thread(&VideoCamera::capture_loop, this);
        // Поток публикации с фиксированной частотой
        publish_thread_ = std::thread(&VideoCamera::publish_loop, this);

        RCLCPP_INFO(this->get_logger(), "Camera started (stable FPS mode)");
    }

    ~VideoCamera()
    {
        running_ = false;
        if (capture_thread_.joinable()) capture_thread_.join();
        if (publish_thread_.joinable()) publish_thread_.join();
        cap_.release();
        RCLCPP_INFO(this->get_logger(), "Camera stopped");
    }

private:
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    std::thread capture_thread_;
    std::thread publish_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> frame_ready_;
    std::mutex frame_mutex_;
    cv::Mat latest_frame_;
    int target_fps_;

    void capture_loop()
    {
        cv::Mat frame;
        while (running_ && rclcpp::ok()) {
            if (!cap_.grab()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            if (!cap_.retrieve(frame) || frame.empty())
                continue;

            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                frame.copyTo(latest_frame_);   // только последний кадр
                frame_ready_ = true;
            }
        }
    }

    void publish_loop()
    {
        const auto period = std::chrono::microseconds(1000000 / target_fps_);
        auto next_time = std::chrono::steady_clock::now();

        sensor_msgs::msg::Image msg;
        msg.encoding = "bgr8";
        msg.is_bigendian = false;
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

            msg.height = static_cast<uint32_t>(frame.rows);
            msg.width  = static_cast<uint32_t>(frame.cols);
            msg.step   = static_cast<uint32_t>(frame.step);
            msg.data.assign(frame.datastart, frame.dataend);
            msg.header.stamp = this->now();

            publisher_->publish(msg);

            ++count;
            auto now = this->now();
            if ((now - last_log).seconds() >= 1.0) {
                RCLCPP_INFO(this->get_logger(), "Camera publish FPS: %d (size %dx%d)",
                            count, frame.cols, frame.rows);
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
