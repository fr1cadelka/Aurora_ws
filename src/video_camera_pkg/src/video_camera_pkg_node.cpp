#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <thread>
#include <atomic>
#include <memory>
#include <chrono>
#include <cstdlib>
#include <queue>
#include <fcntl.h>
#include <unistd.h>

class VideoCamera : public rclcpp::Node
{
public:
    VideoCamera() : Node("video_camera_node"), running_(true), capture_running_(false)
    {
        RCLCPP_INFO(this->get_logger(), "=== VideoCamera Node (ULTRA LOW LATENCY) ===");

        // ==== ПАРАМЕТРЫ ДЛЯ НАСТРОЙКИ РАЗРЕШЕНИЯ И FPS ====

        // Параметры RTSP URL
        this->declare_parameter<std::string>("rtsp_url_0", "rtsp://admin:aurora_2050@192.168.31.166:554/ch1/sub/av_stream");
        this->declare_parameter<std::string>("rtsp_url_1", "rtsp://admin:aurora_2050@192.168.31.166:554/ch2/sub/av_stream");

        // Параметры разрешения
        this->declare_parameter<int>("frame_width", 1080);    // Ширина кадра
        this->declare_parameter<int>("frame_height", 720);   // Высота кадра
        this->declare_parameter<int>("fps", 30);             // Частота кадров

        // Параметры для отображения
        this->declare_parameter<bool>("show_window", true);  // Показывать окно или нет

        rtsp_urls_[0] = this->get_parameter("rtsp_url_0").as_string();
        rtsp_urls_[1] = this->get_parameter("rtsp_url_1").as_string();

        frame_width_ = this->get_parameter("frame_width").as_int();
        frame_height_ = this->get_parameter("frame_height").as_int();
        target_fps_ = this->get_parameter("fps").as_int();
        show_window_ = this->get_parameter("show_window").as_bool();

        RCLCPP_INFO(this->get_logger(), "Configuration:");
        RCLCPP_INFO(this->get_logger(), "  Resolution: %dx%d", frame_width_, frame_height_);
        RCLCPP_INFO(this->get_logger(), "  Target FPS: %d", target_fps_);
        RCLCPP_INFO(this->get_logger(), "  Show Window: %s", show_window_ ? "YES" : "NO");
        RCLCPP_INFO(this->get_logger(), "RTSP URLs:");
        for (int i = 0; i < 2; ++i) {
            RCLCPP_INFO(this->get_logger(), "  [%d] %s", i, rtsp_urls_[i].c_str());
        }

        // Публикатор изображений с QoS для минимальной задержки
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera_image", qos);

        // Подписка на команду переключения потока
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/set_rtsp_stream", 10,
            std::bind(&VideoCamera::on_set_stream, this, std::placeholders::_1));

        // Запускаем первый поток
        current_stream_id_ = 0;
        start_capture(rtsp_urls_[0]);

        RCLCPP_INFO(this->get_logger(), "VideoCamera Node ready!");
    }

    ~VideoCamera()
    {
        running_ = false;
        stop_capture();
        if (capture_thread_.joinable()) capture_thread_.join();
        if (show_window_) cv::destroyAllWindows();
        RCLCPP_INFO(this->get_logger(), "VideoCamera Node shutdown");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    std::string rtsp_urls_[2];
    int current_stream_id_;
    std::atomic<bool> running_;
    std::atomic<bool> capture_running_;
    std::thread capture_thread_;
    cv::VideoCapture cap_;

    // Параметры настройки
    int frame_width_;
    int frame_height_;
    int target_fps_;
    bool show_window_;

    // Для измерения реальной задержки
    std::chrono::steady_clock::time_point frame_capture_time_;
    rclcpp::Time frame_publish_time_;

    void start_capture(const std::string& url)
    {
        stop_capture();

        // МАКСИМАЛЬНО АГРЕССИВНЫЕ НАСТРОЙКИ FFMPEG
        setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS",
               "rtsp_transport;tcp|"
               "fflags;nobuffer|"
               "flags;low_delay|"
               "avioflags;direct|"
               "stimeout;1000000|"
               "reorder_queue_size;0|"
               "buffer_size;1024|"
               "max_delay;0|"
               "strict;experimental|"
               "threads;1|"
               "analyzeduration;0|"
               "probesize;32",
               1);

        // Открываем захват
        cap_.open(url, cv::CAP_FFMPEG);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream: %s", url.c_str());
            return;
        }

        // ==== УСТАНАВЛИВАЕМ ПАРАМЕТРЫ РАЗРЕШЕНИЯ И FPS ====

        RCLCPP_INFO(this->get_logger(), "Setting capture parameters:");

        // Устанавливаем разрешение
        bool width_set = cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
        bool height_set = cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
        RCLCPP_INFO(this->get_logger(), "  Width: %d (%s)", frame_width_, width_set ? "OK" : "FAILED");
        RCLCPP_INFO(this->get_logger(), "  Height: %d (%s)", frame_height_, height_set ? "OK" : "FAILED");

        // Устанавливаем FPS
        bool fps_set = cap_.set(cv::CAP_PROP_FPS, target_fps_);
        RCLCPP_INFO(this->get_logger(), "  FPS: %d (%s)", target_fps_, fps_set ? "OK" : "FAILED");

        // Дополнительные параметры
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('H','2','6','4'));

        // Получаем реальные значения (могут отличаться от запрошенных)
        int actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        int actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        double actual_fps = cap_.get(cv::CAP_PROP_FPS);

        RCLCPP_INFO(this->get_logger(), "Actual capture parameters:");
        RCLCPP_INFO(this->get_logger(), "  Resolution: %dx%d", actual_width, actual_height);
        RCLCPP_INFO(this->get_logger(), "  FPS: %.2f", actual_fps);

        // Если разрешение не поддерживается, пробуем стандартные
        if (actual_width == 0 || actual_height == 0) {
            RCLCPP_WARN(this->get_logger(), "Resolution not supported, trying standard values");
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        }

        // Создаем окно для отображения (если нужно)
        if (show_window_) {
            cv::namedWindow("Camera Stream", cv::WINDOW_NORMAL);
            cv::resizeWindow("Camera Stream", std::min(800, frame_width_ * 2), std::min(600, frame_height_ * 2));
            cv::moveWindow("Camera Stream", 100, 100);
        }

        capture_running_ = true;
        if (capture_thread_.joinable()) capture_thread_.join();
        capture_thread_ = std::thread([this]() {
            cv::Mat frame;
            rclcpp::Time last_log = this->now();
            unsigned int frame_count = 0;
            unsigned int consecutive_failures = 0;

            // Вычисляем задержку между кадрами для FPS
            auto frame_interval = std::chrono::milliseconds(1000 / target_fps_);
            auto last_frame_time = std::chrono::steady_clock::now();

            while (running_ && capture_running_) {
                auto frame_start = std::chrono::steady_clock::now();

                // Захватываем кадр
                if (!cap_.grab()) {
                    consecutive_failures++;
                    if (consecutive_failures > 3) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                             "Grab failed. Reconnecting...");
                        cap_.release();
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                        cap_.open(rtsp_urls_[current_stream_id_], cv::CAP_FFMPEG);
                        if (cap_.isOpened()) {
                            // Переустанавливаем параметры
                            cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
                            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
                            cap_.set(cv::CAP_PROP_FPS, target_fps_);
                            consecutive_failures = 0;
                            RCLCPP_INFO(this->get_logger(), "Reconnected");
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }

                // Извлекаем кадр
                if (!cap_.retrieve(frame)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }

                consecutive_failures = 0;

                if (frame.empty()) {
                    continue;
                }

                // Проверяем размер кадра
                if (frame.cols != frame_width_ || frame.rows != frame_height_) {
                    // Если размер отличается, изменяем размер (это добавляет задержку)
                    if (frame.cols > 0 && frame.rows > 0) {
                        cv::resize(frame, frame, cv::Size(frame_width_, frame_height_), 0, 0, cv::INTER_NEAREST);
                    }
                }

                // Сохраняем время захвата для измерения задержки
                frame_capture_time_ = std::chrono::steady_clock::now();

                // Показываем видео в окне (если нужно)
                if (show_window_) {
                    cv::imshow("Camera Stream", frame);
                    cv::waitKey(1);
                }

                // Публикуем кадр
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                msg->header.stamp = this->now();
                msg->header.frame_id = "camera_frame";
                publisher_->publish(*msg);
                frame_publish_time_ = this->now();

                // Логирование FPS и задержки
                frame_count++;
                auto now = this->now();
                if ((now - last_log).seconds() >= 1.0) {
                    auto now_time = std::chrono::steady_clock::now();
                    auto total_delay = std::chrono::duration_cast<std::chrono::milliseconds>(
                                           now_time - frame_capture_time_).count();

                    RCLCPP_INFO(this->get_logger(),
                                "[VideoCamera] FPS: %d, Size: %dx%d, Resolution: %dx%d, Delay: %ldms",
                                frame_count, frame.cols, frame.rows, frame_width_, frame_height_, total_delay);
                    frame_count = 0;
                    last_log = now;
                }

                // Управление FPS - ждем если кадры приходят быстрее чем нужно
                auto frame_end = std::chrono::steady_clock::now();
                auto frame_duration = std::chrono::duration_cast<std::chrono::milliseconds>(frame_end - frame_start);

                if (frame_duration < frame_interval) {
                    std::this_thread::sleep_for(frame_interval - frame_duration);
                }
            }

            cap_.release();
            if (show_window_) cv::destroyWindow("Camera Stream");
            RCLCPP_INFO(this->get_logger(), "Capture thread finished");
        });
    }

    void stop_capture()
    {
        capture_running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        if (cap_.isOpened()) {
            cap_.release();
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
        start_capture(rtsp_urls_[new_id]);
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
