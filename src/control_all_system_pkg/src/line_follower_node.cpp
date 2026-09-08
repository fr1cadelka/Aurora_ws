#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <algorithm>
#include <chrono>

class LineFollower : public rclcpp::Node {
public:
    LineFollower() : Node("line_follower"), frame_counter_(0) {
        RCLCPP_INFO(this->get_logger(), "=== LINE FOLLOWER NODE ===");

        // ===== 1. ПОДПИСЧИК НА ВИДЕО ОТ UDP =====
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_image/compressed",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(&LineFollower::image_callback, this, std::placeholders::_1));

        // ===== 2. ПУБЛИКАТОРЫ =====
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/line_follower/cmd_vel", 10);

        original_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/line_follower/original_image", 10);
        mask_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/line_follower/mask_image", 10);
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/line_follower/debug_image", 10);

        // ===== 3. ОБЪЯВЛЯЕМ ПАРАМЕТРЫ =====
        declare_parameters();
        load_parameters();

        prev_error_ = 0.0;
        integral_ = 0.0;
        last_valid_time_ = this->now();

        // ===== 4. СОЗДАЁМ ОКНО С ПОЛЗУНКАМИ =====
        create_control_window();

        RCLCPP_INFO(this->get_logger(), "=== LINE FOLLOWER INITIALIZED ===");
        RCLCPP_INFO(this->get_logger(), "Control window with sliders opened");
        RCLCPP_INFO(this->get_logger(), "Publishing to:");
        RCLCPP_INFO(this->get_logger(), "  - /line_follower/cmd_vel (correction commands)");
        RCLCPP_INFO(this->get_logger(), "  - /line_follower/original_image (FULL image)");
        RCLCPP_INFO(this->get_logger(), "  - /line_follower/mask_image (ROI mask)");
        RCLCPP_INFO(this->get_logger(), "  - /line_follower/debug_image (ROI debug)");
    }

    ~LineFollower() {
        cv::destroyWindow("Controls");
    }

private:
    // ===== ПАРАМЕТРЫ =====
    double kp_ = 0.5, ki_ = 0.0, kd_ = 0.1;
    double cruise_speed_ = 0.3;
    double roi_top_ratio_ = 0.4;
    int lost_timeout_ms_ = 3000;
    bool debug_ = true;

    // HSV параметры
    int low_h1_ = 0, high_h1_ = 20;
    int low_h2_ = 160, high_h2_ = 179;
    int low_s_ = 80, high_s_ = 255;
    int low_v_ = 80, high_v_ = 255;
    int min_area_ = 500;
    int min_aspect_ = 30;

    // ===== ПЕРЕМЕННЫЕ ДЛЯ ТРЕКБАРОВ =====
    int track_kp_ = 50, track_ki_ = 0, track_kd_ = 10;
    int track_cruise_speed_ = 30;
    int track_roi_top_ratio_ = 40;
    int track_low_h1_ = 0, track_high_h1_ = 20;
    int track_low_h2_ = 160, track_high_h2_ = 179;
    int track_low_s_ = 80, track_high_s_ = 255;
    int track_low_v_ = 80, track_high_v_ = 255;
    int track_min_area_ = 500;
    int track_min_aspect_ = 30;

    // ===== СОСТОЯНИЕ =====
    double prev_error_ = 0.0;
    double integral_ = 0.0;
    rclcpp::Time last_valid_time_;
    int frame_counter_;

    // ===== ROS =====
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr original_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

    // ===== МЕТОДЫ =====
    void declare_parameters() {
        this->declare_parameter("kp", 0.5);
        this->declare_parameter("ki", 0.0);
        this->declare_parameter("kd", 0.1);
        this->declare_parameter("cruise_speed", 0.3);
        this->declare_parameter("roi_top_ratio", 0.4);
        this->declare_parameter("lost_timeout_ms", 3000);
        this->declare_parameter("debug", true);

        this->declare_parameter("hue_low1", 0);
        this->declare_parameter("hue_high1", 20);
        this->declare_parameter("hue_low2", 160);
        this->declare_parameter("hue_high2", 179);
        this->declare_parameter("sat_low", 80);
        this->declare_parameter("sat_high", 255);
        this->declare_parameter("val_low", 80);
        this->declare_parameter("val_high", 255);
        this->declare_parameter("min_area", 500);
        this->declare_parameter("min_aspect", 30);
    }

    void load_parameters() {
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();
        cruise_speed_ = this->get_parameter("cruise_speed").as_double();
        roi_top_ratio_ = this->get_parameter("roi_top_ratio").as_double();
        lost_timeout_ms_ = this->get_parameter("lost_timeout_ms").as_int();
        debug_ = this->get_parameter("debug").as_bool();

        low_h1_ = this->get_parameter("hue_low1").as_int();
        high_h1_ = this->get_parameter("hue_high1").as_int();
        low_h2_ = this->get_parameter("hue_low2").as_int();
        high_h2_ = this->get_parameter("hue_high2").as_int();
        low_s_ = this->get_parameter("sat_low").as_int();
        high_s_ = this->get_parameter("sat_high").as_int();
        low_v_ = this->get_parameter("val_low").as_int();
        high_v_ = this->get_parameter("val_high").as_int();
        min_area_ = this->get_parameter("min_area").as_int();
        min_aspect_ = this->get_parameter("min_aspect").as_int();

        // Синхронизация трекбаров
        sync_trackbars();
    }

    void sync_trackbars() {
        track_kp_ = static_cast<int>(kp_ * 100);
        track_ki_ = static_cast<int>(ki_ * 100);
        track_kd_ = static_cast<int>(kd_ * 100);
        track_cruise_speed_ = static_cast<int>(cruise_speed_ * 100);
        track_roi_top_ratio_ = static_cast<int>(roi_top_ratio_ * 100);
        track_low_h1_ = low_h1_;
        track_high_h1_ = high_h1_;
        track_low_h2_ = low_h2_;
        track_high_h2_ = high_h2_;
        track_low_s_ = low_s_;
        track_high_s_ = high_s_;
        track_low_v_ = low_v_;
        track_high_v_ = high_v_;
        track_min_area_ = min_area_;
        track_min_aspect_ = min_aspect_;
    }

    void update_from_trackbars() {
        kp_ = track_kp_ / 100.0;
        ki_ = track_ki_ / 100.0;
        kd_ = track_kd_ / 100.0;
        cruise_speed_ = track_cruise_speed_ / 100.0;
        roi_top_ratio_ = track_roi_top_ratio_ / 100.0;
        low_h1_ = track_low_h1_;
        high_h1_ = track_high_h1_;
        low_h2_ = track_low_h2_;
        high_h2_ = track_high_h2_;
        low_s_ = track_low_s_;
        high_s_ = track_high_s_;
        low_v_ = track_low_v_;
        high_v_ = track_high_v_;
        min_area_ = track_min_area_;
        min_aspect_ = track_min_aspect_;
    }

    void create_control_window() {
        cv::namedWindow("Controls", cv::WINDOW_NORMAL);
        cv::resizeWindow("Controls", 400, 700);
        cv::moveWindow("Controls", 0, 0);

        // HSV параметры
        cv::createTrackbar("Hue Low 1", "Controls", &track_low_h1_, 179);
        cv::createTrackbar("Hue High 1", "Controls", &track_high_h1_, 179);
        cv::createTrackbar("Hue Low 2", "Controls", &track_low_h2_, 179);
        cv::createTrackbar("Hue High 2", "Controls", &track_high_h2_, 179);
        cv::createTrackbar("Sat Low", "Controls", &track_low_s_, 255);
        cv::createTrackbar("Sat High", "Controls", &track_high_s_, 255);
        cv::createTrackbar("Val Low", "Controls", &track_low_v_, 255);
        cv::createTrackbar("Val High", "Controls", &track_high_v_, 255);

        // Фильтры
        cv::createTrackbar("Min Area", "Controls", &track_min_area_, 5000);
        cv::createTrackbar("Min Aspect*10", "Controls", &track_min_aspect_, 100);

        // PID
        cv::createTrackbar("Kp*100", "Controls", &track_kp_, 200);
        cv::createTrackbar("Ki*100", "Controls", &track_ki_, 100);
        cv::createTrackbar("Kd*100", "Controls", &track_kd_, 200);

        // Скорость и ROI
        cv::createTrackbar("Cruise Speed*100", "Controls", &track_cruise_speed_, 100);
        cv::createTrackbar("ROI Top %", "Controls", &track_roi_top_ratio_, 90);

        RCLCPP_INFO(this->get_logger(), "Control window created with all sliders");
    }

    cv::Mat get_roi(const cv::Mat& frame) {
        int roi_top = static_cast<int>(frame.rows * roi_top_ratio_);
        return frame(cv::Rect(0, roi_top, frame.cols, frame.rows - roi_top));
    }

    void publish_image(const cv::Mat& img, const std::string& encoding,
                       rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub) {
        if (!pub || img.empty()) return;
        try {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, img).toImageMsg();
            msg->header.stamp = this->now();
            msg->header.frame_id = "camera";
            pub->publish(*msg);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "cv_bridge exception: %s", e.what());
        }
    }

    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        frame_counter_++;

        // ===== ОБНОВЛЯЕМ ПАРАМЕТРЫ ИЗ ТРЕКБАРОВ =====
        update_from_trackbars();

        cv::Mat full_frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (full_frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Empty frame");
            return;
        }

        cv::Mat frame_roi = get_roi(full_frame);

        cv::Mat hsv;
        cv::cvtColor(frame_roi, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask1, mask2, mask_roi;
        cv::inRange(hsv, cv::Scalar(low_h1_, low_s_, low_v_),
                    cv::Scalar(high_h1_, high_s_, high_v_), mask1);
        cv::inRange(hsv, cv::Scalar(low_h2_, low_s_, low_v_),
                    cv::Scalar(high_h2_, high_s_, high_v_), mask2);
        mask_roi = mask1 | mask2;

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask_roi, mask_roi, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask_roi, mask_roi, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask_roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Rect> vertical_rects;
        double min_aspect_ratio = min_aspect_ / 10.0;

        for (const auto& cnt : contours) {
            double area = cv::contourArea(cnt);
            if (area < min_area_) continue;
            cv::Rect rect = cv::boundingRect(cnt);
            double aspect = static_cast<double>(rect.height) / std::max(1, rect.width);
            if (aspect < min_aspect_ratio) continue;
            vertical_rects.push_back(rect);
        }

        std::sort(vertical_rects.begin(), vertical_rects.end(),
                  [](const cv::Rect& a, const cv::Rect& b) { return a.height > b.height; });

        bool both_found = (vertical_rects.size() >= 2);
        cv::Rect ref_line, guide_line;
        double error = 0.0, angular = 0.0;
        std::string status = "NO LINES";

        if (both_found) {
            guide_line = vertical_rects[1];
            ref_line = vertical_rects[0];

            int ref_cx = ref_line.x + ref_line.width / 2;
            int guide_cx = guide_line.x + guide_line.width / 2;
            int frame_center_x = frame_roi.cols / 2;

            error = static_cast<double>(ref_cx - guide_cx) / frame_center_x;
            error = std::clamp(error, -1.0, 1.0);

            integral_ += error;
            integral_ = std::clamp(integral_, -1.0, 1.0);
            double derivative = error - prev_error_;
            angular = kp_ * error + ki_ * integral_ + kd_ * derivative;
            angular = std::clamp(angular, -1.0, 1.0);
            prev_error_ = error;

            const double threshold = 0.02;
            if (error > threshold) status = "RIGHT";
            else if (error < -threshold) status = "LEFT";
            else status = "CENTER";

            last_valid_time_ = this->now();

            if (frame_counter_ % 10 == 0) {
                RCLCPP_INFO(this->get_logger(),
                            "%s | Err: %+.3f | Ang: %+.3f | Lines: %zu",
                            status.c_str(), error, angular, vertical_rects.size());
            }
        } else {
            auto elapsed = (this->now() - last_valid_time_).seconds() * 1000.0;
            if (elapsed > lost_timeout_ms_) {
                angular = 0.0;
                status = "STOPPED";
            } else {
                angular = 0.0;
                status = "SEARCHING";
            }
            integral_ = 0.0;
            prev_error_ = 0.0;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = both_found ? cruise_speed_ : 0.0;
        cmd.angular.z = angular;
        cmd_pub_->publish(cmd);

        if (debug_) {
            // ПОЛНОЕ изображение
            publish_image(full_frame, "bgr8", original_image_pub_);

            // Маска ROI
            publish_image(mask_roi, "mono8", mask_image_pub_);

            // Debug ROI
            cv::Mat debug_frame;
            frame_roi.copyTo(debug_frame);

            if (both_found) {
                cv::rectangle(debug_frame, ref_line, cv::Scalar(0, 255, 0), 3);
                cv::rectangle(debug_frame, guide_line, cv::Scalar(255, 0, 0), 3);
                cv::circle(debug_frame,
                           cv::Point(ref_line.x + ref_line.width/2, ref_line.y + ref_line.height/2),
                           8, cv::Scalar(0, 255, 0), -1);
                cv::circle(debug_frame,
                           cv::Point(guide_line.x + guide_line.width/2, guide_line.y + guide_line.height/2),
                           8, cv::Scalar(255, 0, 0), -1);
                cv::line(debug_frame,
                         cv::Point(ref_line.x + ref_line.width/2, ref_line.y + ref_line.height/2),
                         cv::Point(guide_line.x + guide_line.width/2, guide_line.y + guide_line.height/2),
                         cv::Scalar(0, 255, 255), 2);
            }

            cv::line(debug_frame,
                     cv::Point(frame_roi.cols/2, 0),
                     cv::Point(frame_roi.cols/2, frame_roi.rows),
                     cv::Scalar(255, 255, 0), 2);

            cv::putText(debug_frame, status, cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
            cv::putText(debug_frame, "Err: " + std::to_string(error),
                        cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(255, 255, 255), 2);
            cv::putText(debug_frame, "Lines: " + std::to_string(vertical_rects.size()),
                        cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(255, 255, 255), 2);
            cv::putText(debug_frame, "Ang: " + std::to_string(angular),
                        cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(255, 255, 255), 2);

            publish_image(debug_frame, "bgr8", debug_image_pub_);
        }

        // Небольшая задержка для обновления окна с ползунками
        cv::waitKey(1);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
