#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <vector>
#include <algorithm>

class LineFollower : public rclcpp::Node {
public:
    LineFollower() : Node("line_follower") {
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_image/compressed",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(&LineFollower::image_callback, this, std::placeholders::_1));

        // Публикуем в отдельный топик для UnifiedController
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/line_follower/cmd_vel", 10);

        this->declare_parameter<double>("kp", 0.5);
        this->declare_parameter<double>("ki", 0.0);
        this->declare_parameter<double>("kd", 0.1);
        this->declare_parameter<double>("cruise_speed", 0.5);
        this->declare_parameter<bool>("debug", true);

        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();
        cruise_speed_ = this->get_parameter("cruise_speed").as_double();
        debug_ = this->get_parameter("debug").as_bool();

        prev_error_ = 0.0;
        integral_ = 0.0;

        if (debug_) {
            cv::namedWindow("Original", cv::WINDOW_NORMAL);
            cv::namedWindow("Mask", cv::WINDOW_NORMAL);
            cv::namedWindow("Debug", cv::WINDOW_NORMAL);

            cv::createTrackbar("Low H1", "Debug", &low_h1_, 179, nullptr);
            cv::createTrackbar("High H1", "Debug", &high_h1_, 179, nullptr);
            cv::createTrackbar("Low H2", "Debug", &low_h2_, 179, nullptr);
            cv::createTrackbar("High H2", "Debug", &high_h2_, 179, nullptr);
            cv::createTrackbar("Low S", "Debug", &low_s_, 255, nullptr);
            cv::createTrackbar("High S", "Debug", &high_s_, 255, nullptr);
            cv::createTrackbar("Low V", "Debug", &low_v_, 255, nullptr);
            cv::createTrackbar("High V", "Debug", &high_v_, 255, nullptr);
            cv::createTrackbar("Min Area", "Debug", &min_area_, 5000, nullptr);
            cv::createTrackbar("Min Aspect (H/W)", "Debug", &min_aspect_, 100, nullptr);
            cv::createTrackbar("Kp*100", "Debug", &kp_track_, 200, nullptr);
            cv::createTrackbar("Ki*100", "Debug", &ki_track_, 100, nullptr);
            cv::createTrackbar("Kd*100", "Debug", &kd_track_, 200, nullptr);
        }

        RCLCPP_INFO(this->get_logger(), "LineFollower (RED LINES) started. Kp=%.2f, Ki=%.2f, Kd=%.2f, Speed=%.2f",
                    kp_, ki_, kd_, cruise_speed_);
        RCLCPP_INFO(this->get_logger(), "Publishing to /line_follower/cmd_vel");
    }

private:
    int low_h1_ = 0, high_h1_ = 10, low_h2_ = 160, high_h2_ = 179;
    int low_s_ = 100, high_s_ = 255, low_v_ = 100, high_v_ = 255;
    int min_area_ = 500;
    int min_aspect_ = 30;
    int kp_track_ = 50, ki_track_ = 0, kd_track_ = 10;

    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Empty frame");
            return;
        }

        if (debug_) {
            kp_ = kp_track_ / 100.0;
            ki_ = ki_track_ / 100.0;
            kd_ = kd_track_ / 100.0;
        }

        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Mat mask1, mask2, mask;
        cv::inRange(hsv, cv::Scalar(low_h1_, low_s_, low_v_), cv::Scalar(high_h1_, high_s_, high_v_), mask1);
        cv::inRange(hsv, cv::Scalar(low_h2_, low_s_, low_v_), cv::Scalar(high_h2_, high_s_, high_v_), mask2);
        mask = mask1 | mask2;

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

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
        if (both_found) {
            guide_line = vertical_rects[0];
            ref_line = vertical_rects[1];
        }

        double angular = 0.0;
        std::string status = "NO LINES";
        double error = 0.0;

        if (both_found) {
            int ref_cx = ref_line.x + ref_line.width / 2;
            int guide_cx = guide_line.x + guide_line.width / 2;
            int frame_center_x = frame.cols / 2;

            error = static_cast<double>(ref_cx - guide_cx) / frame_center_x;
            error = std::clamp(error, -1.0, 1.0);

            integral_ += error;
            integral_ = std::clamp(integral_, -1.0, 1.0);
            double derivative = error - prev_error_;
            angular = kp_ * error + ki_ * integral_ + kd_ * derivative;
            angular = std::clamp(angular, -1.0, 1.0);
            prev_error_ = error;

            const double threshold = 0.02;
            if (error > threshold) status = "ПРАВЕЕ (эталон правее)";
            else if (error < -threshold) status = "ЛЕВЕЕ (эталон левее)";
            else status = "РОВНО";

            RCLCPP_INFO(this->get_logger(),
                        "%s | Error: %+.3f | Angular: %+.3f | Ref X: %d | Guide X: %d",
                        status.c_str(), error, angular, ref_cx, guide_cx);
        } else {
            integral_ = 0.0;
            prev_error_ = 0.0;
            angular = 0.0;
            status = "LOST (меньше 2 линий)";
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Only %zu line(s) found. Need 2.", vertical_rects.size());
        }

        // Публикуем Twist (linear.x = cruise_speed если обе линии есть, иначе 0)
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = both_found ? cruise_speed_ : 0.0;
        cmd.angular.z = angular;
        cmd_pub_->publish(cmd);

        // Визуализация (без изменений)
        if (debug_) {
            cv::imshow("Original", frame);
            cv::imshow("Mask", mask);

            cv::Mat debug_frame;
            frame.copyTo(debug_frame);
            if (both_found) {
                cv::rectangle(debug_frame, ref_line, cv::Scalar(0, 255, 0), 2);
                cv::rectangle(debug_frame, guide_line, cv::Scalar(255, 0, 0), 2);
                cv::circle(debug_frame, cv::Point(ref_line.x + ref_line.width/2, ref_line.y + ref_line.height/2), 6, cv::Scalar(0,255,0), -1);
                cv::circle(debug_frame, cv::Point(guide_line.x + guide_line.width/2, guide_line.y + guide_line.height/2), 6, cv::Scalar(255,0,0), -1);
            }
            cv::line(debug_frame, cv::Point(frame.cols/2, 0), cv::Point(frame.cols/2, frame.rows), cv::Scalar(255,255,0), 2);
            cv::putText(debug_frame, status, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255), 2);
            cv::putText(debug_frame, "Error: " + std::to_string(error), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255,255,255), 2);
            cv::putText(debug_frame, "Lines: " + std::to_string(vertical_rects.size()), cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255,255,255), 2);
            cv::imshow("Debug", debug_frame);
            cv::waitKey(1);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    double kp_, ki_, kd_;
    double cruise_speed_;
    double prev_error_, integral_;
    bool debug_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}