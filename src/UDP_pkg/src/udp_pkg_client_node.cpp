#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <sstream>
#include <iomanip>

class UdpClientNode : public rclcpp::Node
{
public:
    UdpClientNode() : Node("udp_client_node"), pipeline_(nullptr), appsink_(nullptr),
        frame_count_(0), last_log_time_(this->now()),
        current_pitch_(0), current_yaw_(0), current_mode_(-1), current_zoom_(0), has_mode_zoom_(false)
    {
        RCLCPP_INFO(this->get_logger(), "=== UDP RTP CLIENT (with jitterbuffer) ===");

        int udp_port = this->declare_parameter("udp_port", 12346);

        cv::namedWindow("UDP Video", cv::WINDOW_NORMAL);
        cv::resizeWindow("UDP Video", 640, 480);
        cv::startWindowThread();

        gst_init(nullptr, nullptr);

        // Пайплайн: udpsrc → rtpjitterbuffer → rtph264depay → h264parse → avdec_h264 → videoconvert → appsink
        std::string pipeline_str =
            "udpsrc port=" + std::to_string(udp_port) + " caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
                                                        "rtpjitterbuffer latency=50 drop-on-latency=true ! "
                                                        "rtph264depay ! "
                                                        "h264parse config-interval=-1 ! "
                                                        "avdec_h264 ! videoconvert ! "
                                                        "video/x-raw,format=BGR ! "
                                                        "appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false";

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (!pipeline_) {
            RCLCPP_ERROR(this->get_logger(), "Pipeline creation failed: %s", error ? error->message : "unknown");
            if (error) g_error_free(error);
            throw std::runtime_error("GStreamer init failed");
        }

        GstElement* sink_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        if (!sink_elem) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get sink element");
            throw std::runtime_error("Missing sink");
        }
        appsink_ = GST_APP_SINK(sink_elem);
        gst_object_unref(sink_elem);

        gst_app_sink_set_emit_signals(appsink_, true);
        gst_app_sink_set_max_buffers(appsink_, 1);
        gst_app_sink_set_drop(appsink_, true);

        g_signal_connect(appsink_, "new-sample", G_CALLBACK(on_new_sample_static), this);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        state_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/tcp_received/angles", 10,
            std::bind(&UdpClientNode::state_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Client ready, listening on port %d", udp_port);
    }

    ~UdpClientNode() {
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
        cv::destroyAllWindows();
        RCLCPP_INFO(this->get_logger(), "Client shutdown");
    }

private:
    GstElement* pipeline_;
    GstAppSink* appsink_;
    unsigned int frame_count_;
    rclcpp::Time last_log_time_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;
    float current_pitch_, current_yaw_;
    int current_mode_;
    float current_zoom_;
    bool has_mode_zoom_;

    void state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            current_pitch_ = msg->data[0];
            current_yaw_ = msg->data[1];
        }
        if (msg->data.size() >= 4) {
            current_mode_ = static_cast<int>(msg->data[2]);
            current_zoom_ = msg->data[3];
            has_mode_zoom_ = true;
        }
    }

    static GstFlowReturn on_new_sample_static(GstAppSink* sink, gpointer user_data) {
        auto* node = static_cast<UdpClientNode*>(user_data);
        return node->process_sample(sink);
    }

    GstFlowReturn process_sample(GstAppSink* sink) {
        GstSample* sample = gst_app_sink_pull_sample(sink);
        if (!sample) return GST_FLOW_ERROR;

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        if (!buffer) { gst_sample_unref(sample); return GST_FLOW_ERROR; }

        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        GstCaps* caps = gst_sample_get_caps(sample);
        GstStructure* structure = gst_caps_get_structure(caps, 0);
        int width, height;
        gst_structure_get_int(structure, "width", &width);
        gst_structure_get_int(structure, "height", &height);

        cv::Mat image(height, width, CV_8UC3, map.data);

        if (!image.empty()) {
            // Нанесение OSD (опционально, можно убрать для максимальной скорости)
            draw_osd(image);
        }

        cv::imshow("UDP Video", image);
        cv::waitKey(1);

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        frame_count_++;
        auto now = this->now();
        if ((now - last_log_time_).seconds() >= 1.0) {
            RCLCPP_INFO(this->get_logger(), "[Client] FPS: %d", frame_count_);
            frame_count_ = 0;
            last_log_time_ = now;
        }

        return GST_FLOW_OK;
    }

    void draw_osd(cv::Mat& image) {
        // Короткая отрисовка – можно закомментировать для уменьшения нагрузки
        std::string mode_str = has_mode_zoom_ ? (std::to_string(current_mode_)) : "N/A";
        std::ostringstream pitch_ss, yaw_ss;
        pitch_ss << std::fixed << std::setprecision(1) << current_pitch_ << "deg";
        yaw_ss << std::fixed << std::setprecision(1) << current_yaw_ << "deg";

        int start_x = 15, start_y = 45, line_h = 40;
        cv::rectangle(image, cv::Point(start_x-10, start_y-35), cv::Point(start_x+250, start_y+3*line_h+10),
                      cv::Scalar(0,0,0), cv::FILLED);
        cv::putText(image, "Pitch: " + pitch_ss.str(), cv::Point(start_x, start_y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,255), 2);
        cv::putText(image, "Yaw: " + yaw_ss.str(), cv::Point(start_x, start_y+line_h),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);
        cv::putText(image, "Mode: " + mode_str, cv::Point(start_x, start_y+2*line_h),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,200,0), 2);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<UdpClientNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("UdpClientNode"), "Fatal: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}