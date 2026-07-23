#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <string>

class UdpServerNode : public rclcpp::Node {
public:
    UdpServerNode() : Node("udp_server_node"), pipeline_(nullptr), appsrc_(nullptr), frame_count_(0), last_log_time_(this->now())
    {
        RCLCPP_INFO(this->get_logger(), "=== UDP RTP SERVER (H.264) ===");

        std::string client_ip = this->declare_parameter("pc_ip", "127.0.0.1");
        int udp_port = this->declare_parameter("udp_port", 12346);

        gst_init(nullptr, nullptr);

        // Пайплайн: appsrc → h264parse → rtph264pay → udpsink
        std::string pipeline_str =
            "appsrc name=src format=time is-live=true block=false ! "
            "h264parse config-interval=-1 ! "
            "rtph264pay config-interval=-1 aggregate-mode=zero-latency mtu=1400 ! "
            "udpsink host=" + client_ip + " port=" + std::to_string(udp_port) + " sync=false";

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (!pipeline_) {
            RCLCPP_ERROR(this->get_logger(), "Pipeline creation failed: %s", error ? error->message : "unknown");
            if (error) g_error_free(error);
            throw std::runtime_error("GStreamer init failed");
        }

        GstElement* src_elem = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
        appsrc_ = GST_APP_SRC(src_elem);
        gst_object_unref(src_elem);
        gst_app_src_set_size(appsrc_, -1);
        gst_app_src_set_stream_type(appsrc_, GST_APP_STREAM_TYPE_STREAM);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_image",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            std::bind(&UdpServerNode::image_callback, this, std::placeholders::_1)
            );

        RCLCPP_INFO(this->get_logger(), "RTP stream to %s:%d", client_ip.c_str(), udp_port);
    }

    ~UdpServerNode() {
        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
        RCLCPP_INFO(this->get_logger(), "UDP Server shutdown");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    GstElement* pipeline_;
    GstAppSrc* appsrc_;
    unsigned int frame_count_;
    rclcpp::Time last_log_time_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (msg->data.empty()) return;

        GstBuffer* buffer = gst_buffer_new_and_alloc(msg->data.size());
        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_WRITE);
        memcpy(map.data, msg->data.data(), msg->data.size());
        gst_buffer_unmap(buffer, &map);

        GST_BUFFER_PTS(buffer) = GST_CLOCK_TIME_NONE;
        GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;

        GstFlowReturn ret = gst_app_src_push_buffer(appsrc_, buffer);
        if (ret != GST_FLOW_OK) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1.0, "Appsrc push failed");
        }

        frame_count_++;
        auto now = this->now();
        if ((now - last_log_time_).seconds() >= 1.0) {
            RCLCPP_INFO(this->get_logger(), "[Server] FPS: %d, Size: %.2f KB", frame_count_, msg->data.size() / 1024.0);
            frame_count_ = 0;
            last_log_time_ = now;
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UdpServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}