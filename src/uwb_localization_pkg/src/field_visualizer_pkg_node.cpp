#include <memory>
#include <vector>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class FieldVisualizer : public rclcpp::Node
{
public:
    FieldVisualizer() : Node("field_visualizer")
    {
        // === Координаты маяков ===
        this->declare_parameter<std::vector<double>>("anchors_x", {0.0, 25.0, 25.0, 0.0});
        this->declare_parameter<std::vector<double>>("anchors_y", {0.0, 0.0, 25.0, 25.0});
        this->declare_parameter<std::vector<double>>("anchors_z", {1.5, 1.5, 1.5, 1.5});

        // === Поле ===
        this->declare_parameter<double>("field_width",  25.0);
        this->declare_parameter<double>("field_height", 25.0);
        this->declare_parameter<double>("lane_width",   1.0);

        // === Визуализация ===
        this->declare_parameter<double>("anchor_radius", 0.3);
        this->declare_parameter<std::string>("frame_id", "map");

        this->get_parameter("anchors_x", anchors_x_);
        this->get_parameter("anchors_y", anchors_y_);
        this->get_parameter("anchors_z", anchors_z_);
        this->get_parameter("field_width",  field_width_);
        this->get_parameter("field_height", field_height_);
        this->get_parameter("lane_width",   lane_width_);
        this->get_parameter("anchor_radius", anchor_radius_);
        this->get_parameter("frame_id", frame_id_);

        num_anchors_ = anchors_x_.size();

        RCLCPP_INFO(this->get_logger(), "=== Field Visualizer ===");
        RCLCPP_INFO(this->get_logger(), "Field: %.2f x %.2f m, lane width: %.2f",
                    field_width_, field_height_, lane_width_);
        for (size_t i = 0; i < num_anchors_; ++i) {
            RCLCPP_INFO(this->get_logger(), "  A%zu: (%.2f, %.2f, %.2f)",
                        i, anchors_x_[i], anchors_y_[i], anchors_z_[i]);
        }

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/uwb/markers", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publish_tfs();

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&FieldVisualizer::publish_markers, this));
    }

private:
    void publish_tfs()
    {
        std::vector<geometry_msgs::msg::TransformStamped> tfs;

        for (size_t i = 0; i < num_anchors_; ++i) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->now();
            t.header.frame_id = frame_id_;
            t.child_frame_id = "anchor_" + std::to_string(i);
            t.transform.translation.x = anchors_x_[i];
            t.transform.translation.y = anchors_y_[i];
            t.transform.translation.z = anchors_z_[i];
            t.transform.rotation.w = 1.0;
            tfs.push_back(t);
        }

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = frame_id_;
        t.child_frame_id = "field";
        t.transform.rotation.w = 1.0;
        tfs.push_back(t);

        tf_broadcaster_->sendTransform(tfs);
    }

    void publish_markers()
    {
        visualization_msgs::msg::MarkerArray arr;

        // Маяки
        for (size_t i = 0; i < num_anchors_; ++i) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = this->now();
            m.ns = "anchors";
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = anchors_x_[i];
            m.pose.position.y = anchors_y_[i];
            m.pose.position.z = anchors_z_[i];
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = anchor_radius_ * 2.0;
            m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 1.0;
            m.lifetime = rclcpp::Duration::from_seconds(2.0);
            arr.markers.push_back(m);

            // Подпись
            visualization_msgs::msg::Marker text;
            text.header.frame_id = frame_id_;
            text.header.stamp = this->now();
            text.ns = "anchor_labels";
            text.id = static_cast<int>(i);
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.position.x = anchors_x_[i];
            text.pose.position.y = anchors_y_[i];
            text.pose.position.z = anchors_z_[i] + 0.5;
            text.pose.orientation.w = 1.0;
            text.scale.z = 0.5;
            text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0; text.color.a = 1.0;
            text.text = "A" + std::to_string(i);
            text.lifetime = rclcpp::Duration::from_seconds(2.0);
            arr.markers.push_back(text);
        }

        // Границы поля
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = frame_id_;
            line.header.stamp = this->now();
            line.ns = "field_border";
            line.id = 100;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.scale.x = 0.05;
            line.color.r = 0.0; line.color.g = 1.0; line.color.b = 0.0; line.color.a = 1.0;

            geometry_msgs::msg::Point p; p.z = 0.05;
            p.x = 0.0;          p.y = 0.0;          line.points.push_back(p);
            p.x = field_width_; p.y = 0.0;          line.points.push_back(p);
            p.x = field_width_; p.y = field_height_; line.points.push_back(p);
            p.x = 0.0;          p.y = field_height_; line.points.push_back(p);
            p.x = 0.0;          p.y = 0.0;          line.points.push_back(p);

            line.lifetime = rclcpp::Duration::from_seconds(2.0);
            arr.markers.push_back(line);
        }

        // Дорожки
        int lane_id = 200;
        for (double x = lane_width_; x < field_width_; x += lane_width_) {
            visualization_msgs::msg::Marker lane;
            lane.header.frame_id = frame_id_;
            lane.header.stamp = this->now();
            lane.ns = "lanes";
            lane.id = lane_id++;
            lane.type = visualization_msgs::msg::Marker::LINE_STRIP;
            lane.action = visualization_msgs::msg::Marker::ADD;
            lane.scale.x = 0.02;
            lane.color.r = 0.5; lane.color.g = 0.5; lane.color.b = 1.0; lane.color.a = 0.5;

            geometry_msgs::msg::Point p; p.z = 0.02;
            p.x = x; p.y = 0.0;           lane.points.push_back(p);
            p.x = x; p.y = field_height_; lane.points.push_back(p);
            lane.lifetime = rclcpp::Duration::from_seconds(2.0);
            arr.markers.push_back(lane);
        }

        marker_pub_->publish(arr);
    }

    std::vector<double> anchors_x_, anchors_y_, anchors_z_;
    size_t num_anchors_ = 0;
    double field_width_, field_height_, lane_width_, anchor_radius_;
    std::string frame_id_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FieldVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
