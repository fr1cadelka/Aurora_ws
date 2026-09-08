#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <queue>
#include <cmath>
#include <algorithm>
#include <cstdint>

class CostmapNode : public rclcpp::Node
{
public:
    CostmapNode()
        : Node("costmap_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
    {
        // Параметры
        this->declare_parameter<std::string>("pointcloud_topic", "/velodyne_points");
        this->declare_parameter<std::string>("output_topic", "/costmap");
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<double>("resolution", 0.05);
        this->declare_parameter<double>("map_width", 20.0);
        this->declare_parameter<double>("map_height", 20.0);
        this->declare_parameter<double>("min_z", 0.15);
        this->declare_parameter<double>("max_z", 2.0);
        this->declare_parameter<double>("min_range", 0.3);
        this->declare_parameter<double>("max_range", 10.0);
        this->declare_parameter<double>("inflation_radius", 0.2);

        pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        resolution_ = this->get_parameter("resolution").as_double();
        map_width_m_ = this->get_parameter("map_width").as_double();
        map_height_m_ = this->get_parameter("map_height").as_double();
        min_z_ = this->get_parameter("min_z").as_double();
        max_z_ = this->get_parameter("max_z").as_double();
        min_range_ = this->get_parameter("min_range").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        inflation_radius_ = this->get_parameter("inflation_radius").as_double();

        // Размер карты в ячейках
        map_width_ = static_cast<int>(map_width_m_ / resolution_);
        map_height_ = static_cast<int>(map_height_m_ / resolution_);
        origin_x_ = -map_width_m_ / 2.0;
        origin_y_ = -map_height_m_ / 2.0;
        inflation_cells_ = static_cast<int>(inflation_radius_ / resolution_);

        // Подписчик
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&CostmapNode::pointCloudCallback, this, std::placeholders::_1));

        // Публишер
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Costmap node started");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 cloud_transformed;

        // 1. Трансформация облака в целевую систему координат
        try
        {
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
            tf2::doTransform(*msg, cloud_transformed, transform);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF failed: %s", ex.what());
            return;
        }

        // 2. Создание карты
        nav_msgs::msg::OccupancyGrid costmap;
        costmap.header.stamp = msg->header.stamp;
        costmap.header.frame_id = target_frame_;
        costmap.info.resolution = static_cast<float>(resolution_);
        costmap.info.width = static_cast<uint32_t>(map_width_);
        costmap.info.height = static_cast<uint32_t>(map_height_);
        costmap.info.origin.position.x = origin_x_;
        costmap.info.origin.position.y = origin_y_;
        costmap.info.origin.position.z = 0.0;
        costmap.info.origin.orientation.w = 1.0;

        // -1 = unknown, 0 = free, 100 = occupied
        //  ВАЖНО: сначала заполняем НЕИЗВЕСТНОСТЬЮ (-1), потом помечаем свободное (0)
        costmap.data.assign(static_cast<size_t>(map_width_) * map_height_, -1);

        // 3. Чтение облака
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_transformed, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_transformed, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_transformed, "z");

        int valid_points = 0;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            const float x = *iter_x;
            const float y = *iter_y;
            const float z = *iter_z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;

            if (z < min_z_ || z > max_z_)
                continue;

            const double range = std::sqrt(x * x + y * y);
            if (range < min_range_ || range > max_range_)
                continue;

            const int map_x = static_cast<int>(std::floor((x - origin_x_) / resolution_));
            const int map_y = static_cast<int>(std::floor((y - origin_y_) / resolution_));

            if (map_x < 0 || map_x >= map_width_ || map_y < 0 || map_y >= map_height_)
                continue;

            const size_t index = static_cast<size_t>(map_y) * map_width_ + map_x;
            costmap.data[index] = 100; // Занято
            valid_points++;
        }

        // 4. Инфляция (оптимизированная через BFS)
        applyInflationBFS(costmap);

        // 5. Публикация
        publisher_->publish(costmap);
    }

    // Используем BFS для быстрого распространения инфляции
    void applyInflationBFS(nav_msgs::msg::OccupancyGrid &costmap)
    {
        if (inflation_cells_ <= 0)
            return;

        std::queue<std::pair<int, int>> q;
        std::vector<std::vector<int>> dist(map_height_, std::vector<int>(map_width_, -1));

        // Инициализация: добавляем все занятые клетки
        for (int y = 0; y < map_height_; ++y)
        {
            for (int x = 0; x < map_width_; ++x)
            {
                size_t idx = static_cast<size_t>(y) * map_width_ + x;
                if (costmap.data[idx] == 100)
                {
                    dist[y][x] = 0;
                    q.push({x, y});
                }
            }
        }

        // Направления (8-связность)
        const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        while (!q.empty())
        {
            auto [cx, cy] = q.front();
            q.pop();

            int current_dist = dist[cy][cx];
            if (current_dist >= inflation_cells_)
                continue;

            for (int i = 0; i < 8; ++i)
            {
                int nx = cx + dx[i];
                int ny = cy + dy[i];

                if (nx < 0 || nx >= map_width_ || ny < 0 || ny >= map_height_)
                    continue;

                if (dist[ny][nx] != -1)
                    continue;

                // Вычисляем стоимость
                int new_dist = current_dist + 1;
                dist[ny][nx] = new_dist;

                // Стоимость инфляции уменьшается с расстоянием
                // int cost = 100 - (100 * new_dist) / inflation_cells_;
                // cost = std::clamp(cost, 0, 100);

                double normalized = static_cast<double>(new_dist) / static_cast<double>(inflation_cells_);
                int cost = static_cast<int>(100.0 * std::exp(-3.0 * normalized)); // Экспоненциальное затухание
                cost = std::clamp(cost, 0, 100);

                // И ВАЖНО: Отсекайте слишком далекие клетки
                if (cost < 10) {
                    cost = 0; // Не раздуваем то, что далеко от препятствия
                }

                size_t nidx = static_cast<size_t>(ny) * map_width_ + nx;
                // Присваиваем, только если текущее значение меньше нового
                if (costmap.data[nidx] < cost)
                    costmap.data[nidx] = cost;

                q.push({nx, ny});
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string pointcloud_topic_;
    std::string output_topic_;
    std::string target_frame_;
    double resolution_;
    double map_width_m_;
    double map_height_m_;
    double min_z_;
    double max_z_;
    double min_range_;
    double max_range_;
    double inflation_radius_;
    int map_width_;
    int map_height_;
    double origin_x_;
    double origin_y_;
    int inflation_cells_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}
