#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <algorithm>
#include <atomic>
#include <thread>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class AreaSelector : public rclcpp::Node
{
public:
    AreaSelector() : rclcpp::Node("area_selector")
    {
        frame_id_    = declare_parameter<std::string>("frame_id", "map");
        path_step_   = declare_parameter<double>("path_step", 1.0);
        turn_points_ = declare_parameter<int>("turn_points", 30);
        edge_offset_ = declare_parameter<double>("edge_offset", 0.5);

        rclcpp::QoS latched(1);
        latched.transient_local().reliable();

        area_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/area_selector/area", latched);
        poly_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/area_selector/polygon", latched);
        path_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/area_selector/path", latched);

        click_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&AreaSelector::onClick, this, std::placeholders::_1));

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&AreaSelector::onGoalPose, this, std::placeholders::_1));

        timer_ = create_wall_timer(1s, std::bind(&AreaSelector::publishAll, this));

        keyboard_thread_ = std::thread([this]() { keyboardLoop(); });

        publishAll();

        RCLCPP_INFO(get_logger(),
                    "1) 4 угла через Publish Point   "
                    "2) 2D Goal Pose = сторона + направление   "
                    "3) ESC = стереть только путь");
    }

    ~AreaSelector() override
    {
        stop_keyboard_ = true;
        if (keyboard_thread_.joinable()) keyboard_thread_.join();
    }

private:
    enum Stage { COLLECT_CORNERS, PICK_START, DONE };

    // =========================================================================
    void onClick(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (stage_ != COLLECT_CORNERS) return;

        clicked_.push_back(msg->point);
        RCLCPP_INFO(get_logger(), "Угол %zu: (%.2f, %.2f)",
                    clicked_.size(), msg->point.x, msg->point.y);

        if (clicked_.size() == 4) {
            stage_ = PICK_START;
            RCLCPP_INFO(get_logger(),
                        "Область задана. 2D Goal Pose — задай сторону и направление.");
        }
        publishAll();
    }

    // =========================================================================
    // Определяем сторону (по позиции) и направление (по ориентации)
    // =========================================================================
    void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (stage_ == COLLECT_CORNERS) {
            RCLCPP_WARN(get_logger(),
                        "Сначала кликни 4 угла области (Publish Point).");
            return;
        }

        auto corners = aabbCorners();
        const double xmin = corners[0].x, xmax = corners[1].x;
        const double ymin = corners[0].y, ymax = corners[2].y;

        const double gx = msg->pose.position.x;
        const double gy = msg->pose.position.y;

        const auto& q = msg->pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        // Главная ось по yaw: горизонтальная (вдоль X) или вертикальная (вдоль Y)
        const bool horiz = std::abs(std::cos(yaw)) >= std::abs(std::sin(yaw));

        if (horiz) {
            // дорожки вдоль X → сторона по gy: снизу или сверху
            start_horiz_    = true;
            start_from_low_ = (gy - ymin) <= (ymax - gy);
            // направление первого прохода по знаку cos(yaw):
            //   yaw ≈ 0   → первый проход вправо (+1)
            //   yaw ≈ π   → первый проход влево  (-1)
            start_sign_ = (std::cos(yaw) >= 0.0) ? +1 : -1;

            RCLCPP_INFO(get_logger(),
                        "Старт: %s, первый проход %s",
                        start_from_low_ ? "снизу" : "сверху",
                        start_sign_ > 0 ? "слева-направо" : "справа-налево");
        } else {
            // дорожки вдоль Y → сторона по gx: слева или справа
            start_horiz_    = false;
            start_from_low_ = (gx - xmin) <= (xmax - gx);   // "слева" = низкий X
            // направление первого прохода по знаку sin(yaw):
            //   yaw ≈ +π/2 → вверх (+1)
            //   yaw ≈ -π/2 → вниз  (-1)
            start_sign_ = (std::sin(yaw) >= 0.0) ? +1 : -1;

            RCLCPP_INFO(get_logger(),
                        "Старт: %s, первый проход %s",
                        start_from_low_ ? "слева" : "справа",
                        start_sign_ > 0 ? "снизу-вверх" : "сверху-вниз");
        }

        buildPath();
        stage_ = DONE;
        publishAll();

        RCLCPP_INFO(get_logger(),
                    "Путь построен: %zu точек. ESC — стереть и выбрать другую сторону.",
                    path_points_.size());
    }

    // =========================================================================
    std::vector<geometry_msgs::msg::Point> aabbCorners() const
    {
        std::vector<geometry_msgs::msg::Point> c(4);
        if (clicked_.empty()) return c;
        double xmin = clicked_[0].x, xmax = clicked_[0].x;
        double ymin = clicked_[0].y, ymax = clicked_[0].y;
        for (auto& p : clicked_) {
            xmin = std::min(xmin, p.x);
            xmax = std::max(xmax, p.x);
            ymin = std::min(ymin, p.y);
            ymax = std::max(ymax, p.y);
        }
        auto mk = [](double x, double y) {
            geometry_msgs::msg::Point p; p.x = x; p.y = y; p.z = 0.0; return p;
        };
        c[0] = mk(xmin, ymin);
        c[1] = mk(xmax, ymin);
        c[2] = mk(xmax, ymax);
        c[3] = mk(xmin, ymax);
        return c;
    }

    // =========================================================================
    void addPoint(double x, double y)
    {
        geometry_msgs::msg::Point p;
        p.x = x; p.y = y; p.z = 0.0;
        path_points_.push_back(p);
    }

    // =========================================================================
    void buildPath()
    {
        path_points_.clear();
        if (clicked_.size() < 3) return;

        auto corners = aabbCorners();
        const double xmin = corners[0].x, xmax = corners[1].x;
        const double ymin = corners[0].y, ymax = corners[2].y;

        const double eo = edge_offset_;
        const double xi0 = xmin + eo, xi1 = xmax - eo;
        const double yi0 = ymin + eo, yi1 = ymax - eo;

        if (xi1 <= xi0 || yi1 <= yi0) {
            RCLCPP_WARN(get_logger(),
                        "Область слишком мала для edge_offset=%.2f м", eo);
            return;
        }

        if (start_horiz_) {
            // ---------- дорожки вдоль X ----------
            std::vector<double> ys;
            if (start_from_low_) {
                for (double y = yi0; y <= yi1 + 1e-9; y += path_step_) ys.push_back(y);
            } else {
                for (double y = yi1; y >= yi0 - 1e-9; y -= path_step_) ys.push_back(y);
            }

            bool ltr = (start_sign_ > 0);

            for (size_t i = 0; i < ys.size(); ++i) {
                const double y = ys[i];
                const double xa = ltr ? xi0 : xi1;
                const double xb = ltr ? xi1 : xi0;
                addPoint(xa, y);
                addPoint(xb, y);

                if (i + 1 < ys.size()) {
                    const double y_next = ys[i + 1];
                    const double dy = y_next - y;
                    const double R_ut = std::abs(dy) / 2.0;
                    const double cx = xb;
                    const double cy = y + dy / 2.0;
                    const double out_sign = ltr ? 1.0 : -1.0;

                    double a0, a1;
                    if (dy > 0) {
                        a0 = -M_PI / 2.0;
                        a1 = (out_sign > 0) ? (M_PI / 2.0) : (-3.0 * M_PI / 2.0);
                    } else {
                        a0 = M_PI / 2.0;
                        a1 = (out_sign > 0) ? (-M_PI / 2.0) : (3.0 * M_PI / 2.0);
                    }

                    const int N = std::max(4, turn_points_);
                    for (int k = 1; k < N; ++k) {
                        const double t = static_cast<double>(k) / N;
                        const double a = a0 + t * (a1 - a0);
                        addPoint(cx + R_ut * std::cos(a), cy + R_ut * std::sin(a));
                    }
                }
                ltr = !ltr;
            }
        } else {
            // ---------- дорожки вдоль Y ----------
            std::vector<double> xs;
            if (start_from_low_) {
                for (double x = xi0; x <= xi1 + 1e-9; x += path_step_) xs.push_back(x);
            } else {
                for (double x = xi1; x >= xi0 - 1e-9; x -= path_step_) xs.push_back(x);
            }

            bool btu = (start_sign_ > 0);   // bottom-to-up для первого прохода

            for (size_t i = 0; i < xs.size(); ++i) {
                const double x = xs[i];
                const double ya = btu ? yi0 : yi1;
                const double yb = btu ? yi1 : yi0;
                addPoint(x, ya);
                addPoint(x, yb);

                if (i + 1 < xs.size()) {
                    const double x_next = xs[i + 1];
                    const double dx = x_next - x;
                    const double R_ut = std::abs(dx) / 2.0;
                    const double cx = x + dx / 2.0;
                    const double cy = yb;
                    const double out_sign = btu ? 1.0 : -1.0;   // наружу вверх/вниз

                    double a0, a1;
                    if (dx > 0) {
                        a0 = M_PI;
                        a1 = (out_sign > 0) ? 0.0 : (2.0 * M_PI);
                    } else {
                        a0 = 0.0;
                        a1 = (out_sign > 0) ? M_PI : (-M_PI);
                    }

                    const int N = std::max(4, turn_points_);
                    for (int k = 1; k < N; ++k) {
                        const double t = static_cast<double>(k) / N;
                        const double a = a0 + t * (a1 - a0);
                        addPoint(cx + R_ut * std::cos(a), cy + R_ut * std::sin(a));
                    }
                }
                btu = !btu;
            }
        }
    }

    // =========================================================================
    void publishAll()
    {
        publishArea();
        publishPath();
    }

    void publishArea()
    {
        const auto stamp = get_clock()->now();
        visualization_msgs::msg::MarkerArray arr;

        if (clicked_.empty()) {
            for (const char* ns : {"clicked", "clicked_label", "area_inset"}) {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = frame_id_;
                m.header.stamp = stamp;
                m.ns = ns;
                m.id = 0;
                m.action = visualization_msgs::msg::Marker::DELETEALL;
                arr.markers.push_back(m);
            }
            area_pub_->publish(arr);

            geometry_msgs::msg::PolygonStamped poly;
            poly.header.frame_id = frame_id_;
            poly.header.stamp = stamp;
            poly_pub_->publish(poly);
            return;
        }

        for (size_t i = 0; i < clicked_.size(); ++i) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = stamp;
            m.ns = "clicked";
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position = clicked_[i];
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = 0.4;
            m.color.r = 1.0f; m.color.g = 0.2f; m.color.b = 0.2f; m.color.a = 1.0f;
            arr.markers.push_back(m);

            visualization_msgs::msg::Marker t;
            t.header.frame_id = frame_id_;
            t.header.stamp = stamp;
            t.ns = "clicked_label";
            t.id = static_cast<int>(i);
            t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            t.action = visualization_msgs::msg::Marker::ADD;
            t.pose.position.x = clicked_[i].x;
            t.pose.position.y = clicked_[i].y;
            t.pose.position.z = clicked_[i].z + 0.8;
            t.pose.orientation.w = 1.0;
            t.scale.z = 0.5;
            t.color.r = 1.0f; t.color.g = 1.0f; t.color.b = 1.0f; t.color.a = 1.0f;
            t.text = std::to_string(i + 1);
            arr.markers.push_back(t);
        }

        if (stage_ >= PICK_START && clicked_.size() >= 3) {
            auto corners = aabbCorners();
            const double eo = edge_offset_;
            std::vector<geometry_msgs::msg::Point> inset = {
                mk(corners[0].x + eo, corners[0].y + eo),
                mk(corners[1].x - eo, corners[0].y + eo),
                mk(corners[1].x - eo, corners[2].y - eo),
                mk(corners[0].x + eo, corners[2].y - eo),
            };
            inset.push_back(inset.front());

            visualization_msgs::msg::Marker line;
            line.header.frame_id = frame_id_;
            line.header.stamp = stamp;
            line.ns = "area_inset";
            line.id = 0;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.scale.x = 0.1;
            line.color.r = 0.0f; line.color.g = 0.9f; line.color.b = 1.0f; line.color.a = 1.0f;
            line.pose.orientation.w = 1.0;
            line.points = inset;
            arr.markers.push_back(line);
        }

        area_pub_->publish(arr);

        if (clicked_.size() >= 3) {
            geometry_msgs::msg::PolygonStamped poly;
            poly.header.frame_id = frame_id_;
            poly.header.stamp = stamp;
            for (const auto & c : clicked_) {
                geometry_msgs::msg::Point32 p32;
                p32.x = static_cast<float>(c.x);
                p32.y = static_cast<float>(c.y);
                p32.z = 0.0f;
                poly.polygon.points.push_back(p32);
            }
            poly_pub_->publish(poly);
        }
    }

    void publishPath()
    {
        visualization_msgs::msg::MarkerArray arr;
        const auto stamp = get_clock()->now();

        visualization_msgs::msg::Marker line;
        line.header.frame_id = frame_id_;
        line.header.stamp = stamp;
        line.ns = "path";
        line.id = 0;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = path_points_.empty()
                          ? visualization_msgs::msg::Marker::DELETEALL
                          : visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.08;
        line.color.r = 1.0f; line.color.g = 0.6f; line.color.b = 0.0f; line.color.a = 1.0f;
        line.pose.orientation.w = 1.0;
        line.points = path_points_;

        arr.markers.push_back(line);
        path_pub_->publish(arr);
    }

    static geometry_msgs::msg::Point mk(double x, double y)
    {
        geometry_msgs::msg::Point p; p.x = x; p.y = y; p.z = 0.0; return p;
    }

    // =========================================================================
    // Клавиатура
    // =========================================================================
    void keyboardLoop()
    {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_cc[VMIN] = 0;
        newt.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (rclcpp::ok() && !stop_keyboard_) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000;

            int ret = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
            if (ret > 0 && FD_ISSET(STDIN_FILENO, &fds)) {
                char c = 0;
                if (read(STDIN_FILENO, &c, 1) == 1 && c == 27) {
                    clearPathOnly();
                }
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    void clearPathOnly()
    {
        path_points_.clear();
        start_horiz_ = true;
        start_from_low_ = true;
        start_sign_ = +1;

        if (clicked_.size() >= 4) stage_ = PICK_START;
        else stage_ = COLLECT_CORNERS;

        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id_;
        m.header.stamp = get_clock()->now();
        m.ns = "path";
        m.id = 0;
        m.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(m);
        path_pub_->publish(arr);

        RCLCPP_INFO(get_logger(),
                    "Путь стёрт. 2D Goal Pose — построй из другой стороны.");
    }

    // =========================================================================
    std::string frame_id_;
    double path_step_;
    double edge_offset_;
    int    turn_points_;

    // параметры траектории, определяются 2D Goal Pose
    bool start_horiz_{true};      // дорожки вдоль X?
    bool start_from_low_{true};   // старт снизу (для horiz) или слева (для vert)?
    int  start_sign_{+1};         // знак направления первого прохода

    Stage stage_{COLLECT_CORNERS};

    std::vector<geometry_msgs::msg::Point> clicked_;
    std::vector<geometry_msgs::msg::Point> path_points_;

    std::atomic<bool> stop_keyboard_{false};
    std::thread keyboard_thread_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr area_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr  poly_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AreaSelector>());
    rclcpp::shutdown();
    return 0;
}
