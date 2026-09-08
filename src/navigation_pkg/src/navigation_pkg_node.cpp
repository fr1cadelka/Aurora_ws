#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode() : Node("navigation_pkg_node")
    {
        RCLCPP_INFO(this->get_logger(), "Navigation Package Node started");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
