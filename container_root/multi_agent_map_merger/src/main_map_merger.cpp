#include "multi_agent_map_merger/map_merger_slam_base_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::vector<std::string> agent_names = {"robot_0", "robot_1"};
    auto node = std::make_shared<MapMergerSlamBaseNode>(agent_names);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
