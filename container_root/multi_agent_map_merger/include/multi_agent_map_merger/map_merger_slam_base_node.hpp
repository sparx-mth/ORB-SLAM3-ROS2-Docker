#pragma once

#include "rclcpp/rclcpp.hpp"
#include "slam_msgs/srv/get_map.hpp"
#include "slam_msgs/msg/map_data.hpp"

#include <unordered_map>
#include <string>
#include <vector>

class MapMergerSlamBaseNode : public rclcpp::Node
{
public:
    MapMergerSlamBaseNode(const std::vector<std::string>& agent_names);

private:
    void tryMergeMaps();
    void mergeMaps(std::shared_ptr<ORB_SLAM3::Map> mapA,
                   std::shared_ptr<ORB_SLAM3::Map> mapB,
                   const Sophus::Sim3f& sim3_AB);

    bool detectOverlap(const slam_msgs::msg::MapData& mapA,
                       const slam_msgs::msg::MapData& mapB);

    std::vector<std::string> agent_names_;
    std::unordered_map<std::string, rclcpp::Client<slam_msgs::srv::GetMap>::SharedPtr> map_clients_;
    std::unordered_map<std::string, slam_msgs::msg::MapData> maps_by_agent_;

    rclcpp::TimerBase::SharedPtr merge_timer_;
};
