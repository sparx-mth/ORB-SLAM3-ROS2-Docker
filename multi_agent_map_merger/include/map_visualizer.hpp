#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <unordered_map>
#include <string>
#include <vector>
#include <set>

#include "slam_msgs/msg/key_frame_full_data.hpp"
#include "ORB_SLAM3/KeyFrame.h"

namespace ORB_SLAM3_Wrapper {

class MapVisualizer
{
public:
    MapVisualizer(rclcpp::Node::SharedPtr node,
                  const std::unordered_map<std::string, std::array<float, 3>>& agent_colors,
                  const std::array<float, 3>& refined_color);

    void visualize(const std::unordered_map<std::string, std::vector<slam_msgs::msg::KeyFrameFullData>>& keyframes_by_agent,
                   const std::set<geometry_msgs::msg::Point>& refined_points);

private:
    visualization_msgs::msg::Marker keyframeToMarker(const slam_msgs::msg::KeyFrameFullData& kf_msg, const std::array<float, 3>& color, int id);
    visualization_msgs::msg::Marker pointToMarker(const geometry_msgs::msg::Point& pt, const std::array<float, 3>& color, int id);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::unordered_map<std::string, std::array<float, 3>> agent_colors_;
    std::array<float, 3> refined_color_;
    rclcpp::Node::SharedPtr node_;
};

} // namespace ORB_SLAM3_Wrapper
