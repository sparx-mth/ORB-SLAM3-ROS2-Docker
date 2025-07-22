#include "multi_agent_map_merger/map_visualizer.hpp"
#include "multi_agent_map_merger/map_merger_slam_base_node.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace ORB_SLAM3_Wrapper {

MapVisualizer::MapVisualizer(rclcpp::Node::SharedPtr node,
                             const std::unordered_map<std::string, std::array<float, 3>>& agent_colors,
                             const std::array<float, 3>& refined_color)
    : node_(node), agent_colors_(agent_colors), refined_color_(refined_color)
{
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/merged_map/visualization", 10);

    publish_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MapVisualizer::periodicPublish, this));
}

void MapVisualizer::visualize(const std::unordered_map<std::string, std::vector<slam_msgs::msg::KeyFrameFullData>>& keyframes_by_agent,
                               const std::set<geometry_msgs::msg::Point>& refined_points)
{
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;

    for (const auto& [agent_name, keyframes] : keyframes_by_agent)
    {
        auto color_it = agent_colors_.find(agent_name);
        if (color_it == agent_colors_.end()) continue;

        const auto& color = color_it->second;

        for (const auto& kf : keyframes)
        {
            marker_array.markers.push_back(keyframeToMarker(kf, color, marker_id++));

            for (const auto& pt : kf.word_pts)
            {
                geometry_msgs::msg::Point p;
                p.x = pt.x;
                p.y = pt.y;
                p.z = pt.z;
                marker_array.markers.push_back(pointToMarker(p, color, marker_id++));
            }
        }
    }

    for (const auto& p : refined_points)
    {
        marker_array.markers.push_back(pointToMarker(p, refined_color_, marker_id++));
    }

    marker_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker MapVisualizer::keyframeToMarker(const slam_msgs::msg::KeyFrameFullData& kf_msg,
                                                                 const std::array<float, 3>& color,
                                                                 int id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->now();
    marker.ns = "keyframes";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = kf_msg.pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;
    return marker;
}

visualization_msgs::msg::Marker MapVisualizer::pointToMarker(const geometry_msgs::msg::Point& pt,
                                                              const std::array<float, 3>& color,
                                                              int id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->now();
    marker.ns = "map_points";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = pt;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = 1.0;
    return marker;
}

void MapVisualizer::periodicPublish()
{
    visualize(latest_keyframes_by_agent_, latest_refined_points_);
}

void MapVisualizer::updateData(const std::unordered_map<std::string, std::vector<slam_msgs::msg::KeyFrameFullData>>& keyframes,
                               const std::set<geometry_msgs::msg::Point>& refined)
{
    latest_keyframes_by_agent_ = keyframes;
    latest_refined_points_ = refined;
}

} // namespace ORB_SLAM3_Wrapper
