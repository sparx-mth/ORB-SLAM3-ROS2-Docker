#pragma once

#include "rclcpp/rclcpp.hpp"
#include "slam_msgs/msg/key_frame_full_data.hpp"
#include "slam_msgs/msg/map_data.hpp"

#include "multi_agent_map_merger/map_visualizer.hpp"
#include "orb_slam3_ros2_wrapper/type_conversion.hpp"

#include "System.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Map.h"
#include "Sim3Solver.h"
#include "ORBmatcher.h"

#include <unordered_map>
#include <vector>
#include <string>
#include <memory>
#include <set>
#include <array>

class MapMergerSlamBaseNode : public rclcpp::Node
{
public:
    explicit MapMergerSlamBaseNode(const std::vector<std::string>& agent_names);

private:
    void tryMergeMaps();

    ORB_SLAM3::KeyFrame* convertMsgToKeyFrame(const slam_msgs::msg::KeyFrameFullData& msg);
    std::vector<ORB_SLAM3::MapPoint*> matchKeyFramesWithORB(
        ORB_SLAM3::KeyFrame* kf1,
        ORB_SLAM3::KeyFrame* kf2);
    bool estimateSim3(
        ORB_SLAM3::KeyFrame* kf1,
        ORB_SLAM3::KeyFrame* kf2,
        const std::vector<ORB_SLAM3::MapPoint*>& matches,
        Sophus::Sim3f& sim3_out);
    
    void mergeMaps(
        const std::string& nameA,
        const std::vector<slam_msgs::msg::KeyFrameFullData>& mapA,
        const std::string& nameB,
        const std::vector<slam_msgs::msg::KeyFrameFullData>& mapB,
        const Sophus::Sim3f& sim3_AB);

    bool getInitialTransform(const std::string& agent_name, Eigen::Affine3f& transform);  // optional

    // Parameters and state
    std::vector<std::string> agent_names_;
    std::shared_ptr<ORB_SLAM3::ORBVocabulary> vocabulary_;
    std::shared_ptr<ORB_SLAM3::Map> global_map_;
    
    std::unordered_map<std::string, rclcpp::Subscription<slam_msgs::msg::KeyFrameFullData>::SharedPtr> keyframe_subs_;
    std::unordered_map<std::string, std::vector<slam_msgs::msg::KeyFrameFullData>> received_keyframes_;
    std::unordered_map<std::string, Eigen::Affine3f> agent_initial_poses_;

    std::set<geometry_msgs::msg::Point> refined_map_points_;

    // Timers and publishers
    rclcpp::TimerBase::SharedPtr merge_timer_;
    rclcpp::TimerBase::SharedPtr viz_timer_;
    rclcpp::Publisher<slam_msgs::msg::MapData>::SharedPtr map_pub_;

    std::unique_ptr<ORB_SLAM3_Wrapper::MapVisualizer> map_visualizer_;
    TypeConversions typeConversions_;
};
