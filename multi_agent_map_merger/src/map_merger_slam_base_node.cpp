#include "multi_agent_map_merger/map_merger_slam_base_node.hpp"
#include "multi_agent_map_merger/map_visualizer.hpp"
#include "ORBmatcher.h"
#include "Thirdparty/DBoW2/DBoW2/ORBVocabulary.h"

using std::placeholders::_1;

MapMergerSlamBaseNode::MapMergerSlamBaseNode(const std::vector<std::string>& agent_names)
    : Node("map_merger_slam_base_node"), agent_names_(agent_names)
{
    std::string vocab_path;
    this->declare_parameter("vocabulary_path", "");
    this->get_parameter("vocabulary_path", vocab_path);

    vocabulary_ = std::make_shared<ORB_SLAM3::ORBVocabulary>();
    if (!vocabulary_->loadFromTextFile(vocab_path)) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load ORB vocabulary from %s", vocab_path.c_str());
        throw std::runtime_error("Vocabulary loading failed");
    }

    // Load agent colors from parameter
    std::unordered_map<std::string, std::array<float, 3>> agent_colors;
    for (const auto& name : agent_names_) {
        std::vector<double> color;
        std::string param_name = name + "_color";
        this->declare_parameter(param_name, std::vector<double>{1.0, 1.0, 1.0});
        this->get_parameter(param_name, color);
        agent_colors[name] = {static_cast<float>(color[0]), static_cast<float>(color[1]), static_cast<float>(color[2])};
    }

    // Load refined point color
    std::vector<double> refined_color_vec{1.0, 1.0, 0.0}; // yellow default
    this->declare_parameter("refined_point_color", refined_color_vec);
    this->get_parameter("refined_point_color", refined_color_vec);
    std::array<float, 3> refined_color = {
        static_cast<float>(refined_color_vec[0]),
        static_cast<float>(refined_color_vec[1]),
        static_cast<float>(refined_color_vec[2])
    };

    map_visualizer_ = std::make_unique<ORB_SLAM3_Wrapper::MapVisualizer>(this->shared_from_this(), agent_colors, refined_color);
    global_map_ = std::make_shared<ORB_SLAM3::Map>();

    for (const auto& name : agent_names_) {
        std::string topic = "/" + name + "/keyframe_full_data";
        auto sub = this->create_subscription<slam_msgs::msg::KeyFrameFullData>(
            topic,
            rclcpp::QoS(10),
            [this, name](const slam_msgs::msg::KeyFrameFullData::SharedPtr msg) {
                received_keyframes_[name].push_back(*msg);
            });
        keyframe_subs_[name] = sub;

        Eigen::Affine3f initial_transform;
        if (getInitialTransform(name, initial_transform)) {
            agent_initial_poses_[name] = initial_transform;
            RCLCPP_INFO(this->get_logger(), "Loaded initial transform for agent %s", name.c_str());
        }
    }

    merge_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&MapMergerSlamBaseNode::tryMergeMaps, this));

    map_pub_ = this->create_publisher<slam_msgs::msg::MapData>("/merged_map", 10);

    RCLCPP_INFO(this->get_logger(), "Map merger initialized with %zu agents.", agent_names_.size());
}

ORB_SLAM3::KeyFrame* MapMergerSlamBaseNode::convertMsgToKeyFrame(const slam_msgs::msg::KeyFrameFullData& msg)
{
    Sophus::SE3f Tcw(
        Eigen::Quaternionf(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z),
        Eigen::Vector3f(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));

    cv::Mat descriptors = typeConversions_.descriptorMsgToCvMat(msg.descriptors);
    std::vector<cv::KeyPoint> keypoints;
    for (const auto& kp_msg : msg.keypoints.keypoints)
    {
        keypoints.emplace_back(
            kp_msg.x, kp_msg.y, kp_msg.size, kp_msg.angle,
            kp_msg.response, kp_msg.octave, kp_msg.class_id);
    }

    std::vector<Eigen::Vector3f> wordPoints;
    for (const auto& pt : msg.word_pts)
    {
        wordPoints.emplace_back(pt.x, pt.y, pt.z);
    }

    auto* kf = ORB_SLAM3::KeyFrame::FromSerialized(msg.id, Tcw, keypoints, descriptors, wordPoints);
    if (kf && vocabulary_)
        kf->SetVocabulary(vocabulary_);
    if (kf)
        kf->ComputeBoW();
    return kf;
}

std::vector<ORB_SLAM3::MapPoint*> MapMergerSlamBaseNode::matchKeyFramesWithORB(
    ORB_SLAM3::KeyFrame* kf1,
    ORB_SLAM3::KeyFrame* kf2)
{
    ORB_SLAM3::ORBmatcher matcher(true);
    std::vector<ORB_SLAM3::MapPoint*> vpMatched12;
    matcher.SearchByBoW(kf1, kf2, vpMatched12);
    return vpMatched12;
}

bool MapMergerSlamBaseNode::estimateSim3(
    ORB_SLAM3::KeyFrame* kf1,
    ORB_SLAM3::KeyFrame* kf2,
    const std::vector<ORB_SLAM3::MapPoint*>& matches,
    Sophus::Sim3f& sim3_out)
{
    ORB_SLAM3::Sim3Solver solver(kf1, kf2, matches, true, {});
    std::vector<bool> vbInliers;
    return solver.iterate(100, vbInliers, sim3_out);
}

void MapMergerSlamBaseNode::mergeMaps(
    const std::string& nameA,
    const std::vector<slam_msgs::msg::KeyFrameFullData>& mapA,
    const std::string& nameB,
    const std::vector<slam_msgs::msg::KeyFrameFullData>& mapB,
    const Sophus::Sim3f& sim3_AB)
{
    for (const auto& kfB_msg : mapB)
    {
        ORB_SLAM3::KeyFrame* kfB = convertMsgToKeyFrame(kfB_msg);
        if (!kfB) continue;

        Sophus::SE3f Tcw_transformed = Sophus::SE3f(sim3_AB.rotationMatrix(), sim3_AB.translation()) * kfB->GetPose();
        kfB->SetPose(Tcw_transformed);

        for (ORB_SLAM3::MapPoint* mp : kfB->GetMapPoints()) {
            if (!mp || mp->isBad()) continue;
            Eigen::Vector3f Pw_trans = sim3_AB * mp->GetWorldPos();
            mp->SetWorldPos(Pw_trans);

            geometry_msgs::msg::Point refined;
            refined.x = Pw_trans.x();
            refined.y = Pw_trans.y();
            refined.z = Pw_trans.z();
            refined_map_points_.insert(refined);
        }

        // Optional: add kfB to global_map_ if needed
    }

    RCLCPP_INFO(this->get_logger(), "Merged %s into %s", nameB.c_str(), nameA.c_str());
}

void MapMergerSlamBaseNode::tryMergeMaps()
{
    std::vector<std::string> agent_names;
    for (const auto& [name, _] : received_keyframes_)
        agent_names.push_back(name);

    for (size_t i = 0; i < agent_names.size(); ++i) {
        for (size_t j = i + 1; j < agent_names.size(); ++j) {
            const auto& nameA = agent_names[i];
            const auto& nameB = agent_names[j];

            const auto& mapA = received_keyframes_[nameA];
            const auto& mapB = received_keyframes_[nameB];

            for (const auto& kfA_msg : mapA) {
                for (const auto& kfB_msg : mapB) {
                    ORB_SLAM3::KeyFrame* kfA = convertMsgToKeyFrame(kfA_msg);
                    ORB_SLAM3::KeyFrame* kfB = convertMsgToKeyFrame(kfB_msg);
                    if (!kfA || !kfB) continue;

                    auto matches = matchKeyFramesWithORB(kfA, kfB);
                    int n_matches = std::count_if(matches.begin(), matches.end(), [](auto* mp){ return mp != nullptr; });
                    if (n_matches < 20) continue;

                    Sophus::Sim3f sim3;
                    if (estimateSim3(kfA, kfB, matches, sim3)) {
                        RCLCPP_INFO(this->get_logger(), "Found Sim3 match between %s and %s", nameA.c_str(), nameB.c_str());
                        mergeMaps(nameA, mapA, nameB, mapB, sim3);

                        map_visualizer_->updateData(received_keyframes_, refined_map_points_);
                        return;
                    }
                }
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "No successful map merges detected this cycle.");
}
