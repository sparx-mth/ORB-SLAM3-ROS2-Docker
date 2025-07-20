#include "multi_agent_map_merger/map_merger_slam_base_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

MapMergerSlamBaseNode::MapMergerSlamBaseNode(const std::vector<std::string>& agent_names)
    : Node("map_merger_slam_base_node"), agent_names_(agent_names)
{
    for (const auto& name : agent_names_) {
        std::string service = "/" + name + "/orb_slam3/get_map_data";
        map_clients_[name] = this->create_client<slam_msgs::srv::GetMap>(service);
    }

    merge_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&MapMergerSlamBaseNode::tryMergeMaps, this));

    RCLCPP_INFO(this->get_logger(), "Map merger initialized with %zu agents.", agent_names_.size());
}

void MapMergerSlamBaseNode::tryMergeMaps()
{
    maps_by_agent_.clear();

    for (const auto& name : agent_names_) {
        auto client = map_clients_[name];
        if (!client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "Service not available for %s", name.c_str());
            continue;
        }

        auto request = std::make_shared<slam_msgs::srv::GetMap::Request>();
        auto future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            maps_by_agent_[name] = response->map;
        }
    }

    std::vector<std::string> agent_names;
    for (const auto& [name, _] : maps_by_agent_)
        agent_names.push_back(name);

    for (size_t i = 0; i < agent_names.size(); ++i) {
        for (size_t j = i + 1; j < agent_names.size(); ++j) {
            const auto& nameA = agent_names[i];
            const auto& nameB = agent_names[j];

            const auto& mapA = maps_by_agent_[nameA];
            const auto& mapB = maps_by_agent_[nameB];

            // Loop over all keyframes in both maps
            for (const auto& kfA_msg : mapA.keyframes) {
                for (const auto& kfB_msg : mapB.keyframes) {

                    // TODO: convert kfA_msg and kfB_msg into ORB_SLAM3::KeyFrame* objects
                    ORB_SLAM3::KeyFrame* kfA = convertMsgToKeyFrame(kfA_msg);
                    ORB_SLAM3::KeyFrame* kfB = convertMsgToKeyFrame(kfB_msg);

                    if (!kfA || !kfB) continue;

                    auto vpMatched12 = matchKeyFramesWithORB(kfA, kfB);

                    int n_matches = std::count_if(vpMatched12.begin(), vpMatched12.end(),
                                                  [](auto* mp){ return mp != nullptr; });

                    if (n_matches < 20) continue;

                    Sim3Solver solver(kfA, kfB, vpMatched12, true, {});
                    std::vector<bool> vbInliers;
                    Sophus::Sim3f sim3;
                    bool success = solver.iterate(100, vbInliers, sim3);

                    if (success) {
                        RCLCPP_INFO(this->get_logger(), "Found Sim3 match between %s and %s", nameA.c_str(), nameB.c_str());
                        mergeMaps(nameA, mapA, nameB, mapB);  // You can apply sim3 inside here
                        return;
                    }
                }
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "No successful map merges detected this cycle.");
}


bool MapMergerSlamBaseNode::detectOverlap(const slam_msgs::msg::MapData& mapA,
                                          const slam_msgs::msg::MapData& mapB)
{
    for (const auto& lmA : mapA.landmarks) {
        for (const auto& lmB : mapB.landmarks) {
            float dx = lmA.position.x - lmB.position.x;
            float dy = lmA.position.y - lmB.position.y;
            float dz = lmA.position.z - lmB.position.z;
            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist < 0.3)
                return true;
        }
    }
    return false;
}

void MapMergerSlamBaseNode::mergeMaps(
    std::shared_ptr<ORB_SLAM3::Map> mapA,
    std::shared_ptr<ORB_SLAM3::Map> mapB,
    const Sophus::Sim3f& sim3_AB)
{
    RCLCPP_INFO(this->get_logger(), "Merging mapB into mapA using estimated Sim3...");

    // 1. Transform all keyframes from mapB to mapA coordinate frame
    for (ORB_SLAM3::KeyFrame* kfB : mapB->GetAllKeyFrames())
    {
        Sophus::SE3f Tcw_B = kfB->GetPose();
        Sophus::SE3f Tcw_A = sim3_AB * Tcw_B;
        kfB->SetPose(Tcw_A);
        kfB->ChangeMap(mapA.get());
        mapA->AddKeyFrame(kfB);
    }

    // 2. Transform and move MapPoints from mapB to mapA
    for (ORB_SLAM3::MapPoint* mpB : mapB->GetAllMapPoints())
    {
        if (!mpB || mpB->isBad()) continue;

        Eigen::Vector3f Pw_B = mpB->GetWorldPos();
        Eigen::Vector3f Pw_A = sim3_AB * Pw_B;

        mpB->SetWorldPos(Pw_A);
        mpB->ChangeMap(mapA.get());
        mapA->AddMapPoint(mpB);
    }

    // 3. Update covisibility connections for merged KeyFrames
    for (ORB_SLAM3::KeyFrame* kf : mapA->GetAllKeyFrames())
    {
        kf->UpdateConnections();
    }

    // 4. Optional: run global bundle adjustment
    RCLCPP_INFO(this->get_logger(), "Running global bundle adjustment...");
    ORB_SLAM3::Optimizer::GlobalBundleAdjustemnt(
        mapA.get(), 10, true, nullptr, nullptr);

    RCLCPP_INFO(this->get_logger(), "Map merge complete.");
}

