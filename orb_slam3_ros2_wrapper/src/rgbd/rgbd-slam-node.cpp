/**
 * @file rgbd-slam-node.cpp
 * @brief Implementation of the RgbdSlamNode Wrapper class.
 * @author Suchetan R S (rssuchetan@gmail.com)
 */
#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>

namespace ORB_SLAM3_Wrapper
{
    RgbdSlamNode::RgbdSlamNode(const std::string &strVocFile,
                               const std::string &strSettingsFile,
                               ORB_SLAM3::System::eSensor sensor)
        : Node("ORB_SLAM3_RGBD_ROS2")
    {
        // Declare parameters (topic names)
        this->declare_parameter("rgb_image_topic_name", rclcpp::ParameterValue("camera/image_raw"));
        this->declare_parameter("depth_image_topic_name", rclcpp::ParameterValue("depth/image_raw"));
        this->declare_parameter("imu_topic_name", rclcpp::ParameterValue("imu"));
        this->declare_parameter("odom_topic_name", rclcpp::ParameterValue("odom"));
        this->declare_parameter("lidar_topic_name", rclcpp::ParameterValue("lidar/points"));

        // ROS Subscribers
        rgbSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, this->get_parameter("rgb_image_topic_name").as_string());
        depthSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, this->get_parameter("depth_image_topic_name").as_string());
        syncApproximate_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *rgbSub_, *depthSub_);
        syncApproximate_->registerCallback(&RgbdSlamNode::RGBDCallback, this);

        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(this->get_parameter("imu_topic_name").as_string(), 1000, std::bind(&RgbdSlamNode::ImuCallback, this, std::placeholders::_1));
        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(this->get_parameter("odom_topic_name").as_string(), 1000, std::bind(&RgbdSlamNode::OdomCallback, this, std::placeholders::_1));
        // ROS Publishers
        mapDataPub_ = this->create_publisher<slam_msgs::msg::MapData>("map_data", 10);
        mapPointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);
#ifdef WITH_TRAVERSABILITY_MAP
        lidarCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions lidarSubOptions;
        lidarSubOptions.callback_group = lidarCallbackGroup_;
        lidarSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(this->get_parameter("lidar_topic_name").as_string(), 1000, std::bind(&RgbdSlamNode::LidarCallback, this, std::placeholders::_1), lidarSubOptions);

        gridmapPub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("global_traversability_map", 10);
        traversabilityPub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("RTQuadtree_struct", rclcpp::QoS(1).transient_local());
        publishOccupancyCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        traversabilityTimer_ = this->create_wall_timer(std::chrono::milliseconds(800), std::bind(&RgbdSlamNode::publishTraversabilityData, this), publishOccupancyCallbackGroup_);
#endif
        visibleLandmarksPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("visible_landmarks", 10);
        visibleLandmarksPose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("visible_landmarks_pose", 10);
        robotPoseMapFrame_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose_slam", 10);
        additions_pub_ = create_publisher<traversability_msgs::msg::KeyFrameAdditions>("traversability_keyframe_additions", 10);
        updates_pub_ = create_publisher<traversability_msgs::msg::KeyFrameUpdates>("traversability_keyframe_updates", 10);
        // Services
        getMapDataService_ = this->create_service<slam_msgs::srv::GetMap>("orb_slam3_get_map_data", std::bind(&RgbdSlamNode::getMapServer, this,
                                                                                                              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        pointsInViewCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        getMapPointsService_ = this->create_service<slam_msgs::srv::GetLandmarksInView>("orb_slam3_get_landmarks_in_view", std::bind(&RgbdSlamNode::getMapPointsInViewServer, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), rmw_qos_profile_services_default, pointsInViewCallbackGroup_);
        // TF
        tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        bool bUseViewer;
        this->declare_parameter("visualization", rclcpp::ParameterValue(true));
        this->get_parameter("visualization", bUseViewer);

        this->declare_parameter("ros_visualization", rclcpp::ParameterValue(false));
        this->get_parameter("ros_visualization", rosViz_);

        this->declare_parameter("robot_base_frame", "base_link");
        this->get_parameter("robot_base_frame", robot_base_frame_id_);

        this->declare_parameter("global_frame", "map");
        this->get_parameter("global_frame", global_frame_);

        this->declare_parameter("odom_frame", "odom");
        this->get_parameter("odom_frame", odom_frame_id_);

        this->declare_parameter("robot_x", rclcpp::ParameterValue(1.0));
        this->get_parameter("robot_x", robot_x_);

        this->declare_parameter("robot_y", rclcpp::ParameterValue(1.0));
        this->get_parameter("robot_y", robot_y_);

        this->declare_parameter("no_odometry_mode", rclcpp::ParameterValue(false));
        this->get_parameter("no_odometry_mode", no_odometry_mode_);

        this->declare_parameter("publish_tf", rclcpp::ParameterValue(true));
        this->get_parameter("publish_tf", publish_tf_);

        this->declare_parameter("map_data_publish_frequency", rclcpp::ParameterValue(1000));
        this->get_parameter("map_data_publish_frequency", map_data_publish_frequency_);

        this->declare_parameter("landmark_publish_frequency", rclcpp::ParameterValue(1000));
        this->get_parameter("landmark_publish_frequency", landmark_publish_frequency_);

        this->declare_parameter("publish_traversability_data", rclcpp::ParameterValue(false));
        this->get_parameter("publish_traversability_data", publish_traversability_data_);

        // Timers
        mapDataCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        mapDataTimer_ = this->create_wall_timer(std::chrono::milliseconds(map_data_publish_frequency_), std::bind(&RgbdSlamNode::publishMapData, this), mapDataCallbackGroup_);
        if (rosViz_)
        {
            mapPointsCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            mapPointsTimer_ = this->create_wall_timer(std::chrono::milliseconds(landmark_publish_frequency_), std::bind(&RgbdSlamNode::publishMapPointCloud, this), mapPointsCallbackGroup_);
        }

        interface_ = std::make_shared<ORB_SLAM3_Wrapper::ORBSLAM3Interface>(strVocFile, strSettingsFile,
                                                                            sensor, bUseViewer, rosViz_, robot_x_,
                                                                            robot_y_, global_frame_, odom_frame_id_, robot_base_frame_id_);

        frequency_tracker_count_ = 0;
        frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO(this->get_logger(), "CONSTRUCTOR END!");
    }

    RgbdSlamNode::~RgbdSlamNode()
    {
        rgbSub_.reset();
        depthSub_.reset();
        imuSub_.reset();
        odomSub_.reset();
        interface_.reset();
        RCLCPP_INFO(this->get_logger(), "DESTRUCTOR!");
    }

    void RgbdSlamNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgIMU)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "ImuCallback");
        // push value to imu buffer.
        interface_->handleIMU(msgIMU);
    }

    void RgbdSlamNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msgOdom)
    {
        std::lock_guard<std::mutex> lock(latestTimeMutex_);
        latestTime_ = msgOdom->header.stamp;
        if (!no_odometry_mode_)
        {   // populate map to odom tf if odometry is being used
            RCLCPP_DEBUG_STREAM(this->get_logger(), "OdomCallback");
            interface_->getMapToOdomTF(msgOdom, tfMapOdom_);
        }
        else
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 4000, "Odometry msg recorded but no odometry mode is true, set to false to use this odometry");
    }

    void RgbdSlamNode::RGBDCallback(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD)
    {
        Sophus::SE3f Tcw;
        if (interface_->trackRGBD(msgRGB, msgD, Tcw))
        {
            isTracked_ = true;
            if (publish_tf_)
            {
                // populate map to base_footprint tf if odometry is not being used
                if (no_odometry_mode_)
                    interface_->getDirectMapToRobotTF(msgRGB->header, tfMapOdom_);
                // publish the tf if publish_tf_ is true
                tfBroadcaster_->sendTransform(tfMapOdom_);
            }
            geometry_msgs::msg::PoseStamped pose;
            interface_->getRobotPose(pose);
            pose.header.stamp = msgRGB->header.stamp;
            robotPoseMapFrame_->publish(pose);

            ++frequency_tracker_count_;
            // publishMapPointCloud();
            // std::thread(&RgbdSlamNode::publishMapPointCloud, this).detach();
        }
    }

#ifdef WITH_TRAVERSABILITY_MAP
    void RgbdSlamNode::LidarCallback(sensor_msgs::msg::PointCloud2::SharedPtr msgLidar)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), "PCLCallback");
        interface_->handleLidarPCL(msgLidar->header.stamp, msgLidar);
    }

    void RgbdSlamNode::publishTraversabilityData()
    {
        if(!publish_traversability_data_)
            return;
        std::lock_guard<std::mutex> lock(latestTimeMutex_);
        auto map = interface_->getTraversabilityData();
        // publish the gridmap and occupancy map.
        map.first.info.origin.position.x = map.first.info.origin.position.x + robot_x_;
        map.first.info.origin.position.y = map.first.info.origin.position.y + robot_y_;
        map.first.header.frame_id = global_frame_;
        map.first.header.stamp = latestTime_;
        gridmapPub_->publish(map.first);
        traversabilityPub_->publish(map.second);
    }
#endif

    void RgbdSlamNode::publishMapPointCloud()
    {
        if (isTracked_)
        {
            // Using high resolution clock to measure time
            auto start = std::chrono::high_resolution_clock::now();

            sensor_msgs::msg::PointCloud2 mapPCL;

            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_create_mapPCL = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Time to create mapPCL object: " << time_create_mapPCL << " seconds");

            interface_->getCurrentMapPoints(mapPCL);

            if (mapPCL.data.size() == 0)
                return;

            auto t2 = std::chrono::high_resolution_clock::now();
            auto time_get_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Time to get current map points: " << time_get_map_points << " seconds");

            mapPointsPub_->publish(mapPCL);
            auto t3 = std::chrono::high_resolution_clock::now();
            auto time_publish_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Time to publish map points: " << time_publish_map_points << " seconds");
            RCLCPP_DEBUG_STREAM(this->get_logger(), "=======================");

            // Calculate the time taken for each line

            // Print the time taken for each line
        }
    }

    bool poseChanged(const geometry_msgs::msg::Pose &a,
                     const geometry_msgs::msg::Pose &b)
    {
        return a.position.x != b.position.x ||
               a.position.y != b.position.y ||
               a.position.z != b.position.z ||
               a.orientation.x != b.orientation.x ||
               a.orientation.y != b.orientation.y ||
               a.orientation.z != b.orientation.z ||
               a.orientation.w != b.orientation.w;
    }

    void RgbdSlamNode::publishMapData()
    {
        if (isTracked_)
        {
            auto start = std::chrono::high_resolution_clock::now();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing map data");
            RCLCPP_INFO_STREAM(this->get_logger(), "Current ORB-SLAM3 tracking frequency: " << frequency_tracker_count_ / std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - frequency_tracker_clock_).count() << " frames / sec");
            frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();
            frequency_tracker_count_ = 0;
            // publish the map data (current active keyframes etc)
            slam_msgs::msg::MapData mapDataMsg;
            interface_->mapDataToMsg(mapDataMsg, true, false);
            mapDataPub_->publish(mapDataMsg);
            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_publishMapData = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Time to create mapdata: " << time_publishMapData << " seconds");
            RCLCPP_INFO_STREAM(this->get_logger(), "*************************");

            traversability_msgs::msg::KeyFrameAdditions additions;
            traversability_msgs::msg::KeyFrameUpdates updates;

            // Validate array sizes
            if (mapDataMsg.graph.poses_id.size() != mapDataMsg.graph.poses.size())
            {
                RCLCPP_ERROR(get_logger(),
                             "Mismatched poses_id (%zu) and poses (%zu) sizes!",
                             mapDataMsg.graph.poses_id.size(), mapDataMsg.graph.poses.size());
                return;
            }

            // Process all poses
            for (size_t i = 0; i < mapDataMsg.graph.poses_id.size(); ++i)
            {
                const int32_t id = mapDataMsg.graph.poses_id[i];
                const auto &pose_stamped = mapDataMsg.graph.poses[i];

                // Convert timestamp to nanoseconds
                double sec = pose_stamped.header.stamp.sec + (pose_stamped.header.stamp.nanosec * pow(10, -9));
                const uint64_t timestamp_ns = sec * 1e9;

                // Check if we've seen this pose before
                auto it = previous_poses_.find(id);
                if (it == previous_poses_.end())
                {
                    // New pose - add to additions
                    traversability_msgs::msg::KeyFrame kf;
                    kf.kf_timestamp_in_nanosec = timestamp_ns;
                    kf.kf_id = id;
                    kf.kf_pose = pose_stamped.pose;
                    kf.kf_pointcloud = sensor_msgs::msg::PointCloud2(); // Empty cloud
                    kf.map_id = 0;
                    additions.keyframes.push_back(kf);
                    previous_poses_[id] = pose_stamped.pose;
                }
                else
                {
                    // Existing pose - check for changes
                    if (poseChanged(it->second, pose_stamped.pose))
                    {
                        traversability_msgs::msg::KeyFrame kf;
                        kf.kf_timestamp_in_nanosec = timestamp_ns;
                        kf.kf_id = id;
                        kf.kf_pose = pose_stamped.pose;
                        kf.kf_pointcloud = sensor_msgs::msg::PointCloud2();
                        kf.map_id = 0;
                        updates.keyframes.push_back(kf);
                        it->second = pose_stamped.pose; // Update stored pose
                    }
                }
            }

            // Publish results
            if (!additions.keyframes.empty())
            {
                additions_pub_->publish(additions);
            }
            if (!updates.keyframes.empty())
            {
                updates_pub_->publish(updates);
            }
            t1 = std::chrono::high_resolution_clock::now();
            time_publishMapData = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Time to create traversability data: " << time_publishMapData << " seconds");
        }
    }

    void RgbdSlamNode::getMapServer(std::shared_ptr<rmw_request_id_t> request_header,
                                    std::shared_ptr<slam_msgs::srv::GetMap::Request> request,
                                    std::shared_ptr<slam_msgs::srv::GetMap::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "GetMap2 service called.");
        slam_msgs::msg::MapData mapDataMsg;
        interface_->mapDataToMsg(mapDataMsg, false, request->tracked_points, request->kf_id_for_landmarks);
        response->data = mapDataMsg;
    }

    void RgbdSlamNode::getMapPointsInViewServer(std::shared_ptr<rmw_request_id_t> request_header,
                                                std::shared_ptr<slam_msgs::srv::GetLandmarksInView::Request> request,
                                                std::shared_ptr<slam_msgs::srv::GetLandmarksInView::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "GetMapPointsInView service called.");
        std::vector<slam_msgs::msg::MapPoint> landmarks;
        std::vector<ORB_SLAM3::MapPoint*> points;
        interface_->mapPointsVisibleFromPose(request->pose, points, 1000, request->max_dist_pose_observation, request->max_angle_pose_observation);
        auto affineMapToPos = interface_->getTypeConversionPtr()->poseToAffine(request->pose);
        auto affinePosToMap = affineMapToPos.inverse().cast<double>();
        // Populate the pose of the points vector into the ros message
        for (auto &point : points)
        {
            slam_msgs::msg::MapPoint landmark;
            Eigen::Vector3f landmark_position = point->GetWorldPos();
            auto position = interface_->getTypeConversionPtr()->vector3fORBToROS(landmark_position);
            position = interface_->getTypeConversionPtr()->transformPointWithReference<Eigen::Vector3f>(affinePosToMap, position);
            // RCLCPP_INFO_STREAM(this->get_logger(), "x: " << position.x() << " y: " << position.y() << " z: " << position.z());
            landmark.position.x = position.x();
            landmark.position.y = position.y();
            landmark.position.z = position.z();
            landmarks.push_back(landmark);
        }
        response->map_points = landmarks;
        auto cloud = interface_->getTypeConversionPtr()->MapPointsToPCL(points);
        visibleLandmarksPub_->publish(cloud);

        // Convert the pose in request to PoseStamped and publish
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = "map"; // Assuming the frame is "map", adjust if needed
        pose_stamped.pose = request->pose;

        // Publish the PoseStamped
        visibleLandmarksPose_->publish(pose_stamped);
    }
}
