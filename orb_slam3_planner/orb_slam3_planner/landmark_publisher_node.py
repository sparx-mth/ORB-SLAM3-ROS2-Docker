#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from slam_msgs.srv import GetAllLandmarksInMap
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2


class LandmarkPublisherNode(Node):
    """
    A ROS2 node for periodically retrieving and publishing 3D landmarks from ORB-SLAM3.

    This node acts as an interface between the SLAM system and downstream modules that
    consume point cloud data. It is designed to be launched per robot in a multi-robot system.

    Key Responsibilities:
    ---------------------
    - Requests 3D landmark data from the ORB-SLAM3 service `/orb_slam3/get_all_landmarks_in_map`.
    - Publishes the resulting landmarks as a `sensor_msgs/PointCloud2` message.
    - Publishes a status string with the number of points received for monitoring purposes.

    Topics Published:
    -----------------
    - /<robot_ns>/orb_slam3/landmarks_raw (PointCloud2): Raw 3D landmark point cloud from ORB-SLAM3.
    - /<robot_ns>/orb_slam3/landmark_status (String): Status message indicating the number of landmarks received.

    Services Used:
    --------------
    - /<robot_ns>/orb_slam3/get_all_landmarks_in_map (GetAllLandmarksInMap):
        Request all known 3D map points from the SLAM system.

    Parameters:
    -----------
    - robot_namespace (str): Namespace of the robot, used to scope all topics and services.

    Usage:
    ------
    This node is typically run in parallel with ORB-SLAM3 for each robot. It runs a periodic
    timer (default: 1 Hz) to fetch the latest 3D map points and makes them available for
    visualization, map merging, or further processing.
    """


    def __init__(self):
        super().__init__('landmark_publisher_node')

        # Declare namespace parameter
        self.declare_parameter('robot_namespace', '')
        self.robot_namespace = self.get_parameter('robot_namespace').value.rstrip('/')
        ns = f'/{self.robot_namespace}' if self.robot_namespace else ''

        # Publisher
        self.raw_landmark_pub = self.create_publisher(
            PointCloud2,
            f'{ns}/orb_slam3/landmarks_raw',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            f'{ns}/orb_slam3/landmark_status',
            10
        )

        # Service client
        service_name = f'{ns}/orb_slam3/get_all_landmarks_in_map'
        self.get_landmarks_client = self.create_client(
            GetAllLandmarksInMap,
            service_name
        )

        self.get_logger().info(f'Waiting for ORB-SLAM3 landmark service: {service_name}')

        # Wait for service with timeout
        service_ready = self.get_landmarks_client.wait_for_service(timeout_sec=10.0)
        if service_ready:
            self.get_logger().info(f'Service ready for {self.robot_namespace}!')
        else:
            self.get_logger().warn(f'Service timeout for {self.robot_namespace}, will retry...')

        # Timer to fetch landmarks periodically
        self.timer = self.create_timer(1.0, self.fetch_and_publish_landmarks)
        self.last_raw_count = 0

    def fetch_and_publish_landmarks(self):
        """
        Send an asynchronous service request to retrieve all current landmarks.

        This function first checks that the SLAM service is ready. If so, it requests
        all available 3D landmarks. The response is handled via a callback.
        If the service is not ready, the function returns without making a request.
        """
        ...

        if not self.get_landmarks_client.service_is_ready():
            return

        request = GetAllLandmarksInMap.Request()
        request.request = True

        future = self.get_landmarks_client.call_async(request)
        future.add_done_callback(self.handle_landmark_response)

    def handle_landmark_response(self, future):
        """
        Handle the response from the landmark service.

        Publishes the received point cloud (if any) to the landmarks topic and
        also publishes a status message with the count of received landmarks.
        Logs any errors encountered in the service response.
        """
        try:
            response = future.result()
            if response and response.landmarks:
                cloud = response.landmarks
                self.raw_landmark_pub.publish(cloud)

                # Count and report how many points received
                points = list(pc2.read_points(cloud, skip_nans=True))
                raw_count = len(points)

                if raw_count != self.last_raw_count:
                    status = String()
                    status.data = f"{self.robot_namespace} - Raw landmarks: {raw_count}"
                    self.status_pub.publish(status)
                    # self.get_logger().info(status.data)
                    self.last_raw_count = raw_count

        except Exception as e:
            self.get_logger().error(f"Error receiving landmarks for {self.robot_namespace}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()