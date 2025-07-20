#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np
import math

class ColorPoseCorrector(Node):
    def __init__(self):
        super().__init__('color_pose_corrector')

        # Parameters
        self.robot_name = 'robot_0'
        self.color = 'blue'
        self.bridge = CvBridge()

        # Data storage
        self.latest_depth_image = None
        self.landmark_position = None  # (x_landmark, y_landmark)
        self.last_pose = None
        self.last_azimuth_deg = None

        # Subscriptions
        self.create_subscription(Image, f'/{self.robot_name}/depth_camera', self.depth_callback, 10)
        self.create_subscription(PoseStamped, f'/{self.robot_name}/robot_pose_slam', self.pose_callback, 10)
        self.create_subscription(Float32, f'/{self.robot_name}/global_azimuth', self.azimuth_callback, 10)

        # Publisher
        self.corrected_pose_pub = self.create_publisher(PoseStamped, f'/{self.robot_name}/corrected_pose_from_color', 10)

        self.get_logger().info("✅ ColorPoseCorrector initialized without message_filters")

    def depth_callback(self, depth_msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

    def pose_callback(self, pose_msg):
        self.last_pose = pose_msg
        self.try_compute()

    def azimuth_callback(self, azimuth_msg):
        self.last_azimuth_deg = azimuth_msg.data
        self.try_compute()

    def try_compute(self):
        if self.last_pose is None or self.last_azimuth_deg is None or self.latest_depth_image is None:
            return

        azimuth_rad = math.radians(self.last_azimuth_deg)
        robot_x = self.last_pose.pose.position.x
        robot_y = self.last_pose.pose.position.y

        # Get depth at center of image (or you can set to bounding box center)
        height, width = self.latest_depth_image.shape
        cx, cy = width // 2, height // 2
        depth = float(self.latest_depth_image[cy, cx])

        if np.isnan(depth) or depth <= 0.0:
            self.get_logger().warn("Invalid depth value. Skipping.")
            return

        self.get_logger().info(f"Pose=({robot_x:.2f}, {robot_y:.2f}), Azimuth={self.last_azimuth_deg:.2f}°, Depth={depth:.2f}m")

        if self.landmark_position is None:
            # First detection – estimate landmark global position
            landmark_x = robot_x + depth * math.cos(azimuth_rad)
            landmark_y = robot_y + depth * math.sin(azimuth_rad)
            self.landmark_position = (landmark_x, landmark_y)
            self.get_logger().info(f"Saved landmark at ({landmark_x:.2f}, {landmark_y:.2f}) ✅")
        else:
            # Second detection – estimate corrected robot pose
            landmark_x, landmark_y = self.landmark_position
            corrected_x = landmark_x - depth * math.cos(azimuth_rad)
            corrected_y = landmark_y - depth * math.sin(azimuth_rad)

            corrected_pose = PoseStamped()
            corrected_pose.header = self.last_pose.header
            corrected_pose.pose.position.x = corrected_x
            corrected_pose.pose.position.y = corrected_y
            corrected_pose.pose.position.z = self.last_pose.pose.position.z
            corrected_pose.pose.orientation = self.last_pose.pose.orientation

            self.corrected_pose_pub.publish(corrected_pose)
            self.get_logger().info(f"Published corrected pose: ({corrected_x:.2f}, {corrected_y:.2f}) 📡")

def main(args=None):
    rclpy.init(args=args)
    node = ColorPoseCorrector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()