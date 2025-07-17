import struct
import rclpy
from rclpy.node import Node
from slam_msgs.srv import GetAllLandmarksInMap
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import std_msgs.msg
import sensor_msgs_py.point_cloud2 as pc2
import time
import numpy as np
import threading

from collections import Counter

class MapMergerNode(Node):
    def __init__(self):
        super().__init__('map_merger_node')

        # List of robots with their initial positions
        self.robots = {
            0: [-5.0, -7.0, 0.0], 
            # 1: [-1.0, 0.0, 0.0],
            # 2 :[5.0, 5.0,0.0],
        }
        self.colors = {
            0: [0, 0, 255],  # Blue (robot 0)
            1: [0, 255, 0],  # Green (robot 1)
            2: [255, 0, 0],  # Red (robot 2)
        }

        self.distance_threshold = 0.1  # Ignore changes greater than 10 cm

        self.frame_counter = {robot_id: 0 for robot_id in self.robots}  # Track frames for each robot
        # Locks for trajectory updates and drawing separately
        self.trajectory_locks = {robot_id: threading.Lock() for robot_id in self.robots}
        self.drawing_locks = {robot_id: threading.Lock() for robot_id in self.robots}  # Separate lock for drawing

        # Create a service client for each robot to get all landmarks in the map
        self.agents_clients = {}
        for robot_id in self.robots:
            service_name = f'/robot_{robot_id}/orb_slam3/get_all_landmarks_in_map'
            self.agents_clients[robot_id] = self.create_client(GetAllLandmarksInMap, service_name)
        
        # Create a publisher for the merged map (PointCloud2)
        self.map_publisher = self.create_publisher(PointCloud2, '/merged_map', 10)

        # Create a publisher for the trajectory (Path visualization in RViz)
        self.trajectory_publishers = {}
        for robot_id in self.robots:
            self.trajectory_publishers[robot_id] = self.create_publisher(Marker, f'/robot_{robot_id}/trajectory_path', 10)

        # Ensure the services are available
        for robot_id, client in self.agents_clients.items():
            while not client.wait_for_service(timeout_sec=30.0):
                self.get_logger().info(f'Service for robot_{robot_id} not available, waiting...')        

        # To store the merged map and robot trajectories
        self.merged_map = []
        self.trajectories = {robot_id: [] for robot_id in self.robots}  # Store trajectory for each robot
        self.last_landmarks = {robot_id: [] for robot_id in self.robots}
        self.last_valid_position = {robot_id: None for robot_id in self.robots}
        self.spike_flags = {robot_id: False for robot_id in self.robots}  # Track if spike was detected
        self.recovery_count = {robot_id: 0 for robot_id in self.robots}  

        self.get_logger().info('Map Merger Node started.')

        # Create a timer that requests the map data every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Add a periodic publisher timer (e.g., every 30 seconds)
        self.periodic_pub_timer = self.create_timer(30.0, self.periodic_publish_callback)

        # Subscribe to each robot's pose for tracking the path/route
        for robot_id in self.robots:
            self.create_subscription(
                PoseStamped,
                f'/robot_{robot_id}/robot_pose_slam',
                lambda msg, robot_id=robot_id: self.pose_callback(msg, robot_id),
                10
            )   


    def timer_callback(self):
        # For each robot, request the map data every 1 second
        for robot_id in self.robots:
            self.get_all_landmarks(robot_id)
    
    def periodic_publish_callback(self):
        # Publish the merged map periodically, even if there is no update
        self.publish_merged_map()

    def get_all_landmarks(self, robot_id):
        # Create a service request for getting all landmarks in the map
        request = GetAllLandmarksInMap.Request()
        request.request = True  # Request the full map

        # Send the request asynchronously
        future = self.agents_clients[robot_id].call_async(request)
        future.add_done_callback(lambda future, robot_id=robot_id: self.handle_landmark_response(future, robot_id))

    def pose_callback(self, msg, robot_id):
        """
        Callback to handle robot pose updates and store the trajectory for each robot.
        The trajectory is then visualized in RViz as a line.
        """
        # Extract the robot's position (x, y, z) from the PoseStamped message
        position = [msg.pose.position.x + self.robots[robot_id][0], msg.pose.position.y + self.robots[robot_id][1], msg.pose.position.z+ self.robots[robot_id][2]]
        
        # Add the new pose directly to the trajectory in global coordinates
        self.add_to_trajectory(robot_id, position)

        # # For debugging, print out the trajectory for the robot
        # self.get_logger().info(f"Robot {robot_id} pose: {position}")

    def handle_landmark_response(self, future, robot_id):
        try:
            # Get the result from the service call
            response = future.result()

            if response:
                self.get_logger().info(f'Received {len(response.landmarks.data)} landmarks from robot_{robot_id}.')

                # Convert PointCloud2 data into a list of points
                points = list(pc2.read_points(response.landmarks, field_names=("x", "y", "z"), skip_nans=True))

                # Filter out invalid points (those with NaN values)
                valid_points = self.filter_invalid_points(points)

                if len(valid_points) != len(points):
                    self.get_logger().warn(f"Skipped {len(points) - len(valid_points)} invalid map points from robot_{robot_id}.")

                # Check for significant change in the map
                if self.is_significant_change(robot_id, valid_points):
                    # Transform landmarks from robot's frame to a global frame
                    transformed_points = self.transform_landmarks_to_global(valid_points, robot_id)

                    # Merge the points into the global map
                    self.merged_map.extend(transformed_points)

                    # Publish the merged map
                    self.publish_merged_map()

                    # Update the last landmarks for this robot
                    self.last_landmarks[robot_id] = valid_points

                else:
                    self.get_logger().info(f'No new landmarks received from robot_{robot_id}.')

        except Exception as e:
            self.get_logger().error(f'Service call failed for robot_{robot_id}: {str(e)}')

    def filter_invalid_points(self, point_cloud):
        # Filter out invalid points (those with NaN values)
        valid_points = []
        for point in point_cloud:
            if not (np.isnan(point[0]) or np.isnan(point[1]) or np.isnan(point[2])):
                valid_points.append(point)
        return valid_points

    def is_significant_change(self, robot_id, new_points):
        """
        Check if the map has changed significantly by comparing the average distance
        between the last N points of the trajectory.
        """
        # Number of points to compare
        num_points_to_compare = min(len(self.last_landmarks[robot_id]), len(new_points), 20)

        if num_points_to_compare < 2:
            return True  # Not enough points to compare, assume a significant change

        total_distance = 0.0

        # Compare the last 'num_points_to_compare' points
        for i in range(num_points_to_compare):
            old_point = self.last_landmarks[robot_id][-num_points_to_compare + i]  # Last N points
            new_point = new_points[i]

            # Convert points to numpy arrays for easy distance calculation
            old_xyz = np.array([old_point[0], old_point[1]], dtype=np.float32)
            new_xyz = np.array([new_point[0], new_point[1]], dtype=np.float32)

            # Calculate the Euclidean distance between the old and new points
            distance = np.linalg.norm(old_xyz - new_xyz)
            total_distance += distance

            self.get_logger().debug(f"Distance between old and new point {i}: {distance:.2f} m")

        # Calculate the average distance
        avg_distance = total_distance / num_points_to_compare

        # Return True if the average distance exceeds the threshold (indicating a significant change)
        if avg_distance > self.distance_threshold:
            self.get_logger().info(f"Significant change detected: avg_distance = {avg_distance:.2f} m")
            return True

        return False


    def transform_landmarks_to_global(self, map_points, robot_id):
        # Transform the points to global coordinates based on robot's position (simple translation here)
        robot_position = np.array(self.robots[robot_id])
        transformed_points_with_id = []  # Now also storing robot_id

        for point in map_points:
            transformed_point_xyz = np.array([point[0], point[1], point[2]]) + robot_position
            # Append the original robot_id to the transformed point
            transformed_points_with_id.append([transformed_point_xyz[0], transformed_point_xyz[1], transformed_point_xyz[2], robot_id])
        
        return transformed_points_with_id

    def remove_duplicates(self, points):
        # Remove duplicate points by spatial hashing (e.g., within a certain distance threshold)
        point_dict = {}
        grid_resolution = 0.1  # 10 cm resolution for the grid

        for point in points:
            grid_key = (int(point[0] / grid_resolution), int(point[1] / grid_resolution), int(point[2] / grid_resolution))
            if grid_key not in point_dict:
                point_dict[grid_key] = point
        
        return list(point_dict.values())

    def publish_merged_map(self):
        # Convert the merged map to a PointCloud2 message and publish it
        if not self.merged_map:  # Check if map is empty
            self.get_logger().info('Merged map is empty, not publishing.')
            return

        # Remove duplicates before publishing
        self.merged_map = self.remove_duplicates(self.merged_map)

        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_footprint"  # Ensure this frame_id is correct relative to your RViz fixed_frame
        
        pc2_msg = PointCloud2()
        pc2_msg.header = header
        pc2_msg.height = 1
        pc2_msg.width = len(self.merged_map)
        
        # Define point fields (x, y, z, rgb)
        pc2_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1)
        ]
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 16  # 3 floats (x, y, z) + 1 UINT32 (rgb)
        pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width
        
        
        
        points_data = bytearray() 

        for point_data in self.merged_map:
            x, y, z = point_data[0], point_data[1], point_data[2]
            original_robot_id = point_data[3]  # Extract the robot_id

            rgb = self.colors.get(original_robot_id, [128, 128, 128])  # Default to grey if ID not found
            rgb_value = (0xFF << 24) | (rgb[0] << 16) | (rgb[1] << 8) | rgb[2]
            
            packed_point = struct.pack('<fffI', x, y, z, rgb_value)
            points_data.extend(packed_point)

        pc2_msg.data = bytes(points_data)  # Convert bytearray to bytes for the message
        
        self.map_publisher.publish(pc2_msg)
        self.get_logger().info(f'Merged map published with {len(self.merged_map)} points.')

    # def draw_trajectory(self, robot_id):
    #     """
    #     Publishes the robot's trajectory (path) to RViz using Marker messages.
    #     Each robot's trajectory is represented as a Line Strip, and the color is based on the robot ID.
    #     """
    #     # Ensure the trajectory has points to draw
    #     if len(self.trajectories[robot_id]) == 0:
    #         self.get_logger().warn(f"No trajectory points for robot {robot_id}.")
    #         return
        
    #     # # Acquire lock to safely access the trajectory data
    #     # with self.trajectory_locks[robot_id]:
    #     # Set color based on robot ID
    #     color = self.colors.get(robot_id, [0.5, 0.5, 0.5])  # Default to gray if robot_id is unknown
        
    #     # Create the Marker message for the robot's trajectory
    #     trajectory_marker = Marker()
    #     trajectory_marker.header.stamp = self.get_clock().now().to_msg()
    #     trajectory_marker.header.frame_id = "base_footprint"  # Frame of reference for the robot's trajectory
    #     trajectory_marker.ns = f"robot_{robot_id}_trajectory"  # Namespace for the robot's trajectory
    #     trajectory_marker.id = 0
    #     trajectory_marker.type = Marker.LINE_STRIP  # Type of marker (line strip for the path)
    #     trajectory_marker.action = Marker.ADD
    #     trajectory_marker.scale.x = 0.1  # Line width in RViz (larger for better visibility)
    #     trajectory_marker.color.a = 1.0  # Opacity (1.0 for fully visible)
    #     trajectory_marker.color.r = float(color[2]) / 255.0  # Red component
    #     trajectory_marker.color.g = float(color[2]) / 255.0  # Green component
    #     trajectory_marker.color.b = float(color[2]) / 255.0  # Blue component

    #     # Add each trajectory point to the Marker message
    #     for point in self.trajectories[robot_id]:
    #         p = Point()
    #         p.x, p.y, p.z = point[0], point[1], point[2]
    #         trajectory_marker.points.append(p)

    #     # Publish the trajectory marker
    #     self.trajectory_publishers[robot_id].publish(trajectory_marker)
    #     self.get_logger().info(f"Published trajectory for robot {robot_id} with {len(trajectory_marker.points)} points.")



    def add_to_trajectory(self, robot_id, new_position):
        """
        Adds the robot's position to the trajectory if it is valid.
        Ignores spikes but allows recovery once the robot returns to expected behavior.
        """
        with self.trajectory_locks[robot_id]:
            # If no valid last position, directly add the first point
            if self.last_valid_position[robot_id] is None:
                self.trajectories[robot_id].append(new_position)
                self.last_valid_position[robot_id] = new_position
                return
            
            # Calculate the distance from the last valid position
            distance = np.linalg.norm(np.array(new_position) - np.array(self.last_valid_position[robot_id]))

            # Define a threshold for spike detection
            spike_threshold = 5.0  # Adjust as needed (e.g., 5 meters)
            if distance > spike_threshold:
                # If the change is too large, consider this a spike
                self.spike_flags[robot_id] = True
                self.get_logger().info(f"Spike detected for robot {robot_id} with distance: {distance:.2f} m")

                # Optionally, track the number of frames since the spike to allow for recovery
                self.recovery_count[robot_id] += 1
                if self.recovery_count[robot_id] > 5:  # Allow 5 frames to stabilize
                    self.spike_flags[robot_id] = False
                    self.recovery_count[robot_id] = 0  # Reset recovery count
                    self.get_logger().info(f"Spike recovery complete for robot {robot_id}")
            else:
                # If distance is within reasonable range, add the new position
                self.spike_flags[robot_id] = False
                self.recovery_count[robot_id] = 0  # Reset recovery counter
                self.trajectories[robot_id].append(new_position)
                self.last_valid_position[robot_id] = new_position
                self.get_logger().info(f"Updating trajectory for robot {robot_id}")

            # After processing, ensure trajectory drawing
            self.schedule_drawing(robot_id)

    def schedule_drawing(self, robot_id):
        """
        Use a separate lock for drawing the trajectory to avoid deadlock.
        """
        if self.drawing_locks[robot_id].locked():
            self.get_logger().info(f"Drawing for robot {robot_id} is already in progress.")
            return

        # Acquire drawing lock asynchronously, in a non-blocking way
        with self.drawing_locks[robot_id]:
            self.draw_trajectory(robot_id)

    def schedule_drawing(self, robot_id):
        """
        Use a separate lock for drawing the trajectory to avoid deadlock.
        """
        if self.drawing_locks[robot_id].locked():
            self.get_logger().info(f"Drawing for robot {robot_id} is already in progress.")
            return

        # Acquire drawing lock asynchronously, in a non-blocking way
        with self.drawing_locks[robot_id]:
            self.draw_trajectory(robot_id)

    def is_stable(self, robot_id):
        """
        Check if the robot's trajectory is stable by comparing the average distance over the last N points.
        If the change is too large (e.g., spike), enter recovery mode.
        """
        window_size = 5  # Number of points to check for stability

        if len(self.trajectories[robot_id]) < window_size:
            return False  # Not enough data to determine stability

        total_distance = 0.0

        # Compare the last 'window_size' points
        for i in range(window_size - 1):
            old_point = self.trajectories[robot_id][-window_size + i]
            new_point = self.trajectories[robot_id][-window_size + i + 1]

            # Compute distance between consecutive points
            distance = np.linalg.norm(np.array(old_point) - np.array(new_point))
            total_distance += distance

        # Calculate the average distance between consecutive points
        avg_distance = total_distance / (window_size - 1)

        # If the average distance is too large, it indicates a spike and we enter recovery mode
        recovery_threshold = 0.5  # Set a threshold for significant jumps (e.g., 50 cm)
        if avg_distance > recovery_threshold:
            self.recovery_flags[robot_id] = True  # Start recovery phase
            self.get_logger().info(f"Significant jump detected for robot {robot_id}. Entering recovery mode.")
            return False

        # Stability threshold: If the average distance is smaller than the threshold, consider it stable
        stability_threshold = 0.05  # Adjust this threshold as needed (e.g., 5 cm)
        return avg_distance < stability_threshold

    def draw_trajectory(self, robot_id):
        """
        Publishes the robot's trajectory (path) to RViz using Marker messages.
        Each robot's trajectory is represented as a Line Strip, and the color is based on the robot ID.
        """
        # Define a color map based on robot ID
        color = self.colors.get(robot_id, [0.5, 0.5, 0.5])  # Default to gray if robot_id is unknown

        # Create the Marker message for the robot's trajectory
        trajectory_marker = Marker()
        trajectory_marker.header.stamp = self.get_clock().now().to_msg()
        trajectory_marker.header.frame_id = "base_footprint"  # Frame of reference for the robot's trajectory
        trajectory_marker.ns = f"robot_{robot_id}_trajectory"  # Namespace for the robot's trajectory
        trajectory_marker.id = 0
        trajectory_marker.type = Marker.LINE_STRIP  # Type of marker (line strip for the path)
        trajectory_marker.action = Marker.ADD
        trajectory_marker.scale.x = 0.05  # Line width in RViz
        trajectory_marker.color.a = 1.0  # Opacity (1.0 for fully visible)
        trajectory_marker.color.r = float(color[2]) / 255.0 * 0.9  # Red component
        trajectory_marker.color.g = float(color[2]) / 255.0 * 0.9  # Green component
        trajectory_marker.color.b = float(color[2]) / 255.0 * 0.9  # Blue component

        # Add each trajectory point to the Marker message
        for point in self.trajectories[robot_id]:
            p = Point()
            p.x, p.y, p.z = point[0], point[1], point[2]
            trajectory_marker.points.append(p)

        # Publish the trajectory marker
        self.trajectory_publishers[robot_id].publish(trajectory_marker)


def main(args=None):
    rclpy.init(args=args)
    node = MapMergerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
