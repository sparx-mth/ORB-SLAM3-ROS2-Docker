import struct
import rclpy
from rclpy.node import Node
from slam_msgs.srv import GetAllLandmarksInMap
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import time
import numpy as np
import std_msgs.msg

# Import the new GlobalMapManager class
from global_map_manager import GlobalMapManager 


class MapMergerNode(Node):
    def __init__(self):
        super().__init__('map_merger_node')

        # List of robots with their initial positions
        self.robots = {
            0: [-3.0, -7.0, 1.0], 
            1: [-1.0, 0.0, 1.65],
            2 :[5.0, 5.0, 1.65],
        }

        # Initialize the GlobalMapManager
        self.map_manager = GlobalMapManager(duplicate_check_radius=0.02)

        # Create a service client for each robot to get all landmarks in the map
        self.agents_clients = {}
        for robot_id in self.robots:
            service_name = f'/robot_{robot_id}/orb_slam3/get_all_landmarks_in_map'
            self.agents_clients[robot_id] = self.create_client(GetAllLandmarksInMap, service_name)

        # Ensure the services are available
        for robot_id, client in self.agents_clients.items():
            while not client.wait_for_service(timeout_sec=30.0):
                self.get_logger().info(f'Service for robot_{robot_id} not available, waiting...')
        
        # Create a publisher for the merged map (PointCloud2)
        self.map_publisher = self.create_publisher(PointCloud2, '/merged_map', 10)

        # To store the merged map
        self.merged_map = []

        self.last_landmarks = {robot_id: [] for robot_id in self.robots}

        self.get_logger().info('Map Merger Node started.')

        # Create a timer that requests the map data every 1 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # For each robot, request the map data every 2 seconds
        for robot_id in self.robots:
            self.get_all_landmarks(robot_id)

    def get_all_landmarks(self, robot_id):
        # Create a service request for getting all landmarks in the map
        request = GetAllLandmarksInMap.Request()
        request.request = True  # Request the full map

        # Send the request asynchronously
        future = self.agents_clients[robot_id].call_async(request)
        future.add_done_callback(lambda future, robot_id=robot_id: self.handle_landmark_response(future, robot_id))

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

                # Check for significant change in the map (you could use the same method for change detection here)
                if self.is_significant_change(robot_id, valid_points):
                    # Transform landmarks from robot's frame to a global frame
                    transformed_points_with_id = self.transform_landmarks_to_global(valid_points, robot_id)

                    # Add points to the merged map using the GlobalMapManager
                    points_added_count = self.map_manager.add_points(transformed_points_with_id)
                    
                    if points_added_count > 0:
                        self.get_logger().info(f'Added {points_added_count} new unique points from robot {robot_id}. Total merged points: {self.map_manager.get_num_points()}')
                    else:
                        self.get_logger().info(f'No new unique points added from robot {robot_id}. Total merged points: {self.map_manager.get_num_points()}')

                    # Always publish the map after a potential update
                    self.publish_merged_map()

                # Update the last landmarks for this robot
                self.last_landmarks[robot_id] = valid_points

            else:
                self.get_logger().info(f'No landmarks received from robot_{robot_id}.')
        except Exception as e:
            self.get_logger().error(f'Service call failed for robot_{robot_id}: {str(e)}')

    def filter_invalid_points(self, point_cloud_data):
        valid_points = []
        for point in point_cloud_data:
            if not (np.isnan(point[0]) or np.isnan(point[1]) or np.isnan(point[2]) or
                    np.isinf(point[0]) or np.isinf(point[1]) or np.isinf(point[2])):
                valid_points.append(point)
        return valid_points

    def publish_merged_map(self): 
        current_merged_map = self.map_manager.get_merged_map_data()

        if not current_merged_map:
            return

        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_footprint" 
        
        pc2_msg = PointCloud2()
        pc2_msg.header = header
        pc2_msg.height = 1
        pc2_msg.width = len(current_merged_map)
        pc2_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1)
        ]
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 16 
        pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width
        
        colors = {
            0: [0, 0, 255],
            1: [0, 255, 0],
            2: [255, 0, 0],
        }
        
        points_data = bytearray() 

        for point_data in current_merged_map:
            x, y, z = point_data[0], point_data[1], point_data[2]
            original_robot_id = point_data[3]
            rgb = colors.get(original_robot_id, [128, 128, 128])
            rgb_value = (0xFF << 24) | (rgb[0] << 16) | (rgb[1] << 8) | rgb[2]
            packed_point = struct.pack('<fffI', x, y, z, rgb_value)
            points_data.extend(packed_point)

        pc2_msg.data = bytes(points_data)
        self.map_publisher.publish(pc2_msg)
        self.get_logger().info(f'Merged map published with {self.map_manager.get_num_points()} points.')

    def is_significant_change(self, robot_id, new_points):
        # This remains largely the same, as it's a local check for each robot's map stream.
        if not self.last_landmarks[robot_id]: 
            return True  

        distance_threshold = 0.2  

        num_points_to_compare = min(len(self.last_landmarks[robot_id]), len(new_points), 5) 

        if num_points_to_compare == 0: 
            return len(self.last_landmarks[robot_id]) != len(new_points)

        for i in range(num_points_to_compare):
            old_point = self.last_landmarks[robot_id][i]
            new_point = new_points[i]
            try:
                old_xyz = np.array([old_point[0], old_point[1], old_point[2]], dtype=np.float32)
                new_xyz = np.array([new_point[0], new_point[1], new_point[2]], dtype=np.float32)
            except Exception as e:
                raise ValueError(f"Could not extract 3D coordinates from points: old_point={old_point}, new_point={new_point}") from e
            distance = np.linalg.norm(old_xyz - new_xyz)
            if distance > distance_threshold:
                return True
        
        if abs(len(self.last_landmarks[robot_id]) - len(new_points)) > 0.1 * len(self.last_landmarks[robot_id]) : 
             return True

        return False

    def transform_landmarks_to_global(self, map_points, robot_id):
        robot_position = np.array(self.robots[robot_id])
        transformed_points_with_id = [] 

        for point in map_points:
            # Try to extract x, y, z regardless of type
            try:
                x, y, z = point[0], point[1], point[2]
            except Exception as e:
                raise ValueError(f"Could not extract 3D coordinates from point: {point}") from e
            point_xyz = np.array([x, y, z], dtype=np.float32)
            transformed_point_xyz = point_xyz + robot_position
            transformed_points_with_id.append([*transformed_point_xyz.tolist(), robot_id])
        
        return transformed_points_with_id

def main(args=None):
    rclpy.init(args=args)
    node = MapMergerNode()
    
    try:
        # Spin the node so the callback is called
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown the node
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
