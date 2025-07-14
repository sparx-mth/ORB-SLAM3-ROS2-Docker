import numpy as np
from scipy.spatial import KDTree
import struct # Although struct is used by MapMergerNode for publishing, not strictly by the manager itself for point storage

class GlobalMapManager:
    def __init__(self, duplicate_check_radius=0.02):
        """
        Initializes the GlobalMapManager.

        Args:
            duplicate_check_radius (float): The minimum distance (in meters)
                                            between two points for them to be
                                            considered unique.
        """
        self.merged_map = [] # Stores [x, y, z, robot_id] for all unique points
        self.kdtree = None
        self.duplicate_check_radius = duplicate_check_radius
        self.points_added_since_last_kdtree_rebuild = 0 # To track when to rebuild

    def add_points(self, new_points_with_id):
        """
        Adds new points to the merged map after performing a spatial duplicate check
        against ALL existing points in the merged map.

        Args:
            new_points_with_id (list): A list of points, where each point is
                                       [x, y, z, robot_id].

        Returns:
            int: The number of truly unique points added to the merged map.
        """
        points_actually_added = 0
        
        # If the merged map is empty, just add all and build the tree
        if not self.merged_map:
            self.merged_map.extend(new_points_with_id)
            self._rebuild_kdtree()
            return len(new_points_with_id)

        # Efficiently update KD-Tree: Rebuild only if a significant number of points
        # have been added since the last rebuild, or if it's currently None.
        # This prevents rebuilding for every single point or small batches.
        # A threshold of, for example, 10% of current map size, or a fixed number (e.g., 100 points)
        # can be used to decide when to rebuild.
        rebuild_threshold = max(50, int(len(self.merged_map) * 0.05)) # Rebuild after 50 new points or 5% map growth

        if self.kdtree is None or self.points_added_since_last_kdtree_rebuild >= rebuild_threshold:
            self._rebuild_kdtree()
            self.points_added_since_last_kdtree_rebuild = 0 # Reset counter

        # If kdtree is still None (e.g., if merged_map became empty after some operation)
        if self.kdtree is None and self.merged_map: # Should not happen if _rebuild_kdtree logic is sound
            self._rebuild_kdtree()


        for new_point_data in new_points_with_id:
            new_point_xyz = new_point_data[:3] # Extract just x, y, z for KDTree query

            # Query the KD-Tree for any existing points within the specified radius
            # This checks against ALL points currently in self.merged_map
            indices_in_radius = self.kdtree.query_ball_point(new_point_xyz, self.duplicate_check_radius)
            
            if not indices_in_radius: # If no points found within the radius, it's a unique point
                self.merged_map.append(new_point_data)
                points_actually_added += 1
                self.points_added_since_last_kdtree_rebuild += 1
                
        # If any points were added, consider rebuilding the tree for the next query
        # This ensures the KDTree is eventually updated with the very latest points,
        # even if the rebuild_threshold wasn't hit immediately.
        if points_actually_added > 0 and self.points_added_since_last_kdtree_rebuild > 0:
            # We don't necessarily rebuild immediately here, just update the counter.
            # The next call to add_points will check the threshold.
            pass 
        
        return points_actually_added

    def _rebuild_kdtree(self):
        """
        Internal method to rebuild the KD-Tree from the current merged_map points.
        This should be called when the merged_map has changed significantly.
        """
        if self.merged_map:
            # Extract just the (x, y, z) coordinates for the KD-Tree
            xyz_points = np.array([p[:3] for p in self.merged_map])
            self.kdtree = KDTree(xyz_points)
            # print(f"DEBUG: KD-Tree rebuilt with {len(self.merged_map)} points.") # For debugging
        else:
            self.kdtree = None
            # print("DEBUG: Merged map is empty, KD-Tree reset.") # For debugging

    def get_merged_map_data(self):
        """
        Returns the current list of merged map points.
        """
        return self.merged_map

    def get_num_points(self):
        """
        Returns the number of points currently in the merged map.
        """
        return len(self.merged_map)

    def clear_map(self):
        """
        Clears the merged map and resets the KDTree.
        """
        self.merged_map = []
        self.kdtree = None
        self.points_added_since_last_kdtree_rebuild = 0