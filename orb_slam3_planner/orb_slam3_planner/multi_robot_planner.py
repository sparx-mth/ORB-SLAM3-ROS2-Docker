#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import numpy as np
import math
import heapq
from collections import defaultdict
import threading
import random


class MultiRobotPlanner(Node):
    """
    Coordinated planner for multiple robots exploring an environment.
    Assigns frontiers to robots to maximize coverage and minimize overlap.
    """

    def __init__(self, robot_ids):
        super().__init__('multi_robot_planner')

        self.robot_ids = robot_ids

        # Map data
        self.occupancy_grid = None
        self.grid_size = 0
        self.cell_size = 0.25
        self.map_range = 160.0

        # Robot states
        self.robot_positions = {rid: None for rid in robot_ids}
        self.robot_goals = {rid: None for rid in robot_ids}
        self.robot_paths = {rid: [] for rid in robot_ids}

        # Planning parameters
        self.exploration_radius = 3  # cells
        self.min_frontier_distance = 5  # cells
        self.max_frontier_distance = 10  # cells
        self.goal_reached_threshold = 3  # cells

        # Visited locations (shared across all robots)
        self.visited_frontiers = set()
        self.frontier_assignments = {}  # frontier -> robot_id

        # Thread safety
        self.planning_lock = threading.Lock()

        # Publishers
        self.goal_publishers = {}
        self.path_publishers = {}

        for robot_id in robot_ids:
            self.goal_publishers[robot_id] = self.create_publisher(
                Point, f'/robot_{robot_id}/goal', 10
            )
            self.path_publishers[robot_id] = self.create_publisher(
                Point, f'/robot_{robot_id}/next_waypoint', 10
            )

        # Subscriptions
        self.create_subscription(
            OccupancyGrid, '/shared_occupancy_grid', self.map_callback, 10
        )

        for robot_id in robot_ids:
            self.create_subscription(
                Point, f'/robot_{robot_id}/grid_position',
                self.create_position_callback(robot_id), 10
            )

        # Timers
        self.create_timer(2.0, self.planning_loop)
        self.create_timer(0.5, self.publish_waypoints)

        self.get_logger().info(f'Multi-Robot Planner initialized for robots: {robot_ids}')

    def create_position_callback(self, robot_id):
        """Factory function to create position callbacks"""

        def callback(msg):
            with self.planning_lock:
                self.robot_positions[robot_id] = (int(msg.x), int(msg.y))

                # Check if robot reached its goal
                if self.robot_goals[robot_id] is not None:
                    gx, gy = self.robot_goals[robot_id]
                    rx, ry = int(msg.x), int(msg.y)
                    distance = math.sqrt((gx - rx) ** 2 + (gy - ry) ** 2)

                    if distance < self.goal_reached_threshold:
                        self.get_logger().info(f'Robot_{robot_id} reached goal at ({gx}, {gy})')
                        self.visited_frontiers.add((gx, gy))

                        # Clear assignment
                        if (gx, gy) in self.frontier_assignments:
                            del self.frontier_assignments[(gx, gy)]

                        self.robot_goals[robot_id] = None
                        self.robot_paths[robot_id] = []

        return callback

    def map_callback(self, msg):
        """Update the occupancy grid"""
        with self.planning_lock:
            self.grid_size = msg.info.width
            self.cell_size = msg.info.resolution
            self.occupancy_grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def planning_loop(self):
        """Main planning loop - assigns frontiers to robots"""
        if self.occupancy_grid is None:
            return

        with self.planning_lock:
            # Find all frontiers
            frontiers = self.find_all_frontiers()

            if not frontiers:
                self.get_logger().warn('No frontiers found!')
                return

            # Get robots that need new goals
            available_robots = []
            for robot_id in self.robot_ids:
                if self.robot_positions[robot_id] is not None and \
                        self.robot_goals[robot_id] is None:
                    available_robots.append(robot_id)

            if not available_robots:
                return

            # Assign frontiers to robots
            assignments = self.assign_frontiers_to_robots(available_robots, frontiers)

            # Plan paths and publish goals
            for robot_id, frontier in assignments.items():
                path = self.plan_path(self.robot_positions[robot_id], frontier)

                if path and len(path) > 1:
                    self.robot_goals[robot_id] = frontier
                    self.robot_paths[robot_id] = path
                    self.frontier_assignments[frontier] = robot_id

                    # Publish goal
                    goal_msg = Point()
                    goal_msg.x = float(frontier[0])
                    goal_msg.y = float(frontier[1])
                    goal_msg.z = 0.0
                    self.goal_publishers[robot_id].publish(goal_msg)

                    self.get_logger().info(
                        f'Assigned frontier ({frontier[0]}, {frontier[1]}) to Robot_{robot_id}'
                    )

    def find_all_frontiers(self):
        """Find all frontier cells in the map"""
        frontiers = []

        for y in range(1, self.grid_size - 1):
            for x in range(1, self.grid_size - 1):
                if self.get_occupancy_value(x, y) != 0:
                    continue  # Only consider free cells

                # Check if it's a frontier (adjacent to unknown)
                has_unknown_neighbor = False
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    if self.get_occupancy_value(x + dx, y + dy) == -1:
                        has_unknown_neighbor = True
                        break

                if has_unknown_neighbor:
                    # Check if it's safe and not already visited
                    if self.is_safe_position(x, y) and (x, y) not in self.visited_frontiers:
                        frontiers.append((x, y))

        return frontiers

    def assign_frontiers_to_robots(self, robot_ids, frontiers):
        """Assign frontiers to robots using a cost-based approach"""
        assignments = {}
        assigned_frontiers = set()

        # Calculate costs for each robot-frontier pair
        costs = defaultdict(lambda: float('inf'))

        for robot_id in robot_ids:
            if self.robot_positions[robot_id] is None:
                continue

            rx, ry = self.robot_positions[robot_id]

            for frontier in frontiers:
                if frontier in assigned_frontiers:
                    continue

                fx, fy = frontier

                # Skip if too close or too far
                distance = math.sqrt((fx - rx) ** 2 + (fy - ry) ** 2)
                if distance < self.min_frontier_distance or distance > self.max_frontier_distance:
                    continue

                # Calculate cost
                cost = self.calculate_frontier_cost(robot_id, frontier, assignments)
                costs[(robot_id, frontier)] = cost

        # Greedy assignment - assign best frontier to each robot
        for robot_id in robot_ids:
            best_frontier = None
            best_cost = float('inf')

            for frontier in frontiers:
                if frontier in assigned_frontiers:
                    continue

                cost = costs.get((robot_id, frontier), float('inf'))
                if cost < best_cost:
                    best_cost = cost
                    best_frontier = frontier

            if best_frontier is not None:
                assignments[robot_id] = best_frontier
                assigned_frontiers.add(best_frontier)

        return assignments

    def calculate_frontier_cost(self, robot_id, frontier, current_assignments):
        """Calculate cost for assigning a frontier to a robot"""
        if self.robot_positions[robot_id] is None:
            return float('inf')

        rx, ry = self.robot_positions[robot_id]
        fx, fy = frontier

        # Base cost is distance
        distance = math.sqrt((fx - rx) ** 2 + (fy - ry) ** 2)
        cost = distance

        # Penalize if other robots are already heading to nearby frontiers
        for other_robot, other_frontier in current_assignments.items():
            if other_robot != robot_id:
                ofx, ofy = other_frontier
                frontier_distance = math.sqrt((fx - ofx) ** 2 + (fy - ofy) ** 2)
                if frontier_distance < 10:  # Too close to another assignment
                    cost += 50 / (frontier_distance + 1)

        # Bonus for unexplored areas (more unknown cells nearby)
        unknown_count = self.count_unknown_neighbors(fx, fy, radius=5)
        cost -= unknown_count * 0.5

        # Small random factor to break ties
        cost += random.uniform(0, 2)

        return cost

    def count_unknown_neighbors(self, x, y, radius=5):
        """Count unknown cells in a radius around a position"""
        count = 0
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    if self.get_occupancy_value(nx, ny) == -1:
                        count += 1
        return count

    def publish_waypoints(self):
        """Publish next waypoints for robots following paths"""
        with self.planning_lock:
            for robot_id in self.robot_ids:
                if not self.robot_paths[robot_id] or self.robot_positions[robot_id] is None:
                    continue

                rx, ry = self.robot_positions[robot_id]
                path = self.robot_paths[robot_id]

                # Find next waypoint on path
                next_waypoint = None
                for i, (wx, wy) in enumerate(path):
                    distance = math.sqrt((wx - rx) ** 2 + (wy - ry) ** 2)
                    if distance > 2:  # cells
                        next_waypoint = (wx, wy)
                        break

                if next_waypoint:
                    waypoint_msg = Point()
                    waypoint_msg.x = float(next_waypoint[0])
                    waypoint_msg.y = float(next_waypoint[1])
                    waypoint_msg.z = 0.0
                    self.path_publishers[robot_id].publish(waypoint_msg)

    def plan_path(self, start, goal):
        """A* path planning"""
        if not start or not goal:
            return None

        sx, sy = start
        gx, gy = goal

        open_set = []
        closed_set = set()
        came_from = {}
        g_score = {start: 0}

        def heuristic(pos):
            return math.sqrt((pos[0] - gx) ** 2 + (pos[1] - gy) ** 2)

        heapq.heappush(open_set, (heuristic(start), 0, start))

        while open_set:
            _, current_g, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return self.smooth_path(path)

            if current in closed_set:
                continue

            closed_set.add(current)
            cx, cy = current

            # Check 8 neighbors
            for dx, dy in [(-1, -1), (-1, 0), (-1, 1), (0, -1),
                           (0, 1), (1, -1), (1, 0), (1, 1)]:
                nx, ny = cx + dx, cy + dy
                neighbor = (nx, ny)

                # Check bounds
                if not (0 <= nx < self.grid_size and 0 <= ny < self.grid_size):
                    continue

                # Check if occupied
                if self.get_occupancy_value(nx, ny) == 1:
                    continue

                if neighbor in closed_set:
                    continue

                # Calculate cost
                move_cost = math.sqrt(2) if dx != 0 and dy != 0 else 1
                tentative_g = g_score[current] + move_cost

                # Add penalty for being close to obstacles
                obstacle_penalty = self.calculate_obstacle_penalty(nx, ny)
                tentative_g += obstacle_penalty

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        return None

    def smooth_path(self, path):
        """Remove unnecessary waypoints from path"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            farthest_idx = current_idx + 1

            for next_idx in range(current_idx + 2, len(path)):
                if self.is_line_clear(path[current_idx], path[next_idx]):
                    farthest_idx = next_idx
                else:
                    break

            smoothed.append(path[farthest_idx])
            current_idx = farthest_idx

        return smoothed

    def is_line_clear(self, start, end):
        """Check if line between two points is clear of obstacles"""
        x0, y0 = start
        x1, y1 = end

        # Bresenham's algorithm
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        dx *= 2
        dy *= 2

        while True:
            # Check current cell and neighbors
            for check_dx in [-1, 0, 1]:
                for check_dy in [-1, 0, 1]:
                    check_x = x + check_dx
                    check_y = y + check_dy
                    if 0 <= check_x < self.grid_size and 0 <= check_y < self.grid_size:
                        if self.get_occupancy_value(check_x, check_y) == 1:
                            return False

            if x == x1 and y == y1:
                break

            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx

        return True

    def calculate_obstacle_penalty(self, x, y):
        """Calculate penalty for being near obstacles"""
        penalty = 0.0
        check_radius = 2

        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    if self.get_occupancy_value(nx, ny) == 1:
                        distance = math.sqrt(dx * dx + dy * dy)
                        if distance > 0:
                            penalty += 2.0 / distance

        return penalty

    def is_safe_position(self, x, y):
        """Check if position is safe (not near obstacles)"""
        safe_distance = 3

        for dx in range(-safe_distance, safe_distance + 1):
            for dy in range(-safe_distance, safe_distance + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    if self.get_occupancy_value(nx, ny) == 1:
                        return False
        return True

    def get_occupancy_value(self, x, y):
        """Get occupancy value: 1=occupied, 0=free, -1=unknown"""
        if self.occupancy_grid is None:
            return -1

        if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
            return -1

        value = self.occupancy_grid[y, x]

        if value == -1:
            return -1
        elif value == 0:
            return 0
        else:
            return 1


def main(args=None):
    rclpy.init(args=args)

    # Initialize planner for three robots
    robot_ids = [0, 1]
    node = MultiRobotPlanner(robot_ids)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()