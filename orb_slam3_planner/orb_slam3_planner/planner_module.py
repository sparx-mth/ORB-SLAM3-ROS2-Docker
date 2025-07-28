import math
import heapq


class FrontierPlanner:
    """
    FrontierPlanner is responsible for identifying unexplored frontiers in the occupancy grid
    and selecting optimal exploration targets based on distance, novelty, information gain,
    alignment with the robot's heading, and coordination with other robots.
    """

    def __init__(self, node):
        """
        Initialize the FrontierPlanner with access to shared state from the main node.
        """
        self.node = node
        self.get_logger = node.get_logger
        self.get_occupancy_value = node.get_occupancy_value
        self.normalize_angle = node.normalize_angle

        self.grid_size = node.grid_size

        self.get_logger().info("FrontierPlanner initialized with multi-robot coordination")

    def find_frontiers(self):
        """
        Scan the occupancy grid to identify frontiers—cells that are known to be free but border unknown space.
        A frontier must be:
        1. A free cell (value = 0)
        2. Adjacent to at least one unknown cell (value = -1)
        3. Not too close to walls/obstacles (value = 100)
        """
        frontiers = []

        # Update grid size in case it changed
        self.grid_size = self.node.grid_size

        # Define minimum distance from walls/obstacles
        min_wall_distance = 2  # cells

        for y in range(min_wall_distance, self.grid_size - min_wall_distance):
            for x in range(min_wall_distance, self.grid_size - min_wall_distance):
                # First check: must be a free cell (0 = free)
                if self.get_occupancy_value(x, y) != 0:
                    continue  # Skip if not a free cell

                # Second check: must have at least one unknown neighbor
                has_unknown_neighbor = False
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                        if self.get_occupancy_value(nx, ny) == -1:
                            has_unknown_neighbor = True
                            break

                if not has_unknown_neighbor:
                    continue  # Skip if no unknown neighbors

                # Third check: must not be too close to walls/obstacles
                too_close_to_wall = False
                for dx in range(-min_wall_distance, min_wall_distance + 1):
                    for dy in range(-min_wall_distance, min_wall_distance + 1):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                            if self.get_occupancy_value(nx, ny) == 100:  # Occupied/wall
                                too_close_to_wall = True
                                break
                    if too_close_to_wall:
                        break

                if not too_close_to_wall:
                    frontiers.append((x, y))

        return frontiers

    def find_best_frontier(self):
        """
        Select the optimal frontier using a multi-factor scoring system including multi-robot coordination.
        """
        if not self.node.robot_pos:
            return None

        frontiers = self.find_frontiers()

        if not frontiers:
            self.get_logger().warn("No frontiers found — starting exploration tour")
            # Return a point for exploration tour
            return self.get_exploration_tour_point()

        rx, ry = self.node.robot_pos
        best_frontier = None
        best_score = float('inf')

        # Log current robot positions and goals for debugging
        if len(self.node.other_robot_positions) > 0 or len(self.node.other_robot_goals) > 0:
            self.get_logger().debug(
                f"Robot {self.node.robot_id} - Other robots: positions={self.node.other_robot_positions}, goals={self.node.other_robot_goals}")

        for fx, fy in frontiers:
            if not self.is_safe_position(fx, fy):
                continue

            # Skip if too close to current position
            distance = math.sqrt((fx - rx) ** 2 + (fy - ry) ** 2)
            if distance < 2:
                continue

            # Calculate comprehensive score
            score = self.calculate_frontier_score(fx, fy, rx, ry)

            if score < best_score:
                best_score = score
                best_frontier = (fx, fy)

        if best_frontier:
            self.get_logger().info(
                f"Robot {self.node.robot_id} selected frontier {best_frontier} with score {best_score:.2f}")
        else:
            # If no good frontier found, try exploration tour
            self.get_logger().info("No suitable frontier found, trying exploration tour")
            return self.get_exploration_tour_point()

        return best_frontier

    def get_exploration_tour_point(self):
        """
        Generate an exploration tour point when no suitable frontier is available.
        This helps robots explore different areas when frontiers are too crowded.
        """
        if not self.node.robot_pos:
            return None

        rx, ry = self.node.robot_pos

        # Calculate a sector for this robot based on its ID
        num_robots = len(self.node.all_robot_ids)
        if num_robots > 1:
            angle_sector = (2 * math.pi * self.node.robot_id) / num_robots
        else:
            angle_sector = 0

        # Add some variation based on time
        time_var = (self.node.get_clock().now().nanoseconds / 1e9) * 0.1
        tour_angle = angle_sector + math.sin(time_var) * 0.5

        # Try multiple distances
        for tour_distance in [8.0, 12.0, 6.0, 15.0]:
            for angle_offset in [0, 0.3, -0.3, 0.6, -0.6]:
                test_angle = tour_angle + angle_offset
                tx = int(rx + tour_distance * math.cos(test_angle))
                ty = int(ry + tour_distance * math.sin(test_angle))

                # Ensure the point is within bounds
                tx = max(5, min(self.node.grid_size - 5, tx))
                ty = max(5, min(self.node.grid_size - 5, ty))

                # Check if it's a valid exploration point (free space)
                if self.get_occupancy_value(tx, ty) == 0 and self.is_safe_position(tx, ty):
                    # Check if far enough from other robots and their goals
                    min_dist_to_others = float('inf')

                    for robot_id, (ox, oy) in self.node.other_robot_positions.items():
                        dist = math.sqrt((tx - ox) ** 2 + (ty - oy) ** 2)
                        min_dist_to_others = min(min_dist_to_others, dist)

                    for robot_id, (gx, gy) in self.node.other_robot_goals.items():
                        dist = math.sqrt((tx - gx) ** 2 + (ty - gy) ** 2)
                        min_dist_to_others = min(min_dist_to_others, dist)

                    if min_dist_to_others > self.node.min_robot_separation:
                        self.get_logger().info(f"Robot {self.node.robot_id} touring to ({tx}, {ty})")
                        return (tx, ty)

        # If no good tour point found, just rotate in place
        self.get_logger().info(f"Robot {self.node.robot_id} will rotate in place")
        return None

    def find_nearest_frontier(self):
        """
        Wrapper function to find a frontier based either on scoring or pure distance.
        """
        if self.node.use_frontier_scoring:
            return self.find_best_frontier()

        # Simple nearest frontier without multi-robot coordination
        if not self.node.robot_pos:
            return None

        frontiers = self.find_frontiers()

        if not frontiers:
            self.get_logger().warn("No frontiers found — exploration halted")
            return None

        rx, ry = self.node.robot_pos
        best_frontier = None
        min_distance = float('inf')

        for fx, fy in frontiers:
            distance = math.sqrt((fx - rx) ** 2 + (fy - ry) ** 2)

            if not self.is_safe_position(fx, fy):
                continue

            if distance < 2:
                continue

            if distance < min_distance:
                min_distance = distance
                best_frontier = (fx, fy)

        return best_frontier

    def calculate_multi_robot_factor(self, fx, fy):
        """
        Calculate a factor that encourages robots to spread out.
        Returns a penalty factor (higher is worse for final score).
        """
        # Start with neutral factor
        separation_penalty = 0.0

        # Check distance to other robot goals (targets)
        # We want to STRONGLY PENALIZE being close to other robot targets
        for robot_id, (gx, gy) in self.node.other_robot_goals.items():
            dist_to_goal = math.sqrt((fx - gx) ** 2 + (fy - gy) ** 2)

            if dist_to_goal < self.node.min_robot_separation * 3:  # Extended range
                # Strong penalty for being close to other targets
                # The closer we are, the worse the penalty
                if dist_to_goal < self.node.min_robot_separation:
                    separation_penalty += 10.0  # Very strong penalty
                else:
                    penalty = 5.0 * (1.0 - (dist_to_goal - self.node.min_robot_separation) / (
                                self.node.min_robot_separation * 2))
                    separation_penalty += penalty

        # Also penalize being close to other robot current positions
        for robot_id, (ox, oy) in self.node.other_robot_positions.items():
            dist_to_robot = math.sqrt((fx - ox) ** 2 + (fy - oy) ** 2)

            if dist_to_robot < self.node.min_robot_separation * 2:
                if dist_to_robot < self.node.min_robot_separation:
                    separation_penalty += 5.0  # Strong penalty
                else:
                    penalty = 2.0 * (
                                1.0 - (dist_to_robot - self.node.min_robot_separation) / self.node.min_robot_separation)
                    separation_penalty += penalty

        return 1.0 + separation_penalty  # Return as multiplicative factor

    def calculate_frontier_score(self, fx, fy, rx, ry):
        """
        Compute a score for a given frontier based on multiple factors.
        Lower score is better.
        """
        # Distance factor - closer is better
        distance = math.sqrt((fx - rx) ** 2 + (fy - ry) ** 2)

        # Novelty factor - unvisited areas are better
        novelty_bonus = 1.0
        for vx, vy in self.node.visited_targets:
            if math.sqrt((fx - vx) ** 2 + (fy - vy) ** 2) < self.node.exploration_radius * 2:
                novelty_bonus = 0.3
                break

        # Information gain factor (unknown cells nearby)
        unknown_count = 0
        check_radius = 3
        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                nx, ny = fx + dx, fy + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    if self.get_occupancy_value(nx, ny) == -1:
                        unknown_count += 1

        information_gain = 1.0 + unknown_count * 0.05  # More unknown cells = better

        # Heading alignment factor
        angle_to_frontier = math.atan2(fy - ry, fx - rx)
        angle_diff = abs(self.normalize_angle(angle_to_frontier - self.node.robot_angle))
        angle_factor = 1.0 + (angle_diff / math.pi) * 0.3  # Smaller angle difference = better

        # Multi-robot coordination factor
        multi_robot_factor = self.calculate_multi_robot_factor(fx, fy)

        # Combined score (lower is better)
        base_score = distance * angle_factor / (novelty_bonus * information_gain)
        final_score = base_score * multi_robot_factor

        # Debug logging for significant coordination effects
        if multi_robot_factor > 2.0:
            self.get_logger().debug(
                f"Frontier ({fx}, {fy}): distance={distance:.1f}, base_score={base_score:.2f}, "
                f"multi_robot_factor={multi_robot_factor:.2f}, final_score={final_score:.2f}"
            )

        return final_score

    def is_safe_position(self, x, y):
        """
        Determine if the specified position is safe by checking for nearby obstacles.
        This is used for validating frontier positions and path planning.
        """
        # Must be a free cell
        if self.get_occupancy_value(x, y) != 0:
            return False

        # Use the same safe distance from the main node
        safe_distance = self.node.safe_distance

        for dx in range(-safe_distance, safe_distance + 1):
            for dy in range(-safe_distance, safe_distance + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    if self.get_occupancy_value(nx, ny) == 100:  # Occupied
                        return False
        return True

    def is_reachable(self, tx, ty):
        """
        Check if there is a valid path from the robot's current position to (tx, ty) using A*.
        """
        if not self.node.robot_pos:
            return False

        path = self.plan_path(self.node.robot_pos, (tx, ty))
        return path is not None and len(path) > 1

    def plan_path(self, start, goal):
        """
        Plan a path from start to goal using A* algorithm.
        """
        if not start or not goal:
            return None

        sx, sy = start
        gx, gy = goal

        # Check if goal is valid
        if self.get_occupancy_value(gx, gy) == 100:  # Goal is occupied
            return None

        # A* implementation
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

                # Smooth path by removing unnecessary waypoints
                smoothed_path = self.smooth_path(path)
                return smoothed_path

            if current in closed_set:
                continue

            closed_set.add(current)
            cx, cy = current

            # Check all 8 neighbors for smoother paths
            for dx, dy in [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]:
                nx, ny = cx + dx, cy + dy
                neighbor = (nx, ny)

                # Check bounds
                if not (0 <= nx < self.grid_size and 0 <= ny < self.grid_size):
                    continue

                # Check if occupied
                if self.node.get_occupancy_value(nx, ny) == 100:  # Occupied
                    continue

                # Check if already visited
                if neighbor in closed_set:
                    continue

                # Calculate tentative g score
                move_cost = math.sqrt(2) if dx != 0 and dy != 0 else 1
                tentative_g = g_score[current] + move_cost

                # Add penalty for being close to obstacles
                obstacle_penalty = self.calculate_obstacle_penalty(nx, ny)
                tentative_g += obstacle_penalty

                # Add penalty for unknown cells (prefer known paths)
                if self.node.get_occupancy_value(nx, ny) == -1:
                    tentative_g += 0.5

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        return None  # No path found

    def calculate_obstacle_penalty(self, x, y):
        """
        Calculate a penalty for being close to obstacles to encourage safer paths.
        """
        penalty = 0.0
        check_radius = 2

        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    if self.node.get_occupancy_value(nx, ny) == 100:  # Occupied
                        distance = math.sqrt(dx * dx + dy * dy)
                        if distance > 0:
                            penalty += 2.0 / distance

        return penalty

    def smooth_path(self, path):
        """
        Smooth the path by removing unnecessary waypoints while ensuring the path remains collision-free.
        """
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            # Try to skip ahead as far as possible
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
        """
        Check if a straight line between two points is clear of obstacles.
        """
        x0, y0 = start
        x1, y1 = end

        # Use Bresenham's algorithm to check all cells along the line
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        dx *= 2
        dy *= 2

        while True:
            # Check current cell and neighbors for safety margin
            for check_dx in [-1, 0, 1]:
                for check_dy in [-1, 0, 1]:
                    check_x = x + check_dx
                    check_y = y + check_dy
                    if 0 <= check_x < self.grid_size and 0 <= check_y < self.grid_size:
                        if self.node.get_occupancy_value(check_x, check_y) == 100:  # Occupied
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