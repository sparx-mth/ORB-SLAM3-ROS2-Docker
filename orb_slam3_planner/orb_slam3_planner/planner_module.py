import math
import heapq


class FrontierPlanner:
    """
    FrontierPlanner is responsible for identifying unexplored frontiers in the occupancy grid
    and selecting optimal exploration targets based on distance, novelty, information gain,
    and alignment with the robot's heading.
    """

    def __init__(self, node):
        """
        Initialize the FrontierPlanner with access to shared state from the main node.

        Args:
            node (rclpy.node.Node): The central node providing shared environment and robot data.
        """
        self.node = node
        self.get_logger = node.get_logger
        self.get_occupancy_value = node.get_occupancy_value
        self.normalize_angle = node.normalize_angle

        self.grid_size = node.grid_size
        self.occupied_threshold = node.occupied_threshold

        self.get_logger().info("FrontierPlanner initialized")

    def find_frontiers(self):
        """
        Scan the occupancy grid to identify frontiers—cells that are known to be free but border unknown space.

        Returns:
            list of (int, int): Grid coordinates of frontier cells.
        """
        frontiers = []

        for y in range(1, self.grid_size - 1):
            for x in range(1, self.grid_size - 1):
                if self.get_occupancy_value(x, y) != 0:
                    continue  # Only consider free cells

                has_unknown_neighbor = False
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    if self.get_occupancy_value(x + dx, y + dy) == -1:
                        has_unknown_neighbor = True
                        break

                if has_unknown_neighbor:
                    frontiers.append((x, y))

        return frontiers

    def find_best_frontier(self):
        """
        Select the optimal frontier using a multi-factor scoring system.
        Returns None if no valid frontiers are found.

        Returns:
            (int, int) or None: Grid coordinates of the best frontier.
        """

        if not self.node.robot_pos:
            return None

        frontiers = self.find_frontiers()

        if not frontiers:
            self.get_logger().warn("No frontiers found — stopping exploration")
            return None

        rx, ry = self.node.robot_pos
        best_frontier = None
        best_score = float('inf')

        for fx, fy in frontiers:
            if not self.is_safe_position(fx, fy):
                continue

            if not self.is_reachable(fx, fy):
                continue

            distance = math.sqrt((fx - rx) ** 2 + (fy - ry) ** 2)
            if distance < 3:
                continue
            if distance > 30:
                continue

            score = self.calculate_frontier_score(fx, fy, rx, ry)

            if score < best_score:
                best_score = score
                best_frontier = (fx, fy)

        return best_frontier

    def find_nearest_frontier(self):
        """
        Wrapper function to find a frontier based either on scoring or pure distance.

        Returns:
            (int, int) or None: Grid coordinates of the chosen frontier.
        """

        if self.node.use_frontier_scoring:
            return self.find_best_frontier()

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

            if distance < 3:
                continue

            if distance > 20:
                continue

            if distance < min_distance:
                min_distance = distance
                best_frontier = (fx, fy)

        return best_frontier

    def calculate_frontier_score(self, fx, fy, rx, ry):
        """
        Compute a score for a given frontier based on distance, novelty, surrounding unknowns, and heading alignment.

        Returns:
            float: Calculated score for the frontier.
        """
        distance = math.sqrt((fx - rx) ** 2 + (fy - ry) ** 2)

        novelty_bonus = 1.0
        for vx, vy in self.node.visited_targets:
            if math.sqrt((fx - vx) ** 2 + (fy - vy) ** 2) < self.node.exploration_radius:
                novelty_bonus = 0.3
                break

        unknown_count = 0
        check_radius = 3
        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                nx, ny = fx + dx, fy + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    if self.get_occupancy_value(nx, ny) == -1:
                        unknown_count += 1

        angle_to_frontier = math.atan2(fy - ry, fx - rx)
        angle_diff = abs(self.normalize_angle(angle_to_frontier - self.node.robot_angle))
        angle_factor = 1.0 - (angle_diff / math.pi) * 0.3

        score = distance / novelty_bonus / (1 + unknown_count * 0.1) / angle_factor
        return score

    def is_safe_position(self, x, y):
        """
        Determine if the specified position is safe by checking for nearby obstacles.

        Returns:
            bool: True if the area is free from obstacles; False otherwise.
        """
        # Use temporal map if SLAM is lost
        prob_grid = self.node.temporal_occupancy_prob if self.node.using_temporal_map else self.node.occupancy_prob

        for dx in range(-self.node.safe_distance, self.node.safe_distance + 1):
            for dy in range(-self.node.safe_distance, self.node.safe_distance + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    if prob_grid[ny, nx] > self.occupied_threshold:
                        return False
        return True

    def is_reachable(self, tx, ty):
        """
        Check if there is a valid path from the robot's current position to (tx, ty) using A*.

        Returns:
            bool: True if a path exists, False otherwise.
        """
        if not self.node.robot_pos:
            return False

        path = self.plan_path(self.node.robot_pos, (tx, ty))
        return path is not None

    def plan_path(self, start, goal):
        """
        Plan a path from start to goal using A* algorithm.

        Args:
            start (tuple): (x, y) starting position in grid coordinates
            goal (tuple): (x, y) goal position in grid coordinates

        Returns:
            list: List of (x, y) waypoints from start to goal, or None if no path exists
        """
        if not start or not goal:
            return None

        sx, sy = start
        gx, gy = goal

        # Use temporal map if SLAM is lost
        grid = self.node.temporal_occupancy_prob if self.node.using_temporal_map else self.node.occupancy_prob

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
                smoothed_path = self.smooth_path(path, grid)
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
                if grid[ny, nx] > self.occupied_threshold:
                    continue

                # Check if already visited
                if neighbor in closed_set:
                    continue

                # Calculate tentative g score
                # Diagonal moves cost sqrt(2), straight moves cost 1
                move_cost = math.sqrt(2) if dx != 0 and dy != 0 else 1
                tentative_g = g_score[current] + move_cost

                # Add penalty for being close to obstacles
                obstacle_penalty = self.calculate_obstacle_penalty(nx, ny, grid)
                tentative_g += obstacle_penalty

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        return None  # No path found

    def calculate_obstacle_penalty(self, x, y, grid):
        """
        Calculate a penalty for being close to obstacles to encourage safer paths.

        Args:
            x, y: Grid coordinates
            grid: Occupancy probability grid

        Returns:
            float: Penalty value
        """
        penalty = 0.0
        check_radius = 2

        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    if grid[ny, nx] > self.occupied_threshold:
                        distance = math.sqrt(dx * dx + dy * dy)
                        if distance > 0:
                            penalty += 2.0 / distance

        return penalty

    def smooth_path(self, path, grid):
        """
        Smooth the path by removing unnecessary waypoints while ensuring the path remains collision-free.

        Args:
            path: List of waypoints
            grid: Occupancy probability grid

        Returns:
            list: Smoothed path
        """
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        current_idx = 0

        while current_idx < len(path) - 1:
            # Try to skip ahead as far as possible
            farthest_idx = current_idx + 1

            for next_idx in range(current_idx + 2, len(path)):
                if self.is_line_clear(path[current_idx], path[next_idx], grid):
                    farthest_idx = next_idx
                else:
                    break

            smoothed.append(path[farthest_idx])
            current_idx = farthest_idx

        return smoothed

    def is_line_clear(self, start, end, grid):
        """
        Check if a straight line between two points is clear of obstacles.

        Args:
            start: (x, y) starting position
            end: (x, y) ending position
            grid: Occupancy probability grid

        Returns:
            bool: True if line is clear
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
                        if grid[check_y, check_x] > self.occupied_threshold:
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