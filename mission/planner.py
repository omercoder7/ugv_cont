"""
A* path planner for navigation using occupancy grid data.
"""

import math
import time
import heapq
from typing import List, Tuple, Dict, Optional

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    np = None


class AStarPlanner:
    """
    A* path planner for smarter navigation using occupancy grid data.
    """

    FREE = 0
    OCCUPIED = 1
    UNKNOWN = 2

    def __init__(self, resolution: float = 0.1, robot_radius: float = 0.2):
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.robot_cells = int(math.ceil(robot_radius / resolution))

        self.current_path: List[Tuple[float, float]] = []
        self.path_goal: Optional[Tuple[float, float]] = None
        self.path_timestamp: float = 0.0
        self.path_cache_ttl: float = 2.0

        self.grid: Optional['np.ndarray'] = None
        self.grid_origin: Tuple[float, float] = (0.0, 0.0)
        self.grid_resolution: float = 0.05

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid cell indices."""
        gx = int((x - self.grid_origin[0]) / self.grid_resolution)
        gy = int((y - self.grid_origin[1]) / self.grid_resolution)
        return (gx, gy)

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid cell indices to world coordinates."""
        x = gx * self.grid_resolution + self.grid_origin[0] + self.grid_resolution / 2
        y = gy * self.grid_resolution + self.grid_origin[1] + self.grid_resolution / 2
        return (x, y)

    def update_grid(self, occupancy_grid: 'np.ndarray', origin: Tuple[float, float],
                    resolution: float):
        """Update the internal grid from SLAM occupancy grid."""
        if HAS_NUMPY:
            self.grid = occupancy_grid.copy()
        self.grid_origin = origin
        self.grid_resolution = resolution

    def is_cell_free(self, gx: int, gy: int) -> bool:
        """Check if a grid cell and surrounding area is free for robot."""
        if self.grid is None:
            return True

        h, w = self.grid.shape

        for dx in range(-self.robot_cells, self.robot_cells + 1):
            for dy in range(-self.robot_cells, self.robot_cells + 1):
                if dx*dx + dy*dy <= self.robot_cells * self.robot_cells:
                    nx, ny = gx + dx, gy + dy
                    if nx < 0 or nx >= w or ny < 0 or ny >= h:
                        return False
                    cell_val = self.grid[ny, nx]
                    if cell_val > 50:
                        return False
        return True

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """A* heuristic: Euclidean distance."""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[Tuple[int, int], float]]:
        """Get valid neighboring cells with movement cost."""
        gx, gy = pos
        neighbors = []

        directions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]

        for dx, dy, cost in directions:
            nx, ny = gx + dx, gy + dy
            if self.is_cell_free(nx, ny):
                neighbors.append(((nx, ny), cost))

        return neighbors

    def find_path(self, start: Tuple[float, float],
                  goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """Find path from start to goal using A*."""
        if self.grid is None:
            return []

        start_cell = self.world_to_grid(*start)
        goal_cell = self.world_to_grid(*goal)

        if not self.is_cell_free(*start_cell):
            return []
        if not self.is_cell_free(*goal_cell):
            return []

        open_set = []
        heapq.heappush(open_set, (0, start_cell))

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {start_cell: 0}
        f_score: Dict[Tuple[int, int], float] = {start_cell: self.heuristic(start_cell, goal_cell)}

        max_iterations = 10000
        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1
            current = heapq.heappop(open_set)[1]

            if current == goal_cell:
                path = []
                while current in came_from:
                    path.append(self.grid_to_world(*current))
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for neighbor, cost in self.get_neighbors(current):
                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_cell)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def get_next_waypoint(self, current_pos: Tuple[float, float],
                          goal: Tuple[float, float],
                          lookahead: float = 0.5) -> Optional[Tuple[float, float]]:
        """Get the next waypoint to navigate toward."""
        now = time.time()
        if (self.path_goal != goal or
            now - self.path_timestamp > self.path_cache_ttl or
            not self.current_path):
            self.current_path = self.find_path(current_pos, goal)
            self.path_goal = goal
            self.path_timestamp = now

        if not self.current_path:
            return None

        accumulated_dist = 0.0
        prev_point = current_pos

        for waypoint in self.current_path:
            dist = math.sqrt((waypoint[0] - prev_point[0])**2 +
                           (waypoint[1] - prev_point[1])**2)
            accumulated_dist += dist

            if accumulated_dist >= lookahead:
                return waypoint

            prev_point = waypoint

        return self.current_path[-1] if self.current_path else None

    def get_direction_to_waypoint(self, current_pos: Tuple[float, float],
                                  current_heading: float,
                                  waypoint: Tuple[float, float]) -> int:
        """Convert waypoint to sector direction."""
        dx = waypoint[0] - current_pos[0]
        dy = waypoint[1] - current_pos[1]
        world_angle = math.atan2(dy, dx)

        relative_angle = world_angle - current_heading

        while relative_angle > math.pi:
            relative_angle -= 2 * math.pi
        while relative_angle < -math.pi:
            relative_angle += 2 * math.pi

        sector = int(round(relative_angle / (math.pi / 6))) % 12

        return sector

    def clear_path(self):
        """Clear cached path."""
        self.current_path = []
        self.path_goal = None
