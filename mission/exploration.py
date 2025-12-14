"""
Exploration tracking for efficient scanning.

Tracks visited cells and provides exploration bias to avoid revisiting.
Also handles stuck detection for invisible obstacles.
"""

import time
import math
from typing import Dict, Tuple, List, Optional


class StuckDetector:
    """Detects when robot is stuck (commanded to move but not moving)."""

    def __init__(self, threshold: float = 0.03, time_window: float = 2.0):
        """
        Args:
            threshold: Minimum movement in meters to not be stuck
            time_window: Time window to check movement over
        """
        self.threshold = threshold
        self.time_window = time_window
        self.positions: List[Tuple[float, float, float]] = []  # (x, y, time)

    def update(self, x: float, y: float) -> bool:
        """
        Update with current position and check if stuck.

        Returns:
            True if robot appears stuck
        """
        now = time.time()
        self.positions.append((x, y, now))

        # Keep only recent positions
        self.positions = [(px, py, t) for px, py, t in self.positions
                          if now - t < self.time_window]

        if len(self.positions) < 3:
            return False

        # Check movement from oldest to newest
        oldest = self.positions[0]
        dist = math.hypot(x - oldest[0], y - oldest[1])
        time_diff = now - oldest[2]

        # Stuck if minimal movement over time window
        return dist < self.threshold and time_diff > 1.0

    def reset(self):
        """Reset stuck detection."""
        self.positions.clear()


class ExplorationTracker:
    """Tracks visited cells and provides exploration bias."""

    def __init__(self, grid_resolution: float = 0.5):
        """
        Args:
            grid_resolution: Size of grid cells in meters
        """
        self.grid_res = grid_resolution
        self.visited: Dict[Tuple[int, int], int] = {}
        self.stuck_detector = StuckDetector()

    def pos_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world position to grid cell."""
        return (int(x / self.grid_res), int(y / self.grid_res))

    def cell_to_pos(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        """Convert grid cell to world position (center of cell)."""
        return (cell[0] * self.grid_res + self.grid_res / 2,
                cell[1] * self.grid_res + self.grid_res / 2)

    def record_visit(self, x: float, y: float) -> int:
        """
        Record a visit to position.

        Returns:
            Visit count for this cell
        """
        cell = self.pos_to_cell(x, y)
        self.visited[cell] = self.visited.get(cell, 0) + 1
        return self.visited[cell]

    def get_visit_count(self, x: float, y: float) -> int:
        """Get visit count for a position."""
        cell = self.pos_to_cell(x, y)
        return self.visited.get(cell, 0)

    def check_stuck(self, x: float, y: float) -> bool:
        """Check if robot is stuck at current position."""
        return self.stuck_detector.update(x, y)

    def reset_stuck(self):
        """Reset stuck detection after recovery."""
        self.stuck_detector.reset()

    def get_exploration_bias(self, x: float, y: float, heading: float,
                              sectors: List[float]) -> float:
        """
        Get angular velocity bias toward less-visited areas.

        Args:
            x, y: Robot position
            heading: Robot heading in radians
            sectors: LiDAR sector distances

        Returns:
            Angular velocity bias (positive = left, negative = right)
        """
        left_visits = 0
        right_visits = 0

        # Sample points ~1m away in different directions
        for angle_offset in [0.3, 0.6, 0.9]:  # ~17, 34, 52 degrees
            # Left side
            lx = x + 1.0 * math.cos(heading + angle_offset)
            ly = y + 1.0 * math.sin(heading + angle_offset)
            left_visits += self.get_visit_count(lx, ly)

            # Right side
            rx = x + 1.0 * math.cos(heading - angle_offset)
            ry = y + 1.0 * math.sin(heading - angle_offset)
            right_visits += self.get_visit_count(rx, ry)

        # Bias toward less visited side
        if left_visits < right_visits - 1:
            return 0.15  # Turn left
        elif right_visits < left_visits - 1:
            return -0.15  # Turn right

        return 0.0

    def get_stats(self) -> dict:
        """Get exploration statistics."""
        if not self.visited:
            return {
                'unique_cells': 0,
                'total_visits': 0,
                'revisit_pct': 0.0,
                'most_visited': 0
            }

        unique = len(self.visited)
        total = sum(self.visited.values())
        revisited = sum(1 for v in self.visited.values() if v > 1)
        most = max(self.visited.values())

        return {
            'unique_cells': unique,
            'total_visits': total,
            'revisit_pct': revisited / unique * 100 if unique > 0 else 0,
            'most_visited': most
        }

    def clear(self):
        """Clear all visit history."""
        self.visited.clear()
        self.stuck_detector.reset()
