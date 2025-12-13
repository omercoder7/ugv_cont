"""
Virtual obstacle tracking for invisible obstacles.

Tracks obstacles that the robot has physically encountered but LiDAR cannot see
(e.g., glass walls, low objects, narrow gaps).
"""

import math
import time
from typing import List, Tuple, Optional

from .ros_interface import publish_virtual_obstacle


class VirtualObstacleTracker:
    """
    Tracks virtual obstacles that the robot has encountered but LiDAR cannot detect.

    Each obstacle entry is (x, y, timestamp, confidence).
    Confidence increases when robot repeatedly gets stuck at same location.
    """

    def __init__(self, radius: float = 0.12, max_age: float = 300.0, max_count: int = 20):
        """
        Args:
            radius: Obstacle radius for clearance calculations (meters)
            max_age: Forget obstacles after this time (seconds)
            max_count: Maximum stored virtual obstacles
        """
        self.obstacles: List[Tuple[float, float, float, int]] = []
        self.radius = radius
        self.max_age = max_age
        self.max_count = max_count

    def add(self, x: float, y: float, publish: bool = True):
        """
        Add a virtual obstacle at the given position.

        Args:
            x, y: World coordinates of the obstacle
            publish: Whether to publish marker for RViz visualization
        """
        now = time.time()

        # Check if there's already a virtual obstacle nearby (avoid duplicates)
        for i, (ox, oy, ts, conf) in enumerate(self.obstacles):
            dist = math.sqrt((x - ox)**2 + (y - oy)**2)
            if dist < 0.3:  # Within 30cm - update existing
                self.obstacles[i] = (ox, oy, now, min(conf + 1, 5))
                print(f"\n[VIRTUAL-OBS] Reinforced obstacle at ({ox:.2f}, {oy:.2f}), conf={conf+1}")
                return

        # Add new virtual obstacle
        self.obstacles.append((x, y, now, 1))
        print(f"\n[VIRTUAL-OBS] Added new obstacle at ({x:.2f}, {y:.2f})")

        # Limit total count
        if len(self.obstacles) > self.max_count:
            self.obstacles.sort(key=lambda o: o[2])  # Sort by timestamp
            self.obstacles = self.obstacles[-self.max_count:]

        # Publish for visualization
        if publish:
            publish_virtual_obstacle(x, y)
            print(f"\n[OBSTACLE] Marked virtual obstacle at ({x:.2f}, {y:.2f})")

    def cleanup(self):
        """Remove old virtual obstacles"""
        now = time.time()
        self.obstacles = [
            (x, y, ts, conf) for x, y, ts, conf in self.obstacles
            if now - ts < self.max_age
        ]

    def get_clearance(self, robot_x: float, robot_y: float, robot_heading: float) -> float:
        """
        Get the minimum clearance to any virtual obstacle in front of the robot.

        Args:
            robot_x, robot_y: Robot position in world frame
            robot_heading: Robot heading in radians

        Returns:
            Distance to nearest virtual obstacle in front arc, or inf if none
        """
        if not self.obstacles or robot_x is None:
            return float('inf')

        min_clearance = float('inf')

        for ox, oy, ts, conf in self.obstacles:
            dx = ox - robot_x
            dy = oy - robot_y
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < 0.1:  # Too close to matter
                continue

            # Check if obstacle is in front of robot (within ±60 degrees)
            angle_to_obs = math.atan2(dy, dx)
            angle_diff = angle_to_obs - robot_heading

            # Normalize to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # Check if in front arc (±60 degrees = ±1.05 radians)
            if abs(angle_diff) < 1.05:
                clearance = dist - self.radius
                if clearance < min_clearance:
                    min_clearance = clearance

        return min_clearance

    def mark(self, x: float, y: float):
        """Publish a virtual obstacle marker for visualization only (no tracking)."""
        publish_virtual_obstacle(x, y)
        print(f"\n[OBSTACLE] Marked virtual obstacle at ({x:.2f}, {y:.2f})")
