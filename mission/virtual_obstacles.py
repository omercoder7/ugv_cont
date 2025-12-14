"""
Virtual obstacle management for invisible obstacles.

When robot detects it's stuck (moving but position not changing),
it places a virtual obstacle that gets treated like a real one.
"""

import time
import math
from typing import List, Tuple, Optional


class VirtualObstacle:
    """Virtual obstacle placed when robot detects invisible obstacle."""

    def __init__(self, x: float, y: float, radius: float = 0.3, ttl: float = 120.0):
        """
        Create virtual obstacle.

        Args:
            x, y: Position in world coordinates
            radius: Obstacle radius for avoidance
            ttl: Time to live in seconds (expires after this)
        """
        self.x = x
        self.y = y
        self.radius = radius
        self.created_at = time.time()
        self.ttl = ttl

    def is_expired(self) -> bool:
        """Check if obstacle has expired."""
        return time.time() - self.created_at > self.ttl

    def distance_to(self, x: float, y: float) -> float:
        """Get distance from point to obstacle center."""
        return math.hypot(x - self.x, y - self.y)

    def __repr__(self):
        age = time.time() - self.created_at
        return f"VirtualObstacle(({self.x:.2f}, {self.y:.2f}), r={self.radius}, age={age:.0f}s)"


class VirtualObstacleManager:
    """Manages virtual obstacles for invisible obstacle detection."""

    def __init__(self, default_radius: float = 0.3, default_ttl: float = 120.0):
        self.obstacles: List[VirtualObstacle] = []
        self.default_radius = default_radius
        self.default_ttl = default_ttl

    def add_obstacle(self, x: float, y: float, heading: float,
                     distance_ahead: float = 0.3) -> Optional[VirtualObstacle]:
        """
        Add virtual obstacle in front of robot.

        Args:
            x, y: Robot position
            heading: Robot heading in radians
            distance_ahead: How far ahead to place obstacle

        Returns:
            New obstacle if added, None if duplicate
        """
        # Place obstacle ahead of robot
        obs_x = x + distance_ahead * math.cos(heading)
        obs_y = y + distance_ahead * math.sin(heading)

        # Check for nearby existing obstacle
        for obs in self.obstacles:
            if obs.distance_to(obs_x, obs_y) < self.default_radius:
                return None  # Already have one here

        new_obs = VirtualObstacle(obs_x, obs_y, self.default_radius, self.default_ttl)
        self.obstacles.append(new_obs)
        return new_obs

    def cleanup_expired(self):
        """Remove expired obstacles."""
        before = len(self.obstacles)
        self.obstacles = [o for o in self.obstacles if not o.is_expired()]
        removed = before - len(self.obstacles)
        if removed > 0:
            print(f"[VIRTUAL] Removed {removed} expired obstacles")

    def get_nearest_in_front(self, x: float, y: float, heading: float,
                              fov_angle: float = math.pi / 3) -> float:
        """
        Get distance to nearest virtual obstacle in front of robot.

        Args:
            x, y: Robot position
            heading: Robot heading in radians
            fov_angle: Field of view angle (default 60 degrees each side)

        Returns:
            Distance to nearest obstacle, or inf if none in front
        """
        self.cleanup_expired()

        min_dist = float('inf')

        for obs in self.obstacles:
            # Angle from robot to obstacle
            angle_to_obs = math.atan2(obs.y - y, obs.x - x)

            # Angle difference (handle wraparound)
            angle_diff = abs(angle_to_obs - heading)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff

            # Check if in front
            if angle_diff < fov_angle:
                dist = obs.distance_to(x, y) - obs.radius
                min_dist = min(min_dist, max(0, dist))

        return min_dist

    def count(self) -> int:
        """Get number of active obstacles."""
        return len(self.obstacles)

    def clear(self):
        """Remove all obstacles."""
        self.obstacles.clear()

    def get_all(self) -> List[VirtualObstacle]:
        """Get all active obstacles."""
        self.cleanup_expired()
        return self.obstacles.copy()
