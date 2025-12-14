"""
Waypoint navigation for exploration.

Places invisible goal points at the furthest free distance,
robot steers toward these goals for efficient exploration.
"""

import math
import time
from typing import Optional, Tuple, List


class Waypoint:
    """A navigation goal point."""

    def __init__(self, x: float, y: float, created_at: Optional[float] = None):
        self.x = x
        self.y = y
        self.created_at = created_at or time.time()

    def distance_to(self, x: float, y: float) -> float:
        """Distance from point to waypoint."""
        return math.hypot(x - self.x, y - self.y)

    def angle_from(self, x: float, y: float) -> float:
        """Angle from point to waypoint in radians."""
        return math.atan2(self.y - y, self.x - x)

    def __repr__(self):
        return f"Waypoint({self.x:.2f}, {self.y:.2f})"


class WaypointNavigator:
    """
    Manages waypoints for exploration.

    Places waypoints at furthest free distance, robot steers toward them.
    """

    def __init__(self, reach_threshold: float = 0.5, min_waypoint_dist: float = 1.0):
        """
        Args:
            reach_threshold: Distance to consider waypoint reached
            min_waypoint_dist: Minimum distance to place new waypoint
        """
        self.current_waypoint: Optional[Waypoint] = None
        self.reach_threshold = reach_threshold
        self.min_waypoint_dist = min_waypoint_dist
        self.waypoints_reached = 0

    def update_waypoint(self, robot_x: float, robot_y: float, robot_heading: float,
                        sectors: List[float], num_sectors: int = 12) -> Optional[Waypoint]:
        """
        Update waypoint based on current sensor data.

        Places waypoint at furthest free distance if needed.

        Args:
            robot_x, robot_y: Robot position
            robot_heading: Robot heading in radians
            sectors: LiDAR sector distances
            num_sectors: Number of sectors

        Returns:
            Current waypoint (may be new or existing)
        """
        # Check if current waypoint is reached
        if self.current_waypoint:
            dist = self.current_waypoint.distance_to(robot_x, robot_y)
            if dist < self.reach_threshold:
                print(f"\n[WAYPOINT] Reached ({self.current_waypoint.x:.1f}, {self.current_waypoint.y:.1f})")
                self.waypoints_reached += 1
                self.current_waypoint = None

        # Create new waypoint if none exists
        if self.current_waypoint is None:
            self.current_waypoint = self._create_waypoint(
                robot_x, robot_y, robot_heading, sectors, num_sectors)

        return self.current_waypoint

    def _create_waypoint(self, robot_x: float, robot_y: float, robot_heading: float,
                         sectors: List[float], num_sectors: int) -> Optional[Waypoint]:
        """Create waypoint at furthest free distance."""
        # Find sector with maximum distance (prefer front sectors)
        best_sector = 0
        best_score = 0

        for i, dist in enumerate(sectors):
            # Score based on distance and preference for front
            # Sectors: 0=front, 1-5=right, 6=back, 7-11=left
            front_bonus = 1.0
            if i in [0, 1, 11]:  # Front arc
                front_bonus = 1.5
            elif i in [2, 10]:  # Front-side
                front_bonus = 1.2

            score = dist * front_bonus
            if score > best_score and dist > self.min_waypoint_dist:
                best_score = score
                best_sector = i

        if best_score == 0:
            return None

        # Convert sector to world coordinates
        sector_angle = (best_sector * 2 * math.pi / num_sectors)
        # Adjust: sector 0 is front, increases counterclockwise
        if best_sector <= num_sectors // 2:
            angle_offset = -sector_angle  # Right side
        else:
            angle_offset = 2 * math.pi - sector_angle  # Left side

        world_angle = robot_heading + angle_offset

        # Place waypoint at 80% of the free distance
        waypoint_dist = sectors[best_sector] * 0.8
        waypoint_dist = max(waypoint_dist, self.min_waypoint_dist)

        wx = robot_x + waypoint_dist * math.cos(world_angle)
        wy = robot_y + waypoint_dist * math.sin(world_angle)

        waypoint = Waypoint(wx, wy)
        print(f"\n[WAYPOINT] New target at ({wx:.1f}, {wy:.1f}), dist={waypoint_dist:.1f}m, sector={best_sector}")
        return waypoint

    def get_steering(self, robot_x: float, robot_y: float,
                     robot_heading: float) -> Tuple[float, float]:
        """
        Get steering toward current waypoint.

        Returns:
            (angular_bias, distance_to_waypoint)
        """
        if self.current_waypoint is None:
            return 0.0, 0.0

        # Angle to waypoint
        target_angle = self.current_waypoint.angle_from(robot_x, robot_y)

        # Angle error (how much to turn)
        angle_error = target_angle - robot_heading

        # Normalize to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Convert to angular velocity (proportional control)
        # Limit to reasonable range
        angular_bias = max(-0.3, min(0.3, angle_error * 0.5))

        dist = self.current_waypoint.distance_to(robot_x, robot_y)
        return angular_bias, dist

    def clear_waypoint(self):
        """Clear current waypoint (e.g., when stuck)."""
        self.current_waypoint = None

    def get_stats(self) -> dict:
        """Get navigation statistics."""
        return {
            'waypoints_reached': self.waypoints_reached,
            'has_waypoint': self.current_waypoint is not None
        }
