"""
Exploration waypoint system for frontier-based navigation.

Sets waypoints at "free ray" points where LiDAR sees furthest,
and navigates toward them for systematic exploration.
"""

import math
import time
from typing import List, Tuple, Optional
from dataclasses import dataclass, field


@dataclass
class ExplorationWaypoint:
    """A waypoint representing an exploration target."""
    x: float
    y: float
    created_at: float = field(default_factory=time.time)
    source_heading: float = 0.0  # Heading when waypoint was created
    distance_when_set: float = 0.0  # How far it was when set
    attempts: int = 0  # How many times we tried to reach it

    def distance_to(self, x: float, y: float) -> float:
        """Calculate distance from waypoint to a position."""
        return math.hypot(self.x - x, self.y - y)

    def angle_from(self, x: float, y: float) -> float:
        """Calculate angle from position to waypoint (radians)."""
        return math.atan2(self.y - y, self.x - x)

    def age(self) -> float:
        """Return age of waypoint in seconds."""
        return time.time() - self.created_at


class WaypointManager:
    """
    Manages exploration waypoints for frontier-based navigation.

    Key features:
    - Sets waypoints at "free ray" points (where LiDAR sees furthest)
    - Maintains minimum distance between waypoints
    - Automatically replaces reached/stale waypoints
    - Provides steering guidance toward active waypoint
    """

    def __init__(self,
                 min_waypoint_distance: float = 2.0,
                 max_waypoints: int = 5,
                 waypoint_reach_threshold: float = 0.5,
                 max_waypoint_age: float = 300.0,
                 min_free_ray_distance: float = 2.0):
        """
        Args:
            min_waypoint_distance: Minimum distance between waypoints (meters)
            max_waypoints: Maximum number of waypoints to maintain
            waypoint_reach_threshold: Distance to consider waypoint reached (meters)
            max_waypoint_age: Remove waypoints older than this (seconds)
            min_free_ray_distance: Minimum LiDAR distance to set waypoint (meters)
        """
        self.min_waypoint_distance = min_waypoint_distance
        self.max_waypoints = max_waypoints
        self.waypoint_reach_threshold = waypoint_reach_threshold
        self.max_waypoint_age = max_waypoint_age
        self.min_free_ray_distance = min_free_ray_distance

        self.waypoints: List[ExplorationWaypoint] = []
        self.active_waypoint: Optional[ExplorationWaypoint] = None
        self.reached_positions: List[Tuple[float, float]] = []  # History of reached waypoints

        # Stats
        self.waypoints_created = 0
        self.waypoints_reached = 0
        self.waypoints_abandoned = 0

    def add_waypoint_from_lidar(self,
                                 robot_x: float,
                                 robot_y: float,
                                 robot_heading: float,
                                 sector_distances: List[float],
                                 sector_index: int) -> Optional[ExplorationWaypoint]:
        """
        Add a waypoint based on a clear LiDAR sector.

        Args:
            robot_x, robot_y: Current robot position
            robot_heading: Current robot heading (radians)
            sector_distances: List of 12 sector distances
            sector_index: Which sector has the free ray

        Returns:
            New waypoint if created, None otherwise
        """
        if len(sector_distances) < 12:
            return None

        distance = sector_distances[sector_index]

        # Only set waypoint if distance is significant
        if distance < self.min_free_ray_distance:
            return None

        # Convert sector to world angle
        # Sector 0 = front, each sector = 30 degrees
        sector_angle = sector_index * (2 * math.pi / 12)
        if sector_index > 6:
            sector_angle = (sector_index - 12) * (2 * math.pi / 12)

        world_angle = robot_heading + sector_angle

        # Calculate waypoint position (slightly less than full distance for safety)
        waypoint_distance = min(distance * 0.8, distance - 0.5)
        waypoint_x = robot_x + waypoint_distance * math.cos(world_angle)
        waypoint_y = robot_y + waypoint_distance * math.sin(world_angle)

        return self.add_waypoint(waypoint_x, waypoint_y, robot_heading, distance)

    def add_waypoint(self,
                     x: float,
                     y: float,
                     source_heading: float = 0.0,
                     distance_when_set: float = 0.0) -> Optional[ExplorationWaypoint]:
        """
        Add a waypoint if it's far enough from existing ones.

        Returns:
            New waypoint if created, None if too close to existing
        """
        # Check distance to all existing waypoints
        for wp in self.waypoints:
            if wp.distance_to(x, y) < self.min_waypoint_distance:
                return None

        # Check distance to recently reached positions
        for rx, ry in self.reached_positions[-10:]:  # Last 10 reached
            if math.hypot(x - rx, y - ry) < self.min_waypoint_distance:
                return None

        # Remove oldest if at capacity
        if len(self.waypoints) >= self.max_waypoints:
            self._remove_oldest_waypoint()

        # Create new waypoint
        waypoint = ExplorationWaypoint(
            x=x, y=y,
            source_heading=source_heading,
            distance_when_set=distance_when_set
        )
        self.waypoints.append(waypoint)
        self.waypoints_created += 1

        # Set as active if none active
        if self.active_waypoint is None:
            self.active_waypoint = waypoint
            print(f"\n[WAYPOINT] New target at ({x:.2f}, {y:.2f}), {distance_when_set:.1f}m away")

        return waypoint

    def update(self, robot_x: float, robot_y: float) -> Optional[ExplorationWaypoint]:
        """
        Update waypoint status based on robot position.

        Returns:
            Current active waypoint (may have changed)
        """
        self._prune_old_waypoints()

        # Check if we reached active waypoint
        if self.active_waypoint:
            dist = self.active_waypoint.distance_to(robot_x, robot_y)

            if dist < self.waypoint_reach_threshold:
                print(f"\n[WAYPOINT] Reached target at ({self.active_waypoint.x:.2f}, {self.active_waypoint.y:.2f})!")
                self._mark_reached(self.active_waypoint)
                self._select_next_waypoint(robot_x, robot_y)

        return self.active_waypoint

    def get_steering_bias(self,
                          robot_x: float,
                          robot_y: float,
                          robot_heading: float) -> Tuple[float, float]:
        """
        Get steering bias toward active waypoint.

        Returns:
            (angle_error, distance) - angle error in radians, distance to waypoint
            Returns (0, 0) if no active waypoint
        """
        if not self.active_waypoint:
            return 0.0, 0.0

        # Calculate angle to waypoint
        target_angle = self.active_waypoint.angle_from(robot_x, robot_y)

        # Calculate angle error (how much we need to turn)
        angle_error = target_angle - robot_heading

        # Normalize to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        distance = self.active_waypoint.distance_to(robot_x, robot_y)

        return angle_error, distance

    def mark_blocked(self, robot_x: float, robot_y: float):
        """
        Mark that we're blocked trying to reach active waypoint.
        Switch to another waypoint if too many attempts.
        """
        if not self.active_waypoint:
            return

        self.active_waypoint.attempts += 1

        if self.active_waypoint.attempts >= 3:
            print(f"\n[WAYPOINT] Abandoning blocked target ({self.active_waypoint.x:.2f}, {self.active_waypoint.y:.2f}) after {self.active_waypoint.attempts} attempts")
            self._abandon_waypoint(self.active_waypoint)
            self._select_next_waypoint(robot_x, robot_y)

    def switch_waypoint(self, robot_x: float, robot_y: float):
        """Force switch to a different waypoint."""
        if self.active_waypoint and len(self.waypoints) > 1:
            # Move current to end of list (lower priority)
            self.waypoints.remove(self.active_waypoint)
            self.waypoints.append(self.active_waypoint)
            self._select_next_waypoint(robot_x, robot_y)

    def find_best_sector_for_waypoint(self,
                                       sector_distances: List[float],
                                       map_scores: List[float] = None,
                                       dead_ends: List[bool] = None) -> Optional[int]:
        """
        Find the best sector to set a new waypoint.

        Prefers sectors with:
        1. Long clear distance (free ray)
        2. High exploration score (unexplored area)
        3. Not a dead-end

        Returns:
            Best sector index, or None if no good sector
        """
        if len(sector_distances) < 12:
            return None

        best_sector = None
        best_score = 0.0

        for i, dist in enumerate(sector_distances):
            # Skip short distances
            if dist < self.min_free_ray_distance:
                continue

            # Skip dead-ends
            if dead_ends and i < len(dead_ends) and dead_ends[i]:
                continue

            # Base score from distance
            score = min(dist / 5.0, 1.0)  # Cap at 5m

            # Bonus for unexplored areas
            if map_scores and i < len(map_scores):
                score += map_scores[i] * 0.5

            # Slight preference for forward direction
            if i in [0, 1, 11]:
                score += 0.1

            if score > best_score:
                best_score = score
                best_sector = i

        return best_sector

    def _select_next_waypoint(self, robot_x: float, robot_y: float):
        """Select the best next waypoint to pursue."""
        if not self.waypoints:
            self.active_waypoint = None
            return

        # Sort by distance (prefer closer waypoints)
        self.waypoints.sort(key=lambda wp: wp.distance_to(robot_x, robot_y))

        # Select closest that hasn't been attempted too many times
        for wp in self.waypoints:
            if wp.attempts < 3:
                self.active_waypoint = wp
                print(f"\n[WAYPOINT] New target: ({wp.x:.2f}, {wp.y:.2f}), {wp.distance_to(robot_x, robot_y):.1f}m away")
                return

        # All waypoints have been tried - clear and start fresh
        print("\n[WAYPOINT] All waypoints exhausted, clearing list")
        self.waypoints.clear()
        self.active_waypoint = None

    def _mark_reached(self, waypoint: ExplorationWaypoint):
        """Mark a waypoint as reached."""
        self.reached_positions.append((waypoint.x, waypoint.y))
        if len(self.reached_positions) > 50:
            self.reached_positions.pop(0)

        if waypoint in self.waypoints:
            self.waypoints.remove(waypoint)

        if self.active_waypoint == waypoint:
            self.active_waypoint = None

        self.waypoints_reached += 1

    def _abandon_waypoint(self, waypoint: ExplorationWaypoint):
        """Abandon a waypoint that can't be reached."""
        if waypoint in self.waypoints:
            self.waypoints.remove(waypoint)

        if self.active_waypoint == waypoint:
            self.active_waypoint = None

        self.waypoints_abandoned += 1

    def _remove_oldest_waypoint(self):
        """Remove the oldest waypoint."""
        if self.waypoints:
            oldest = min(self.waypoints, key=lambda wp: wp.created_at)
            self.waypoints.remove(oldest)
            if self.active_waypoint == oldest:
                self.active_waypoint = None

    def _prune_old_waypoints(self):
        """Remove waypoints that are too old."""
        current_time = time.time()
        old_waypoints = [wp for wp in self.waypoints
                        if current_time - wp.created_at > self.max_waypoint_age]

        for wp in old_waypoints:
            self.waypoints.remove(wp)
            if self.active_waypoint == wp:
                self.active_waypoint = None

    def get_stats(self) -> dict:
        """Get waypoint statistics."""
        return {
            'active': self.active_waypoint is not None,
            'total_waypoints': len(self.waypoints),
            'created': self.waypoints_created,
            'reached': self.waypoints_reached,
            'abandoned': self.waypoints_abandoned
        }

    def clear(self):
        """Clear all waypoints."""
        self.waypoints.clear()
        self.active_waypoint = None
        self.reached_positions.clear()
