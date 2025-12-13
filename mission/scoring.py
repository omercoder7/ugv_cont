"""
Direction and trajectory scoring for navigation.
"""

import math
from typing import List, Tuple, Dict, Optional, TYPE_CHECKING

from .utils import sector_to_angle

if TYPE_CHECKING:
    from .vfh import VFHController
    from .memory import VisitedTracker


class TrajectoryScorer:
    """
    Enhanced DWA-style trajectory generation and scoring.

    Improvements over basic DWA:
    - Fixed velocity samples instead of resolution-based (more predictable)
    - Optimized collision checking using sector projection
    - Better scoring weights tuned for slow robots
    - Progress-based scoring to ensure forward motion
    """

    def __init__(self, max_speed: float = 0.10, max_yaw_rate: float = 0.5,
                 robot_radius: float = 0.09):
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.robot_radius = robot_radius

        # Trajectory simulation parameters
        self.predict_time = 1.5  # seconds to simulate ahead
        self.dt = 0.1  # simulation timestep

        # Fixed velocity samples for consistent behavior
        # Linear: 5 samples from 0 to max_speed
        self.linear_samples = [0.0, 0.25, 0.5, 0.75, 1.0]  # multiplied by max_speed
        # Angular: 9 samples from -max_yaw to +max_yaw
        self.angular_samples = [-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0]  # multiplied by max_yaw_rate

        # Scoring weights (tuned for slow exploration robot)
        self.clearance_weight = 3.0   # Safety is paramount
        self.velocity_weight = 1.5    # Prefer moving over stopping
        self.heading_weight = 1.0     # Prefer straight paths
        self.progress_weight = 1.0    # Reward forward progress

        # Collision threshold
        self.min_clearance = robot_radius + 0.05  # collision if closer than this

        # Cache for obstacle points (computed once per update)
        self._obstacle_cache = None
        self._cache_distances = None

    def set_corridor_mode(self, enabled: bool):
        """Switch to corridor mode with tighter constraints."""
        if enabled:
            # Fewer samples, more cautious
            self.linear_samples = [0.3, 0.5, 0.7]  # slower speeds
            self.angular_samples = [-0.5, -0.25, 0.0, 0.25, 0.5]  # less turning
            self.clearance_weight = 4.0  # even more safety-focused
        else:
            # Normal exploration mode
            self.linear_samples = [0.0, 0.25, 0.5, 0.75, 1.0]
            self.angular_samples = [-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0]
            self.clearance_weight = 3.0

    def simulate_trajectory(self, x: float, y: float, yaw: float,
                           linear: float, angular: float,
                           duration: float = None) -> List[Tuple[float, float, float]]:
        """
        Simulate robot trajectory using differential drive kinematics.

        Args:
            x, y, yaw: Starting pose
            linear: Linear velocity (m/s)
            angular: Angular velocity (rad/s)
            duration: Simulation duration (default: self.predict_time)

        Returns:
            List of (x, y, theta) poses along trajectory
        """
        if duration is None:
            duration = self.predict_time

        trajectory = [(x, y, yaw)]
        steps = int(duration / self.dt)

        for _ in range(steps):
            # Differential drive kinematics
            yaw += angular * self.dt
            x += linear * math.cos(yaw) * self.dt
            y += linear * math.sin(yaw) * self.dt
            trajectory.append((x, y, yaw))

        return trajectory

    def _update_obstacle_cache(self, sector_distances: List[float]):
        """Update cached obstacle points if distances changed."""
        if self._cache_distances == sector_distances:
            return

        self._cache_distances = sector_distances.copy()
        self._obstacle_cache = []

        num_sectors = len(sector_distances)
        for sector, dist in enumerate(sector_distances):
            # Consider all obstacles from 1cm to 2.5m
            # CRITICAL: Don't ignore close obstacles! 0.05m is very close and must be detected
            # Only skip 0.0 (blind spots where LiDAR has no reading)
            if 0.01 < dist < 2.5:
                angle = sector_to_angle(sector, num_sectors)
                ox = dist * math.cos(angle)
                oy = dist * math.sin(angle)
                self._obstacle_cache.append((ox, oy, dist))

    def trajectory_min_clearance(self, trajectory: List[Tuple[float, float, float]],
                                  sector_distances: List[float]) -> float:
        """
        Calculate minimum clearance along trajectory.

        Uses optimized checking - only checks against nearby obstacles.

        Args:
            trajectory: List of (x, y, theta) poses
            sector_distances: Current LiDAR sector distances

        Returns:
            Minimum clearance (0 if collision detected)
        """
        self._update_obstacle_cache(sector_distances)

        if not self._obstacle_cache:
            return float('inf')  # No obstacles nearby

        min_clearance = float('inf')

        for tx, ty, _ in trajectory:
            for ox, oy, obs_dist in self._obstacle_cache:
                # Distance from trajectory point to obstacle
                d = math.sqrt((tx - ox)**2 + (ty - oy)**2)

                # Quick collision check
                if d < self.min_clearance:
                    return 0.0  # Collision!

                min_clearance = min(min_clearance, d)

        return min_clearance

    def score_trajectory(self, trajectory: List[Tuple[float, float, float]],
                        sector_distances: List[float],
                        linear: float, angular: float,
                        target_yaw: float = 0.0) -> Tuple[float, Dict[str, float]]:
        """
        Score trajectory using multiple criteria.

        All scores are normalized to 0-1 range before weighting.

        Args:
            trajectory: Simulated trajectory poses
            sector_distances: Current LiDAR distances
            linear: Commanded linear velocity
            angular: Commanded angular velocity
            target_yaw: Target heading (default: 0 = forward)

        Returns:
            (total_score, breakdown_dict)
        """
        if not trajectory or len(trajectory) < 2:
            return float('-inf'), {"invalid": -1000}

        breakdown = {}

        # 1. CLEARANCE SCORE: How far from obstacles?
        min_clearance = self.trajectory_min_clearance(trajectory, sector_distances)
        if min_clearance == 0:
            return float('-inf'), {"collision": -1000}

        # Normalize: clearance of 1.5m = perfect score
        clearance_score = min(1.0, min_clearance / 1.5)
        breakdown["clearance"] = clearance_score

        # 2. VELOCITY SCORE: Prefer moving over stopping
        velocity_score = linear / self.max_speed if self.max_speed > 0 else 0
        breakdown["velocity"] = velocity_score

        # 3. HEADING SCORE: Prefer straight paths (less turning)
        # 1.0 when going straight, lower when turning
        heading_score = 1.0 - min(1.0, abs(angular) / self.max_yaw_rate)
        breakdown["heading"] = heading_score

        # 4. PROGRESS SCORE: Reward forward distance traveled
        start_x, start_y, _ = trajectory[0]
        end_x, end_y, _ = trajectory[-1]
        forward_dist = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        # Normalize: 0.3m traveled = perfect score (for 1.5s at low speed)
        progress_score = min(1.0, forward_dist / 0.3)
        breakdown["progress"] = progress_score

        # Weighted sum
        total = (
            self.clearance_weight * clearance_score +
            self.velocity_weight * velocity_score +
            self.heading_weight * heading_score +
            self.progress_weight * progress_score
        )
        breakdown["total"] = total

        return total, breakdown

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def compute_best_velocity(self, x: float, y: float, yaw: float,
                             current_v: float, current_w: float,
                             sector_distances: List[float],
                             target_yaw: float = 0.0,
                             committed_direction: str = None) -> Tuple[float, float, float]:
        """
        Main DWA function: find best velocity command.

        Samples candidate trajectories and returns the best one.

        Args:
            x, y, yaw: Current robot pose (can be 0,0,0 for local frame)
            current_v, current_w: Current velocities (for smoothness)
            sector_distances: LiDAR sector distances
            target_yaw: Target heading
            committed_direction: "left" or "right" to bias sampling

        Returns:
            (best_linear, best_angular, best_score)
        """
        best_v, best_w = 0.0, 0.0
        best_score = float('-inf')
        valid_count = 0

        # Generate candidate velocities
        for lin_mult in self.linear_samples:
            linear = lin_mult * self.max_speed

            for ang_mult in self.angular_samples:
                angular = ang_mult * self.max_yaw_rate

                # If committed to a direction, bias the samples
                if committed_direction == "left" and angular < -0.1:
                    continue  # Skip hard right turns
                elif committed_direction == "right" and angular > 0.1:
                    continue  # Skip hard left turns

                # Simulate trajectory
                trajectory = self.simulate_trajectory(x, y, yaw, linear, angular)

                # Score it
                score, breakdown = self.score_trajectory(
                    trajectory, sector_distances, linear, angular, target_yaw
                )

                if score > float('-inf'):
                    valid_count += 1

                if score > best_score:
                    best_score = score
                    best_v, best_w = linear, angular

        return best_v, best_w, best_score

    def compute_best_velocity_fast(self, sector_distances: List[float],
                                    committed_direction: str = None) -> Tuple[float, float, float]:
        """
        Faster version using local frame (robot at origin facing forward).

        This is more efficient when we don't need global coordinates.
        """
        return self.compute_best_velocity(
            x=0.0, y=0.0, yaw=0.0,
            current_v=0.0, current_w=0.0,
            sector_distances=sector_distances,
            target_yaw=0.0,
            committed_direction=committed_direction
        )


class UnifiedDirectionScorer:
    """
    Unified scoring system that consolidates all direction evaluation criteria.
    """

    PRESETS = {
        "exploration": {
            "clearance": 1.0,
            "distance": 1.5,
            "freedom": 1.5,
            "frontier": 4.0,
            "visited": 2.0,
            "continuity": 0.5,
            "forward_bias": 0.3,
        },
        "safety": {
            "clearance": 4.0,
            "distance": 2.0,
            "freedom": 2.0,
            "frontier": 0.5,
            "visited": 0.5,
            "continuity": 1.0,
            "forward_bias": 0.2,
        },
        "corridor": {
            "clearance": 3.0,
            "distance": 1.0,
            "freedom": 1.0,
            "frontier": 2.0,
            "visited": 1.0,
            "continuity": 2.0,
            "forward_bias": 0.5,
        },
    }

    def __init__(self, num_sectors: int = 12):
        self.num_sectors = num_sectors
        self.previous_sector: Optional[int] = None
        self.weights = self.PRESETS["exploration"].copy()

    def set_mode(self, mode: str):
        """Set scoring weights based on navigation mode."""
        if mode in self.PRESETS:
            self.weights = self.PRESETS[mode].copy()

    def set_weights(self, **kwargs):
        """Override individual weights."""
        for key, value in kwargs.items():
            if key in self.weights:
                self.weights[key] = value

    def score_sector(
        self,
        sector: int,
        sector_distances: List[float],
        min_distance: float,
        vfh_controller: 'VFHController',
        map_scores: Optional[List[float]] = None,
        dead_ends: Optional[List[bool]] = None,
        visit_counts: Optional[List[int]] = None,
        current_position: Optional[Tuple[float, float]] = None,
    ) -> Tuple[float, Dict[str, float]]:
        """Score a single sector using unified criteria."""
        dist = sector_distances[sector]
        breakdown = {}

        if dist == 0.0:
            return -1000.0, {"invalid": -1000.0}
        if dist < 0.1:
            return -800.0, {"critical_obstacle": -800.0}
        if dist < min_distance:
            return -500.0, {"blocked": -500.0}
        if dead_ends and dead_ends[sector]:
            return -400.0, {"dead_end": -400.0}

        clearance = 1.0 if vfh_controller.is_path_clear(sector) else 0.3
        breakdown["clearance"] = clearance

        max_dist = 3.0
        distance_score = min(dist / max_dist, 1.0)
        breakdown["distance"] = distance_score

        freedom = vfh_controller.calculate_space_freedom(sector, sector_distances)
        breakdown["freedom"] = freedom

        frontier = map_scores[sector] if map_scores else 0.5
        breakdown["frontier"] = frontier

        if visit_counts and visit_counts[sector] is not None:
            vc = visit_counts[sector]
            if vc == 0:
                visited_score = 1.0
            elif vc == 1:
                visited_score = 0.5
            elif vc == 2:
                visited_score = 0.25
            else:
                visited_score = 0.1
        else:
            visited_score = 0.7
        breakdown["visited"] = visited_score

        if self.previous_sector is not None:
            angle_diff = abs(sector - self.previous_sector)
            if angle_diff > self.num_sectors // 2:
                angle_diff = self.num_sectors - angle_diff
            continuity = 1.0 - (angle_diff / (self.num_sectors // 2))
        else:
            continuity = 0.5
        breakdown["continuity"] = continuity

        if sector in [0]:
            forward_bias = 1.0
        elif sector in [1, 11]:
            forward_bias = 0.8
        elif sector in [2, 10]:
            forward_bias = 0.5
        else:
            forward_bias = 0.2
        breakdown["forward_bias"] = forward_bias

        pocket_penalty = 0.0
        if dist < 2.0:
            left_1 = sector_distances[(sector + 1) % self.num_sectors]
            left_2 = sector_distances[(sector + 2) % self.num_sectors]
            right_1 = sector_distances[(sector - 1) % self.num_sectors]
            right_2 = sector_distances[(sector - 2) % self.num_sectors]

            left_min = min(left_1, left_2)
            right_min = min(right_1, right_2)

            if left_min < dist and right_min < dist:
                estimated_width = left_min + right_min
                MIN_TURN_WIDTH = 0.4

                if estimated_width < MIN_TURN_WIDTH:
                    pocket_penalty = -1.0
                    breakdown["pocket"] = pocket_penalty

        total = (
            self.weights["clearance"] * clearance +
            self.weights["distance"] * distance_score +
            self.weights["freedom"] * freedom +
            self.weights["frontier"] * frontier +
            self.weights["visited"] * visited_score +
            self.weights["continuity"] * continuity +
            self.weights["forward_bias"] * forward_bias +
            pocket_penalty
        )

        return total, breakdown

    def find_best_sector(
        self,
        sector_distances: List[float],
        min_distance: float,
        vfh_controller: 'VFHController',
        map_scores: Optional[List[float]] = None,
        dead_ends: Optional[List[bool]] = None,
        visited_tracker: Optional['VisitedTracker'] = None,
        current_position: Optional[Tuple[float, float]] = None,
        current_heading: Optional[float] = None,
    ) -> Tuple[Optional[int], float, Dict[str, float]]:
        """Find the best sector to move toward."""
        best_sector = None
        best_score = float('-inf')
        best_breakdown = {}

        visit_counts = None
        if visited_tracker and current_position:
            visit_counts = []
            heading = current_heading if current_heading is not None else 0.0
            for sector in range(self.num_sectors):
                dist = sector_distances[sector]
                if dist > 0.1:
                    robot_angle = sector_to_angle(sector, self.num_sectors)
                    world_angle = heading + robot_angle
                    look_ahead = min(dist, 1.5)
                    target_x = current_position[0] + look_ahead * math.cos(world_angle)
                    target_y = current_position[1] + look_ahead * math.sin(world_angle)
                    visit_counts.append(visited_tracker.get_visit_count(target_x, target_y))
                else:
                    visit_counts.append(None)

        for sector in range(self.num_sectors):
            score, breakdown = self.score_sector(
                sector, sector_distances, min_distance, vfh_controller,
                map_scores, dead_ends, visit_counts, current_position
            )
            if score > best_score:
                best_score = score
                best_sector = sector
                best_breakdown = breakdown

        if best_sector is not None:
            self.previous_sector = best_sector

        return best_sector, best_score, best_breakdown
