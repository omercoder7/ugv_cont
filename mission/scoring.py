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
    DWA-style trajectory generation and scoring.
    Predicts multiple trajectories and scores them based on:
    - Clearance: Distance from obstacles
    - Heading: Alignment with target direction
    - Velocity: Prefer higher speeds
    - Smoothness: Prefer straight paths
    """

    def __init__(self, max_speed: float = 0.10, max_yaw_rate: float = 1.0):
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate

        self.v_resolution = 0.02
        self.yaw_resolution = 0.2

        self.predict_time = 1.5
        self.dt = 0.1

        self.heading_weight = 0.3
        self.clearance_weight = 0.4
        self.velocity_weight = 0.2
        self.smoothness_weight = 0.1

        self.robot_radius = 0.15

    def predict_trajectory(self, x: float, y: float, yaw: float,
                          v: float, w: float) -> List[Tuple[float, float, float]]:
        """Predict robot trajectory for given velocity command."""
        trajectory = [(x, y, yaw)]

        for _ in range(int(self.predict_time / self.dt)):
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            trajectory.append((x, y, yaw))

        return trajectory

    def check_trajectory_collision(self, trajectory: List[Tuple[float, float, float]],
                                   sector_distances: List[float]) -> bool:
        """Check if trajectory collides with obstacles."""
        num_sectors = len(sector_distances)
        obstacle_points = []
        for sector, dist in enumerate(sector_distances):
            if 0.1 < dist < 2.0:
                angle = sector_to_angle(sector, num_sectors)
                ox = dist * math.cos(angle)
                oy = dist * math.sin(angle)
                obstacle_points.append((ox, oy))

        for tx, ty, _ in trajectory:
            for ox, oy in obstacle_points:
                dist = math.sqrt((tx - ox)**2 + (ty - oy)**2)
                if dist < self.robot_radius + 0.10:
                    return True

        return False

    def score_trajectory(self, trajectory: List[Tuple[float, float, float]],
                        sector_distances: List[float],
                        target_yaw: float = 0.0,
                        current_v: float = 0.0,
                        current_w: float = 0.0) -> float:
        """Score trajectory using multiple criteria."""
        if not trajectory or len(trajectory) < 2:
            return float('-inf')

        final_x, final_y, final_yaw = trajectory[-1]

        heading_error = abs(self._normalize_angle(target_yaw - final_yaw))
        heading_score = 1.0 - (heading_error / math.pi)

        min_clearance = float('inf')
        num_sectors = len(sector_distances)
        obstacle_points = []
        for sector, dist in enumerate(sector_distances):
            if 0.1 < dist < 3.0:
                angle = sector_to_angle(sector, num_sectors)
                ox = dist * math.cos(angle)
                oy = dist * math.sin(angle)
                obstacle_points.append((ox, oy))

        for tx, ty, _ in trajectory:
            for ox, oy in obstacle_points:
                d = math.sqrt((tx - ox)**2 + (ty - oy)**2)
                min_clearance = min(min_clearance, d)

        clearance_score = min(1.0, min_clearance / 1.0) if min_clearance < float('inf') else 1.0

        v = math.sqrt((trajectory[-1][0] - trajectory[0][0])**2 +
                     (trajectory[-1][1] - trajectory[0][1])**2) / self.predict_time
        velocity_score = min(1.0, v / self.max_speed)

        first_yaw = trajectory[0][2]
        yaw_change = abs(self._normalize_angle(final_yaw - first_yaw))
        smoothness_score = 1.0 - min(1.0, yaw_change / math.pi)

        total = (self.heading_weight * heading_score +
                self.clearance_weight * clearance_score +
                self.velocity_weight * velocity_score +
                self.smoothness_weight * smoothness_score)

        return total

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
                             max_accel: float = 0.2,
                             max_yaw_accel: float = 0.8) -> Tuple[float, float, float]:
        """Main DWA function: find best velocity command."""
        min_v = max(0.0, current_v - max_accel * self.dt)
        max_v = min(self.max_speed, current_v + max_accel * self.dt)
        min_w = max(-self.max_yaw_rate, current_w - max_yaw_accel * self.dt)
        max_w = min(self.max_yaw_rate, current_w + max_yaw_accel * self.dt)

        best_v, best_w = 0.0, 0.0
        best_score = float('-inf')

        v = min_v
        while v <= max_v:
            w = min_w
            while w <= max_w:
                trajectory = self.predict_trajectory(x, y, yaw, v, w)

                if self.check_trajectory_collision(trajectory, sector_distances):
                    w += self.yaw_resolution
                    continue

                score = self.score_trajectory(
                    trajectory, sector_distances, target_yaw, current_v, current_w
                )

                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w

                w += self.yaw_resolution
            v += self.v_resolution

        return best_v, best_w, best_score


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
