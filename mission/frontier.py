"""
Frontier-based exploration for autonomous scanning.

Based on professional algorithms:
- explore_lite (ROS2): https://github.com/robo-friends/m-explore-ros2
- Wavefront Frontier Detector (WFD): https://arxiv.org/pdf/1806.03581
- Husarion tutorials: https://husarion.com/tutorials/ros2-tutorials/10-exploration/

Key concepts:
1. Frontiers = boundaries between explored and unexplored space
2. Cost function: C = distance_cost - gain * frontier_size + orientation_cost
3. Information gain: prefer frontiers revealing more unexplored area
4. Min frontier size: ignore tiny frontiers (likely dead ends or noise)
5. Progress timeout: abandon goals if no progress made
"""

import math
import time
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass, field


@dataclass
class Frontier:
    """A frontier (boundary between explored and unexplored)."""
    sector: int  # Primary LiDAR sector
    distance: float  # Distance to frontier
    size: float  # Estimated frontier size (width)
    info_gain: float  # Expected information gain
    angle: float  # Angle from robot front (radians)
    cost: float = 0.0  # Computed cost (lower = better)

    def __post_init__(self):
        """Ensure valid values."""
        self.size = max(0.1, self.size)
        self.info_gain = max(0.0, self.info_gain)


class FrontierExplorer:
    """
    Professional frontier-based exploration.

    Uses explore_lite-style cost function:
    cost = potential_scale * distance - gain_scale * size + orientation_scale * |angle|

    Lower cost = better frontier to explore.
    """

    def __init__(self,
                 num_sectors: int = 12,
                 # Cost function weights (like explore_lite)
                 potential_scale: float = 1.0,  # Distance penalty
                 gain_scale: float = 2.0,  # Size/info gain bonus
                 orientation_scale: float = 0.5,  # Turning penalty
                 # Thresholds
                 min_frontier_size: float = 0.3,  # Minimum frontier width (meters)
                 min_frontier_distance: float = 0.5,  # Minimum distance to consider
                 max_frontier_distance: float = 5.0,  # Maximum useful distance
                 # Progress tracking
                 progress_timeout: float = 10.0,  # Abandon goal if no progress
                 min_progress: float = 0.1):  # Minimum movement to count as progress
        """
        Args:
            num_sectors: Number of LiDAR sectors
            potential_scale: Weight for distance (higher = prefer closer)
            gain_scale: Weight for information gain (higher = prefer larger frontiers)
            orientation_scale: Weight for turning (higher = prefer straight ahead)
            min_frontier_size: Ignore frontiers smaller than this (meters)
            min_frontier_distance: Ignore frontiers closer than this
            max_frontier_distance: Clip distances beyond this
            progress_timeout: Seconds without progress before abandoning goal
            min_progress: Minimum meters moved to count as progress
        """
        self.num_sectors = num_sectors
        self.potential_scale = potential_scale
        self.gain_scale = gain_scale
        self.orientation_scale = orientation_scale
        self.min_frontier_size = min_frontier_size
        self.min_frontier_distance = min_frontier_distance
        self.max_frontier_distance = max_frontier_distance
        self.progress_timeout = progress_timeout
        self.min_progress = min_progress

        # Current target tracking
        self.current_target: Optional[Frontier] = None
        self.target_start_time: float = 0
        self.target_start_pos: Optional[Tuple[float, float]] = None
        self.last_progress_time: float = 0
        self.last_progress_pos: Optional[Tuple[float, float]] = None

        # Store all frontiers for telemetry
        self.last_frontiers: List[Frontier] = []

        # Statistics
        self.frontiers_explored = 0
        self.frontiers_abandoned = 0

    def sector_to_angle(self, sector: int) -> float:
        """Convert sector index to angle from front (radians, -pi to pi).

        Sectors: 0=front, 1-6=left (positive angles), 7-11=right (negative angles)
        Convention: positive=left, negative=right
        """
        if sector == 0:
            return 0.0
        elif sector <= self.num_sectors // 2:
            # Left side and back: positive angle
            return sector * 2 * math.pi / self.num_sectors
        else:
            # Right side: negative angle
            return -(self.num_sectors - sector) * 2 * math.pi / self.num_sectors

    def estimate_frontier_size(self, sectors: List[float], sector_idx: int) -> float:
        """
        Estimate frontier size based on adjacent sector readings.

        Larger open areas suggest bigger frontiers with more to explore.
        """
        distance = sectors[sector_idx]
        if distance < self.min_frontier_distance:
            return 0.0

        # Check adjacent sectors for continuity
        left_idx = (sector_idx - 1) % self.num_sectors
        right_idx = (sector_idx + 1) % self.num_sectors

        left_dist = sectors[left_idx]
        right_dist = sectors[right_idx]

        # Estimate width based on sector angle and distance
        sector_angle = 2 * math.pi / self.num_sectors  # ~30 degrees for 12 sectors
        base_width = 2 * distance * math.tan(sector_angle / 2)

        # Bonus for adjacent open sectors (indicates larger frontier)
        continuity_bonus = 0.0
        if left_dist > self.min_frontier_distance:
            continuity_bonus += 0.5
        if right_dist > self.min_frontier_distance:
            continuity_bonus += 0.5

        return base_width * (1.0 + continuity_bonus)

    def calculate_info_gain(self, sectors: List[float], sector_idx: int,
                           visited_cells: Dict[Tuple[int, int], int],
                           robot_x: float, robot_y: float, robot_heading: float,
                           grid_res: float) -> float:
        """
        Calculate expected information gain for exploring this frontier.

        Higher gain = more unexplored cells in this direction.
        """
        distance = sectors[sector_idx]
        if distance < self.min_frontier_distance:
            return 0.0

        angle = self.sector_to_angle(sector_idx)
        world_angle = robot_heading + angle

        # Sample points along this direction
        info_gain = 0.0
        sample_distances = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]

        for sample_dist in sample_distances:
            if sample_dist > distance:
                break

            sample_x = robot_x + sample_dist * math.cos(world_angle)
            sample_y = robot_y + sample_dist * math.sin(world_angle)

            cell_x = int(sample_x / grid_res)
            cell_y = int(sample_y / grid_res)

            visit_count = visited_cells.get((cell_x, cell_y), 0)

            # Unvisited cells have high info gain
            if visit_count == 0:
                info_gain += 1.0
            elif visit_count == 1:
                info_gain += 0.3  # Some value for rarely visited
            # Heavily visited = no info gain

        # Bonus for distance (further = more unexplored beyond)
        distance_bonus = min(distance / self.max_frontier_distance, 1.0)

        return info_gain * (1.0 + distance_bonus)

    def detect_frontiers(self, sectors: List[float],
                        visited_cells: Dict[Tuple[int, int], int],
                        robot_x: float, robot_y: float,
                        robot_heading: float,
                        grid_res: float = 0.5) -> List[Frontier]:
        """
        Detect all valid frontiers from current sensor data.

        Returns list of Frontier objects sorted by cost (best first).
        """
        frontiers = []

        for sector_idx, distance in enumerate(sectors):
            # Skip too close or too far
            if distance < self.min_frontier_distance:
                continue

            # Clip max distance
            distance = min(distance, self.max_frontier_distance)

            # Estimate frontier size
            size = self.estimate_frontier_size(sectors, sector_idx)
            if size < self.min_frontier_size:
                continue  # Too small, likely dead end or noise

            # Calculate information gain
            info_gain = self.calculate_info_gain(
                sectors, sector_idx, visited_cells,
                robot_x, robot_y, robot_heading, grid_res
            )

            # Get angle from front
            angle = self.sector_to_angle(sector_idx)

            # Calculate cost (explore_lite style)
            # Lower cost = better frontier
            cost = (self.potential_scale * distance -
                   self.gain_scale * (size + info_gain) +
                   self.orientation_scale * abs(angle))

            frontiers.append(Frontier(
                sector=sector_idx,
                distance=distance,
                size=size,
                info_gain=info_gain,
                angle=angle,
                cost=cost
            ))

        # Sort by cost (lowest first = best)
        frontiers.sort(key=lambda f: f.cost)
        return frontiers

    def check_progress(self, robot_x: float, robot_y: float) -> bool:
        """
        Check if robot is making progress toward current target.

        Returns False if stuck (triggers goal abandonment).
        """
        if self.current_target is None:
            return True

        now = time.time()

        # Initialize progress tracking
        if self.last_progress_pos is None:
            self.last_progress_pos = (robot_x, robot_y)
            self.last_progress_time = now
            return True

        # Calculate movement since last progress check
        dx = robot_x - self.last_progress_pos[0]
        dy = robot_y - self.last_progress_pos[1]
        movement = math.hypot(dx, dy)

        if movement >= self.min_progress:
            # Made progress, reset timer
            self.last_progress_pos = (robot_x, robot_y)
            self.last_progress_time = now
            return True

        # Check timeout
        if now - self.last_progress_time > self.progress_timeout:
            return False  # Stuck, abandon goal

        return True

    def select_frontier(self, sectors: List[float],
                       visited_cells: Dict[Tuple[int, int], int],
                       robot_x: float, robot_y: float,
                       robot_heading: float,
                       grid_res: float = 0.5) -> Tuple[Optional[Frontier], bool]:
        """
        Select best frontier to explore.

        Returns:
            (frontier, is_dead_end)
            frontier: Best frontier to explore, or None
            is_dead_end: True if no valid frontiers found
        """
        # Check progress on current target
        if not self.check_progress(robot_x, robot_y):
            print(f"\n[FRONTIER] No progress for {self.progress_timeout}s, abandoning target")
            self.frontiers_abandoned += 1
            self.current_target = None
            self.last_progress_pos = None

        # Detect all frontiers
        frontiers = self.detect_frontiers(
            sectors, visited_cells,
            robot_x, robot_y, robot_heading, grid_res
        )

        # Store all frontiers for telemetry
        self.last_frontiers = frontiers

        if not frontiers:
            return None, True

        # Select best frontier
        best = frontiers[0]

        # Check if this is a new target
        if (self.current_target is None or
            best.sector != self.current_target.sector):
            self.current_target = best
            self.target_start_time = time.time()
            self.target_start_pos = (robot_x, robot_y)
            self.last_progress_pos = (robot_x, robot_y)
            self.last_progress_time = time.time()

        return best, False

    def get_steering(self, frontier: Frontier) -> float:
        """
        Get angular velocity to steer toward frontier.

        Returns angular velocity (positive = left, negative = right).
        """
        if frontier is None:
            return 0.0

        # Proportional control with limits
        # Stronger steering for larger angles
        kp = 0.5
        max_angular = 0.35

        angular = kp * frontier.angle
        angular = max(-max_angular, min(max_angular, angular))

        return angular

    def get_all_frontier_costs(self) -> List[Tuple[int, float, float]]:
        """
        Get all frontier costs for telemetry.

        Returns:
            List of (sector, angle_degrees, cost) tuples sorted by cost
        """
        return [(f.sector, math.degrees(f.angle), f.cost) for f in self.last_frontiers]

    def mark_explored(self):
        """Mark current target as explored (reached)."""
        if self.current_target is not None:
            self.frontiers_explored += 1
            self.current_target = None
            self.last_progress_pos = None

    def reset(self):
        """Reset exploration state (after backup/recovery)."""
        self.current_target = None
        self.last_progress_pos = None
        self.last_progress_time = 0
        self.last_frontiers = []

    def get_stats(self) -> dict:
        """Get exploration statistics."""
        return {
            'frontiers_explored': self.frontiers_explored,
            'frontiers_abandoned': self.frontiers_abandoned,
            'has_target': self.current_target is not None,
            'num_frontiers': len(self.last_frontiers)
        }
