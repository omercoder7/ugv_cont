"""
Frontier-based exploration for autonomous scanning.

Based on explore_lite and WFD (Wavefront Frontier Detector) algorithms.
Uses LiDAR readings + visited cells as proxy for occupancy grid.

Key concept: Frontiers are boundaries between explored and unexplored space.
The robot navigates to frontiers to maximize exploration efficiency.
Dead ends are avoided implicitly - they have no frontiers (nothing unknown beyond).

References:
- explore_lite: https://github.com/robo-friends/m-explore-ros2
- WFD algorithm: https://arxiv.org/pdf/1806.03581
"""

import math
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass


@dataclass
class FrontierCandidate:
    """A potential frontier direction to explore."""
    sector: int  # LiDAR sector index
    distance: float  # Free distance in this direction
    visit_count: int  # How many times we've visited cells in this direction
    angle_from_front: float  # Angle offset from front (radians)
    score: float  # Combined exploration score


class FrontierExplorer:
    """
    Frontier-based exploration using LiDAR + visited cells.

    Approximates frontier detection without full occupancy grid by:
    - Long LiDAR reading = likely frontier (unexplored space ahead)
    - Low visit count = unexplored area
    - Combines into a cost function similar to explore_lite

    Cost function: score = λ1*distance + λ2*novelty - λ3*turn_cost
    """

    def __init__(self,
                 num_sectors: int = 12,
                 min_frontier_distance: float = 0.8,
                 distance_weight: float = 1.0,
                 novelty_weight: float = 2.0,
                 turn_weight: float = 0.3):
        """
        Args:
            num_sectors: Number of LiDAR sectors
            min_frontier_distance: Minimum distance to consider a frontier
            distance_weight: Weight for distance in scoring (λ1)
            novelty_weight: Weight for novelty/unvisited bonus (λ2)
            turn_weight: Penalty for turning away from front (λ3)
        """
        self.num_sectors = num_sectors
        self.min_frontier_distance = min_frontier_distance
        self.distance_weight = distance_weight
        self.novelty_weight = novelty_weight
        self.turn_weight = turn_weight

        # Dead end detection
        self.dead_end_threshold = 0.5  # If best score below this, it's a dead end
        self.consecutive_dead_ends = 0
        self.max_dead_ends_before_turn = 3

    def sector_to_angle(self, sector: int) -> float:
        """Convert sector index to angle offset from front (radians)."""
        # Sector 0 = front, increases counterclockwise
        # Returns angle in [-pi, pi] relative to front
        angle = sector * 2 * math.pi / self.num_sectors
        if angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def find_frontiers(self, sectors: List[float],
                       visited_cells: Dict[Tuple[int, int], int],
                       robot_x: float, robot_y: float,
                       robot_heading: float,
                       grid_resolution: float = 0.5) -> List[FrontierCandidate]:
        """
        Find frontier candidates from LiDAR sectors.

        Args:
            sectors: LiDAR distance readings per sector
            visited_cells: Dict of (cell_x, cell_y) -> visit count
            robot_x, robot_y: Robot position
            robot_heading: Robot heading in radians
            grid_resolution: Grid cell size for visited tracking

        Returns:
            List of FrontierCandidate sorted by score (best first)
        """
        candidates = []

        for sector_idx, distance in enumerate(sectors):
            # Skip sectors with obstacles too close
            if distance < self.min_frontier_distance:
                continue

            # Calculate angle from front
            angle_offset = self.sector_to_angle(sector_idx)
            world_angle = robot_heading + angle_offset

            # Sample points along this direction to check visit counts
            visit_count = 0
            sample_distances = [0.5, 1.0, 1.5, 2.0]  # meters

            for sample_dist in sample_distances:
                if sample_dist > distance:
                    break
                sample_x = robot_x + sample_dist * math.cos(world_angle)
                sample_y = robot_y + sample_dist * math.sin(world_angle)

                # Convert to grid cell
                cell_x = int(sample_x / grid_resolution)
                cell_y = int(sample_y / grid_resolution)
                visit_count += visited_cells.get((cell_x, cell_y), 0)

            # Calculate novelty (inverse of visit count)
            # More visits = less novel = lower score
            novelty = 1.0 / (1.0 + visit_count * 0.5)

            # Turn cost (prefer going straight)
            turn_cost = abs(angle_offset)

            # Combined score (higher = better frontier)
            score = (self.distance_weight * distance +
                     self.novelty_weight * novelty -
                     self.turn_weight * turn_cost)

            candidates.append(FrontierCandidate(
                sector=sector_idx,
                distance=distance,
                visit_count=visit_count,
                angle_from_front=angle_offset,
                score=score
            ))

        # Sort by score (best first)
        candidates.sort(key=lambda c: c.score, reverse=True)
        return candidates

    def select_best_frontier(self, sectors: List[float],
                             visited_cells: Dict[Tuple[int, int], int],
                             robot_x: float, robot_y: float,
                             robot_heading: float,
                             grid_resolution: float = 0.5) -> Tuple[Optional[int], bool]:
        """
        Select the best frontier direction.

        Args:
            sectors: LiDAR distance readings per sector
            visited_cells: Dict of (cell_x, cell_y) -> visit count
            robot_x, robot_y: Robot position
            robot_heading: Robot heading in radians
            grid_resolution: Grid cell size

        Returns:
            (best_sector, is_dead_end)
            best_sector: Index of best sector to navigate toward, or None
            is_dead_end: True if no good frontiers found (should turn around)
        """
        candidates = self.find_frontiers(
            sectors, visited_cells, robot_x, robot_y,
            robot_heading, grid_resolution
        )

        if not candidates:
            self.consecutive_dead_ends += 1
            return None, True

        best = candidates[0]

        # Check if best frontier is good enough
        if best.score < self.dead_end_threshold:
            self.consecutive_dead_ends += 1
            if self.consecutive_dead_ends >= self.max_dead_ends_before_turn:
                return None, True
        else:
            self.consecutive_dead_ends = 0

        return best.sector, False

    def get_steering_toward_frontier(self, target_sector: int,
                                     current_sectors: List[float]) -> float:
        """
        Get angular velocity to steer toward frontier.

        Args:
            target_sector: Target sector index
            current_sectors: Current LiDAR readings

        Returns:
            Angular velocity (positive = left, negative = right)
        """
        angle_offset = self.sector_to_angle(target_sector)

        # Proportional control with limit
        angular_vel = max(-0.3, min(0.3, angle_offset * 0.4))

        return angular_vel

    def is_dead_end(self, sectors: List[float],
                    visited_cells: Dict[Tuple[int, int], int],
                    robot_x: float, robot_y: float,
                    robot_heading: float,
                    grid_resolution: float = 0.5) -> bool:
        """
        Check if current position is a dead end.

        A dead end is when:
        - All directions have obstacles close by, OR
        - All directions lead to heavily visited areas

        Returns:
            True if this is a dead end
        """
        # Check if any sector has a good frontier
        candidates = self.find_frontiers(
            sectors, visited_cells, robot_x, robot_y,
            robot_heading, grid_resolution
        )

        if not candidates:
            return True

        # Check if best candidate is good enough
        best_score = candidates[0].score
        return best_score < self.dead_end_threshold

    def reset(self):
        """Reset dead end counter (call after successful turn-around)."""
        self.consecutive_dead_ends = 0

    def get_stats(self) -> dict:
        """Get exploration statistics."""
        return {
            'consecutive_dead_ends': self.consecutive_dead_ends
        }
