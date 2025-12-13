"""
Dead-end detection and prediction.
"""

from typing import List, Set, Tuple
from ..utils import pos_to_cell


def is_dead_end_by_sectors(sector_distances: List[float],
                           blocked_threshold: float = 0.5) -> bool:
    """
    Detect if current position is a dead-end by checking if 3+ direction
    groups are blocked.

    Direction groups:
    - FRONT: sectors 11, 0, 1
    - LEFT: sectors 2, 3, 4
    - BACK: sectors 5, 6, 7
    - RIGHT: sectors 8, 9, 10

    Returns True if 3 or more direction groups are blocked.
    """
    blocked_groups = 0

    # FRONT (sectors 11, 0, 1)
    front_dists = [sector_distances[11], sector_distances[0], sector_distances[1]]
    front_valid = [d for d in front_dists if d > 0.1]
    if front_valid and max(front_valid) < blocked_threshold:
        blocked_groups += 1

    # LEFT (sectors 2, 3, 4)
    left_dists = [sector_distances[2], sector_distances[3], sector_distances[4]]
    left_valid = [d for d in left_dists if d > 0.1]
    if left_valid and max(left_valid) < blocked_threshold:
        blocked_groups += 1

    # BACK (sectors 5, 6, 7)
    back_dists = [sector_distances[5], sector_distances[6], sector_distances[7]]
    back_valid = [d for d in back_dists if d > 0.1]
    if back_valid and max(back_valid) < blocked_threshold:
        blocked_groups += 1

    # RIGHT (sectors 8, 9, 10)
    right_dists = [sector_distances[8], sector_distances[9], sector_distances[10]]
    right_valid = [d for d in right_dists if d > 0.1]
    if right_valid and max(right_valid) < blocked_threshold:
        blocked_groups += 1

    return blocked_groups >= 3


def detect_small_pocket(sector_distances: List[float],
                        look_ahead_distance: float,
                        min_pocket_size: float = 0.6) -> bool:
    """
    Detect if robot is heading into a small pocket too tight to turn in.

    Args:
        sector_distances: 12 sector distances
        look_ahead_distance: How far ahead to check
        min_pocket_size: Minimum width needed for maneuvering

    Returns:
        True if small pocket detected ahead
    """
    front_dist = sector_distances[0]

    if front_dist >= look_ahead_distance:
        return False

    left_diag = min(sector_distances[1], sector_distances[2])
    right_diag = min(sector_distances[10], sector_distances[11])

    # If both diagonals are closer than front + margin, it's a narrowing pocket
    if left_diag < front_dist + 0.2 and right_diag < front_dist + 0.2:
        estimated_width = min(left_diag, right_diag) * 2
        if estimated_width < min_pocket_size:
            return True

    return False


class DeadEndTracker:
    """Tracks known dead-end positions."""

    def __init__(self, grid_resolution: float = 0.5):
        self.dead_end_positions: Set[Tuple[int, int]] = set()
        self.grid_resolution = grid_resolution

    def mark(self, x: float, y: float):
        """Mark a position as a dead-end."""
        cell = pos_to_cell(x, y, self.grid_resolution)
        if cell not in self.dead_end_positions:
            self.dead_end_positions.add(cell)
            print(f"\n[DEAD-END] Marked cell {cell} as dead-end at ({x:.2f}, {y:.2f})")

    def is_in_dead_end(self, x: float, y: float) -> bool:
        """Check if a position is in a known dead-end area."""
        cell = pos_to_cell(x, y, self.grid_resolution)
        return cell in self.dead_end_positions

    def clear(self):
        """Clear all dead-end markers."""
        self.dead_end_positions.clear()
