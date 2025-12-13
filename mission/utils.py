"""
Utility functions for autonomous scanning and navigation.
"""

import math
from typing import Tuple

from .constants import NUM_SECTORS, GRID_RESOLUTION, ACTUAL_MAX_ANGULAR_VEL


def sector_to_angle(sector: int, num_sectors: int = NUM_SECTORS) -> float:
    """
    Convert sector number to angle in radians (0 = front, positive = left).

    This is the UNIFIED implementation - use this everywhere instead of
    inline calculations. Handles wrap-around correctly.

    Args:
        sector: Sector index (0 = front, increases counter-clockwise)
        num_sectors: Total number of sectors (default 12)

    Returns:
        Angle in radians, range [-pi, pi]
    """
    sector_width = 2 * math.pi / num_sectors
    if sector <= num_sectors // 2:
        return sector * sector_width  # 0 to pi
    else:
        return (sector - num_sectors) * sector_width  # -pi to 0


def pos_to_cell(x: float, y: float, resolution: float = GRID_RESOLUTION) -> Tuple[int, int]:
    """
    Convert world position to grid cell coordinates.

    UNIFIED implementation - use this everywhere instead of class methods.

    Args:
        x, y: World position in meters
        resolution: Grid cell size in meters

    Returns:
        (cell_x, cell_y) tuple
    """
    return (int(x / resolution), int(y / resolution))


def calculate_turn_duration(turn_degrees: float, turn_speed: float = 1.0) -> float:
    """
    Calculate how long a turn will take at given speed.

    Args:
        turn_degrees: Absolute turn angle in degrees
        turn_speed: Angular velocity factor (1.0 = max speed)

    Returns:
        Duration in seconds
    """
    actual_vel = ACTUAL_MAX_ANGULAR_VEL * abs(turn_speed)
    if actual_vel < 0.01:
        return 999.0  # Avoid division by zero
    return abs(turn_degrees) * (math.pi / 180) / actual_vel
