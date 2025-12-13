"""
LiDAR processing and sector distance computation.
"""

from typing import List
from ..constants import NUM_SECTORS, LIDAR_ROTATION_SECTORS
from ..ros_interface import get_lidar_scan


def lidar_to_robot_sector(lidar_sector: int) -> int:
    """Convert LiDAR sector to robot-frame sector (accounting for rotation)"""
    return (lidar_sector + LIDAR_ROTATION_SECTORS) % NUM_SECTORS


def robot_to_lidar_sector(robot_sector: int) -> int:
    """Convert robot-frame sector to LiDAR sector"""
    return (robot_sector - LIDAR_ROTATION_SECTORS) % NUM_SECTORS


def compute_sector_distances(scan_ranges: List[float],
                             robot_radius: float = 0.18) -> List[float]:
    """
    Compute minimum distance for each sector in ROBOT frame.

    Args:
        scan_ranges: Raw LiDAR range data
        robot_radius: Distance threshold for body filtering

    Returns:
        List of 12 sector distances in robot frame
    """
    if not scan_ranges:
        return [5.0] * NUM_SECTORS

    n = len(scan_ranges)
    points_per_sector = n // NUM_SECTORS

    # Compute distances in LiDAR frame first
    lidar_distances = [5.0] * NUM_SECTORS

    for lidar_sector in range(NUM_SECTORS):
        start_idx = lidar_sector * points_per_sector
        end_idx = start_idx + points_per_sector

        sector_ranges = scan_ranges[start_idx:end_idx]
        valid = [r for r in sector_ranges if robot_radius < r < 10.0]

        if valid:
            lidar_distances[lidar_sector] = min(valid)
        else:
            # Check for very close obstacles
            any_reading = [r for r in sector_ranges if 0.02 < r < 10.0]
            if any_reading:
                min_reading = min(any_reading)
                if min_reading < robot_radius:
                    lidar_distances[lidar_sector] = min_reading
                else:
                    lidar_distances[lidar_sector] = 0.0
            else:
                # No readings - BLIND SPOT
                lidar_distances[lidar_sector] = 0.0

    # Convert to robot frame (rotate by calibrated offset)
    sector_distances = [5.0] * NUM_SECTORS
    for robot_sector in range(NUM_SECTORS):
        lidar_sector = robot_to_lidar_sector(robot_sector)
        sector_distances[robot_sector] = lidar_distances[lidar_sector]

    return sector_distances


class LidarProcessor:
    """Handles LiDAR scan processing and sector distance computation."""

    def __init__(self):
        self.scan_ranges: List[float] = []
        self.sector_distances: List[float] = [10.0] * NUM_SECTORS

    def update_scan(self) -> bool:
        """Fetch latest LiDAR scan and compute sector distances."""
        ranges = get_lidar_scan()
        if ranges:
            self.scan_ranges = ranges
            self.sector_distances = compute_sector_distances(ranges)
            return True
        return False

    def lidar_to_robot_sector(self, lidar_sector: int) -> int:
        """Convert LiDAR sector to robot-frame sector."""
        return lidar_to_robot_sector(lidar_sector)

    def robot_to_lidar_sector(self, robot_sector: int) -> int:
        """Convert robot-frame sector to LiDAR sector."""
        return robot_to_lidar_sector(robot_sector)
