"""
Avoider subpackage - components for obstacle avoidance.

This package contains modular components used by SectorObstacleAvoider:
- core: Main SectorObstacleAvoider class
- lidar: LiDAR processing and sector distance computation
- dead_ends: Dead-end detection and tracking
- stuck: Stuck detection algorithms
- maneuvers: Movement maneuvers (turn, backup)
- movement_thread: Continuous movement at 30Hz
"""

# Main class export
from .core import SectorObstacleAvoider

# Component exports
from .lidar import LidarProcessor, compute_sector_distances, lidar_to_robot_sector, robot_to_lidar_sector
from .dead_ends import DeadEndTracker, is_dead_end_by_sectors, detect_small_pocket
from .stuck import StuckDetector
from .maneuvers import (
    calculate_turn_duration, execute_turn_in_place,
    calculate_backup_turn_params, calculate_avoidance_turn_degrees
)
from .movement_thread import MovementThread

__all__ = [
    'SectorObstacleAvoider',
    'LidarProcessor',
    'compute_sector_distances',
    'lidar_to_robot_sector',
    'robot_to_lidar_sector',
    'DeadEndTracker',
    'is_dead_end_by_sectors',
    'detect_small_pocket',
    'StuckDetector',
    'calculate_turn_duration',
    'execute_turn_in_place',
    'calculate_backup_turn_params',
    'calculate_avoidance_turn_degrees',
    'MovementThread',
]
