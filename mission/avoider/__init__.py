"""
Avoider subpackage - LiDAR processing utilities.

This package contains the LiDAR processing module used by FSMAvoider.
"""

from .lidar import LidarProcessor, compute_sector_distances, lidar_to_robot_sector, robot_to_lidar_sector

__all__ = [
    'LidarProcessor',
    'compute_sector_distances',
    'lidar_to_robot_sector',
    'robot_to_lidar_sector',
]
