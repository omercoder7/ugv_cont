"""
Mission package for autonomous scanning and navigation.

This package contains the refactored code from auto_scan.py, split into
logical modules for better maintainability.
"""

from .constants import *
from .utils import sector_to_angle, pos_to_cell, calculate_turn_duration
from .state import RobotState, StateContext
from .avoider import SectorObstacleAvoider
from .waypoints import WaypointManager, ExplorationWaypoint
from .main import main, ensure_slam_running, check_prerequisites, start_driver

__all__ = [
    'SectorObstacleAvoider',
    'RobotState',
    'StateContext',
    'WaypointManager',
    'ExplorationWaypoint',
    'main',
    'ensure_slam_running',
    'check_prerequisites',
    'start_driver',
    'sector_to_angle',
    'pos_to_cell',
    'calculate_turn_duration',
]
