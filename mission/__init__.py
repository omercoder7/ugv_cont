"""
Mission package for autonomous scanning and navigation.

Modules:
- fsm_avoider: Main obstacle avoidance FSM
- frontier: Frontier-based exploration (like explore_lite)
- exploration: Visited cell tracking, stuck detection
- virtual_obstacles: Invisible obstacle management
- pro_avoidance: TTC and environment detection
"""

from .constants import *
from .fsm_avoider import FSMAvoider, State
from .frontier import FrontierExplorer, Frontier
from .exploration import ExplorationTracker, StuckDetector
from .virtual_obstacles import VirtualObstacle, VirtualObstacleManager
from .main import main, ensure_slam_running, check_prerequisites, start_driver

__all__ = [
    'FSMAvoider',
    'State',
    'FrontierExplorer',
    'Frontier',
    'ExplorationTracker',
    'StuckDetector',
    'VirtualObstacle',
    'VirtualObstacleManager',
    'main',
    'ensure_slam_running',
    'check_prerequisites',
    'start_driver',
]
