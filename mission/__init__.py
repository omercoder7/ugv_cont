"""
Mission package for autonomous scanning and navigation.

Modules:
- simple_cost_nav: Next-Best-View (NBV) navigation with goal point selection
- frontier: Frontier-based exploration (like explore_lite)
- exploration: Visited cell tracking, stuck detection
- virtual_obstacles: Invisible obstacle management
- pro_avoidance: TTC and environment detection
"""

from .constants import *
from .frontier import FrontierExplorer, Frontier
from .exploration import ExplorationTracker, StuckDetector
from .virtual_obstacles import VirtualObstacle, VirtualObstacleManager

__all__ = [
    'FrontierExplorer',
    'Frontier',
    'ExplorationTracker',
    'StuckDetector',
    'VirtualObstacle',
    'VirtualObstacleManager',
]
