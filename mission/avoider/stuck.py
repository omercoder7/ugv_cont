"""
Stuck detection algorithms.
"""

import math
import time
from typing import Optional, Tuple, List


class StuckDetector:
    """
    Detects when the robot is stuck using efficiency-based detection.

    Uses CollisionVerifier's efficiency history when available, with odometry
    fallback. Detects stuck when movement efficiency drops below threshold.
    """

    def __init__(self, efficiency_threshold: float = 0.35,
                 history_max_len: int = 10,
                 min_data_time: float = 0.25):
        """
        Args:
            efficiency_threshold: Below this ratio = stuck (0.35 = 35%)
            history_max_len: Number of position samples to keep
            min_data_time: Minimum time window for efficiency calculation
        """
        self.efficiency_threshold = efficiency_threshold
        self.history_max_len = history_max_len
        self.min_data_time = min_data_time

        # State tracking
        self.last_position: Optional[Tuple[float, float]] = None
        self.last_position_time: float = 0
        self.sensor_history: List[Tuple[float, float, float]] = []  # (time, x, y)
        self.stuck_position: Optional[Tuple[float, float]] = None
        self.stuck_cooldown: int = 0

    def reset(self):
        """Reset all tracking state."""
        self.last_position = None
        self.last_position_time = 0
        self.sensor_history = []
        self.stuck_position = None

    def check(self, is_driving: bool, commanded_linear: float,
              current_position: Optional[Tuple[float, float]],
              collision_verifier=None) -> bool:
        """
        Check if robot is stuck.

        Args:
            is_driving: Whether robot is trying to drive forward
            commanded_linear: Commanded linear velocity (m/s)
            current_position: Current (x, y) position from odometry
            collision_verifier: Optional CollisionVerifier for efficiency data

        Returns:
            True if stuck is detected
        """
        if not is_driving or commanded_linear <= 0.01:
            self.reset()
            return False

        if self.stuck_cooldown > 0:
            self.stuck_cooldown -= 1
            return False

        if current_position is None:
            return False

        current_time = time.time()

        # METHOD 1: Use CollisionVerifier's efficiency data if available
        if collision_verifier and hasattr(collision_verifier, 'efficiency_history'):
            cv_efficiency = collision_verifier.efficiency_history
            if len(cv_efficiency) >= 5:
                avg_efficiency = sum(cv_efficiency[-5:]) / 5
                if avg_efficiency < self.efficiency_threshold:
                    print(f"\n[STUCK] CollisionVerifier efficiency={avg_efficiency:.0%}")
                    print(f"  Position: ({current_position[0]:.2f}, {current_position[1]:.2f})")
                    self.stuck_position = current_position
                    return True

        # METHOD 2: Fallback to odometry-based efficiency check
        if self.last_position is None:
            self.last_position = current_position
            self.last_position_time = current_time
            self.sensor_history = [(current_time, current_position[0], current_position[1])]
            return False

        # Track history of positions
        self.sensor_history.append((current_time, current_position[0], current_position[1]))

        # Keep only recent samples
        while len(self.sensor_history) > self.history_max_len:
            self.sensor_history.pop(0)

        # Need enough samples for efficiency calculation
        if len(self.sensor_history) >= 5:
            oldest = self.sensor_history[0]
            newest = self.sensor_history[-1]
            dt = newest[0] - oldest[0]

            if dt > self.min_data_time:
                odom_delta = math.hypot(newest[1] - oldest[1], newest[2] - oldest[2])
                expected_change = commanded_linear * dt

                if expected_change > 0.015:
                    efficiency = odom_delta / expected_change
                    if efficiency < self.efficiency_threshold:
                        print(f"\n[STUCK] Efficiency={efficiency:.0%} (moved {odom_delta:.3f}m, expected {expected_change:.3f}m)")
                        print(f"  Position: ({current_position[0]:.2f}, {current_position[1]:.2f})")
                        self.stuck_position = current_position
                        self.sensor_history = []
                        return True

        # Update tracking periodically
        time_elapsed = current_time - self.last_position_time
        if time_elapsed > 0.5:
            self.last_position = current_position
            self.last_position_time = current_time

        return False

    def set_cooldown(self, count: int):
        """Set cooldown iterations after recovering from stuck."""
        self.stuck_cooldown = count
