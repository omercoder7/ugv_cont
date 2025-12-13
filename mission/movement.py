"""
Movement control functions for velocity smoothing and adaptive speed.
"""

import math
import time
from typing import Tuple

from .constants import (
    LIDAR_FRONT_OFFSET, SPEED_SCALE_FAR_DISTANCE,
    SPEED_SCALE_NEAR_DISTANCE, SPEED_SCALE_MIN_FACTOR,
    ACTUAL_MAX_ANGULAR_VEL
)


class VelocitySmoother:
    """
    Applies smoothing to velocity commands using:
    1. Exponential Moving Average (EMA) for smooth transitions
    2. Acceleration limiting to prevent jerky motion
    """

    def __init__(self, max_linear_accel: float = 0.15, max_angular_accel: float = 1.5,
                 ema_alpha: float = 0.4):
        self.max_linear_accel = max_linear_accel
        self.max_angular_accel = max_angular_accel
        self.ema_alpha = ema_alpha

        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_cmd_time = time.time()

    def smooth(self, target_linear: float, target_angular: float) -> Tuple[float, float]:
        """
        Apply smoothing to velocity commands.

        Args:
            target_linear: Target linear velocity (m/s)
            target_angular: Target angular velocity (rad/s)

        Returns:
            (smoothed_linear, smoothed_angular)
        """
        current_time = time.time()
        dt = current_time - self.last_cmd_time
        self.last_cmd_time = current_time

        dt = max(0.05, min(dt, 0.5))

        # Apply acceleration limits
        max_linear_change = self.max_linear_accel * dt
        max_angular_change = self.max_angular_accel * dt

        linear_diff = target_linear - self.last_linear
        if abs(linear_diff) > max_linear_change:
            target_linear = self.last_linear + math.copysign(max_linear_change, linear_diff)

        angular_diff = target_angular - self.last_angular
        if abs(angular_diff) > max_angular_change:
            target_angular = self.last_angular + math.copysign(max_angular_change, angular_diff)

        # Apply EMA smoothing
        smoothed_linear = self.ema_alpha * target_linear + (1 - self.ema_alpha) * self.last_linear
        smoothed_angular = self.ema_alpha * target_angular + (1 - self.ema_alpha) * self.last_angular

        self.last_linear = smoothed_linear
        self.last_angular = smoothed_angular

        return smoothed_linear, smoothed_angular

    def reset(self):
        """Reset smoother state"""
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_cmd_time = time.time()


def compute_adaptive_speed(front_distance: float, base_speed: float) -> float:
    """
    Calculate adaptive speed based on distance to nearest obstacle.

    Robot slows down as it approaches obstacles for:
    - Tighter maneuvering in close quarters
    - More reaction time near obstacles
    - Full speed in open areas

    Args:
        front_distance: Distance to nearest obstacle in front arc (meters, from LiDAR)
        base_speed: Base linear speed (m/s)

    Returns:
        Scaled linear speed
    """
    # Subtract LiDAR offset to get actual distance from robot front
    actual_distance = front_distance - LIDAR_FRONT_OFFSET

    if actual_distance >= SPEED_SCALE_FAR_DISTANCE:
        return base_speed
    elif actual_distance <= SPEED_SCALE_NEAR_DISTANCE:
        return base_speed * SPEED_SCALE_MIN_FACTOR
    else:
        distance_range = SPEED_SCALE_FAR_DISTANCE - SPEED_SCALE_NEAR_DISTANCE
        distance_ratio = (actual_distance - SPEED_SCALE_NEAR_DISTANCE) / distance_range
        speed_factor = SPEED_SCALE_MIN_FACTOR + (1.0 - SPEED_SCALE_MIN_FACTOR) * distance_ratio
        return base_speed * speed_factor


def calculate_turn_duration(degrees: float, speed: float = 1.0) -> float:
    """
    Calculate how long a turn will take based on calibrated angular velocity.

    Args:
        degrees: Angle to rotate (absolute value)
        speed: Speed factor (0.1 to 1.0)

    Returns:
        Duration in seconds
    """
    speed = max(0.1, min(1.0, speed))
    actual_vel = ACTUAL_MAX_ANGULAR_VEL * speed

    if actual_vel < 0.01:
        return 999.0

    radians = abs(degrees) * (math.pi / 180)
    return radians / actual_vel


def calculate_backup_distance(start_pos: Tuple[float, float],
                               current_pos: Tuple[float, float]) -> float:
    """
    Calculate how far the robot has backed up.

    Args:
        start_pos: Starting position (x, y)
        current_pos: Current position (x, y)

    Returns:
        Distance backed up in meters
    """
    if start_pos is None or current_pos is None:
        return 0.0

    dx = current_pos[0] - start_pos[0]
    dy = current_pos[1] - start_pos[1]
    return math.hypot(dx, dy)
