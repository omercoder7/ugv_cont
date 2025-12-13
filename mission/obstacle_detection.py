"""
Obstacle detection and classification for navigation.

Analyzes LiDAR sector distances to detect obstacle shapes (round, thin, flat)
and trapped/dead-end states.
"""

import json
import os
from typing import List, Tuple, Optional, Dict

from .constants import BODY_THRESHOLD_FRONT, NUM_SECTORS
from .collision import CollisionVerifier


def load_avoidance_profiles() -> dict:
    """
    Load obstacle-specific avoidance profiles from calibration file.

    Profiles contain:
    - shape: 'round', 'flat', or 'thin'
    - extra_margin: Additional distance margin (meters)
    - min_turn_deg: Minimum turn angle for this obstacle
    - backup_mult: Multiplier for backup duration
    """
    default_profiles = {
        'default': {'shape': 'flat', 'extra_margin': 0.0, 'min_turn_deg': 45, 'backup_mult': 1.0}
    }

    try:
        calib_file = CollisionVerifier.CALIBRATION_FILE
        if os.path.exists(calib_file):
            with open(calib_file, 'r') as f:
                data = json.load(f)
                profiles = data.get('avoidance_profiles', {})
                if profiles:
                    default_profiles.update(profiles)
                    custom_count = len([k for k in profiles if k != 'default'])
                    if custom_count > 0:
                        print(f"[Avoidance] Loaded {custom_count} obstacle-specific avoidance profiles")
    except Exception as e:
        print(f"[Avoidance] Using default profiles: {e}")

    return default_profiles


def detect_thin_obstacle(sector_distances: List[float]) -> bool:
    """
    Detect if current obstacle is thin (like chair leg, pole).

    Thin obstacles show a single sector blocked while adjacent sectors are clear.
    Must distinguish from doorways which have similar pattern but with open sides.
    """
    if len(sector_distances) < NUM_SECTORS:
        return False

    front = sector_distances[0]
    front_left = sector_distances[1]
    front_right = sector_distances[11]
    left_side = sector_distances[2]
    right_side = sector_distances[10]

    if front < 0.5:
        left_clear = front_left > front + 0.3
        right_clear = front_right > front + 0.3

        # Doorway check: both sides very open
        both_sides_very_open = left_side > 0.8 and right_side > 0.8
        if both_sides_very_open and left_clear and right_clear:
            return False  # Doorway

        # Door frame check
        if left_side < 0.5 and right_side < 0.5:
            return False  # Doorway with door frame

        if left_clear and right_clear:
            return True

    return False


def detect_round_obstacle(sector_distances: List[float]) -> bool:
    """
    Detect if current obstacle has round/curved shape.

    Round obstacles show gradually changing distances across sectors.
    Must distinguish from corners (two walls meeting).
    """
    if len(sector_distances) < NUM_SECTORS:
        return False

    front = sector_distances[0]
    front_left = sector_distances[1]
    front_right = sector_distances[11]
    left_side = sector_distances[2]
    right_side = sector_distances[10]
    left_back = sector_distances[3]
    right_back = sector_distances[9]

    if front < 0.6:
        # Corner check: walls extending to sides
        if left_side < 0.6 or right_side < 0.6:
            return False

        # Corner check: walls extending backward
        if left_back < 0.8 or right_back < 0.8:
            return False

        # Check for "curve" pattern - symmetric and both further than center
        side_diff = abs(front_left - front_right)
        both_further = front_left > front and front_right > front

        if side_diff < 0.15 and both_further:
            return True

    return False


def get_front_arc_min(sector_distances: List[float],
                      body_threshold: float = BODY_THRESHOLD_FRONT) -> float:
    """
    Get minimum distance in front arc (sectors 11, 0, 1).

    Args:
        sector_distances: List of 12 sector distances
        body_threshold: Minimum distance to consider valid (filters noise, not real obstacles)

    Returns:
        Minimum distance in front arc
    """
    front_readings = [sector_distances[11], sector_distances[0], sector_distances[1]]

    # Filter out readings below body_threshold (likely noise or sensor error)
    filtered = [d for d in front_readings if d > body_threshold]

    if filtered:
        return min(filtered)
    else:
        # All readings below threshold - this means VERY CLOSE obstacle, not "clear"!
        # Return the actual minimum reading (even if below threshold)
        # Only return 0.0 if all readings are exactly 0 (blind spot)
        actual_min = min(front_readings)
        if actual_min > 0.01:
            # Real obstacle very close - return actual distance
            return actual_min
        else:
            # All readings are ~0 (blind spot) - assume blocked for safety
            return 0.05  # Assume 5cm - triggers backing up


def get_side_clearances(sector_distances: List[float]) -> Tuple[float, float]:
    """
    Get left and right clearance distances.

    Returns:
        (left_clearance, right_clearance) tuple in meters
    """
    left = min(sector_distances[1], sector_distances[2])
    right = min(sector_distances[10], sector_distances[11])
    return left, right


def check_trapped_state(sector_distances: List[float],
                        dynamic_min_dist: float,
                        margin: float = 0.40) -> dict:
    """
    Check if robot is trapped in a dead-end or tight space.

    Args:
        sector_distances: List of 12 sector distances
        dynamic_min_dist: Current minimum distance threshold
        margin: Additional margin for triggering trapped detection

    Returns:
        Dict with keys: front_blocked, left_blocked, right_blocked, rear_clear, is_trapped
    """
    front_arc_min = get_front_arc_min(sector_distances)
    left_dist, right_dist = get_side_clearances(sector_distances)
    rear_dists = [sector_distances[5], sector_distances[6], sector_distances[7]]

    threshold = dynamic_min_dist + margin

    result = {
        'front_blocked': front_arc_min < threshold,
        'left_blocked': left_dist < threshold,
        'right_blocked': right_dist < threshold,
        'rear_clear': min(rear_dists) > dynamic_min_dist,
        'front_arc_min': front_arc_min,
        'left_dist': left_dist,
        'right_dist': right_dist,
    }
    result['is_trapped'] = (result['front_blocked'] and result['left_blocked'] and
                            result['right_blocked'] and result['rear_clear'])
    return result


def get_avoidance_params(sector_distances: List[float],
                         front_distance: float,
                         avoidance_profiles: dict) -> Tuple[dict, Optional[str]]:
    """
    Get avoidance parameters based on current obstacle detection.

    Args:
        sector_distances: List of 12 sector distances
        front_distance: Distance to front obstacle
        avoidance_profiles: Loaded avoidance profiles dict

    Returns:
        tuple: (params_dict, obstacle_name or None)
    """
    default_params = {
        'shape': 'flat',
        'extra_margin': 0.0,
        'min_turn_deg': 45,
        'backup_mult': 1.0
    }
    params = avoidance_profiles.get('default', default_params).copy()
    detected_obstacle = None

    # Check for round obstacle pattern
    if front_distance < 0.6 and detect_round_obstacle(sector_distances):
        for name, profile in avoidance_profiles.items():
            if name == 'default':
                continue
            if profile.get('shape') == 'round':
                params = profile.copy()
                detected_obstacle = name
                break

        if detected_obstacle is None:
            params = {
                'shape': 'round',
                'extra_margin': 0.10,
                'min_turn_deg': 60,
                'backup_mult': 1.3
            }
            detected_obstacle = "round_obstacle (uncalibrated)"

    # Check for thin obstacle pattern
    elif front_distance < 0.5 and detect_thin_obstacle(sector_distances):
        for name, profile in avoidance_profiles.items():
            if name == 'default':
                continue
            if profile.get('shape') == 'thin':
                params = profile.copy()
                detected_obstacle = name
                break

        if detected_obstacle is None:
            params = {
                'shape': 'thin',
                'extra_margin': 0.0,
                'min_turn_deg': 35,
                'backup_mult': 1.0
            }
            detected_obstacle = "thin_obstacle (uncalibrated)"

    return params, detected_obstacle
