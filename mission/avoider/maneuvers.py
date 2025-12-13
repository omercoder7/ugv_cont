"""
Movement maneuvers for obstacle avoidance.
"""

import math
import subprocess
from typing import Callable

from ..constants import CONTAINER_NAME, ACTUAL_MAX_ANGULAR_VEL


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


def execute_turn_in_place(degrees: float, speed: float = 1.0,
                          stop_callback: Callable = None) -> bool:
    """
    Turn the robot in place by a specific number of degrees.
    Uses calibrated timing for accurate rotation.

    BLOCKING - waits for turn to complete.

    Args:
        degrees: Angle to rotate (positive = left/CCW, negative = right/CW)
        speed: Command speed (0.1 to 1.0, default max)
        stop_callback: Optional callback to stop robot on error

    Returns:
        True if turn completed successfully
    """
    if abs(degrees) < 5:
        return True

    speed = max(0.1, min(1.0, speed))
    duration = calculate_turn_duration(degrees, speed)
    angular = speed if degrees > 0 else -speed

    print(f"\n[TURN] Rotating {degrees:.0f}° {'left' if degrees > 0 else 'right'} (duration: {duration:.1f}s)")

    # Scale to robot's expected range (-1 to 1)
    x_val = 0.0
    z_val = max(-1.0, min(1.0, -angular / 1.0))  # INVERTED

    serial_cmd = f'{{"T":"13","X":{x_val:.2f},"Z":{z_val:.2f}}}'
    stop_cmd = '{"T":"13","X":0.00,"Z":0.00}'

    iterations = int(duration / 0.05)

    bash_script = f'''
for i in $(seq 1 {iterations}); do
    echo '{serial_cmd}' > /dev/ttyAMA0
    sleep 0.05
done
echo '{stop_cmd}' > /dev/ttyAMA0
echo '{stop_cmd}' > /dev/ttyAMA0
'''

    try:
        subprocess.run(
            ['docker', 'exec', CONTAINER_NAME, 'bash', '-c', bash_script],
            timeout=duration + 5,
            capture_output=True
        )
        return True
    except subprocess.TimeoutExpired:
        print("Turn timed out, stopping...")
        if stop_callback:
            stop_callback()
        return False
    except Exception as e:
        print(f"Turn error: {e}")
        if stop_callback:
            stop_callback()
        return False


def calculate_backup_turn_params(backup_duration: float,
                                 turn_degrees: float,
                                 linear_speed: float) -> dict:
    """
    Calculate parameters for a backup-then-turn maneuver.

    Args:
        backup_duration: How long to back up
        turn_degrees: How many degrees to turn
        linear_speed: Robot's linear speed

    Returns:
        Dict with backup_speed, backup_duration, turn_speed, turn_duration
    """
    turn_degrees = max(-45, min(45, turn_degrees))  # Cap at 45°
    turn_speed = 1.0
    turn_duration = calculate_turn_duration(turn_degrees, turn_speed)
    angular_speed = turn_speed if turn_degrees > 0 else -turn_speed

    return {
        'backup_speed': linear_speed,
        'backup_duration': backup_duration,
        'turn_speed': angular_speed,
        'turn_duration': turn_duration
    }


def calculate_avoidance_turn_degrees(obstacle_sector: int,
                                     obstacle_distance: float,
                                     min_distance: float,
                                     num_sectors: int = 12) -> float:
    """
    Calculate how many degrees to turn based on obstacle position.

    Uses a polynomial relationship to determine turn angle based on obstacle
    position. Obstacles directly ahead need larger turns.

    Args:
        obstacle_sector: Sector where obstacle was detected (0 = front)
        obstacle_distance: Distance to obstacle in meters
        min_distance: Minimum safe distance threshold
        num_sectors: Number of sectors (default 12)

    Returns:
        Turn angle in degrees (positive = left, negative = right)
    """
    sector_degrees = 360 // num_sectors

    # Calculate base turn angle
    if obstacle_sector <= num_sectors // 2:
        # Obstacle on left side - turn right
        base_angle = -(obstacle_sector + 2) * sector_degrees
    else:
        # Obstacle on right side - turn left
        sectors_from_right = num_sectors - obstacle_sector
        base_angle = (sectors_from_right + 2) * sector_degrees

    # Adjust based on distance - closer obstacles need sharper turns
    if obstacle_distance < min_distance * 0.5:
        multiplier = 1.5  # Very close - sharper turn
    elif obstacle_distance < min_distance:
        multiplier = 1.2
    else:
        multiplier = 1.0

    return base_angle * multiplier
