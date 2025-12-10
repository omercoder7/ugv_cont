#!/usr/bin/env python3
"""
Rotation utilities for UGV Robot

Provides functions to rotate the robot in place (left or right).
Supports both "forward" rotation (normal) and "backward" rotation (inverted).

Usage:
    from rotation_utils import rotate_left, rotate_right

    rotate_left(duration=2.0, speed=0.3)   # Turn left for 2 seconds
    rotate_right(duration=2.0, speed=0.3)  # Turn right for 2 seconds
"""
import subprocess
import time

CONTAINER_NAME = "ugv_rpi_ros_humble"

# Calibration: Robot actual turn rate at Z=1.0 command
# Measured: 6.5s command gave ~110° instead of 90°, so robot is faster than 0.24
# Adjusted: 0.24 * (110/90) ≈ 0.29 rad/s
ACTUAL_MAX_ANGULAR_VEL = 0.29  # rad/s at Z=1.0


def send_cmd(linear_x: float, angular_z: float):
    """
    Send velocity command to robot via serial port.

    Args:
        linear_x: Forward/backward speed (-1 to 1 range, scaled from m/s)
        angular_z: Rotation speed (-1 to 1 range, scaled from rad/s)

    Note: Both linear AND angular are INVERTED due to motor wiring on this robot.
    """
    # Scale to robot's expected range (-1 to 1)
    x_val = max(-1.0, min(1.0, -linear_x / 0.3))   # INVERTED
    z_val = max(-1.0, min(1.0, -angular_z / 1.0))  # INVERTED

    serial_cmd = f'{{"T":"13","X":{x_val:.2f},"Z":{z_val:.2f}}}'
    try:
        result = subprocess.run(
            ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
             f"echo '{serial_cmd}' > /dev/ttyAMA0"],
            timeout=1,
            capture_output=True
        )
        if result.returncode != 0:
            print(f"  Error: {result.stderr.decode()}")
    except Exception as e:
        print(f"Error sending command: {e}")


def send_cmd_for_duration(linear_x: float, angular_z: float, duration: float):
    """
    Send velocity command continuously for a duration using a single docker exec.
    This avoids the ~80ms overhead per command when using multiple docker exec calls.

    Args:
        linear_x: Forward/backward speed (m/s scale)
        angular_z: Rotation speed (rad/s scale)
        duration: How long to send the command (seconds)
    """
    # Scale to robot's expected range (-1 to 1)
    x_val = max(-1.0, min(1.0, -linear_x / 0.3))   # INVERTED
    z_val = max(-1.0, min(1.0, -angular_z / 1.0))  # INVERTED

    serial_cmd = f'{{"T":"13","X":{x_val:.2f},"Z":{z_val:.2f}}}'
    stop_cmd = '{"T":"13","X":0.00,"Z":0.00}'

    # Calculate iterations (50ms interval = 20Hz inside container)
    iterations = int(duration / 0.05)

    # Run a bash loop inside container for consistent timing
    bash_script = f'''
for i in $(seq 1 {iterations}); do
    echo '{serial_cmd}' > /dev/ttyAMA0
    sleep 0.05
done
echo '{stop_cmd}' > /dev/ttyAMA0
echo '{stop_cmd}' > /dev/ttyAMA0
'''

    try:
        result = subprocess.run(
            ['docker', 'exec', CONTAINER_NAME, 'bash', '-c', bash_script],
            timeout=duration + 5,
            capture_output=True
        )
        if result.returncode != 0:
            print(f"  Error: {result.stderr.decode()}")
    except subprocess.TimeoutExpired:
        print("Command timed out, stopping robot...")
        stop()
    except Exception as e:
        print(f"Error: {e}")
        stop()


def stop():
    """Stop the robot immediately."""
    send_cmd(0.0, 0.0)
    time.sleep(0.1)
    send_cmd(0.0, 0.0)  # Send twice to ensure it's received


def rotate_left(duration: float = 1.0, speed: float = 1.0, backward: bool = False):
    """
    Rotate the robot in place to the LEFT (counter-clockwise when viewed from above).

    Args:
        duration: How long to rotate in seconds
        speed: Command speed (0.1 to 1.0, maps to Z command value)
        backward: If True, rotate in the "backward" direction (inverted)

    Returns:
        float: Approximate angle rotated in degrees (calibrated estimate)
    """
    speed = max(0.1, min(1.0, speed))
    angular = speed if not backward else -speed

    # Calibrated actual angular velocity
    actual_vel = ACTUAL_MAX_ANGULAR_VEL * speed
    estimated_degrees = actual_vel * duration * 57.3

    print(f"Rotating LEFT {'(backward)' if backward else '(forward)'} for {duration:.1f}s (~{estimated_degrees:.0f}°)")

    # Use efficient single docker exec with internal loop
    send_cmd_for_duration(0.0, angular, duration)

    print(f"Rotation complete.")
    return estimated_degrees


def rotate_right(duration: float = 1.0, speed: float = 1.0, backward: bool = False):
    """
    Rotate the robot in place to the RIGHT (clockwise when viewed from above).

    Args:
        duration: How long to rotate in seconds
        speed: Command speed (0.1 to 1.0, maps to Z command value)
        backward: If True, rotate in the "backward" direction (inverted)

    Returns:
        float: Approximate angle rotated in degrees (calibrated estimate)
    """
    speed = max(0.1, min(1.0, speed))
    angular = -speed if not backward else speed

    # Calibrated actual angular velocity
    actual_vel = ACTUAL_MAX_ANGULAR_VEL * speed
    estimated_degrees = actual_vel * duration * 57.3

    print(f"Rotating RIGHT {'(backward)' if backward else '(forward)'} for {duration:.1f}s (~{estimated_degrees:.0f}°)")

    # Use efficient single docker exec with internal loop
    send_cmd_for_duration(0.0, angular, duration)

    print(f"Rotation complete.")
    return estimated_degrees


def rotate_degrees(degrees: float, speed: float = 1.0, backward: bool = False):
    """
    Rotate the robot by a specific number of degrees.

    Args:
        degrees: Angle to rotate (positive = left, negative = right)
        speed: Command speed (0.1 to 1.0, default max)
        backward: If True, rotate in the "backward" direction

    Returns:
        float: The degrees requested (actual rotation may vary)
    """
    if abs(degrees) < 1:
        return 0.0

    # Calculate duration using CALIBRATED angular velocity
    # actual_vel = ACTUAL_MAX_ANGULAR_VEL * speed (in rad/s)
    # duration = angle_rad / actual_vel
    actual_vel = ACTUAL_MAX_ANGULAR_VEL * speed
    angle_rad = abs(degrees) * (3.14159 / 180)
    duration = angle_rad / actual_vel

    print(f"Rotating {abs(degrees):.0f}° {'left' if degrees > 0 else 'right'} (duration: {duration:.1f}s)")

    if degrees > 0:
        rotate_left(duration=duration, speed=speed, backward=backward)
    else:
        rotate_right(duration=duration, speed=speed, backward=backward)

    return degrees
