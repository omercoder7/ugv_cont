#!/usr/bin/env python3
"""
Movement Calibration Script for UGV Robot

Tests and calibrates:
1. Rotation (angular velocity at Z=1.0)
2. Forward movement (linear velocity at X=1.0)
3. Backward movement (linear velocity at X=-1.0)

Uses odometry feedback to measure actual displacement.

Usage:
    ./calibrate_movement.py [--test rotation|forward|backward|all]
"""

import subprocess
import time
import math
import argparse
import json
import sys

CONTAINER_NAME = "ugv_rpi_ros_humble"

# Current calibration values (to be updated)
CURRENT_ANGULAR_VEL = 0.29  # rad/s at Z=1.0
CURRENT_LINEAR_VEL = 0.3    # m/s at X=1.0


def send_cmd(linear_x: float, angular_z: float):
    """Send velocity command to robot"""
    # Scale and invert (motor wiring)
    x_val = max(-1.0, min(1.0, -linear_x / 0.3))
    z_val = max(-1.0, min(1.0, -angular_z / 1.0))

    serial_cmd = f'{{"T":"13","X":{x_val:.2f},"Z":{z_val:.2f}}}'
    subprocess.run(
        ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
         f"echo '{serial_cmd}' > /dev/ttyAMA0"],
        timeout=1, capture_output=True
    )


def stop_robot():
    """Stop the robot"""
    for _ in range(3):
        send_cmd(0.0, 0.0)
        time.sleep(0.05)


def get_odom():
    """Get current odometry from ROS2"""
    try:
        result = subprocess.run(
            ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
             'source /opt/ros/humble/setup.bash && timeout 1 ros2 topic echo /odom --once 2>/dev/null'],
            capture_output=True, text=True, timeout=5
        )

        output = result.stdout

        # Parse position
        x, y, z = 0.0, 0.0, 0.0
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        lines = output.split('\n')
        in_position = False
        in_orientation = False

        for i, line in enumerate(lines):
            if 'position:' in line:
                in_position = True
                in_orientation = False
            elif 'orientation:' in line:
                in_position = False
                in_orientation = True
            elif in_position:
                if 'x:' in line:
                    x = float(line.split(':')[1].strip())
                elif 'y:' in line:
                    y = float(line.split(':')[1].strip())
                elif 'z:' in line and 'qz' not in line:
                    z = float(line.split(':')[1].strip())
                    in_position = False
            elif in_orientation:
                if 'x:' in line:
                    qx = float(line.split(':')[1].strip())
                elif 'y:' in line:
                    qy = float(line.split(':')[1].strip())
                elif 'z:' in line:
                    qz = float(line.split(':')[1].strip())
                elif 'w:' in line:
                    qw = float(line.split(':')[1].strip())
                    in_orientation = False

        # Calculate yaw from quaternion
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        return x, y, yaw
    except Exception as e:
        print(f"Error getting odom: {e}")
        return None, None, None


def calibrate_rotation():
    """Calibrate rotation by commanding rotation and asking user to measure"""
    print("\n" + "="*60)
    print("ROTATION CALIBRATION (Manual Measurement)")
    print("="*60)
    print("\nThis test will rotate the robot for a set duration.")
    print("You need to observe and measure the actual rotation.")
    print("\nINSTRUCTIONS:")
    print("  1. Mark the robot's starting orientation (use tape or note a reference)")
    print("  2. Watch the robot rotate")
    print("  3. After it stops, measure how many degrees it rotated\n")

    input("Press ENTER when ready to start rotation test...")

    # Command rotation at Z=1.0 for a set duration
    test_duration = 10.0  # 10 seconds for easier measurement
    loop_rate = 30  # Hz (matching new movement thread)
    loop_period = 1.0 / loop_rate

    print(f"\nRotating at Z=1.0 for {test_duration}s...")
    print("(Watch the robot and count rotations or estimate degrees)")

    start_time = time.time()
    while time.time() - start_time < test_duration:
        send_cmd(0.0, 1.0)  # Full angular speed
        time.sleep(loop_period)

    stop_robot()
    print("\nRotation complete!")

    # Ask user for measurement
    while True:
        try:
            user_input = input("\nHow many DEGREES did the robot rotate? (e.g., 90, 180, 360): ")
            observed_degrees = float(user_input)
            break
        except ValueError:
            print("Please enter a number")

    observed_radians = math.radians(observed_degrees)
    actual_angular_vel = observed_radians / test_duration

    print(f"\nResults:")
    print(f"  Observed rotation: {observed_degrees:.1f}°")
    print(f"  Duration: {test_duration:.1f}s")
    print(f"  Calculated angular velocity: {actual_angular_vel:.3f} rad/s at Z=1.0")
    print(f"  Previous calibration: {CURRENT_ANGULAR_VEL:.3f} rad/s")

    if CURRENT_ANGULAR_VEL > 0:
        print(f"  Difference: {((actual_angular_vel - CURRENT_ANGULAR_VEL) / CURRENT_ANGULAR_VEL * 100):.1f}%")

    return actual_angular_vel


def calibrate_forward():
    """Calibrate forward movement by driving straight and asking user to measure"""
    print("\n" + "="*60)
    print("FORWARD MOVEMENT CALIBRATION (Manual Measurement)")
    print("="*60)
    print("\nThis test will drive the robot forward for a set duration.")
    print("You need to measure the distance traveled.")
    print("\nINSTRUCTIONS:")
    print("  1. Mark the robot's starting position")
    print("  2. Watch the robot drive forward")
    print("  3. After it stops, measure distance traveled in centimeters\n")

    input("Press ENTER when ready to start forward test...")

    # Command forward at the typical speed used in auto_scan
    test_duration = 5.0  # 5 seconds
    test_speed = 0.06  # Same as auto_scan default speed
    loop_rate = 30
    loop_period = 1.0 / loop_rate

    print(f"\nDriving forward at commanded {test_speed} m/s for {test_duration}s...")
    print("(Watch and prepare to measure distance)")

    start_time = time.time()
    while time.time() - start_time < test_duration:
        send_cmd(test_speed, 0.0)
        time.sleep(loop_period)

    stop_robot()
    print("\nForward movement complete!")

    # Ask user for measurement
    while True:
        try:
            user_input = input("\nHow many CENTIMETERS did the robot move forward? (e.g., 30, 50): ")
            observed_cm = float(user_input)
            break
        except ValueError:
            print("Please enter a number")

    observed_meters = observed_cm / 100.0
    actual_linear_vel = observed_meters / test_duration

    # Calculate scaling factor for send_cmd
    # test_speed (0.06) should result in actual_linear_vel
    # Current scaling: x_val = -linear_x / 0.3
    # So at test_speed=0.06: x_val = -0.06/0.3 = -0.2
    # If actual velocity is different, we need to adjust the divisor
    effective_max_vel = test_speed * (actual_linear_vel / test_speed) if test_speed > 0 else actual_linear_vel

    print(f"\nResults:")
    print(f"  Observed distance: {observed_cm:.1f}cm ({observed_meters:.3f}m)")
    print(f"  Duration: {test_duration:.1f}s")
    print(f"  Actual velocity at cmd={test_speed}: {actual_linear_vel:.3f} m/s")
    print(f"  Expected velocity: {test_speed:.3f} m/s")
    print(f"  Ratio: {(actual_linear_vel / test_speed * 100):.1f}%")

    return actual_linear_vel


def calibrate_backward():
    """Calibrate backward movement by asking user to measure"""
    print("\n" + "="*60)
    print("BACKWARD MOVEMENT CALIBRATION (Manual Measurement)")
    print("="*60)
    print("\nThis test will drive the robot backward for a set duration.")
    print("You need to measure the distance traveled.")
    print("\nINSTRUCTIONS:")
    print("  1. Mark the robot's starting position")
    print("  2. Watch the robot drive backward")
    print("  3. After it stops, measure distance traveled in centimeters\n")

    input("Press ENTER when ready to start backward test...")

    # Command backward at same magnitude as forward test
    test_duration = 5.0
    test_speed = -0.06  # Same magnitude as forward, negative for backward
    loop_rate = 30
    loop_period = 1.0 / loop_rate

    print(f"\nDriving backward at commanded {abs(test_speed)} m/s for {test_duration}s...")
    print("(Watch and prepare to measure distance)")

    start_time = time.time()
    while time.time() - start_time < test_duration:
        send_cmd(test_speed, 0.0)
        time.sleep(loop_period)

    stop_robot()
    print("\nBackward movement complete!")

    # Ask user for measurement
    while True:
        try:
            user_input = input("\nHow many CENTIMETERS did the robot move backward? (e.g., 30, 50): ")
            observed_cm = float(user_input)
            break
        except ValueError:
            print("Please enter a number")

    observed_meters = observed_cm / 100.0
    actual_backward_vel = observed_meters / test_duration

    print(f"\nResults:")
    print(f"  Observed distance: {observed_cm:.1f}cm ({observed_meters:.3f}m)")
    print(f"  Duration: {test_duration:.1f}s")
    print(f"  Actual velocity at cmd={abs(test_speed)}: {actual_backward_vel:.3f} m/s")
    print(f"  Expected velocity: {abs(test_speed):.3f} m/s")
    print(f"  Ratio: {(actual_backward_vel / abs(test_speed) * 100):.1f}%")

    return actual_backward_vel


def run_all_calibrations():
    """Run all calibration tests and save results"""
    results = {}

    print("\n" + "#"*60)
    print("# FULL MOVEMENT CALIBRATION SUITE")
    print("#"*60)

    # Run each test
    angular_vel = calibrate_rotation()
    if angular_vel:
        results['angular_velocity_at_z1'] = angular_vel

    forward_vel = calibrate_forward()
    if forward_vel:
        results['forward_velocity_at_x1'] = forward_vel

    backward_vel = calibrate_backward()
    if backward_vel:
        results['backward_velocity_at_x1'] = backward_vel

    # Print summary
    print("\n" + "="*60)
    print("CALIBRATION SUMMARY")
    print("="*60)

    if 'angular_velocity_at_z1' in results:
        print(f"\nRotation (Z=1.0):")
        print(f"  Measured: {results['angular_velocity_at_z1']:.3f} rad/s")
        print(f"  Current:  {CURRENT_ANGULAR_VEL:.3f} rad/s")

    if 'forward_velocity_at_x1' in results:
        print(f"\nForward (X=1.0):")
        print(f"  Measured: {results['forward_velocity_at_x1']:.3f} m/s")
        print(f"  Current:  {CURRENT_LINEAR_VEL:.3f} m/s")

    if 'backward_velocity_at_x1' in results:
        print(f"\nBackward (X=-1.0):")
        print(f"  Measured: {results['backward_velocity_at_x1']:.3f} m/s")

    # Save results
    results_file = '/tmp/movement_calibration.json'
    with open(results_file, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nResults saved to: {results_file}")

    # Print recommended code changes
    print("\n" + "="*60)
    print("RECOMMENDED CODE CHANGES for auto_scan.py")
    print("="*60)

    if 'angular_velocity_at_z1' in results:
        print(f"\n# Update ACTUAL_MAX_ANGULAR_VEL:")
        print(f"ACTUAL_MAX_ANGULAR_VEL = {results['angular_velocity_at_z1']:.3f}  # rad/s at Z=1.0")

    if 'forward_velocity_at_x1' in results:
        print(f"\n# Update send_cmd linear scaling:")
        print(f"# x_val = -linear_x / {results['forward_velocity_at_x1']:.3f}")

    return results


def main():
    parser = argparse.ArgumentParser(description='Calibrate UGV movement')
    parser.add_argument('--test', '-t', choices=['rotation', 'forward', 'backward', 'all'],
                        default='all', help='Which test to run (default: all)')
    args = parser.parse_args()

    # Check if robot is ready
    print("Checking robot connection...")
    try:
        result = subprocess.run(
            ['docker', 'exec', CONTAINER_NAME, 'bash', '-c', 'ls /dev/ttyAMA0'],
            capture_output=True, timeout=5
        )
        if result.returncode != 0:
            print("ERROR: Cannot access robot serial port")
            sys.exit(1)
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    print("Robot ready!\n")

    if args.test == 'rotation':
        calibrate_rotation()
    elif args.test == 'forward':
        calibrate_forward()
    elif args.test == 'backward':
        calibrate_backward()
    else:
        run_all_calibrations()


if __name__ == '__main__':
    main()
