#!/usr/bin/env python3
"""
LiDAR Orientation Calibration Tool

Run this script when the robot's FRONT is facing a CLEAR open space.
It will analyze the LiDAR sectors and determine the correct rotation offset.

Usage:
    1. Position robot so FRONT faces open space (no obstacles within 1m)
    2. Run: python3 calibrate_lidar.py
    3. Note the recommended LIDAR_ROTATION_SECTORS value
    4. Update auto_scan.py with that value

The script assumes:
    - LiDAR angle_min = 0° (sector 0 starts at 0°)
    - Sectors go counterclockwise
    - Clear space (robot front) should map to robot sector 0
"""
import subprocess
import sys

CONTAINER_NAME = "ugv_rpi_ros_humble"
NUM_SECTORS = 12
ROBOT_RADIUS = 0.18


def get_lidar_scan():
    """Fetch current LiDAR scan data"""
    result = subprocess.run(
        ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
         '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from sensor_msgs.msg import LaserScan
rclpy.init()
node = rclpy.create_node('scan_reader')
msg = None
def cb(m): global msg; msg = m
sub = node.create_subscription(LaserScan, '/scan', cb, 1)
for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.1)
    if msg: break
if msg:
    print(','.join([f'{r:.3f}' for r in msg.ranges]))
node.destroy_node()
rclpy.shutdown()
"'''],
        capture_output=True, text=True, timeout=5
    )

    if result.stdout.strip():
        return [float(x) for x in result.stdout.strip().split(',')]
    return None


def compute_sector_distances(ranges):
    """Compute minimum distance for each LiDAR sector"""
    n = len(ranges)
    points_per_sector = n // NUM_SECTORS
    distances = []

    for sector in range(NUM_SECTORS):
        start_idx = sector * points_per_sector
        end_idx = start_idx + points_per_sector
        sector_ranges = ranges[start_idx:end_idx]

        valid = [r for r in sector_ranges if ROBOT_RADIUS < r < 10.0]
        if valid:
            distances.append(min(valid))
        else:
            any_reading = [r for r in sector_ranges if 0.02 < r < 10.0]
            if any_reading:
                distances.append(min(any_reading))
            else:
                distances.append(0.0)

    return distances


def find_clear_direction(distances):
    """Find the sector with maximum clearance (likely robot front)"""
    # Find sector with max distance
    max_dist = max(distances)
    max_sector = distances.index(max_dist)

    # Also check neighboring sectors for consistency
    # The clear direction should have multiple clear sectors
    best_sector = max_sector
    best_score = 0

    for center in range(NUM_SECTORS):
        # Score = sum of distances in 3-sector arc
        score = 0
        for offset in [-1, 0, 1]:
            sector = (center + offset) % NUM_SECTORS
            score += distances[sector]

        if score > best_score:
            best_score = score
            best_sector = center

    return best_sector, distances[best_sector]


def main():
    print("=" * 60)
    print("LiDAR ORIENTATION CALIBRATION")
    print("=" * 60)
    print()
    print("IMPORTANT: Robot FRONT must be facing CLEAR open space!")
    print("           (No obstacles within 1 meter in front)")
    print()

    # Get scan data
    print("Reading LiDAR data...")
    ranges = get_lidar_scan()

    if not ranges:
        print("ERROR: Could not read LiDAR data")
        print("Make sure the container is running and LiDAR is active")
        sys.exit(1)

    # Compute sector distances
    distances = compute_sector_distances(ranges)

    # Display all sectors
    print("\n" + "=" * 60)
    print("LiDAR SECTOR DISTANCES (raw LiDAR frame)")
    print("=" * 60)

    for i, dist in enumerate(distances):
        angle_start = i * 30
        angle_end = (i + 1) * 30
        bar = "█" * min(int(dist * 5), 15)
        status = "CLEAR" if dist >= 0.8 else ("close" if dist > 0.2 else "BLIND/BODY")
        print(f"  LiDAR {i:2d} ({angle_start:3d}°-{angle_end:3d}°): {dist:5.2f}m {bar:15s} {status}")

    # Find the clear direction
    clear_sector, clear_dist = find_clear_direction(distances)

    print("\n" + "=" * 60)
    print("ANALYSIS")
    print("=" * 60)
    print(f"Clearest direction: LiDAR sector {clear_sector} ({clear_sector * 30}°-{(clear_sector + 1) * 30}°)")
    print(f"Distance in clear direction: {clear_dist:.2f}m")

    # Calculate required rotation offset
    # We want: robot_sector_0 (FRONT) = lidar_sector_clear
    # Formula: robot_sector = (lidar_sector + offset) % 12
    # So: 0 = (clear_sector + offset) % 12
    # Therefore: offset = (12 - clear_sector) % 12
    #
    # BUT in the code we use: lidar_sector = (robot_sector - offset) % 12
    # So robot_sector_0 → lidar_sector = (0 - offset) % 12 = -offset % 12 = (12 - offset) % 12
    # We want this to equal clear_sector
    # So: (12 - offset) % 12 = clear_sector
    # Therefore: offset = (12 - clear_sector) % 12

    recommended_offset = (12 - clear_sector) % 12

    # Also calculate with the lidar_to_robot formula for verification
    # lidar_to_robot: robot = (lidar + offset) % 12
    # We want: when lidar = clear_sector, robot = 0
    # So: 0 = (clear_sector + offset) % 12
    # offset = -clear_sector % 12 = (12 - clear_sector) % 12

    print(f"\nTo map this to ROBOT sector 0 (FRONT):")
    print(f"  LIDAR_ROTATION_SECTORS = {recommended_offset}")

    # Verify the mapping
    print("\n" + "=" * 60)
    print("VERIFICATION (with recommended offset)")
    print("=" * 60)

    labels = ["FRONT", "F-L", "LEFT", "L-BACK", "BACK-L", "BACK",
              "BACK-R", "R-BACK", "RIGHT", "R-F", "F-R", "FRONT-R"]

    for robot_sector in range(NUM_SECTORS):
        lidar_sector = (robot_sector - recommended_offset) % NUM_SECTORS
        dist = distances[lidar_sector]
        bar = "█" * min(int(dist * 5), 15)
        print(f"  Robot {robot_sector:2d} {labels[robot_sector]:7s} <- LiDAR {lidar_sector:2d}: {dist:5.2f}m {bar}")

    print("\n" + "=" * 60)
    print("RECOMMENDATION")
    print("=" * 60)
    print(f"\nUpdate auto_scan.py line ~48:")
    print(f"  LIDAR_ROTATION_SECTORS = {recommended_offset}")
    print()

    # Check if current value matches
    print("Current value in auto_scan.py:")
    try:
        with open('/home/ws/ugv_cont/auto_scan.py', 'r') as f:
            for line in f:
                if 'LIDAR_ROTATION_SECTORS' in line and '=' in line and not line.strip().startswith('#'):
                    print(f"  {line.strip()}")
                    current = int(line.split('=')[1].split('#')[0].strip())
                    if current == recommended_offset:
                        print("\n  ✓ Current value is CORRECT!")
                    else:
                        print(f"\n  ✗ Current value ({current}) differs from recommended ({recommended_offset})")
                        print("    Consider updating if robot front is correctly facing clear space")
                    break
    except:
        pass


if __name__ == '__main__':
    main()
