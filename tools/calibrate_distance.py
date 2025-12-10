#!/usr/bin/env python3
"""
LiDAR Distance Calibration Script

Place an object (like a wall or box) at a KNOWN distance from the robot's FRONT
and this script will show you what the LiDAR reports.

Usage:
    1. Place obstacle at a known distance (e.g., 30cm, 50cm, 100cm)
    2. Run: ./calibrate_distance.py
    3. Compare reported distance to actual distance
    4. Calculate correction factor if needed

The robot should be stationary and facing the obstacle directly.
"""

import subprocess
import time
import sys
import statistics

CONTAINER_NAME = "ugv_rpi_ros_humble"
NUM_SECTORS = 12
LIDAR_ROTATION_SECTORS = 3  # Same as auto_scan.py

def get_scan_ranges():
    """Get current LiDAR scan ranges"""
    try:
        result = subprocess.run(
            ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
             'source /opt/ros/humble/setup.bash && timeout 1 ros2 topic echo /scan --once 2>/dev/null'],
            capture_output=True, text=True, timeout=5
        )

        if result.returncode != 0:
            return None

        # Parse ranges from output
        output = result.stdout
        ranges_start = output.find('ranges:')
        if ranges_start == -1:
            return None

        ranges_end = output.find(']', ranges_start)
        ranges_str = output[ranges_start+8:ranges_end]

        # Parse the array
        ranges_str = ranges_str.replace('[', '').replace('\n', ' ')
        ranges = []
        for val in ranges_str.split(','):
            val = val.strip()
            if val:
                try:
                    ranges.append(float(val))
                except ValueError:
                    continue

        return ranges if len(ranges) > 0 else None

    except Exception as e:
        print(f"Error getting scan: {e}")
        return None

def ranges_to_sectors(ranges):
    """Convert raw LiDAR ranges to 12 sectors with rotation correction"""
    if not ranges:
        return [0.0] * NUM_SECTORS

    n = len(ranges)
    points_per_sector = n // NUM_SECTORS
    sector_distances = []

    for sector in range(NUM_SECTORS):
        start_idx = sector * points_per_sector
        end_idx = start_idx + points_per_sector
        sector_ranges = ranges[start_idx:end_idx]

        # Filter valid readings
        valid = [r for r in sector_ranges if 0.05 < r < 10.0]
        if valid:
            sector_distances.append(min(valid))  # Use minimum (closest obstacle)
        else:
            sector_distances.append(0.0)

    # Apply rotation correction
    corrected = [0.0] * NUM_SECTORS
    for robot_sector in range(NUM_SECTORS):
        lidar_sector = (robot_sector - LIDAR_ROTATION_SECTORS) % NUM_SECTORS
        corrected[robot_sector] = sector_distances[lidar_sector]

    return corrected

def main():
    print("=" * 60)
    print("LiDAR Distance Calibration")
    print("=" * 60)
    print("\nPlace an object at a KNOWN distance in front of the robot.")
    print("The robot should be stationary and facing the obstacle.")
    print("\nPress Enter to take a reading, or 'q' to quit.\n")

    readings = []

    while True:
        try:
            cmd = input("Press Enter to measure (or 'q' to quit): ")
            if cmd.lower() == 'q':
                break

            # Take multiple samples
            samples = []
            print("Sampling...", end="", flush=True)
            for _ in range(5):
                ranges = get_scan_ranges()
                if ranges:
                    sectors = ranges_to_sectors(ranges)
                    front_dist = sectors[0]  # Sector 0 = FRONT
                    if front_dist > 0.01:
                        samples.append(front_dist)
                print(".", end="", flush=True)
                time.sleep(0.2)
            print()

            if samples:
                avg = statistics.mean(samples)
                std = statistics.stdev(samples) if len(samples) > 1 else 0
                min_val = min(samples)
                max_val = max(samples)

                print(f"\n  FRONT sector distance:")
                print(f"    Average:  {avg:.3f} m ({avg*100:.1f} cm)")
                print(f"    Min:      {min_val:.3f} m ({min_val*100:.1f} cm)")
                print(f"    Max:      {max_val:.3f} m ({max_val*100:.1f} cm)")
                print(f"    StdDev:   {std:.4f} m")
                print(f"    Samples:  {len(samples)}")

                # Ask for actual distance
                actual = input("\n  What is the ACTUAL distance in cm? (or Enter to skip): ")
                if actual.strip():
                    try:
                        actual_m = float(actual) / 100.0
                        error = avg - actual_m
                        error_pct = (error / actual_m) * 100 if actual_m > 0 else 0

                        print(f"\n  Comparison:")
                        print(f"    Reported: {avg:.3f} m ({avg*100:.1f} cm)")
                        print(f"    Actual:   {actual_m:.3f} m ({actual} cm)")
                        print(f"    Error:    {error:.3f} m ({error*100:.1f} cm)")
                        print(f"    Error %:  {error_pct:+.1f}%")

                        readings.append({
                            'reported': avg,
                            'actual': actual_m,
                            'error': error,
                            'error_pct': error_pct
                        })
                    except ValueError:
                        print("  Invalid number, skipping comparison")
            else:
                print("  No valid readings obtained")

            print()

        except KeyboardInterrupt:
            print("\nInterrupted")
            break

    # Summary
    if readings:
        print("\n" + "=" * 60)
        print("CALIBRATION SUMMARY")
        print("=" * 60)

        for i, r in enumerate(readings, 1):
            print(f"\n  Reading {i}:")
            print(f"    Actual:   {r['actual']*100:.1f} cm")
            print(f"    Reported: {r['reported']*100:.1f} cm")
            print(f"    Error:    {r['error_pct']:+.1f}%")

        avg_error_pct = statistics.mean([r['error_pct'] for r in readings])
        print(f"\n  Average error: {avg_error_pct:+.1f}%")

        if abs(avg_error_pct) > 5:
            correction = 1 / (1 + avg_error_pct/100)
            print(f"\n  Recommended correction factor: {correction:.3f}")
            print("  (Multiply reported distances by this factor)")
        else:
            print("\n  Distances are within ±5%, no correction needed")

    print("\n" + "=" * 60)
    print("Current auto_scan.py settings:")
    print(f"  min_distance: 0.45 m (45 cm)")
    print(f"  DANGER_DISTANCE: min_distance + 0.10 = 0.55 m (55 cm)")
    print("=" * 60)

if __name__ == '__main__':
    main()
