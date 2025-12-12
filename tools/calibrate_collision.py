#!/usr/bin/env python3
"""
Collision Detection Calibration Script for UGV Robot

Calibrates the CollisionVerifier FSM by collecting movement efficiency data
in different scenarios. Supports CUSTOM-NAMED obstacles for comprehensive
calibration against different materials and surfaces.

Built-in scenarios:
- clear_path: Robot moves freely (hallway, open room)
- soft_block: Carpet edge, gentle resistance
- hard_block: Wall, furniture, immovable obstacle

Custom obstacles (examples):
- glass_door: Transparent glass that LiDAR can't see
- chair_leg: Thin metal/wood pole
- carpet_edge: Transition between surfaces
- rug: Thick rug that adds resistance
- cable: Cable on floor robot might roll over

Usage:
    ./calibrate_collision.py                        # Interactive calibration
    ./calibrate_collision.py --clear 30             # Collect clear path data
    ./calibrate_collision.py --obstacle glass 20    # Collect 20s of 'glass' data
    ./calibrate_collision.py --list                 # List all calibrated obstacles
    ./calibrate_collision.py --show                 # Show current calibration
    ./calibrate_collision.py --compute              # Recompute thresholds

Controls during collection:
    SPACE - Pause/resume data collection
    q     - Stop collection and save
"""

import subprocess
import time
import math
import argparse
import json
import sys
import os
import select
import termios
import tty
import signal
from typing import Optional, Tuple, List, Dict

CONTAINER_NAME = "ugv_rpi_ros_humble"
CALIBRATION_FILE = "/home/ws/ugv_cont/config/collision_verifier_calibration.json"

# Movement parameters for calibration
CALIBRATION_SPEED = 0.06  # m/s - slow speed for accurate measurement


class KeyboardHandler:
    """Non-blocking keyboard input handler"""

    def __init__(self):
        self.old_settings = None
        self.paused = False
        self.quit = False

    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *args):
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def check_key(self) -> Optional[str]:
        """Check for keypress without blocking"""
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == ' ':
                self.paused = not self.paused
                return 'pause'
            elif key == 'q':
                self.quit = True
                return 'quit'
            return key
        return None


def send_cmd(linear_x: float, angular_z: float):
    """Send velocity command to robot"""
    # Scale and invert (motor wiring)
    x_val = max(-1.0, min(1.0, -linear_x / 0.3))
    z_val = max(-1.0, min(1.0, -angular_z / 1.0))

    serial_cmd = f'{{"T":"13","X":{x_val:.2f},"Z":{z_val:.2f}}}'
    try:
        subprocess.run(
            ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
             f"echo '{serial_cmd}' > /dev/ttyAMA0"],
            timeout=1, capture_output=True
        )
    except subprocess.TimeoutExpired:
        pass


def stop_robot():
    """Stop the robot"""
    for _ in range(5):
        send_cmd(0.0, 0.0)
        time.sleep(0.05)


def get_position() -> Optional[Tuple[float, float]]:
    """Get current (x, y) position from odometry"""
    try:
        result = subprocess.run(
            ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
             '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from nav_msgs.msg import Odometry
rclpy.init()
node = rclpy.create_node('odom_reader')
msg = None
def cb(m): global msg; msg = m
sub = node.create_subscription(Odometry, '/odom', cb, 1)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.1)
    if msg: break
if msg:
    print(f'{msg.pose.pose.position.x:.4f},{msg.pose.pose.position.y:.4f}')
node.destroy_node()
rclpy.shutdown()
"'''],
            capture_output=True, text=True, timeout=5
        )

        if result.stdout.strip():
            parts = result.stdout.strip().split(',')
            return (float(parts[0]), float(parts[1]))
    except Exception as e:
        print(f"Error getting position: {e}")
    return None


def get_front_lidar_distance() -> float:
    """Get front LiDAR distance (average of front sectors)"""
    try:
        result = subprocess.run(
            ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
             '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from sensor_msgs.msg import LaserScan
import math
rclpy.init()
node = rclpy.create_node('scan_reader')
msg = None
def cb(m): global msg; msg = m
sub = node.create_subscription(LaserScan, '/scan', cb, 1)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.1)
    if msg: break
if msg:
    # Front sector (around 270 degrees due to LiDAR mounting)
    n = len(msg.ranges)
    front_idx = int(n * 0.75)  # 270 degrees
    front_range = 30  # +/- 30 samples
    readings = []
    for i in range(-front_range, front_range):
        idx = (front_idx + i) % n
        r = msg.ranges[idx]
        if 0.1 < r < 10:
            readings.append(r)
    if readings:
        print(f'{min(readings):.3f}')
    else:
        print('10.0')
node.destroy_node()
rclpy.shutdown()
"'''],
            capture_output=True, text=True, timeout=5
        )

        if result.stdout.strip():
            return float(result.stdout.strip())
    except Exception:
        pass
    return 10.0


def load_calibration() -> Dict:
    """Load existing calibration data"""
    if os.path.exists(CALIBRATION_FILE):
        try:
            with open(CALIBRATION_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Error loading calibration: {e}")

    # Default structure with support for custom obstacles
    return {
        'thresholds': {
            'clear_min': 0.70,
            'suspicious': 0.50,
            'blocked': 0.25
        },
        'raw_data': {
            'clear_path': [],
            'soft_block': [],
            'hard_block': []
            # Custom obstacles are added dynamically, e.g.:
            # 'glass_door': [...],
            # 'chair_leg': [...]
        },
        'obstacle_categories': {
            # Maps obstacle names to their category (soft/hard) for threshold computation
            'soft_block': 'soft',
            'hard_block': 'hard',
            'clear_path': 'clear'
            # Custom: 'glass_door': 'hard', 'carpet_edge': 'soft', etc.
        },
        'metadata': {
            'robot_id': 'ugv_beast',
            'calibrated_at': None,
            'samples_collected': 0
        }
    }


def save_calibration(data: Dict):
    """Save calibration data"""
    # Ensure directory exists
    os.makedirs(os.path.dirname(CALIBRATION_FILE), exist_ok=True)

    # Count total samples across all obstacles (including custom)
    total_samples = sum(len(samples) for samples in data['raw_data'].values())

    data['metadata']['calibrated_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
    data['metadata']['samples_collected'] = total_samples
    data['metadata']['obstacles_calibrated'] = list(data['raw_data'].keys())

    with open(CALIBRATION_FILE, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"\nCalibration saved to {CALIBRATION_FILE}")


def collect_data(label: str, duration: float, data: Dict, category: str = None) -> Dict:
    """
    Collect calibration data for a specific scenario.

    Args:
        label: Obstacle name ('clear_path', 'soft_block', 'hard_block', or custom like 'glass_door')
        duration: How long to collect data (seconds)
        data: Existing calibration data dict
        category: For custom obstacles, specify 'soft' or 'hard' to affect threshold computation
    """
    print(f"\n{'='*60}")
    print(f"Collecting '{label}' data for {duration} seconds")
    print(f"{'='*60}")

    # Ensure raw_data has entry for this obstacle
    if label not in data['raw_data']:
        data['raw_data'][label] = []

    # Handle category assignment for custom obstacles
    if 'obstacle_categories' not in data:
        data['obstacle_categories'] = {'clear_path': 'clear', 'soft_block': 'soft', 'hard_block': 'hard'}

    if label == 'clear_path':
        print("Drive the robot FORWARD in a CLEAR area (hallway, open room)")
        print("The robot will move at constant speed - ensure no obstacles ahead!")
        data['obstacle_categories'][label] = 'clear'
    elif label == 'soft_block':
        print("Drive the robot into a SOFT obstacle (carpet edge, soft mat)")
        print("The robot should experience gentle resistance but still move slowly")
        data['obstacle_categories'][label] = 'soft'
    elif label == 'hard_block':
        print("Drive the robot into a HARD obstacle (wall, furniture, glass)")
        print("The robot should be completely blocked from moving")
        data['obstacle_categories'][label] = 'hard'
    else:
        # Custom obstacle - use provided category or ask
        if category:
            data['obstacle_categories'][label] = category
            cat_name = 'HARD (fully blocks)' if category == 'hard' else 'SOFT (partial resistance)'
            print(f"Custom obstacle '{label}' categorized as: {cat_name}")
        else:
            print(f"Custom obstacle: '{label}'")
            if label not in data['obstacle_categories']:
                data['obstacle_categories'][label] = 'hard'  # Default to hard

        print(f"Drive the robot into this obstacle")
        print(f"Category: {data['obstacle_categories'].get(label, 'hard').upper()}")

    print("\nControls:")
    print("  SPACE - Pause/resume collection")
    print("  q     - Stop and finish")
    print("\nStarting in 3 seconds...")
    time.sleep(3)

    samples = data['raw_data'].get(label, [])
    start_time = time.time()

    last_position: Optional[Tuple[float, float]] = None
    last_time = time.time()
    sample_count = 0

    with KeyboardHandler() as kb:
        try:
            while time.time() - start_time < duration and not kb.quit:
                # Check keyboard
                key = kb.check_key()
                if key == 'pause':
                    stop_robot()
                    print(f"\n{'PAUSED' if kb.paused else 'RESUMED'} - press SPACE to toggle")
                    continue

                if kb.paused:
                    time.sleep(0.1)
                    continue

                # Send movement command
                send_cmd(CALIBRATION_SPEED, 0.0)

                # Get current position
                current_pos = get_position()
                current_time = time.time()

                if current_pos and last_position:
                    dt = current_time - last_time

                    if dt > 0.1:  # Sample every 100ms
                        # Calculate efficiency
                        dx = current_pos[0] - last_position[0]
                        dy = current_pos[1] - last_position[1]
                        actual_dist = math.hypot(dx, dy)
                        expected_dist = CALIBRATION_SPEED * dt

                        if expected_dist > 0.005:
                            efficiency = actual_dist / expected_dist
                            efficiency = min(efficiency, 2.0)

                            # Get front distance
                            front_dist = get_front_lidar_distance()

                            # Store sample
                            samples.append({
                                'efficiency': efficiency,
                                'front_dist': front_dist,
                                'timestamp': current_time
                            })
                            sample_count += 1

                            # Progress display
                            elapsed = current_time - start_time
                            print(f"\r  {elapsed:.1f}s | Samples: {sample_count} | "
                                  f"Efficiency: {efficiency:.0%} | Front: {front_dist:.2f}m   ", end="")

                        last_position = current_pos
                        last_time = current_time
                else:
                    last_position = current_pos
                    last_time = current_time

                time.sleep(0.05)

        finally:
            stop_robot()

    # Update data
    data['raw_data'][label] = samples
    print(f"\n\nCollected {sample_count} new samples ({len(samples)} total for '{label}')")

    return data


def compute_thresholds(data: Dict) -> Dict:
    """Compute detection thresholds from collected data (including custom obstacles)"""
    print("\n" + "="*60)
    print("Computing Thresholds from ALL Calibrated Obstacles")
    print("="*60)

    # Get categories mapping
    categories = data.get('obstacle_categories', {
        'clear_path': 'clear', 'soft_block': 'soft', 'hard_block': 'hard'
    })

    # Collect all samples by category
    clear_effs = []
    soft_effs = []
    hard_effs = []

    print(f"\nProcessing obstacles by category:")

    for obstacle_name, samples in data['raw_data'].items():
        if not samples:
            continue

        cat = categories.get(obstacle_name, 'hard')  # Default to hard
        effs = [s['efficiency'] for s in samples]

        if cat == 'clear':
            clear_effs.extend(effs)
            print(f"  CLEAR: {obstacle_name} ({len(samples)} samples)")
        elif cat == 'soft':
            soft_effs.extend(effs)
            print(f"  SOFT:  {obstacle_name} ({len(samples)} samples)")
        else:  # hard
            hard_effs.extend(effs)
            print(f"  HARD:  {obstacle_name} ({len(samples)} samples)")

    if len(clear_effs) < 10:
        print(f"\nNeed at least 10 clear_path samples (have {len(clear_effs)})")
        print("Run: ./calibrate_collision.py --clear 30")
        return data

    # Use defaults if no hard samples
    if not hard_effs:
        hard_effs = [0.1]
        print(f"\nNo hard obstacle samples - using default hard_effs=[0.1]")
        print("Run: ./calibrate_collision.py --obstacle <name> --hard 15")

    # Statistics for clear path
    clear_mean = sum(clear_effs) / len(clear_effs)
    clear_min = min(clear_effs)
    clear_variance = sum((e - clear_mean) ** 2 for e in clear_effs) / len(clear_effs)
    clear_std = math.sqrt(clear_variance) if clear_variance > 0 else 0.1

    # Statistics for hard obstacles
    hard_mean = sum(hard_effs) / len(hard_effs)
    hard_max = max(hard_effs)

    print(f"\n" + "-"*40)
    print(f"Statistics:")
    print(f"  Clear path ({len(clear_effs)} samples):")
    print(f"    Mean efficiency: {clear_mean:.0%}")
    print(f"    Min efficiency:  {clear_min:.0%}")
    print(f"    Std deviation:   {clear_std:.0%}")

    print(f"  Hard obstacles ({len(hard_effs)} samples):")
    print(f"    Mean efficiency: {hard_mean:.0%}")
    print(f"    Max efficiency:  {hard_max:.0%}")

    if soft_effs:
        soft_mean = sum(soft_effs) / len(soft_effs)
        print(f"  Soft obstacles ({len(soft_effs)} samples):")
        print(f"    Mean efficiency: {soft_mean:.0%}")

    # Compute thresholds
    clear_min_thresh = max(hard_max + 0.10, clear_mean - 2 * clear_std)
    blocked_thresh = (hard_max + clear_min_thresh) / 2 - 0.05
    suspicious_thresh = (blocked_thresh + clear_min_thresh) / 2

    # Ensure sensible bounds
    blocked_thresh = max(0.15, min(blocked_thresh, 0.40))
    suspicious_thresh = max(blocked_thresh + 0.10, min(suspicious_thresh, 0.60))
    clear_min_thresh = max(suspicious_thresh + 0.10, clear_min_thresh)

    # Update thresholds
    data['thresholds'] = {
        'clear_min': round(clear_min_thresh, 2),
        'suspicious': round(suspicious_thresh, 2),
        'blocked': round(blocked_thresh, 2)
    }

    print(f"\nComputed Thresholds:")
    print(f"  Clear path:  >= {clear_min_thresh:.0%}  (robot is moving freely)")
    print(f"  Suspicious:  < {suspicious_thresh:.0%}  (triggers slow probing)")
    print(f"  Blocked:     < {blocked_thresh:.0%}  (confirms invisible obstacle)")

    return data


def show_calibration():
    """Display current calibration"""
    data = load_calibration()

    print("\n" + "="*60)
    print("Current Collision Verifier Calibration")
    print("="*60)

    thresh = data.get('thresholds', {})
    print(f"\nThresholds:")
    print(f"  Clear path:  >= {thresh.get('clear_min', 0.70):.0%}")
    print(f"  Suspicious:  < {thresh.get('suspicious', 0.50):.0%}")
    print(f"  Blocked:     < {thresh.get('blocked', 0.25):.0%}")

    categories = data.get('obstacle_categories', {})
    raw = data.get('raw_data', {})

    # Group obstacles by category
    clear_obs = []
    soft_obs = []
    hard_obs = []

    for name, samples in raw.items():
        count = len(samples) if samples else 0
        cat = categories.get(name, 'hard')

        if cat == 'clear':
            clear_obs.append((name, count))
        elif cat == 'soft':
            soft_obs.append((name, count))
        else:
            hard_obs.append((name, count))

    print(f"\nCalibrated Obstacles:")
    if clear_obs:
        print(f"  CLEAR (baseline):")
        for name, count in clear_obs:
            print(f"    - {name}: {count} samples")
    if soft_obs:
        print(f"  SOFT (partial resistance):")
        for name, count in soft_obs:
            print(f"    - {name}: {count} samples")
    if hard_obs:
        print(f"  HARD (full block):")
        for name, count in hard_obs:
            print(f"    - {name}: {count} samples")

    meta = data.get('metadata', {})
    print(f"\nMetadata:")
    print(f"  Robot ID:      {meta.get('robot_id', 'unknown')}")
    print(f"  Calibrated at: {meta.get('calibrated_at', 'never')}")
    print(f"  Total samples: {meta.get('samples_collected', 0)}")


def list_obstacles():
    """List all calibrated obstacles with their statistics"""
    data = load_calibration()
    categories = data.get('obstacle_categories', {})
    raw = data.get('raw_data', {})

    print("\n" + "="*60)
    print("All Calibrated Obstacles")
    print("="*60)

    if not raw:
        print("\nNo obstacles calibrated yet.")
        print("Run: ./calibrate_collision.py --obstacle <name> --hard 15")
        return

    print(f"\n{'Obstacle':<20} {'Category':<10} {'Samples':<10} {'Mean Eff':<10} {'Min Eff':<10}")
    print("-" * 60)

    for name, samples in sorted(raw.items()):
        if not samples:
            continue

        cat = categories.get(name, 'hard')
        effs = [s['efficiency'] for s in samples]
        mean_eff = sum(effs) / len(effs)
        min_eff = min(effs)

        print(f"{name:<20} {cat:<10} {len(samples):<10} {mean_eff:.0%}{'':<6} {min_eff:.0%}")


def main():
    parser = argparse.ArgumentParser(
        description='Calibrate collision detection with custom obstacles',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ./calibrate_collision.py --clear 30              # Collect clear path baseline
  ./calibrate_collision.py --obstacle glass 20     # Collect 'glass' obstacle (default: hard)
  ./calibrate_collision.py --obstacle rug 15 --soft  # 'rug' as soft obstacle
  ./calibrate_collision.py --list                  # Show all calibrated obstacles
  ./calibrate_collision.py --compute               # Recompute thresholds
        """)

    parser.add_argument('--clear', type=float, metavar='SEC',
                        help='Collect clear path data for SEC seconds')
    parser.add_argument('--block', type=float, metavar='SEC',
                        help='Collect hard block data for SEC seconds')
    parser.add_argument('--soft-block', type=float, metavar='SEC', dest='soft_block',
                        help='Collect soft block data for SEC seconds')

    # Custom obstacle support
    parser.add_argument('--obstacle', nargs=2, metavar=('NAME', 'SEC'),
                        help='Collect data for custom-named obstacle for SEC seconds')
    parser.add_argument('--hard', action='store_true',
                        help='Mark custom obstacle as HARD (fully blocking) - default')
    parser.add_argument('--soft', action='store_true',
                        help='Mark custom obstacle as SOFT (partial resistance)')

    parser.add_argument('--compute', action='store_true',
                        help='Compute thresholds from existing data')
    parser.add_argument('--show', action='store_true',
                        help='Show current calibration and thresholds')
    parser.add_argument('--list', action='store_true',
                        help='List all calibrated obstacles with stats')
    parser.add_argument('--reset', action='store_true',
                        help='Reset all calibration data')

    args = parser.parse_args()

    # Handle keyboard interrupt
    def signal_handler(sig, frame):
        print("\n\nInterrupted - stopping robot...")
        stop_robot()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    if args.show:
        show_calibration()
        return

    if args.list:
        list_obstacles()
        return

    if args.reset:
        if os.path.exists(CALIBRATION_FILE):
            os.remove(CALIBRATION_FILE)
            print("Calibration reset")
        return

    # Load existing data
    data = load_calibration()

    if args.compute:
        data = compute_thresholds(data)
        save_calibration(data)
        return

    if args.clear:
        data = collect_data('clear_path', args.clear, data)
        data = compute_thresholds(data)
        save_calibration(data)
        return

    if args.block:
        data = collect_data('hard_block', args.block, data)
        data = compute_thresholds(data)
        save_calibration(data)
        return

    if args.soft_block:
        data = collect_data('soft_block', args.soft_block, data)
        data = compute_thresholds(data)
        save_calibration(data)
        return

    # Handle custom obstacle
    if args.obstacle:
        obstacle_name = args.obstacle[0]
        duration = float(args.obstacle[1])

        # Determine category (default to hard)
        category = 'soft' if args.soft else 'hard'

        data = collect_data(obstacle_name, duration, data, category=category)
        data = compute_thresholds(data)
        save_calibration(data)
        return

    # Interactive mode
    print("\n" + "="*60)
    print("Collision Detection Calibration - Interactive Mode")
    print("="*60)
    print("\nThis tool calibrates the collision verifier to detect")
    print("invisible obstacles (obstacles LiDAR can't see).")
    print("\nRecommended steps:")
    print("  1. Collect 30+ seconds of clear path data")
    print("  2. Collect 15+ seconds of obstacle data (hard or soft)")
    print("  3. Add custom obstacles as needed")
    print("  4. Compute thresholds")

    while True:
        print("\n" + "-"*40)
        print("Options:")
        print("  1. Collect clear path data (30s)")
        print("  2. Collect hard block data (15s)")
        print("  3. Collect soft block data (15s)")
        print("  4. Add CUSTOM obstacle (you name it!)")
        print("  5. Compute thresholds")
        print("  6. Show current calibration")
        print("  7. List all obstacles")
        print("  8. Reset all data")
        print("  q. Quit")

        choice = input("\nSelect option: ").strip()

        if choice == '1':
            data = collect_data('clear_path', 30, data)
            save_calibration(data)
        elif choice == '2':
            data = collect_data('hard_block', 15, data)
            save_calibration(data)
        elif choice == '3':
            data = collect_data('soft_block', 15, data)
            save_calibration(data)
        elif choice == '4':
            # Custom obstacle
            print("\n--- Add Custom Obstacle ---")
            name = input("Obstacle name (e.g., glass_door, chair_leg, rug): ").strip()
            if not name:
                print("Cancelled")
                continue

            cat_choice = input("Category - (h)ard or (s)oft? [h]: ").strip().lower()
            category = 'soft' if cat_choice == 's' else 'hard'

            try:
                duration = float(input("Collection duration (seconds) [15]: ").strip() or "15")
            except ValueError:
                duration = 15

            data = collect_data(name, duration, data, category=category)
            save_calibration(data)
        elif choice == '5':
            data = compute_thresholds(data)
            save_calibration(data)
        elif choice == '6':
            show_calibration()
        elif choice == '7':
            list_obstacles()
        elif choice == '8':
            confirm = input("Reset ALL data? (y/N): ").strip().lower()
            if confirm == 'y':
                data = {
                    'thresholds': data.get('thresholds', {}),
                    'raw_data': {},
                    'obstacle_categories': {},
                    'metadata': {'robot_id': 'ugv_beast', 'calibrated_at': None, 'samples_collected': 0}
                }
                save_calibration(data)
                print("All data reset (thresholds preserved)")
        elif choice == 'q':
            break
        else:
            print("Invalid option")


if __name__ == '__main__':
    main()
