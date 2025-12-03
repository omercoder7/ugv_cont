#!/usr/bin/env python3
"""
Autonomous Scanning Mode for UGV Robot
Based on sector-based obstacle avoidance algorithm

Divides LiDAR scan into 12 sectors (30° each), finds clearest path,
and navigates with minimal rotation.

Usage:
    ./auto_scan.py [--speed 0.15] [--min-dist 0.4] [--duration 60]

Emergency Stop:
    - Press SPACE or 's' to immediately stop
    - Press 'r' to resume after stop
    - Press 'q' to quit
    - Press Ctrl+C to force quit

References:
    - github.com/Rad-hi/Obstacle-Avoidance-ROS (algorithm concept)
    - github.com/vinay06vinay/Turtlebot3-Obstacle-Avoidance-ROS2
"""
import sys
import subprocess
import signal
import time
import math
import argparse
import threading
import select
import termios
import tty
from typing import List, Tuple, Optional

CONTAINER_NAME = "ugv_rpi_ros_humble"
NUM_SECTORS = 12  # 360° / 12 = 30° per sector

# LiDAR is mounted rotated 90° (1.571 rad) in URDF
# Based on actual scan analysis:
#   LiDAR sector 2-3 (60°-120°): NO DATA - blind spot
#   Other sectors: have data
#
# The robot should drive towards sectors WITH DATA, not blind spots!
# Mapping: Robot FRONT = LiDAR sector 9 (270°)
#          Robot BACK = LiDAR sector 3 (90°) - blind spot
#
# To convert: robot_sector = (lidar_sector + 3) % 12
#   - lidar sector 9 (robot front) -> robot sector 0
#   - lidar sector 3 (blind) -> robot sector 6 (back)
LIDAR_ROTATION_SECTORS = 3  # Add 3 to lidar sector to get robot sector


class KeyboardMonitor:
    """Non-blocking keyboard input monitor"""

    def __init__(self):
        self.running = True
        self.emergency_stop = False
        self.quit_requested = False
        self.paused = False
        self.old_settings = None

    def start(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        self.thread = threading.Thread(target=self._monitor, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except:
                pass

    def _monitor(self):
        try:
            tty.setcbreak(sys.stdin.fileno())
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key in ' sS':
                        self.emergency_stop = True
                        self.paused = True
                        print("\n*** EMERGENCY STOP - Press 'r' to resume ***")
                    elif key in 'qQ':
                        self.quit_requested = True
                        print("\n*** QUIT REQUESTED ***")
                    elif key in 'rR':
                        self.emergency_stop = False
                        self.paused = False
                        print("\n*** RESUMING ***")
                    elif key in 'pP':
                        self.paused = not self.paused
                        print(f"\n*** {'PAUSED' if self.paused else 'RESUMED'} ***")
        except:
            pass
        finally:
            if self.old_settings:
                try:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                except:
                    pass


class SectorObstacleAvoider:
    """
    Sector-based obstacle avoidance.

    Algorithm:
    1. Divide 360° LiDAR scan into 12 sectors (30° each)
    2. For each sector, find minimum distance to obstacle
    3. Find sectors where min distance > threshold (clear paths)
    4. Choose clearest path requiring minimal rotation
    5. Rotate towards that direction, then move forward
    """

    def __init__(self, linear_speed: float = 0.10, min_distance: float = 0.5,
                 duration: float = 60.0):
        self.linear_speed = min(linear_speed, 0.12)  # Cap max speed at 0.12 m/s
        self.min_distance = max(min_distance, 0.5)   # Min safe distance 0.5m
        self.duration = duration
        self.running = False

        # Motion parameters
        self.turn_speed = 0.35  # Slower turning
        self.sector_degrees = 360 // NUM_SECTORS  # 30°

        # State
        self.scan_ranges: List[float] = []
        self.sector_distances: List[float] = [10.0] * NUM_SECTORS
        self.current_heading_sector = 0  # Front = sector 0

        # Keyboard monitor
        self.keyboard = KeyboardMonitor()

        # Statistics
        self.start_time = 0
        self.obstacles_avoided = 0
        self.total_rotations = 0

    def send_cmd(self, linear_x: float, angular_z: float):
        """Send velocity command directly to serial port (bypasses ROS2)"""
        # Convert to robot's expected format: {"T":"13","X":linear,"Z":angular}
        # Scale: linear is in m/s, angular in rad/s
        # The robot expects values roughly in range -1 to 1
        x_val = max(-1.0, min(1.0, linear_x / 0.3))  # Scale to -1 to 1
        z_val = max(-1.0, min(1.0, angular_z / 1.0))  # Scale to -1 to 1

        serial_cmd = f'{{"T":"13","X":{x_val:.2f},"Z":{z_val:.2f}}}'
        try:
            subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
                 f"echo '{serial_cmd}' > /dev/ttyAMA0"],
                timeout=1,
                capture_output=True
            )
        except:
            pass

    def stop(self):
        """Stop the robot"""
        for _ in range(3):
            self.send_cmd(0.0, 0.0)
            time.sleep(0.05)

    def emergency_stop(self):
        """Hardware-level emergency stop"""
        self.stop()
        try:
            subprocess.run(['./stop_robot.sh'], timeout=2,
                          capture_output=True, cwd='/home/ws/ugv_cont')
        except:
            pass

    def update_scan(self) -> bool:
        """Fetch latest LiDAR scan and compute sector distances"""
        try:
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
                self.scan_ranges = [float(x) for x in result.stdout.strip().split(',')]
                self._compute_sector_distances()
                return True
            return False

        except Exception as e:
            print(f"\nScan error: {e}")
            return False

    def lidar_to_robot_sector(self, lidar_sector: int) -> int:
        """Convert LiDAR sector to robot-frame sector (accounting for 90° rotation)"""
        # LiDAR is rotated 90° CCW, so add rotation offset
        return (lidar_sector + LIDAR_ROTATION_SECTORS) % NUM_SECTORS

    def robot_to_lidar_sector(self, robot_sector: int) -> int:
        """Convert robot-frame sector to LiDAR sector"""
        return (robot_sector - LIDAR_ROTATION_SECTORS) % NUM_SECTORS

    def _compute_sector_distances(self):
        """Compute minimum distance for each sector (in ROBOT frame, not LiDAR frame)"""
        if not self.scan_ranges:
            return

        n = len(self.scan_ranges)
        points_per_sector = n // NUM_SECTORS

        # Robot dimensions (from URDF ugv_beast):
        #   - LiDAR at X=0.044m from center
        #   - Front wheels at X=0.083m, rear at X=-0.072m
        #   - Width ~0.14m total
        #   - Robot body/arms can extend to ~0.17m from LiDAR
        # ROBOT_RADIUS = 0.18m - anything closer is robot body
        ROBOT_RADIUS = 0.18

        # Compute distances in LiDAR frame first
        lidar_distances = [5.0] * NUM_SECTORS

        for lidar_sector in range(NUM_SECTORS):
            start_idx = lidar_sector * points_per_sector
            end_idx = start_idx + points_per_sector

            sector_ranges = self.scan_ranges[start_idx:end_idx]
            valid = [r for r in sector_ranges if ROBOT_RADIUS < r < 10.0]

            if valid:
                lidar_distances[lidar_sector] = min(valid)
            else:
                any_reading = [r for r in sector_ranges if 0.02 < r < 10.0]
                if any_reading and min(any_reading) < ROBOT_RADIUS:
                    # Only robot body detected - mark as blocked (can't drive through body)
                    lidar_distances[lidar_sector] = 0.0
                else:
                    # No readings (nan/cropped) - BLIND SPOT - mark as BLOCKED!
                    # Don't drive into areas we can't see!
                    lidar_distances[lidar_sector] = 0.0

        # Convert to robot frame (rotate by 90°)
        # Robot sector 0 (front) = LiDAR sector 3
        for robot_sector in range(NUM_SECTORS):
            lidar_sector = self.robot_to_lidar_sector(robot_sector)
            self.sector_distances[robot_sector] = lidar_distances[lidar_sector]

    def find_clearest_sector(self) -> int:
        """Find the sector with the most clearance"""
        # Score each sector by distance (prefer front if equal)
        best_sector = 0
        best_score = -1

        for sector in range(NUM_SECTORS):
            dist = self.sector_distances[sector]
            if dist < 0.01:  # Skip blind spots
                continue

            # Prefer front (sector 0) by giving it a small bonus
            # Rotation cost: 0 for front, increases as we go around
            if sector <= NUM_SECTORS // 2:
                rotation_cost = sector * 0.1  # 0 to 0.6
            else:
                rotation_cost = (NUM_SECTORS - sector) * 0.1  # 0.5 to 0.1

            score = dist - rotation_cost
            if score > best_score:
                best_score = score
                best_sector = sector

        return best_sector

    def compute_velocity(self) -> Tuple[float, float]:
        """
        Simple algorithm: Go towards the clearest path.
        If front is clear enough, go straight. Otherwise, turn towards clearest sector.
        """
        front_dist = self.sector_distances[0]

        # Find the clearest sector
        clearest = self.find_clearest_sector()
        clearest_dist = self.sector_distances[clearest]

        # If front is clear, go straight ahead
        if front_dist >= self.min_distance:
            linear = self.linear_speed
            angular = 0.0
            status = f"[FORWARD] front={front_dist:.2f}m, clearest=sector{clearest}({clearest_dist:.2f}m)"

        # Front is blocked - need to turn towards clearest path
        elif clearest_dist >= self.min_distance:
            # Turn towards the clearest sector
            # Sectors: 0=FRONT, 1-5=LEFT side, 6-11=RIGHT side
            if clearest == 0:
                # Front is clearest but blocked? Shouldn't happen, turn right
                linear = 0.0
                angular = -self.turn_speed
            elif clearest <= 5:
                # Sectors 1-5 are on LEFT side -> Turn left (positive angular)
                linear = 0.0
                angular = self.turn_speed
            else:
                # Sectors 6-11 are on RIGHT side -> Turn right (negative angular)
                linear = 0.0
                angular = -self.turn_speed
            self.total_rotations += 1
            status = f"[TURN] front={front_dist:.2f}m -> sector{clearest}({clearest_dist:.2f}m)"

        else:
            # Everything is blocked - back up
            linear = -self.linear_speed * 0.5
            angular = self.turn_speed  # Turn while backing up
            self.obstacles_avoided += 1
            status = f"[BACKUP] all blocked, best={clearest_dist:.2f}m"

        print(f"\r{status:<65}", end="", flush=True)
        return linear, angular

    def print_sector_display(self):
        """Print a visual representation of sector distances (in ROBOT frame)"""
        print("\n" + "="*55)
        print("Sector Distances (ROBOT FRAME - sector 0 = FRONT):")
        print("  LiDAR rotation corrected. Blind spots marked as 0.00m")
        # Labels for 12 sectors starting from front, going counterclockwise
        labels = ["FRONT", "F-L", "LEFT", "L-BACK", "BACK-L", "BACK",
                  "BACK-R", "R-BACK", "RIGHT", "R-F", "F-R", "FRONT-R"]
        for i in range(NUM_SECTORS):
            dist = self.sector_distances[i]
            if dist < 0.01:
                bar = "BLIND"
                clear = "⊘"
            else:
                bar = "█" * min(int(dist * 5), 12)
                clear = "✓" if dist > self.min_distance else "✗"
            print(f"  {i:2d} {labels[i]:7s}: {dist:5.2f}m {bar:12s} {clear}")
        print("="*55)

    def run(self):
        """Main control loop"""
        self.running = True
        self.start_time = time.time()

        print("\n" + "="*60)
        print("AUTONOMOUS SCANNING MODE (Sector-based)")
        print("="*60)
        print(f"Linear speed:    {self.linear_speed} m/s")
        print(f"Min distance:    {self.min_distance} m")
        print(f"Scan duration:   {'unlimited' if self.duration == 0 else f'{self.duration}s'}")
        print(f"Sectors:         {NUM_SECTORS} ({self.sector_degrees}° each)")
        print(f"LiDAR rotation:  90° (corrected in software)")
        print("-"*60)
        print("Controls:")
        print("  SPACE/s  - Emergency stop")
        print("  p        - Pause/Resume")
        print("  r        - Resume after stop")
        print("  q        - Quit")
        print("="*60 + "\n")

        self.keyboard.start()
        last_status_time = time.time()

        try:
            while self.running:
                elapsed = time.time() - self.start_time

                # Check duration limit
                if self.duration > 0 and elapsed >= self.duration:
                    print(f"\n\nDuration ({self.duration}s) completed!")
                    break

                # Check quit
                if self.keyboard.quit_requested:
                    print("\nQuit requested")
                    break

                # Check emergency stop / pause
                if self.keyboard.emergency_stop or self.keyboard.paused:
                    self.stop()
                    time.sleep(0.2)
                    continue

                # Update sensors
                if not self.update_scan():
                    time.sleep(0.1)
                    continue

                # Compute and send velocity
                linear, angular = self.compute_velocity()
                self.send_cmd(linear, angular)

                # Print detailed status periodically
                if time.time() - last_status_time > 10:
                    self.print_sector_display()
                    last_status_time = time.time()

                # Control loop rate (~5 Hz)
                time.sleep(0.2)

        except KeyboardInterrupt:
            print("\n\nInterrupted (Ctrl+C)")
        finally:
            print("\n\nStopping...")
            self.emergency_stop()
            self.keyboard.stop()
            self.running = False

            # Summary
            total_time = time.time() - self.start_time
            print("\n" + "="*60)
            print("SCAN SUMMARY")
            print("="*60)
            print(f"Total time:         {total_time:.1f}s")
            print(f"Obstacles avoided:  {self.obstacles_avoided}")
            print(f"Direction changes:  {self.total_rotations}")
            print("="*60)


def check_prerequisites():
    """Check if container and required topics are available"""
    result = subprocess.run(
        ['docker', 'ps', '--filter', f'name={CONTAINER_NAME}', '--format', '{{.Names}}'],
        capture_output=True, text=True
    )
    if CONTAINER_NAME not in result.stdout:
        print(f"Error: Container '{CONTAINER_NAME}' is not running.")
        print("Start it with: ./start_ugv_service.sh")
        return False

    result = subprocess.run(
        ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
         'source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null | grep /scan'],
        capture_output=True, text=True
    )
    if '/scan' not in result.stdout:
        print("Error: /scan topic not found.")
        print("Make sure bringup_lidar.launch.py is running")
        return False

    return True


def start_driver():
    """Start ugv_driver for motor control if not already running"""
    # Check if driver is already running
    result = subprocess.run(
        ['docker', 'exec', CONTAINER_NAME, 'pgrep', '-f', 'ugv_driver'],
        capture_output=True, text=True
    )
    if result.returncode == 0:
        print("ugv_driver already running")
        return True

    print("Starting ugv_driver for motor control...")
    subprocess.Popen(
        ['docker', 'exec', '-d', CONTAINER_NAME, 'bash', '-c',
         'source /opt/ros/humble/setup.bash && '
         'source /root/ugv_ws/install/setup.bash && '
         'ros2 run ugv_bringup ugv_driver'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(2)

    # Verify it started
    result = subprocess.run(
        ['docker', 'exec', CONTAINER_NAME, 'pgrep', '-f', 'ugv_driver'],
        capture_output=True, text=True
    )
    if result.returncode == 0:
        print("ugv_driver started successfully")
        return True
    else:
        print("Warning: ugv_driver may not have started properly")
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Autonomous scanning with sector-based obstacle avoidance',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ./auto_scan.py                      # Default: 60s at 0.10 m/s
  ./auto_scan.py --duration 120       # 2 minute scan
  ./auto_scan.py --duration 0         # Unlimited
  ./auto_scan.py --speed 0.08         # Slower scanning
  ./auto_scan.py --min-dist 0.5       # More cautious

Algorithm:
  Divides 360° LiDAR scan into 12 sectors (30° each).
  Finds clearest path and navigates with minimal rotation.
  Based on: github.com/Rad-hi/Obstacle-Avoidance-ROS
        """
    )
    parser.add_argument('--speed', '-s', type=float, default=0.10,
                        help='Linear speed m/s (default: 0.10, max: 0.12)')
    parser.add_argument('--min-dist', '-m', type=float, default=0.5,
                        help='Min obstacle distance m (default: 0.5)')
    parser.add_argument('--duration', '-d', type=float, default=60,
                        help='Scan duration seconds, 0=unlimited (default: 60)')
    args = parser.parse_args()

    if not check_prerequisites():
        sys.exit(1)

    # Start motor driver if needed
    start_driver()

    avoider = SectorObstacleAvoider(
        linear_speed=args.speed,
        min_distance=args.min_dist,
        duration=args.duration
    )

    def signal_handler(sig, frame):
        avoider.running = False
        avoider.emergency_stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    avoider.run()


if __name__ == '__main__':
    main()
