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
# BUT the LiDAR driver crops angles 225°-315° (rear) AND there's a blind spot at 60°-120°
# Based on actual scan analysis:
#   LiDAR sector 0-1 (0°-60°): CLEAR - actual readings
#   LiDAR sector 2-3 (60°-120°): NO DATA - blind spot/cropped
#   LiDAR sector 4-8 (120°-270°): CLEAR - actual readings
#   LiDAR sector 9-11 (270°-360°): CLEAR - close obstacles
#
# The robot should drive towards sectors WITH DATA, not blind spots!
# After 90° TF rotation:
#   Robot FRONT = LiDAR ~270° (sector 9) - has data
#   Robot BACK = LiDAR ~90° (sector 3) - NO DATA (blind spot)
#   Robot LEFT = LiDAR ~180° (sector 6) - has data
#   Robot RIGHT = LiDAR ~0° (sector 0) - has data
#
# So robot_sector = (lidar_sector + 9) % 12  (270° offset, not 90°)
LIDAR_ROTATION_SECTORS = 9  # LiDAR 270° = Robot front


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
        self.side_clearance = 0.4  # Require clearance on sides too

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

    def is_sector_safe(self, sector: int) -> bool:
        """Check if a sector and its neighbors are safe (not a narrow passage)"""
        dist = self.sector_distances[sector]
        if dist < self.min_distance:
            return False

        # Also check adjacent sectors to avoid narrow passages (like closets)
        left_neighbor = (sector + 1) % NUM_SECTORS
        right_neighbor = (sector - 1) % NUM_SECTORS

        # Require some clearance on sides too
        left_dist = self.sector_distances[left_neighbor]
        right_dist = self.sector_distances[right_neighbor]

        # If going into a narrow corridor (both sides close), avoid it
        if left_dist < self.side_clearance and right_dist < self.side_clearance:
            return False

        return True

    def find_best_direction(self) -> Tuple[int, float]:
        """
        Find the best direction (sector) to move.

        Returns:
            (sector_index, angular_velocity)
            sector_index: 0 = front, 6 = back
            angular_velocity: positive = left, negative = right
        """
        # Find all safe sectors (distance > threshold AND not narrow passage)
        clear_sectors = []
        for i in range(NUM_SECTORS):
            if self.is_sector_safe(i):
                clear_sectors.append(i)

        if not clear_sectors:
            # No safe path - find widest opening (max distance with best side clearance)
            def opening_score(sector):
                dist = self.sector_distances[sector]
                left = self.sector_distances[(sector + 1) % NUM_SECTORS]
                right = self.sector_distances[(sector - 1) % NUM_SECTORS]
                return dist + (left + right) * 0.3  # Prefer wider openings

            best_sector = max(range(NUM_SECTORS), key=opening_score)
            self.obstacles_avoided += 1
        else:
            # Find clear sector requiring minimal rotation from front (sector 0)
            # Sector 0 = front, sectors 1-5 = left, sectors 7-11 = right, 6 = back
            def rotation_cost(sector):
                if sector <= NUM_SECTORS // 2:
                    return sector  # Left turn
                else:
                    return NUM_SECTORS - sector  # Right turn

            best_sector = min(clear_sectors, key=rotation_cost)

        # Calculate angular velocity to face the chosen sector
        if best_sector == 0:
            # Already facing the best direction
            return 0, 0.0
        elif best_sector <= NUM_SECTORS // 2:
            # Turn left (positive angular velocity)
            return best_sector, self.turn_speed
        else:
            # Turn right (negative angular velocity)
            return best_sector, -self.turn_speed

    def compute_velocity(self) -> Tuple[float, float]:
        """Compute velocity command based on current scan"""
        front_dist = self.sector_distances[0]
        front_left_dist = self.sector_distances[1]
        front_right_dist = self.sector_distances[NUM_SECTORS - 1]

        best_sector, turn_angular = self.find_best_direction()

        # Determine linear and angular velocity
        if front_dist < self.min_distance:
            # Obstacle directly ahead - stop and rotate towards clear sector
            linear = 0.0
            angular = turn_angular if turn_angular != 0 else self.turn_speed
            self.obstacles_avoided += 1
            status = f"[BLOCKED] Front={front_dist:.2f}m -> sector {best_sector}"
        elif front_dist < self.min_distance * 1.5:
            # Slow down and turn towards clearer path
            linear = self.linear_speed * 0.5
            angular = turn_angular * 0.7  # Turn while moving slowly
            self.total_rotations += 1 if turn_angular != 0 else 0
            status = f"[SLOW+TURN] Front={front_dist:.2f}m -> sector {best_sector}"
        elif best_sector != 0:
            # Need to rotate to face clearer direction
            linear = self.linear_speed * 0.5
            angular = turn_angular
            self.total_rotations += 1
            status = f"[ROTATE] -> sector {best_sector} ({self.sector_distances[best_sector]:.2f}m)"
        else:
            # Path is clear - full speed with slight wandering
            linear = self.linear_speed
            if front_left_dist > front_right_dist + 0.3:
                angular = 0.1  # Wander slightly left
            elif front_right_dist > front_left_dist + 0.3:
                angular = -0.1  # Wander slightly right
            else:
                angular = 0.0
            status = f"[SCAN] Clear, front={front_dist:.2f}m"

        print(f"\r{status:<60}", end="", flush=True)
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


def start_slam_rviz():
    """Start SLAM and RViz using rviz.sh slam-opt"""
    import os

    script_dir = os.path.dirname(os.path.abspath(__file__))
    rviz_script = os.path.join(script_dir, 'rviz.sh')

    if not os.path.exists(rviz_script):
        print(f"Warning: rviz.sh not found at {rviz_script}")
        return None

    # Get DISPLAY from environment or use default
    display = os.environ.get('DISPLAY', '192.168.0.81:0.0')
    os.environ['DISPLAY'] = display

    print(f"Starting SLAM with RViz (DISPLAY={display})...")
    print("This will open RViz in a separate process...")

    # Start rviz.sh slam-opt in background
    env = os.environ.copy()
    env['DISPLAY'] = display

    proc = subprocess.Popen(
        ['bash', rviz_script, 'slam-opt'],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True  # Detach from parent - keeps running after script ends
    )

    print("Waiting for SLAM and RViz to initialize...")
    time.sleep(12)  # Give time for bringup + SLAM + RViz

    return proc


def main():
    parser = argparse.ArgumentParser(
        description='Autonomous scanning with sector-based obstacle avoidance',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ./auto_scan.py                      # Default: 60s at 0.10 m/s with SLAM
  ./auto_scan.py --duration 120       # 2 minute scan
  ./auto_scan.py --duration 0         # Unlimited
  ./auto_scan.py --speed 0.08         # Slower scanning
  ./auto_scan.py --min-dist 0.5       # More cautious
  ./auto_scan.py --no-slam            # Skip SLAM/RViz startup

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
    parser.add_argument('--no-slam', action='store_true',
                        help='Skip automatic SLAM/RViz startup')
    args = parser.parse_args()

    # Start SLAM and RViz first (unless --no-slam)
    rviz_proc = None
    if not args.no_slam:
        rviz_proc = start_slam_rviz()
        if rviz_proc:
            print("SLAM and RViz started. RViz will remain open after scan completes.")
        else:
            print("Warning: Could not start SLAM/RViz. Continuing without visualization.")

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
