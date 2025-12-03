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

        # Stuck detection
        self.last_position = None  # (x, y) from odometry
        self.last_position_time = 0
        self.stuck_counter = 0
        self.blocked_sectors = set()  # Sectors that led to being stuck
        self.stuck_cooldown = 0  # Cooldown after recovering from stuck

        # Smoothing parameters (based on DWA/VFH research)
        # Exponential Moving Average for velocity smoothing
        self.ema_alpha = 0.3  # Lower = smoother but slower response (0.2-0.4 typical)
        self.last_linear = 0.0
        self.last_angular = 0.0

        # Acceleration limits (m/s² and rad/s²)
        self.max_linear_accel = 0.15  # Max change in linear velocity per second
        self.max_angular_accel = 0.8  # Max change in angular velocity per second
        self.last_cmd_time = time.time()

        # State hysteresis to prevent rapid state switching
        self.current_state = "FWD"
        self.state_hold_counter = 0
        self.min_state_hold = 2  # Hold state for at least N iterations

    def smooth_velocity(self, target_linear: float, target_angular: float) -> Tuple[float, float]:
        """
        Apply smoothing to velocity commands using:
        1. Exponential Moving Average (EMA) for smooth transitions
        2. Acceleration limiting to prevent jerky motion

        Based on DWA/VFH research for smooth robot motion.
        """
        current_time = time.time()
        dt = current_time - self.last_cmd_time
        self.last_cmd_time = current_time

        # Clamp dt to reasonable range
        dt = max(0.05, min(dt, 0.5))

        # 1. Apply acceleration limits
        max_linear_change = self.max_linear_accel * dt
        max_angular_change = self.max_angular_accel * dt

        # Limit linear velocity change
        linear_diff = target_linear - self.last_linear
        if abs(linear_diff) > max_linear_change:
            target_linear = self.last_linear + math.copysign(max_linear_change, linear_diff)

        # Limit angular velocity change
        angular_diff = target_angular - self.last_angular
        if abs(angular_diff) > max_angular_change:
            target_angular = self.last_angular + math.copysign(max_angular_change, angular_diff)

        # 2. Apply Exponential Moving Average (EMA) smoothing
        # smoothed = alpha * new + (1 - alpha) * old
        smoothed_linear = self.ema_alpha * target_linear + (1 - self.ema_alpha) * self.last_linear
        smoothed_angular = self.ema_alpha * target_angular + (1 - self.ema_alpha) * self.last_angular

        # Update last values
        self.last_linear = smoothed_linear
        self.last_angular = smoothed_angular

        return smoothed_linear, smoothed_angular

    def send_cmd(self, linear_x: float, angular_z: float):
        """Send velocity command directly to serial port (bypasses ROS2)"""
        # Convert to robot's expected format: {"T":"13","X":linear,"Z":angular}
        # Scale: linear is in m/s, angular in rad/s
        # The robot expects values roughly in range -1 to 1
        # NOTE: Both linear AND angular are INVERTED due to motor wiring
        x_val = max(-1.0, min(1.0, -linear_x / 0.3))  # INVERTED + Scale to -1 to 1
        z_val = max(-1.0, min(1.0, -angular_z / 1.0))  # INVERTED + Scale to -1 to 1

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

    def send_smoothed_cmd(self, linear_x: float, angular_z: float):
        """Send velocity command with smoothing applied"""
        smoothed_linear, smoothed_angular = self.smooth_velocity(linear_x, angular_z)
        self.send_cmd(smoothed_linear, smoothed_angular)

    def stop(self):
        """Stop the robot"""
        # Reset smoothing state for clean stop
        self.last_linear = 0.0
        self.last_angular = 0.0
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

    def get_odometry(self) -> Optional[Tuple[float, float]]:
        """Get current position from odometry"""
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
    rclpy.spin_once(node, timeout_sec=0.05)
    if msg: break
if msg:
    print(f'{msg.pose.pose.position.x:.4f},{msg.pose.pose.position.y:.4f}')
node.destroy_node()
rclpy.shutdown()
"'''],
                capture_output=True, text=True, timeout=3
            )
            if result.stdout.strip():
                parts = result.stdout.strip().split(',')
                return (float(parts[0]), float(parts[1]))
            return None
        except:
            return None

    def check_if_stuck(self, is_driving: bool) -> bool:
        """
        Check if robot is stuck (driving but not moving).
        Returns True if stuck is detected.
        """
        if not is_driving:
            # Not trying to move, so can't be stuck
            self.stuck_counter = 0
            return False

        # Decrease cooldown
        if self.stuck_cooldown > 0:
            self.stuck_cooldown -= 1
            return False

        current_pos = self.get_odometry()
        current_time = time.time()

        if current_pos is None:
            return False

        if self.last_position is None:
            self.last_position = current_pos
            self.last_position_time = current_time
            return False

        # Check movement over last ~1 second (5 iterations at 0.2s each)
        time_diff = current_time - self.last_position_time
        if time_diff < 1.0:
            return False

        # Calculate distance moved
        dx = current_pos[0] - self.last_position[0]
        dy = current_pos[1] - self.last_position[1]
        distance = math.sqrt(dx*dx + dy*dy)

        # Expected distance at current speed (with some margin)
        expected_distance = self.linear_speed * time_diff * 0.3  # 30% of expected

        # Update position tracking
        self.last_position = current_pos
        self.last_position_time = current_time

        if distance < expected_distance:
            # Not moving as expected
            self.stuck_counter += 1
            if self.stuck_counter >= 2:  # Stuck for 2+ checks (~2 seconds)
                return True
        else:
            # Moving fine, reset counter and clear blocked sectors gradually
            self.stuck_counter = 0
            if self.blocked_sectors and time_diff > 5.0:
                # Clear one blocked sector after moving well for a while
                self.blocked_sectors.pop() if self.blocked_sectors else None

        return False

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

    def sector_to_angle(self, sector: int) -> float:
        """Convert sector number to angle in radians (0 = front, positive = left)"""
        # Sector 0 = front (0°), sectors go counterclockwise
        # Sector 6 = back (180°)
        if sector <= 6:
            return sector * (math.pi / 6)  # 0 to π
        else:
            return (sector - 12) * (math.pi / 6)  # -π to 0

    def find_best_direction(self) -> Tuple[int, float]:
        """
        VFH-inspired algorithm: Find the best direction using a cost function.

        Cost = target_weight * target_cost
             + current_weight * current_cost
             + previous_weight * previous_cost

        Returns (best_sector, best_score)
        """
        # Weights (inspired by VFH)
        TARGET_WEIGHT = 3.0      # Prefer forward direction
        CLEARANCE_WEIGHT = 2.0   # Prefer clear paths
        PREVIOUS_WEIGHT = 1.0    # Smooth trajectory (reduce oscillation)

        best_sector = 0
        best_score = -999

        for sector in range(NUM_SECTORS):
            dist = self.sector_distances[sector]

            # Skip blind spots and blocked sectors
            if dist < self.min_distance:
                continue

            # Skip sectors that led to being stuck (invisible obstacles)
            if sector in self.blocked_sectors:
                continue

            # Calculate angle from front (0 = front, ±π = back)
            angle = abs(self.sector_to_angle(sector))

            # Target cost: prefer front direction (angle = 0)
            target_cost = 1.0 - (angle / math.pi)  # 1.0 at front, 0.0 at back

            # Clearance cost: prefer more clearance
            clearance_cost = min(dist / 2.0, 1.0)  # Normalize to 0-1

            # Previous direction cost: prefer continuing in same direction
            if hasattr(self, 'previous_sector'):
                prev_angle = abs(self.sector_to_angle(self.previous_sector))
                curr_angle = abs(self.sector_to_angle(sector))
                # Smaller difference = higher score
                angle_diff = abs(curr_angle - prev_angle)
                previous_cost = 1.0 - (angle_diff / math.pi)
            else:
                previous_cost = 0.5  # Neutral for first iteration

            # Combined score
            score = (TARGET_WEIGHT * target_cost +
                    CLEARANCE_WEIGHT * clearance_cost +
                    PREVIOUS_WEIGHT * previous_cost)

            if score > best_score:
                best_score = score
                best_sector = sector

        return best_sector, best_score

    def compute_velocity(self) -> Tuple[float, float]:
        """
        VFH-inspired smooth obstacle avoidance.

        Key improvements over simple algorithm:
        1. Proportional steering (not just hard left/right)
        2. Speed reduction when turning
        3. Previous direction weight to reduce oscillation
        4. Smooth transitions between states
        5. Backward motion when too close to obstacles
        """
        front_dist = self.sector_distances[0]
        front_left = self.sector_distances[1] if self.sector_distances[1] > 0.01 else 0
        front_right = self.sector_distances[11] if self.sector_distances[11] > 0.01 else 0

        # Find best direction using VFH-style cost function
        best_sector, best_score = self.find_best_direction()
        best_dist = self.sector_distances[best_sector]

        # Calculate steering angle (proportional to sector offset)
        # Sector 0 = 0°, Sector 1 = +30°, Sector 11 = -30°
        steer_angle = self.sector_to_angle(best_sector)

        # Check if front arc (sectors 11, 0, 1) is reasonably clear
        front_arc_clear = (front_dist >= self.min_distance or
                          front_left >= self.min_distance or
                          front_right >= self.min_distance)

        # Danger zone threshold - back up if anything is closer than this
        DANGER_DISTANCE = 0.3  # 30cm - too close!

        # Check minimum distance in front arc
        front_arc_min = min(
            front_dist if front_dist > 0.01 else 10.0,
            front_left if front_left > 0.01 else 10.0,
            front_right if front_right > 0.01 else 10.0
        )

        if front_arc_min < DANGER_DISTANCE:
            # TOO CLOSE! Back up while turning towards best clear direction
            linear = -self.linear_speed * 0.6
            # Turn towards the best clear direction (using VFH cost function)
            # This ensures we back up AND orient towards where we want to go
            if best_sector <= 5:
                angular = self.turn_speed   # Best is on left side, turn left
            else:
                angular = -self.turn_speed  # Best is on right side, turn right
            self.obstacles_avoided += 1
            status = f"[DANGER] {front_arc_min:.2f}m! backing -> s{best_sector}"

        elif front_dist >= self.min_distance:
            # Front is clear - drive forward with slight steering adjustment
            linear = self.linear_speed

            # Apply gentle steering towards best direction if it's not straight ahead
            if best_sector == 0:
                angular = 0.0
            elif best_sector <= 2 or best_sector >= 10:
                # Best is slightly off-center - gentle correction
                angular = steer_angle * 0.3  # Proportional, damped
            else:
                # Best is significantly off-center - prepare to turn
                angular = steer_angle * 0.2

            status = f"[FWD] f={front_dist:.1f}m best=s{best_sector} steer={math.degrees(angular):.0f}°"

        elif front_arc_clear and best_dist >= self.min_distance:
            # Front blocked but can steer around - drive with steering
            # Reduce speed proportionally to how much we need to turn
            turn_factor = abs(steer_angle) / math.pi  # 0 to 1
            linear = self.linear_speed * (1.0 - turn_factor * 0.5)  # Slow down when turning
            angular = steer_angle * 0.5  # Proportional steering

            status = f"[STEER] f={front_dist:.1f}m -> s{best_sector} ang={math.degrees(angular):.0f}°"

        elif best_dist >= self.min_distance:
            # Need to turn in place towards clear direction
            # Back up slightly while turning if front is very close
            if front_dist < self.min_distance * 0.8:
                linear = -self.linear_speed * 0.3  # Gentle backward
            else:
                linear = 0.0
            # Turn at fixed rate towards best sector
            if best_sector <= 6:
                angular = self.turn_speed  # Turn left
            else:
                angular = -self.turn_speed  # Turn right

            self.total_rotations += 1
            status = f"[TURN] f={front_dist:.1f}m -> s{best_sector}({best_dist:.1f}m)"

        else:
            # Everything blocked - back up while turning
            linear = -self.linear_speed * 0.5
            angular = self.turn_speed
            self.obstacles_avoided += 1
            status = f"[BACK] blocked, best=s{best_sector}({best_dist:.1f}m)"

        # Remember this direction for next iteration (smooth trajectory)
        self.previous_sector = best_sector

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
        print(f"Stuck detection: Enabled (uses odometry)")
        print(f"Motion smooth:   EMA(α={self.ema_alpha}) + accel limits")
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

                # Check if stuck (driving but not moving)
                is_driving = abs(linear) > 0.01
                if self.check_if_stuck(is_driving):
                    # Robot is stuck on invisible obstacle!
                    # Mark current direction as blocked and back up
                    if hasattr(self, 'previous_sector'):
                        self.blocked_sectors.add(self.previous_sector)
                        # Also block adjacent sectors
                        self.blocked_sectors.add((self.previous_sector + 1) % NUM_SECTORS)
                        self.blocked_sectors.add((self.previous_sector - 1) % NUM_SECTORS)

                    print(f"\n*** STUCK DETECTED! Blocking sectors: {self.blocked_sectors} ***")
                    self.obstacles_avoided += 1
                    self.stuck_counter = 0
                    self.stuck_cooldown = 10  # Wait 10 iterations before checking again

                    # Back up and turn
                    for _ in range(10):  # Back up for ~2 seconds
                        self.send_cmd(-self.linear_speed * 0.5, self.turn_speed)
                        time.sleep(0.2)
                    self.stop()

                    # Reset position tracking after recovery maneuver
                    self.last_position = None
                    continue

                # Send smoothed velocity command
                self.send_smoothed_cmd(linear, angular)

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
            print(f"Stuck detections:   {len(self.blocked_sectors) // 3}")  # Approx, since we block 3 at a time
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
