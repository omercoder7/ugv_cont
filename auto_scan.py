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
from typing import List, Tuple, Optional, Set, Dict
import json
import os

CONTAINER_NAME = "ugv_rpi_ros_humble"

# Visited locations grid settings
GRID_RESOLUTION = 0.5  # 50cm grid cells
VISITED_FILE = "/tmp/visited_locations.json"
NUM_SECTORS = 12  # 360° / 12 = 30° per sector

# Calibration: Robot actual turn rate at Z=1.0 command
# Measured: 6.5s command gave ~110° instead of 90°, adjusted from 0.24
ACTUAL_MAX_ANGULAR_VEL = 0.29  # rad/s at Z=1.0

# LiDAR orientation calibration:
# Run calibrate_lidar.py with robot FRONT facing clear space to determine this value.
#
# Physical mounting: LiDAR 0° is offset from robot chassis front by ~270°
# Calibration shows: Robot FRONT = LiDAR sector 9 (270°-300°)
#
# Formula: robot_sector = (lidar_sector + offset) % 12
#          lidar_sector = (robot_sector - offset) % 12
# With offset=3: Robot sector 0 (FRONT) <- LiDAR sector 9
LIDAR_ROTATION_SECTORS = 3  # Calibrated: LiDAR 270° = Robot FRONT


class KeyboardMonitor:
    """Non-blocking keyboard input monitor"""

    def __init__(self):
        self.running = True
        self.emergency_stop = False
        self.quit_requested = False
        self.paused = False
        self.old_settings = None

    def start(self):
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            self.thread = threading.Thread(target=self._monitor, daemon=True)
            self.thread.start()
        except termios.error:
            # Not a TTY (e.g., running in non-interactive mode)
            print("Note: Keyboard controls disabled (non-interactive mode)")
            self.old_settings = None

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


class VisitedTracker:
    """
    Tracks visited locations using a grid-based approach.
    Divides the map into cells and marks cells as visited when the robot passes through.
    """

    def __init__(self, resolution: float = GRID_RESOLUTION):
        self.resolution = resolution  # Grid cell size in meters
        self.visited_cells: Set[Tuple[int, int]] = set()
        self.visit_counts: Dict[Tuple[int, int], int] = {}  # How many times visited
        self.last_cell: Optional[Tuple[int, int]] = None
        self.total_cells_visited = 0

        # Load existing visited locations if available
        self.load()

    def pos_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world position to grid cell coordinates"""
        cell_x = int(x / self.resolution)
        cell_y = int(y / self.resolution)
        return (cell_x, cell_y)

    def cell_to_pos(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        """Convert grid cell to world position (center of cell)"""
        x = (cell[0] + 0.5) * self.resolution
        y = (cell[1] + 0.5) * self.resolution
        return (x, y)

    def mark_visited(self, x: float, y: float) -> bool:
        """
        Mark a position as visited. Returns True if this is a new cell.
        """
        cell = self.pos_to_cell(x, y)

        # Skip if same cell as last time
        if cell == self.last_cell:
            return False

        self.last_cell = cell
        is_new = cell not in self.visited_cells

        self.visited_cells.add(cell)
        self.visit_counts[cell] = self.visit_counts.get(cell, 0) + 1

        if is_new:
            self.total_cells_visited += 1

        return is_new

    def is_visited(self, x: float, y: float) -> bool:
        """Check if a position has been visited"""
        cell = self.pos_to_cell(x, y)
        return cell in self.visited_cells

    def get_visit_count(self, x: float, y: float) -> int:
        """Get how many times a position has been visited"""
        cell = self.pos_to_cell(x, y)
        return self.visit_counts.get(cell, 0)

    def get_unvisited_direction(self, current_x: float, current_y: float,
                                 sector_distances: List[float],
                                 min_distance: float) -> Optional[int]:
        """
        Find a sector direction that leads to unvisited cells.
        Returns sector number (0-11) or None if all reachable cells are visited.

        Priority:
        1. Unvisited cells that are reachable (clear path)
        2. Less-visited cells
        3. None if all directions lead to visited areas
        """
        current_cell = self.pos_to_cell(current_x, current_y)

        best_sector = None
        best_score = -999

        for sector in range(NUM_SECTORS):
            dist = sector_distances[sector]

            # Skip blocked/close sectors
            if dist < min_distance:
                continue

            # Calculate where we'd end up going this direction
            # Sector 0 = front (0°), sector 3 = left (90°), etc.
            angle = sector * (2 * math.pi / NUM_SECTORS)

            # Look ahead by the clearance distance (capped at 2m)
            look_ahead = min(dist, 2.0)
            target_x = current_x + look_ahead * math.cos(angle)
            target_y = current_y + look_ahead * math.sin(angle)

            target_cell = self.pos_to_cell(target_x, target_y)

            # Score based on visit count (lower = better)
            visit_count = self.visit_counts.get(target_cell, 0)

            # Prefer unvisited (visit_count=0), then less visited
            # Also factor in clearance and prefer forward direction
            clearance_score = min(dist / 3.0, 1.0)  # 0-1 based on clearance
            direction_score = 1.0 - (abs(sector - 6) / 6.0) if sector <= 6 else 1.0 - (abs(sector - 6) / 6.0)
            # Prefer front (sector 0, 1, 11)
            if sector in [0, 1, 11]:
                direction_score = 1.0
            elif sector in [2, 10]:
                direction_score = 0.7
            else:
                direction_score = 0.3

            # Visit score: heavily penalize visited cells
            if visit_count == 0:
                visit_score = 10.0  # Big bonus for unvisited
            elif visit_count == 1:
                visit_score = 2.0   # Small bonus for visited once
            else:
                visit_score = -visit_count  # Penalty for revisiting

            total_score = visit_score + clearance_score + direction_score

            if total_score > best_score:
                best_score = total_score
                best_sector = sector

        return best_sector

    def get_coverage_stats(self) -> Dict:
        """Get statistics about coverage"""
        if not self.visited_cells:
            return {"cells_visited": 0, "area_covered": 0.0, "revisit_rate": 0.0}

        total_visits = sum(self.visit_counts.values())
        unique_cells = len(self.visited_cells)
        revisit_rate = (total_visits - unique_cells) / total_visits if total_visits > 0 else 0

        return {
            "cells_visited": unique_cells,
            "area_covered": unique_cells * (self.resolution ** 2),  # m²
            "total_visits": total_visits,
            "revisit_rate": revisit_rate,
        }

    def save(self):
        """Save visited locations to file"""
        data = {
            "resolution": self.resolution,
            "visited_cells": list(self.visited_cells),
            "visit_counts": {f"{k[0]},{k[1]}": v for k, v in self.visit_counts.items()},
        }
        try:
            with open(VISITED_FILE, 'w') as f:
                json.dump(data, f)
            print(f"\n[SAVE] Saved {len(self.visited_cells)} visited cells to {VISITED_FILE}")
        except Exception as e:
            print(f"\n[SAVE] Failed to save visited locations: {e}")

    def load(self):
        """Load visited locations from file"""
        if not os.path.exists(VISITED_FILE):
            return

        try:
            with open(VISITED_FILE, 'r') as f:
                data = json.load(f)

            self.resolution = data.get("resolution", GRID_RESOLUTION)
            self.visited_cells = set(tuple(c) for c in data.get("visited_cells", []))
            self.visit_counts = {
                tuple(map(int, k.split(','))): v
                for k, v in data.get("visit_counts", {}).items()
            }
            self.total_cells_visited = len(self.visited_cells)
            print(f"\n[LOAD] Loaded {len(self.visited_cells)} visited cells from {VISITED_FILE}")
        except Exception as e:
            print(f"\n[LOAD] Failed to load visited locations: {e}")

    def clear(self):
        """Clear all visited locations"""
        self.visited_cells.clear()
        self.visit_counts.clear()
        self.last_cell = None
        self.total_cells_visited = 0
        if os.path.exists(VISITED_FILE):
            os.remove(VISITED_FILE)
        print("\n[CLEAR] Cleared all visited locations")


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

    def __init__(self, linear_speed: float = 0.04, min_distance: float = 0.45,
                 duration: float = 60.0, clear_visited: bool = False):
        # Cap max speed at 0.10 m/s - rf2o odometry overestimates at higher speeds
        # causing mismatch between RViz display and actual robot position
        self.linear_speed = min(linear_speed, 0.10)
        # Reduced min distance to 0.35m floor (robot is ~17cm wide, need clearance)
        self.min_distance = max(min_distance, 0.35)
        self.duration = duration
        self.running = False

        # Motion parameters
        self.turn_speed = 0.35  # Slower turning
        self.sector_degrees = 360 // NUM_SECTORS  # 30°

        # State
        self.scan_ranges: List[float] = []
        self.sector_distances: List[float] = [10.0] * NUM_SECTORS
        self.current_heading_sector = 0  # Front = sector 0
        self.current_position: Optional[Tuple[float, float]] = None

        # Visited location tracking
        self.visited_tracker = VisitedTracker()
        if clear_visited:
            self.visited_tracker.clear()

        # Keyboard monitor
        self.keyboard = KeyboardMonitor()

        # Statistics
        self.start_time = 0
        self.obstacles_avoided = 0
        self.total_rotations = 0

        # Stuck detection (LiDAR-based, NOT odometry-based)
        self.last_position = None  # (x, y) from odometry (for virtual obstacle marking only)
        self.last_position_time = 0
        self.stuck_counter = 0
        self.stuck_time_counter = 0  # Counter for stuck detection
        self.blocked_sectors = set()  # Sectors that led to being stuck
        self.stuck_cooldown = 0  # Cooldown after recovering from stuck
        self.driving_into_obstacle_time = 0  # Time spent driving toward unchanging obstacle
        self.scan_unchanged_time = 0  # Time LiDAR scan hasn't changed while driving
        self.emergency_maneuver = False  # True when in danger/backing up mode

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

        # Avoidance direction - alternates to help navigate passages
        self.avoidance_direction = None  # "left" or "right" or None
        self.avoidance_intensity = 1.0   # Increases if still stuck after turning
        self.avoidance_start_time = 0    # When we started this avoidance maneuver
        self.last_avoidance_direction = "right"  # Track last direction to alternate
        self.avoidance_attempt_count = 0  # Count attempts in same area
        self.last_avoidance_position = None  # Position where we last avoided

        # Dead-end detection
        self.dead_end_positions: Set[Tuple[int, int]] = set()  # Grid cells marked as dead ends

        # Continuous movement thread
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.movement_lock = threading.Lock()
        self.movement_thread = None
        self.movement_running = False

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

    def is_dead_end(self) -> bool:
        """
        Detect if current position is a dead-end by checking if 3+ direction
        groups are blocked. Direction groups:
        - FRONT: sectors 11, 0, 1
        - LEFT: sectors 2, 3, 4
        - BACK: sectors 5, 6, 7
        - RIGHT: sectors 8, 9, 10

        Returns True if 3 or more direction groups are blocked.
        """
        BLOCKED_THRESHOLD = 0.5  # Consider blocked if closer than this

        # Count blocked direction groups
        blocked_groups = 0

        # FRONT (sectors 11, 0, 1)
        front_dists = [self.sector_distances[11], self.sector_distances[0], self.sector_distances[1]]
        front_valid = [d for d in front_dists if d > 0.1]  # Exclude blind spots
        if front_valid and max(front_valid) < BLOCKED_THRESHOLD:
            blocked_groups += 1

        # LEFT (sectors 2, 3, 4)
        left_dists = [self.sector_distances[2], self.sector_distances[3], self.sector_distances[4]]
        left_valid = [d for d in left_dists if d > 0.1]
        if left_valid and max(left_valid) < BLOCKED_THRESHOLD:
            blocked_groups += 1

        # BACK (sectors 5, 6, 7)
        back_dists = [self.sector_distances[5], self.sector_distances[6], self.sector_distances[7]]
        back_valid = [d for d in back_dists if d > 0.1]
        if back_valid and max(back_valid) < BLOCKED_THRESHOLD:
            blocked_groups += 1

        # RIGHT (sectors 8, 9, 10)
        right_dists = [self.sector_distances[8], self.sector_distances[9], self.sector_distances[10]]
        right_valid = [d for d in right_dists if d > 0.1]
        if right_valid and max(right_valid) < BLOCKED_THRESHOLD:
            blocked_groups += 1

        return blocked_groups >= 3

    def mark_dead_end(self, x: float, y: float):
        """Mark a position as a dead-end that should be avoided"""
        cell = self.visited_tracker.pos_to_cell(x, y)
        if cell not in self.dead_end_positions:
            self.dead_end_positions.add(cell)
            print(f"\n[DEAD-END] Marked cell {cell} as dead-end at ({x:.2f}, {y:.2f})")

    def is_in_dead_end(self, x: float, y: float) -> bool:
        """Check if a position is in a known dead-end area"""
        cell = self.visited_tracker.pos_to_cell(x, y)
        return cell in self.dead_end_positions

    def start_movement_thread(self):
        """Start the continuous movement thread for smooth motion"""
        if self.movement_thread is not None and self.movement_thread.is_alive():
            return  # Already running

        self.movement_running = True
        self.movement_thread = threading.Thread(target=self._movement_loop, daemon=True)
        self.movement_thread.start()
        print("[MOTION] Started continuous movement thread")

    def stop_movement_thread(self):
        """Stop the continuous movement thread"""
        self.movement_running = False
        if self.movement_thread is not None:
            self.movement_thread.join(timeout=1.0)
        self.movement_thread = None

    def set_target_velocity(self, linear: float, angular: float):
        """Set target velocity for the movement thread (thread-safe)"""
        with self.movement_lock:
            self.target_linear = linear
            self.target_angular = angular

    def _movement_loop(self):
        """
        Continuous movement loop running at ~10Hz.
        Sends smoothed velocity commands for smooth motion instead of
        discrete start-stop movements.
        """
        LOOP_RATE = 10  # Hz
        loop_period = 1.0 / LOOP_RATE

        while self.movement_running:
            loop_start = time.time()

            # Get current target velocity (thread-safe)
            with self.movement_lock:
                target_lin = self.target_linear
                target_ang = self.target_angular

            # Apply smoothing and send command
            # Skip if paused or emergency stopped
            if not self.keyboard.emergency_stop and not self.keyboard.paused:
                smoothed_lin, smoothed_ang = self.smooth_velocity(target_lin, target_ang)
                self.send_cmd(smoothed_lin, smoothed_ang)

            # Maintain loop rate
            elapsed = time.time() - loop_start
            sleep_time = loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

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

    def turn_in_place(self, degrees: float, speed: float = 1.0):
        """
        Turn the robot in place by a specific number of degrees.
        Uses calibrated timing for accurate rotation.

        Args:
            degrees: Angle to rotate (positive = left/CCW, negative = right/CW)
            speed: Command speed (0.1 to 1.0, default max)
        """
        if abs(degrees) < 5:  # Ignore tiny rotations
            return

        speed = max(0.1, min(1.0, speed))

        # Calculate duration using CALIBRATED angular velocity
        actual_vel = ACTUAL_MAX_ANGULAR_VEL * speed
        angle_rad = abs(degrees) * (math.pi / 180)
        duration = angle_rad / actual_vel

        # Determine direction
        angular = speed if degrees > 0 else -speed

        print(f"\n[TURN] Rotating {degrees:.0f}° {'left' if degrees > 0 else 'right'} (duration: {duration:.1f}s)")

        # Scale to robot's expected range (-1 to 1)
        x_val = 0.0  # No linear motion
        z_val = max(-1.0, min(1.0, -angular / 1.0))  # INVERTED

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
            subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'bash', '-c', bash_script],
                timeout=duration + 5,
                capture_output=True
            )
        except subprocess.TimeoutExpired:
            print("Turn timed out, stopping...")
            self.stop()
        except Exception as e:
            print(f"Turn error: {e}")
            self.stop()

        self.total_rotations += 1

    def backup_then_turn(self, backup_duration: float = 0.8, turn_degrees: float = 45):
        """
        Execute obstacle avoidance maneuver: back up, then turn in place.

        Args:
            backup_duration: How long to back up in seconds
            turn_degrees: How many degrees to turn (positive = left, negative = right)
        """
        print(f"\n[AVOID] Backing up for {backup_duration:.1f}s, then turning {turn_degrees:.0f}°")

        # First: Back up
        # Motor is INVERTED: positive X = forward, negative X = backward
        # So we need NEGATIVE x_val to go backward
        x_val = max(-1.0, min(1.0, -self.linear_speed * 0.6 / 0.3))  # NEGATIVE = backward
        serial_cmd = f'{{"T":"13","X":{x_val:.2f},"Z":0.00}}'
        stop_cmd = '{"T":"13","X":0.00,"Z":0.00}'

        iterations = int(backup_duration / 0.05)
        bash_script = f'''
for i in $(seq 1 {iterations}); do
    echo '{serial_cmd}' > /dev/ttyAMA0
    sleep 0.05
done
echo '{stop_cmd}' > /dev/ttyAMA0
'''

        try:
            subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'bash', '-c', bash_script],
                timeout=backup_duration + 3,
                capture_output=True
            )
        except Exception as e:
            print(f"Backup error: {e}")
            self.stop()
            return

        time.sleep(0.1)  # Brief pause between maneuvers

        # Second: Turn in place
        self.turn_in_place(turn_degrees, speed=1.0)

        self.obstacles_avoided += 1

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

    def publish_virtual_obstacle(self, x: float, y: float):
        """
        Publish a virtual obstacle marker at the given position.
        This helps SLAM know there's an invisible obstacle (low object, narrow gap, etc.)
        """
        try:
            # Publish a PointCloud2 with a single point at the obstacle location
            # This will be picked up by the costmap and marked as an obstacle
            subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
                 f'''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

rclpy.init()
node = rclpy.create_node('virtual_obstacle_pub')

# Publish visualization marker (visible in RViz)
marker_pub = node.create_publisher(Marker, '/virtual_obstacles', 10)

marker = Marker()
marker.header.frame_id = 'odom'
marker.header.stamp = node.get_clock().now().to_msg()
marker.ns = 'stuck_obstacles'
marker.id = int({x}*1000 + {y}*100) % 10000
marker.type = Marker.CYLINDER
marker.action = Marker.ADD
marker.pose.position.x = {x}
marker.pose.position.y = {y}
marker.pose.position.z = 0.1
marker.pose.orientation.w = 1.0
marker.scale.x = 0.3  # 30cm diameter
marker.scale.y = 0.3
marker.scale.z = 0.2  # 20cm tall
marker.color.r = 1.0
marker.color.g = 0.0
marker.color.b = 0.0
marker.color.a = 0.8
marker.lifetime.sec = 300  # Keep for 5 minutes

# Publish multiple times to ensure delivery
for _ in range(3):
    marker_pub.publish(marker)
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
print('Published virtual obstacle at {x:.2f}, {y:.2f}')
"'''],
                capture_output=True, text=True, timeout=5
            )
            print(f"\n[OBSTACLE] Marked virtual obstacle at ({x:.2f}, {y:.2f})")
        except Exception as e:
            print(f"\nFailed to publish virtual obstacle: {e}")

    def get_imu_acceleration(self) -> Optional[float]:
        """Get current linear acceleration magnitude from IMU"""
        try:
            result = subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
                 '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from sensor_msgs.msg import Imu
rclpy.init()
node = rclpy.create_node('imu_reader')
msg = None
def cb(m): global msg; msg = m
sub = node.create_subscription(Imu, '/imu/data', cb, 1)
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=0.05)
    if msg: break
if msg:
    # Linear acceleration magnitude (excluding gravity on z)
    ax, ay = msg.linear_acceleration.x, msg.linear_acceleration.y
    import math
    print(f'{math.sqrt(ax*ax + ay*ay):.4f}')
node.destroy_node()
rclpy.shutdown()
"'''],
                capture_output=True, text=True, timeout=2
            )
            if result.stdout.strip():
                return float(result.stdout.strip())
            return None
        except:
            return None

    def check_if_stuck(self, is_driving: bool, commanded_linear: float = 0.0) -> bool:
        """
        Check if robot is stuck by detecting unchanging LiDAR distance.
        Returns True if stuck is detected.

        Simple approach: If we're driving forward but front distance isn't
        decreasing, we're stuck. LiDAR directly measures obstacle distance -
        trust it over odometry!
        """
        if not is_driving or commanded_linear <= 0.01:
            # Not driving forward, reset
            self.stuck_time_counter = 0
            return False

        if self.stuck_cooldown > 0:
            self.stuck_cooldown -= 1
            return False

        # Robot body appears at ~0.1-0.25m in side/back sectors
        # Use same thresholds as compute_velocity() for consistency
        BODY_THRESHOLD_FRONT = 0.12  # Front sectors: real obstacles can be close
        BODY_THRESHOLD_SIDE = 0.25   # Side/back sectors: may have body readings

        # Check front and side sectors with appropriate thresholds
        sectors_to_check = [
            (0, self.sector_distances[0], BODY_THRESHOLD_FRONT),   # FRONT
            (1, self.sector_distances[1], BODY_THRESHOLD_FRONT),   # F-L
            (11, self.sector_distances[11], BODY_THRESHOLD_FRONT), # FRONT-R
            (4, self.sector_distances[4], BODY_THRESHOLD_SIDE),    # BACK-L
            (5, self.sector_distances[5], BODY_THRESHOLD_SIDE),    # BACK
        ]

        # Find minimum distance, filtering out body readings and blind spots
        front_min = 999
        for sector, d, threshold in sectors_to_check:
            if d > threshold:  # Skip body and blind readings
                front_min = min(front_min, d)

        # If front_min is still 999, no valid readings - don't trigger stuck
        if front_min >= 999:
            self.stuck_time_counter = 0
            return False

        # If there's a close obstacle we're driving into, that's stuck
        if front_min < 0.40:
            self.stuck_time_counter += 1

            # 3 iterations = ~0.6s of driving into close obstacle = stuck
            if self.stuck_time_counter >= 3:
                print(f"\n[STUCK] Driving into obstacle at {front_min:.2f}m!")
                print(f"  Front sectors: s0={self.sector_distances[0]:.2f} s1={self.sector_distances[1]:.2f} s4={self.sector_distances[4]:.2f} s5={self.sector_distances[5]:.2f}")
                self.stuck_position = self.get_odometry()
                self.stuck_time_counter = 0
                return True
        else:
            self.stuck_time_counter = 0

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
                # Check for very close obstacles (closer than robot radius)
                any_reading = [r for r in sector_ranges if 0.02 < r < 10.0]
                if any_reading:
                    min_reading = min(any_reading)
                    if min_reading < ROBOT_RADIUS:
                        # Very close obstacle! Use actual distance, not 0
                        # This is critical for stuck detection
                        lidar_distances[lidar_sector] = min_reading
                    else:
                        # Readings exist but filtered - shouldn't happen
                        lidar_distances[lidar_sector] = 0.0
                else:
                    # No readings (nan/cropped) - BLIND SPOT
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
        Now includes visited location tracking to prefer unvisited areas.

        Cost = target_weight * target_cost
             + clearance_weight * clearance_cost
             + previous_weight * previous_cost
             + unvisited_weight * unvisited_cost

        Returns (best_sector, best_score)
        """
        # Weights (inspired by VFH + exploration)
        TARGET_WEIGHT = 2.0       # Prefer forward direction
        CLEARANCE_WEIGHT = 2.0    # Prefer clear paths
        PREVIOUS_WEIGHT = 1.0     # Smooth trajectory (reduce oscillation)
        UNVISITED_WEIGHT = 5.0    # STRONGLY prefer unvisited areas

        best_sector = 0
        best_score = -999

        # Get current position for visited checking
        current_x, current_y = 0.0, 0.0
        if self.current_position:
            current_x, current_y = self.current_position

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

            # UNVISITED COST: Check if direction leads to unvisited area
            # Calculate target position in this direction
            sector_angle = sector * (2 * math.pi / NUM_SECTORS)
            look_ahead = min(dist, 1.5)  # Look 1.5m ahead or to obstacle
            target_x = current_x + look_ahead * math.cos(sector_angle)
            target_y = current_y + look_ahead * math.sin(sector_angle)

            visit_count = self.visited_tracker.get_visit_count(target_x, target_y)

            # Score: unvisited = 1.0, visited once = 0.3, visited more = 0.0
            if visit_count == 0:
                unvisited_cost = 1.0
            elif visit_count == 1:
                unvisited_cost = 0.3
            else:
                unvisited_cost = 0.0

            # Combined score
            score = (TARGET_WEIGHT * target_cost +
                    CLEARANCE_WEIGHT * clearance_cost +
                    PREVIOUS_WEIGHT * previous_cost +
                    UNVISITED_WEIGHT * unvisited_cost)

            if score > best_score:
                best_score = score
                best_sector = sector

        return best_sector, best_score

    def calculate_turn_degrees(self, obstacle_sector: int, obstacle_distance: float) -> float:
        """
        Calculate how many degrees to turn based on obstacle position.

        Args:
            obstacle_sector: Sector with closest obstacle (0-11)
            obstacle_distance: Distance to obstacle in meters

        Returns:
            Degrees to turn (positive = left, negative = right)
        """
        # Base turn angle depends on obstacle distance
        # Closer obstacle = bigger turn needed
        if obstacle_distance < 0.3:
            base_degrees = 60  # Very close - big turn
        elif obstacle_distance < 0.4:
            base_degrees = 45  # Close - moderate turn
        elif obstacle_distance < 0.5:
            base_degrees = 35  # Medium - smaller turn
        else:
            base_degrees = 25  # Far - gentle turn

        # Adjust based on obstacle position (which sector)
        # Sectors 0, 11 = directly ahead, need bigger turn
        # Sectors 1, 10 = slightly off, need less turn
        # Sectors 2, 9 = more off-center, need even less
        if obstacle_sector in [0, 11]:  # Dead ahead
            base_degrees *= 1.2
        elif obstacle_sector in [1, 10]:  # Slightly off
            base_degrees *= 1.0
        elif obstacle_sector in [2, 9]:  # More off-center
            base_degrees *= 0.8

        return min(90, max(25, base_degrees))  # Clamp between 25-90°

    def compute_velocity(self) -> Tuple[float, float]:
        """
        Obstacle avoidance using discrete maneuvers:
        1. When obstacle detected: back up, then turn in place by calculated degrees
        2. When clear: go straight
        3. Maintain committed direction throughout avoidance

        Returns:
            (linear, angular) velocity command, or (None, None) if discrete maneuver was executed
        """
        front_dist = self.sector_distances[0]
        front_left = self.sector_distances[1] if self.sector_distances[1] > 0.01 else 0
        front_right = self.sector_distances[11] if self.sector_distances[11] > 0.01 else 0

        # Find best direction using VFH-style cost function
        best_sector, best_score = self.find_best_direction()
        best_dist = self.sector_distances[best_sector]

        # With LIDAR_ROTATION_SECTORS=7:
        # - Blind spots (LiDAR 60-120°) map to Robot sectors 9-10 (R-F, F-R)
        # - Robot body/close readings in sectors 6-8 (BACK-R, R-BACK, RIGHT)
        #
        # Front sectors (0, 1, 11) should be reliable for obstacle detection.
        # Use lower threshold for front (real obstacles can be close).
        BODY_THRESHOLD_FRONT = 0.12  # Front sectors: real obstacles can be close
        BODY_THRESHOLD_SIDE = 0.25   # Side/back sectors: may have body readings

        narrow_front = [
            self.sector_distances[11],  # FRONT-R
            self.sector_distances[0],   # FRONT
            self.sector_distances[1],   # F-L
        ]

        # Find minimum distance in front arc
        # For front sectors, use lower threshold (0.12m) - real obstacles can be close
        front_arc_min = 10.0
        closest_sector = 0
        sector_map = [11, 0, 1]
        for i, d in enumerate(narrow_front):
            # Skip blind spots (0) and very close body readings (< 0.12m)
            if d > BODY_THRESHOLD_FRONT and d < front_arc_min:
                front_arc_min = d
                closest_sector = sector_map[i]

        # If all narrow front sectors are body/blind, check wider arc
        if front_arc_min >= 10.0:
            # Fall back to sectors 10, 0, 1, 4 (skip 2,3 blind)
            wider_front = [
                (10, self.sector_distances[10]),  # F-R - use side threshold
                (0, self.sector_distances[0]),    # FRONT - use front threshold
                (1, self.sector_distances[1]),    # F-L - use front threshold
                (4, self.sector_distances[4]),    # BACK-L (might see around)
            ]
            for sec, d in wider_front:
                threshold = BODY_THRESHOLD_FRONT if sec in [0, 1, 11] else BODY_THRESHOLD_SIDE
                if d > threshold and d < front_arc_min:
                    front_arc_min = d
                    closest_sector = sec

        # Danger zone threshold - execute backup+turn maneuver if closer than this
        # Using min_distance directly so we back up as soon as obstacle is too close
        DANGER_DISTANCE = self.min_distance  # 0.65m - back up when obstacle is within min safe distance

        # Check if path is clear enough to drive straight
        path_clear = front_arc_min >= self.min_distance

        if front_arc_min < DANGER_DISTANCE:
            # OBSTACLE TOO CLOSE! Execute discrete avoidance maneuver
            self.emergency_maneuver = True

            # Pick avoidance direction - ALTERNATE to help navigate passages
            if self.avoidance_direction is None:
                # Check if we're in the same area as last avoidance (within 0.5m)
                same_area = False
                if self.current_position and self.last_avoidance_position:
                    dx = self.current_position[0] - self.last_avoidance_position[0]
                    dy = self.current_position[1] - self.last_avoidance_position[1]
                    dist_from_last = math.sqrt(dx*dx + dy*dy)
                    same_area = dist_from_last < 0.5

                # Get clearance on both sides
                s1 = self.sector_distances[1] if self.sector_distances[1] > BODY_THRESHOLD_FRONT else 0
                s4 = self.sector_distances[4] if self.sector_distances[4] > BODY_THRESHOLD_SIDE else 0
                left_clearance = max(s1, s4)

                s10 = self.sector_distances[10] if self.sector_distances[10] > BODY_THRESHOLD_SIDE else 0
                s5 = self.sector_distances[5] if self.sector_distances[5] > BODY_THRESHOLD_SIDE else 0
                right_clearance = max(s10, s5)

                # Check if this looks like a passage (both sides have similar clearance)
                clearance_ratio = min(left_clearance, right_clearance) / max(left_clearance, right_clearance, 0.01)
                is_passage = clearance_ratio > 0.5 and left_clearance > 0.3 and right_clearance > 0.3

                if same_area:
                    # We're stuck in same area - ALTERNATE direction!
                    self.avoidance_attempt_count += 1
                    if self.avoidance_attempt_count >= 2:
                        # Tried same direction twice, switch!
                        self.avoidance_direction = "right" if self.last_avoidance_direction == "left" else "left"
                        self.avoidance_attempt_count = 0
                        print(f"\n[PASSAGE] Alternating to {self.avoidance_direction} (attempt #{self.avoidance_attempt_count})")
                    else:
                        # First attempt in this area, use clearance
                        if left_clearance >= right_clearance:
                            self.avoidance_direction = "left"
                        else:
                            self.avoidance_direction = "right"
                elif is_passage:
                    # Looks like a passage - alternate from last direction
                    self.avoidance_direction = "right" if self.last_avoidance_direction == "left" else "left"
                    print(f"\n[PASSAGE] Detected passage, trying {self.avoidance_direction} (L:{left_clearance:.2f}m R:{right_clearance:.2f}m)")
                else:
                    # Normal obstacle - pick side with more clearance
                    if left_clearance >= right_clearance:
                        self.avoidance_direction = "left"
                    else:
                        self.avoidance_direction = "right"

                # Save state
                self.last_avoidance_direction = self.avoidance_direction
                self.last_avoidance_position = self.current_position
                self.avoidance_intensity = 1.0
                self.avoidance_start_time = time.time()
                print(f"\n[AVOID] Chose {self.avoidance_direction} (L:{left_clearance:.2f}m R:{right_clearance:.2f}m)")

            # Calculate turn degrees dynamically based on obstacle
            turn_degrees = self.calculate_turn_degrees(closest_sector, front_arc_min)

            # Apply committed direction
            if self.avoidance_direction == "right":
                turn_degrees = -turn_degrees

            # Increase turn if we've been trying to avoid for a while
            time_avoiding = time.time() - self.avoidance_start_time
            if time_avoiding > 2.0:
                turn_degrees *= 1.3  # 30% more turn if stuck
                print(f"\n[STUCK] Increasing turn to {abs(turn_degrees):.0f}°")

            # Execute backup + turn maneuver
            backup_duration = 0.6 if front_arc_min > 0.3 else 1.0  # Back up more if very close
            self.backup_then_turn(backup_duration=backup_duration, turn_degrees=turn_degrees)

            # Return None to indicate maneuver was executed
            return None, None

        elif path_clear:
            # PATH IS CLEAR - Go straight!
            linear = self.linear_speed
            angular = 0.0
            self.emergency_maneuver = False

            # Clear the avoidance state - we escaped!
            if self.avoidance_direction is not None:
                print(f"\n[CLEAR] Path clear, resetting avoidance state")
                self.avoidance_direction = None
                self.avoidance_intensity = 1.0
                self.avoidance_attempt_count = 0  # Reset attempts on success

            status = f"[FWD] clear={front_arc_min:.2f}m - going straight"

        elif best_dist >= self.min_distance and best_sector not in [0, 11, 1]:
            # Only steer if best direction is NOT the front sectors
            # If front is clear enough (best_sector is 0, 1, or 11), just go straight
            steer_angle = self.sector_to_angle(best_sector)
            turn_factor = abs(steer_angle) / math.pi
            linear = self.linear_speed * (1.0 - turn_factor * 0.3)

            # Use committed direction if we have one
            if self.avoidance_direction == "left":
                angular = abs(steer_angle) * 0.3
            elif self.avoidance_direction == "right":
                angular = -abs(steer_angle) * 0.3
            else:
                angular = steer_angle * 0.3

            self.emergency_maneuver = False
            status = f"[STEER] f={front_arc_min:.2f}m -> s{best_sector}"

        elif best_dist >= self.min_distance:
            # Best sector is front (0, 1, or 11) - go straight!
            linear = self.linear_speed
            angular = 0.0
            self.emergency_maneuver = False

            if self.avoidance_direction is not None:
                print(f"\n[CLEAR] Front is best direction, resetting commitment")
                self.avoidance_direction = None

            status = f"[FWD] front ok={front_arc_min:.2f}m s{best_sector}"

        else:
            # All directions have obstacles - back up and turn to find clear path
            turn_degrees = self.calculate_turn_degrees(closest_sector, front_arc_min)

            if self.avoidance_direction is None:
                if best_sector <= 5:
                    self.avoidance_direction = "left"
                else:
                    self.avoidance_direction = "right"
                self.avoidance_start_time = time.time()

            if self.avoidance_direction == "right":
                turn_degrees = -turn_degrees

            # Always back up first, then turn (using calibrated rotation)
            backup_duration = 0.8 if front_arc_min > 0.3 else 1.2
            print(f"\n[BLOCKED] Backing up and turning {turn_degrees:.0f}° to find clear path")
            self.backup_then_turn(backup_duration=backup_duration, turn_degrees=turn_degrees)
            return None, None

        # Remember this direction for next iteration
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

    def test_front_sector(self):
        """
        Calibration test: Place obstacle directly in front of robot.
        The sector with lowest distance should be sector 0 (FRONT).
        If not, LIDAR_ROTATION_SECTORS needs adjustment.
        """
        if not self.update_scan():
            print("ERROR: Could not read LiDAR data")
            return

        print("\n" + "="*60)
        print("FRONT SECTOR CALIBRATION TEST")
        print("="*60)
        print("Hold an obstacle directly in FRONT of the robot (< 0.5m)")
        print("The sector with lowest distance should be sector 0 (FRONT)")
        print("-"*60)

        # Find sectors with close obstacles (excluding blind spots)
        close_sectors = []
        for i, d in enumerate(self.sector_distances):
            if 0.1 < d < 0.6:
                close_sectors.append((i, d))

        if not close_sectors:
            print("No close obstacles detected (0.1m - 0.6m range)")
            print("Please hold something in front of the robot and try again")
        else:
            close_sectors.sort(key=lambda x: x[1])  # Sort by distance
            print("\nDetected obstacles:")
            labels = ["FRONT", "F-L", "LEFT", "L-BACK", "BACK-L", "BACK",
                      "BACK-R", "R-BACK", "RIGHT", "R-F", "F-R", "FRONT-R"]
            for sector, dist in close_sectors:
                marker = " *** LIKELY FRONT ***" if sector == close_sectors[0][0] else ""
                print(f"  Sector {sector:2d} ({labels[sector]:7s}): {dist:.2f}m{marker}")

            closest_sector = close_sectors[0][0]
            if closest_sector == 0:
                print(f"\n✓ Calibration CORRECT: Closest sector is 0 (FRONT)")
            else:
                # Calculate what offset would fix this
                needed_offset = (LIDAR_ROTATION_SECTORS - closest_sector) % 12
                print(f"\n✗ Calibration ERROR: Closest sector is {closest_sector}, should be 0")
                print(f"  Current LIDAR_ROTATION_SECTORS = {LIDAR_ROTATION_SECTORS}")
                print(f"  Try changing to: LIDAR_ROTATION_SECTORS = {needed_offset}")

        print("="*60)

    def run(self):
        """Main control loop with continuous movement thread for smooth motion"""
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
        print(f"Motion smooth:   Continuous thread @10Hz + EMA(α={self.ema_alpha})")
        print(f"Dead-end detect: Enabled (skip 3+ blocked directions)")
        print("-"*60)
        print("Controls:")
        print("  SPACE/s  - Emergency stop")
        print("  p        - Pause/Resume")
        print("  r        - Resume after stop")
        print("  q        - Quit")
        print("="*60 + "\n")

        self.keyboard.start()
        self.start_movement_thread()  # Start continuous movement for smooth motion
        last_status_time = time.time()
        dead_ends_found = 0

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
                    self.set_target_velocity(0.0, 0.0)  # Stop via movement thread
                    time.sleep(0.2)
                    continue

                # Update sensors
                if not self.update_scan():
                    time.sleep(0.1)
                    continue

                # Update current position for visited tracking
                pos = self.get_odometry()
                if pos:
                    self.current_position = pos
                    is_new = self.visited_tracker.mark_visited(pos[0], pos[1])
                    if is_new:
                        stats = self.visited_tracker.get_coverage_stats()
                        print(f"\n[EXPLORE] New cell! Total: {stats['cells_visited']} cells, {stats['area_covered']:.1f}m²")

                    # Check for dead-end at current position
                    if self.is_dead_end():
                        if not self.is_in_dead_end(pos[0], pos[1]):
                            self.mark_dead_end(pos[0], pos[1])
                            dead_ends_found += 1
                            # Back out of dead-end
                            print(f"\n[DEAD-END] Backing out of dead-end area!")
                            self.set_target_velocity(0.0, 0.0)  # Stop movement thread
                            self.backup_then_turn(backup_duration=1.2, turn_degrees=120)
                            continue

                # Compute velocity (may execute discrete maneuvers)
                linear, angular = self.compute_velocity()

                # If compute_velocity returned None, a discrete maneuver was executed
                # Skip the rest of this iteration
                if linear is None:
                    # After discrete maneuver, reset target velocity
                    self.set_target_velocity(0.0, 0.0)
                    continue

                # Check if stuck (driving but not moving)
                is_driving = linear > 0.01  # Only forward motion can be "stuck"

                if self.check_if_stuck(is_driving, linear):
                    # Robot is stuck on invisible obstacle!
                    # Mark current direction as blocked
                    if hasattr(self, 'previous_sector'):
                        self.blocked_sectors.add(self.previous_sector)
                        # Also block adjacent sectors
                        self.blocked_sectors.add((self.previous_sector + 1) % NUM_SECTORS)
                        self.blocked_sectors.add((self.previous_sector - 1) % NUM_SECTORS)

                    print(f"\n*** STUCK DETECTED! Blocking sectors: {self.blocked_sectors} ***")
                    self.stuck_counter = 0
                    self.stuck_cooldown = 5  # Wait 5 iterations before checking again

                    # Publish virtual obstacle marker at stuck position (visible in RViz)
                    if hasattr(self, 'stuck_position') and self.stuck_position:
                        stuck_x, stuck_y = self.stuck_position
                        self.publish_virtual_obstacle(stuck_x, stuck_y)

                    # Use calibrated backup + turn maneuver
                    # Pick direction based on committed direction or default
                    if self.avoidance_direction is None:
                        self.avoidance_direction = "left"  # Default
                        self.avoidance_start_time = time.time()

                    self.set_target_velocity(0.0, 0.0)  # Stop movement thread during maneuver
                    turn_degrees = 60 if self.avoidance_direction == "left" else -60
                    self.backup_then_turn(backup_duration=1.0, turn_degrees=turn_degrees)

                    # Reset position tracking after recovery maneuver
                    self.last_position = None
                    continue

                # Set target velocity for the continuous movement thread
                # During emergency maneuvers, bypass smoothing for faster response
                if self.emergency_maneuver:
                    # Reset smoothing state and send direct command
                    self.last_linear = linear
                    self.last_angular = angular
                    self.send_cmd(linear, angular)
                else:
                    # Set target velocity - movement thread will smooth and send
                    self.set_target_velocity(linear, angular)

                # Print detailed status periodically
                if time.time() - last_status_time > 10:
                    self.print_sector_display()
                    last_status_time = time.time()

                # Control loop rate (~5 Hz) - just for decision making
                # Movement thread runs at 10Hz for smooth motion
                time.sleep(0.2)

        except KeyboardInterrupt:
            print("\n\nInterrupted (Ctrl+C)")
        finally:
            print("\n\nStopping...")
            self.set_target_velocity(0.0, 0.0)
            self.stop_movement_thread()
            self.emergency_stop()
            self.keyboard.stop()
            self.running = False

            # Save visited locations for next run
            self.visited_tracker.save()

            # Summary
            total_time = time.time() - self.start_time
            stats = self.visited_tracker.get_coverage_stats()
            print("\n" + "="*60)
            print("SCAN SUMMARY")
            print("="*60)
            print(f"Total time:         {total_time:.1f}s")
            print(f"Obstacles avoided:  {self.obstacles_avoided}")
            print(f"Direction changes:  {self.total_rotations}")
            print(f"Stuck detections:   {len(self.blocked_sectors) // 3}")  # Approx, since we block 3 at a time
            print(f"Dead-ends found:    {dead_ends_found}")
            print("-"*60)
            print("EXPLORATION COVERAGE:")
            print(f"  Cells visited:    {stats['cells_visited']}")
            print(f"  Area covered:     {stats['area_covered']:.2f} m²")
            print(f"  Total visits:     {stats.get('total_visits', 0)}")
            print(f"  Revisit rate:     {stats.get('revisit_rate', 0)*100:.1f}%")
            print("="*60)


def check_prerequisites():
    """Ensure container and bringup are running using helper script"""
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ensure_script = os.path.join(script_dir, 'ensure_bringup.sh')

    # Check if helper script exists
    if not os.path.exists(ensure_script):
        print(f"Warning: {ensure_script} not found, falling back to manual check")
        # Fallback to basic container check
        result = subprocess.run(
            ['docker', 'ps', '--filter', f'name={CONTAINER_NAME}', '--format', '{{.Names}}'],
            capture_output=True, text=True
        )
        if CONTAINER_NAME not in result.stdout:
            print(f"Error: Container '{CONTAINER_NAME}' is not running.")
            print("Start it with: ./start_ugv_service.sh")
            return False
        return True

    # Use helper script to ensure bringup is running
    print("Checking robot bringup...")
    result = subprocess.run(
        [ensure_script, '--wait'],
        capture_output=True, text=True,
        timeout=30
    )

    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr)
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
  ./auto_scan.py                      # Default: 60s at 0.06 m/s
  ./auto_scan.py --duration 120       # 2 minute scan
  ./auto_scan.py --duration 0         # Unlimited
  ./auto_scan.py --speed 0.08         # Slower scanning
  ./auto_scan.py --min-dist 0.7       # More cautious

Algorithm:
  Divides 360° LiDAR scan into 12 sectors (30° each).
  Finds clearest path and navigates with minimal rotation.
  Based on: github.com/Rad-hi/Obstacle-Avoidance-ROS
        """
    )
    parser.add_argument('--speed', '-s', type=float, default=0.06,
                        help='Linear speed m/s (default: 0.06, max: 0.08)')
    parser.add_argument('--min-dist', '-m', type=float, default=0.45,
                        help='Min obstacle distance m (default: 0.45)')
    parser.add_argument('--duration', '-d', type=float, default=60,
                        help='Scan duration seconds, 0=unlimited (default: 60)')
    parser.add_argument('--clear-visited', '-c', action='store_true',
                        help='Clear visited locations history before starting')
    parser.add_argument('--test-front', action='store_true',
                        help='Test front sector calibration (place obstacle in front)')
    args = parser.parse_args()

    if not check_prerequisites():
        sys.exit(1)

    # Start motor driver if needed
    start_driver()

    avoider = SectorObstacleAvoider(
        linear_speed=args.speed,
        min_distance=args.min_dist,
        duration=args.duration,
        clear_visited=args.clear_visited
    )

    # Run calibration test if requested
    if args.test_front:
        avoider.test_front_sector()
        sys.exit(0)

    def signal_handler(sig, frame):
        avoider.running = False
        avoider.emergency_stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    avoider.run()


if __name__ == '__main__':
    main()
