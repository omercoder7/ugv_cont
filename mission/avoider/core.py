"""
Sector-based obstacle avoidance main class.
"""

import sys
import subprocess
import signal
import time
import math
import threading
import json
import os
from typing import List, Tuple, Optional, Set, Dict

# Try to import numpy
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    np = None

# Optional sensor logging
try:
    from mapping.robot.sensor_logger import SensorLogger
    HAS_SENSOR_LOGGER = True
except ImportError:
    HAS_SENSOR_LOGGER = False

# Import from parent package (mission/)
from ..constants import (
    CONTAINER_NAME, GRID_RESOLUTION, VISITED_FILE, NUM_SECTORS,
    LINEAR_VEL_RATIO, LIDAR_ROTATION_SECTORS,
    LIDAR_FRONT_OFFSET, ROBOT_WIDTH, ROBOT_LENGTH, ROBOT_HALF_WIDTH,
    MIN_CORRIDOR_WIDTH, SAFE_TURN_CLEARANCE, MAX_BACKUP_TIME, MIN_BACKUP_DISTANCE,
    DIRECTION_CHANGE_PAUSE, WHEEL_ENCODER_STUCK_THRESHOLD, WHEEL_ENCODER_STUCK_TIME,
    SPEED_SCALE_FAR_DISTANCE, SPEED_SCALE_NEAR_DISTANCE, SPEED_SCALE_MIN_FACTOR,
    BODY_THRESHOLD_FRONT, BODY_THRESHOLD_SIDE,
    DANGER_DISTANCE_MARGIN, CORRIDOR_DANGER_MARGIN
)
from ..utils import sector_to_angle, pos_to_cell
from ..state import RobotState, StateContext
from ..collision import CollisionVerifier, CollisionVerifierState
from ..vfh import VFHController
from ..scoring import TrajectoryScorer, UnifiedDirectionScorer
from ..planner import AStarPlanner
from ..monitors import KeyboardMonitor, CorridorDetectorWithHysteresis
from ..memory import EscapeMemory, VisitedTracker
from ..ros_interface import (
    send_velocity_cmd, get_odometry, get_wheel_encoders,
    get_imu_acceleration, get_lidar_scan, publish_virtual_obstacle,
    check_rf2o_health, ensure_slam_running
)
from ..movement import VelocitySmoother, compute_adaptive_speed
from ..obstacle_detection import (
    load_avoidance_profiles, detect_thin_obstacle, detect_round_obstacle,
    get_front_arc_min as _get_front_arc_min,
    get_side_clearances as _get_side_clearances,
    check_trapped_state as _check_trapped_state,
    get_avoidance_params as _get_avoidance_params
)
from ..virtual_obstacles import VirtualObstacleTracker

# Import from avoider subpackage
from .lidar import compute_sector_distances
from .dead_ends import is_dead_end_by_sectors, detect_small_pocket, DeadEndTracker
from .stuck import StuckDetector
from .maneuvers import execute_turn_in_place, calculate_turn_duration as calc_turn_duration

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
                 duration: float = 60.0, clear_visited: bool = False, use_vfh: bool = False,
                 log_sensors: bool = False, log_output_dir: str = "/tmp/exploration_data"):
        # Cap max speed at 0.10 m/s - rf2o odometry overestimates at higher speeds
        # causing mismatch between RViz display and actual robot position
        self.linear_speed = min(linear_speed, 0.10)
        self.use_vfh = use_vfh  # Use VFH algorithm with state machine

        # Sensor logging for two-phase mapping
        self.log_sensors = log_sensors and HAS_SENSOR_LOGGER
        self.sensor_logger: Optional[SensorLogger] = None
        if self.log_sensors:
            self.sensor_logger = SensorLogger(output_dir=log_output_dir)
        # Min distance floor - robot is ~17cm wide
        # Allow down to 0.25m for tight spaces (LiDAR offset already adds 0.37m)
        self.min_distance = max(min_distance, 0.25)
        # LiDAR threshold = min_distance + LiDAR offset from robot front
        # This compensates for LiDAR being mounted ~37cm behind robot front
        self.lidar_min_distance = self.min_distance + LIDAR_FRONT_OFFSET
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
        self.current_heading: Optional[float] = None  # Robot heading in radians

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

        # Stuck detection using efficiency-based algorithm
        self.stuck_detector = StuckDetector(efficiency_threshold=0.35)
        self.stuck_counter = 0
        self.blocked_sectors = set()  # Sectors that led to being stuck
        self.emergency_maneuver = False  # True when in danger/backing up mode
        self.prev_front_distance = None  # For rapid approach detection

        # Virtual obstacle tracking - stores invisible obstacles the robot has encountered
        self.virtual_obstacle_tracker = VirtualObstacleTracker(
            radius=0.12, max_age=300.0, max_count=20
        )

        # Velocity smoother (based on DWA/VFH research)
        # Uses EMA and acceleration limiting for smooth motion
        self.velocity_smoother = VelocitySmoother(
            max_linear_accel=0.15,
            max_angular_accel=0.8,
            ema_alpha=0.7  # Higher = faster response
        )

        # Avoidance direction - alternates to help navigate passages
        self.avoidance_direction = None  # "left" or "right" or None
        self.avoidance_intensity = 1.0   # Increases if still stuck after turning
        self.avoidance_start_time = 0    # When we started this avoidance maneuver
        self.last_avoidance_direction = "right"  # Track last direction to alternate
        self.avoidance_attempt_count = 0  # Count attempts in same area
        self.last_avoidance_position = None  # Position where we last avoided
        self.backup_start_position = None  # Position when backup started (for distance tracking)

        # Dead-end detection
        self.dead_end_positions: Set[Tuple[int, int]] = set()  # Grid cells marked as dead ends

        # Continuous movement thread
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.movement_lock = threading.Lock()
        self.movement_thread = None
        self.movement_running = False
        # Maneuver mode: None = normal, "backup" = backing up, "turn" = turning, "stop" = stationary
        self.maneuver_mode = None
        self.maneuver_end_time = 0
        self.maneuver_speed = 0.0  # For backup: linear speed, for turn: angular speed
        # Maneuver queue (FIFO): list of (mode, speed, duration) tuples
        self.maneuver_queue = []

        # VFH Controller for histogram smoothing and hysteresis
        self.vfh = VFHController(num_sectors=NUM_SECTORS, smoothing_window=3)

        # Corridor detector with hysteresis to prevent oscillation
        self.corridor_detector = CorridorDetectorWithHysteresis()

        # Escape memory - tracks which maneuvers worked in stuck situations
        self.escape_memory = EscapeMemory(grid_resolution=0.5)

        # DWA-style trajectory scorer for the AVOIDING state
        self.trajectory_scorer = TrajectoryScorer(max_speed=self.linear_speed, max_yaw_rate=1.0)

        # Unified direction scorer - consolidates all scoring systems
        self.unified_scorer = UnifiedDirectionScorer(num_sectors=NUM_SECTORS)

        # A* path planner for smarter navigation
        self.path_planner = AStarPlanner(
            resolution=0.05,  # Match SLAM map resolution
            robot_radius=ROBOT_HALF_WIDTH + 0.05  # Robot half-width + safety margin
        )
        self.current_goal: Optional[Tuple[float, float]] = None  # Current navigation goal
        self.use_path_planning: bool = True  # Enable/disable path planning

        # Cache for map exploration scores (expensive subprocess call)
        # Default to 1.0 (fully unexplored) so if SLAM fails, robot doesn't
        # penalize any direction - relies on clearance/freedom instead
        # (0.5 was bad: made robot think everything was "partially explored")
        self._map_scores_cache = [1.0] * NUM_SECTORS
        self._MAP_SCORES_CACHE_TTL = 3.0  # Refresh every 3 seconds
        self._map_scores_cache_time = -self._MAP_SCORES_CACHE_TTL  # Force fresh fetch on first call
        self._map_scores_fetch_failed = False  # Track if SLAM is working

        # Cache for dead-end detection
        self._dead_end_cache = [True] * NUM_SECTORS  # Safe default: assume all blocked
        self._DEAD_END_CACHE_TTL = 2.0  # Refresh every 2 seconds
        self._dead_end_cache_time = -self._DEAD_END_CACHE_TTL  # Force fresh fetch on first call

        # State machine for cleaner navigation logic
        self.robot_state = RobotState.STOPPED
        self.state_context = StateContext()
        self.state_context.state_start_time = time.time()

        # Minimum time to stay in each state (prevents oscillation)
        self.min_state_duration = {
            RobotState.FORWARD: 0.3,      # At least 0.3s forward before changing
            RobotState.CORRIDOR: 0.3,     # Same as FORWARD for corridor navigation
            RobotState.TURNING: 0.5,      # Complete turn before evaluating
            RobotState.BACKING_UP: 0.5,   # Complete backup before evaluating
            RobotState.AVOIDING: 0.5,     # Stay in avoidance mode briefly
            RobotState.STUCK_RECOVERY: 1.5,  # Give recovery time to work
            RobotState.STOPPED: 0.1,      # Can exit stopped quickly
        }

        # Collision verifier FSM - detects invisible obstacles by tracking movement efficiency
        # When LiDAR shows clear but odometry shows no movement, there's an invisible obstacle
        self.collision_verifier = CollisionVerifier()

        # Load obstacle avoidance profiles from calibration
        # Different obstacles need different avoidance strategies (round vs flat vs thin)
        self.avoidance_profiles = load_avoidance_profiles()

    def get_avoidance_params(self, front_distance: float) -> Tuple[dict, Optional[str]]:
        """Get avoidance parameters based on current obstacle detection."""
        return _get_avoidance_params(self.sector_distances, front_distance, self.avoidance_profiles)

    def get_front_arc_min(self, body_threshold: float = BODY_THRESHOLD_FRONT) -> float:
        """Get minimum distance in front arc (sectors 11, 0, 1)."""
        return _get_front_arc_min(self.sector_distances, body_threshold)

    def get_side_clearances(self) -> Tuple[float, float]:
        """Get left and right clearance distances."""
        return _get_side_clearances(self.sector_distances)

    def check_trapped_state(self, dynamic_min_dist: float, margin: float = 0.40) -> dict:
        """Check if robot is trapped in a dead-end or tight space."""
        return _check_trapped_state(self.sector_distances, dynamic_min_dist, margin)

    def _detect_thin_obstacle(self) -> bool:
        """Detect if current obstacle is thin (like chair leg, pole)."""
        return detect_thin_obstacle(self.sector_distances)

    def _detect_round_obstacle(self) -> bool:
        """Detect if current obstacle has round/curved shape."""
        return detect_round_obstacle(self.sector_distances)

    def smooth_velocity(self, target_linear: float, target_angular: float) -> Tuple[float, float]:
        """
        Apply smoothing to velocity commands.
        Delegates to VelocitySmoother instance for EMA and acceleration limiting.
        """
        return self.velocity_smoother.smooth(target_linear, target_angular)

    def get_adaptive_speed(self, front_distance: float) -> float:
        """
        Calculate adaptive speed based on distance to nearest obstacle.
        Delegates to standalone compute_adaptive_speed function.
        """
        return compute_adaptive_speed(front_distance, self.linear_speed)

    def is_dead_end(self) -> bool:
        """Detect if current position is a dead-end (3+ direction groups blocked)."""
        return is_dead_end_by_sectors(self.sector_distances)

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

    def predict_dead_end_ahead(self, look_ahead_distance: float = 1.5) -> bool:
        """PREDICTIVE dead-end detection - warns BEFORE robot gets trapped."""
        front_dist = self.sector_distances[0]
        if front_dist > look_ahead_distance:
            return False

        # Map-based dead-end check
        dead_ends = self.get_dead_end_sectors()
        if dead_ends[0] or (dead_ends[1] and dead_ends[11]):
            return True

        # Small pocket detection - too tight to turn
        if detect_small_pocket(self.sector_distances, look_ahead_distance):
            print(f"\n[POCKET] Detected small pocket ahead")
            return True

        # Check if trapped - front close AND sides constrained with no escape
        front_close = front_dist < self.lidar_min_distance + 0.4
        sides_constrained = (
            self.sector_distances[1] < self.lidar_min_distance + 0.15 or
            self.sector_distances[11] < self.lidar_min_distance + 0.15
        )
        if front_close and sides_constrained:
            escape_sectors = [2, 3, 9, 10]
            has_escape = any(
                self.sector_distances[s] >= self.lidar_min_distance and not dead_ends[s]
                for s in escape_sectors
            )
            if not has_escape:
                return True

        return False

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
            # FIX: Clear sensor history when starting significant turn
            # Old "front distance" entries point to different world locations after turning
            if abs(angular) > 0.25 and abs(self.target_angular) <= 0.1:
                self.stuck_detector.reset()
            self.target_linear = linear
            self.target_angular = angular

    def queue_maneuver(self, mode: str, speed: float, duration: float):
        """Add a maneuver to the queue (thread-safe)"""
        with self.movement_lock:
            self.maneuver_queue.append((mode, speed, duration))

    def _movement_loop(self):
        """
        Continuous movement loop running at ~30Hz.
        Sends smoothed velocity commands for smooth motion instead of
        discrete start-stop movements.
        Also handles backup, turn, and stop maneuvers via queue.
        """
        LOOP_RATE = 30  # Hz - higher frequency for smoother motion
        loop_period = 1.0 / LOOP_RATE

        while self.movement_running:
            loop_start = time.time()

            # Skip if paused or emergency stopped
            if self.keyboard.emergency_stop or self.keyboard.paused:
                time.sleep(loop_period)
                continue

            current_time = time.time()

            # Check if current maneuver is done and get next from queue (thread-safe)
            with self.movement_lock:
                maneuver_just_ended = False
                ended_mode = None

                if self.maneuver_mode is not None and current_time >= self.maneuver_end_time:
                    ended_mode = self.maneuver_mode
                    self.maneuver_mode = None
                    maneuver_just_ended = True

                # If no active maneuver, check queue for next one
                if self.maneuver_mode is None and self.maneuver_queue:
                    mode, speed, duration = self.maneuver_queue.pop(0)
                    self.maneuver_mode = mode
                    self.maneuver_speed = speed
                    self.maneuver_end_time = current_time + duration
                    print(f"[MANEUVER] Starting {mode}: speed={speed:.2f}, duration={duration:.2f}s", end="", flush=True)
                    maneuver_just_ended = False  # We have a new maneuver, not ended

                # If maneuver just ended and queue is empty, resume forward motion immediately
                skip_smoothing = False
                if maneuver_just_ended and not self.maneuver_queue and ended_mode == "turn":
                    # After turn completes, immediately start moving forward
                    # Don't wait for main loop's slow subprocess calls
                    self.target_linear = self.linear_speed
                    self.target_angular = 0.0
                    # Set smoothing state to target so we start at full speed
                    self.velocity_smoother.last_linear = self.linear_speed
                    self.velocity_smoother.last_angular = 0.0
                    skip_smoothing = True  # Bypass smoothing this iteration

                # Copy maneuver state while holding lock to avoid race condition
                current_maneuver_mode = self.maneuver_mode
                current_maneuver_speed = self.maneuver_speed

            # Execute current maneuver or normal operation (using copied values)
            if current_maneuver_mode is not None:
                if current_maneuver_mode == "backup":
                    # Send backup command (negative linear, no angular)
                    # At 0.15 m/s, this should move ~15cm per second
                    self.send_cmd(-abs(current_maneuver_speed), 0.0)
                elif current_maneuver_mode == "turn":
                    # Send turn command (no linear, angular based on speed sign)
                    # At speed=1.0, this sends Z=1.0 which turns at ~0.29 rad/s calibrated
                    self.send_cmd(0.0, current_maneuver_speed)
                elif current_maneuver_mode == "stop":
                    # Stationary - send zero
                    self.send_cmd(0.0, 0.0)
            else:
                # Normal operation - get target velocity (thread-safe)
                with self.movement_lock:
                    target_lin = self.target_linear
                    target_ang = self.target_angular

                # Apply smoothing (unless just finished a maneuver)
                if skip_smoothing:
                    # Send full speed immediately after maneuver
                    self.send_cmd(target_lin, target_ang)
                else:
                    smoothed_lin, smoothed_ang = self.smooth_velocity(target_lin, target_ang)
                    self.send_cmd(smoothed_lin, smoothed_ang)

            # Maintain loop rate
            elapsed = time.time() - loop_start
            sleep_time = loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def send_cmd(self, linear_x: float, angular_z: float):
        """Send velocity command directly to serial port (bypasses ROS2)"""
        send_velocity_cmd(linear_x, angular_z)

    def send_smoothed_cmd(self, linear_x: float, angular_z: float):
        """Send velocity command with smoothing applied"""
        smoothed_linear, smoothed_angular = self.smooth_velocity(linear_x, angular_z)
        self.send_cmd(smoothed_linear, smoothed_angular)

    def turn_in_place(self, degrees: float, speed: float = 1.0):
        """Turn the robot in place by a specific number of degrees (BLOCKING)."""
        if execute_turn_in_place(degrees, speed, stop_callback=self.stop):
            self.total_rotations += 1

    def backup_then_turn(self, backup_duration: float = 0.5, turn_degrees: float = 30):
        """
        Execute quick obstacle avoidance: brief backup then small turn.
        NON-BLOCKING - queues maneuvers and returns immediately.

        Args:
            backup_duration: How long to back up (default 0.5s)
            turn_degrees: How many degrees to turn (positive = left, negative = right)
        """
        # Use smaller, quicker turns (30° = ~1s turn time)
        turn_degrees = max(-45, min(45, turn_degrees))  # Cap at 45°
        turn_speed = 1.0
        turn_duration = calc_turn_duration(turn_degrees, turn_speed)
        angular_speed = turn_speed if turn_degrees > 0 else -turn_speed

        # Clear queue and set up new maneuvers atomically
        # This prevents any gap where movement thread might send wrong commands
        with self.movement_lock:
            self.maneuver_queue.clear()
            # Queue the new maneuvers BEFORE clearing current mode
            # This ensures movement thread always has something to do
            self.maneuver_queue.append(("backup", self.linear_speed, backup_duration))
            self.maneuver_queue.append(("turn", angular_speed, turn_duration))
            # Now clear current maneuver - next iteration will pick up backup
            self.maneuver_mode = None
            self.maneuver_end_time = time.time()  # Expire immediately

        # FULLY NON-BLOCKING: Return immediately, let movement thread handle maneuvers
        # No sleep here - main loop will continue but maneuver_mode blocks normal commands

        # Clear sensor history - after a turn, "front distance" points somewhere else
        # Old history entries would compare distances to different directions
        self.stuck_detector.reset()

        self.obstacles_avoided += 1
        self.total_rotations += 1

    def stop(self):
        """Stop the robot"""
        # Reset smoothing state for clean stop
        self.velocity_smoother.last_linear = 0.0
        self.velocity_smoother.last_angular = 0.0
        for _ in range(3):
            self.send_cmd(0.0, 0.0)
            time.sleep(0.05)

    def clear_tracking_data(self):
        """
        Clear all tracking data/caches to prevent stale data from affecting
        subsequent runs. This fixes the RViz erratic display bug where the
        robot appears to move crazily even when stationary.
        """
        # Clear stuck detector state (used for stuck detection)
        self.stuck_detector.reset()

        # Clear collision verifier state
        if hasattr(self, 'collision_verifier') and self.collision_verifier:
            self.collision_verifier.reset()

        # Clear avoidance state
        self.avoidance_direction = None
        self.avoidance_start_time = 0
        self.avoidance_attempt_count = 0
        self.last_avoidance_position = None
        self.backup_start_position = None

        # Clear state context
        if hasattr(self, 'state_context'):
            self.state_context.clear_lock()
            self.state_context.consecutive_avoidances = 0

        # Clear escape memory current attempt
        if hasattr(self, 'escape_memory') and self.escape_memory:
            self.escape_memory.current_attempt = None

        # Clear maneuver queue
        if hasattr(self, 'maneuver_queue'):
            self.maneuver_queue.clear()
        self.maneuver_mode = None
        self.maneuver_end_time = 0

        print("[CLEANUP] Cleared all tracking data and caches")

    def emergency_stop(self):
        """Hardware-level emergency stop"""
        self.stop()
        # Clear all tracking data to prevent stale state
        self.clear_tracking_data()
        try:
            subprocess.run(['./stop_robot.sh'], timeout=2,
                          capture_output=True, cwd='/home/ws/ugv_cont')
        except:
            pass

    def fetch_odometry(self) -> Optional[Tuple[float, float]]:
        """Get current position and heading from odometry. Also updates self.current_heading."""
        result = get_odometry()
        if result:
            x, y, yaw = result
            self.current_heading = yaw  # Update heading
            return (x, y)
        return None

    def fetch_wheel_encoders(self) -> Optional[Tuple[float, float]]:
        """Get raw wheel encoder values from /odom/odom_raw topic"""
        return get_wheel_encoders()

    def mark_virtual_obstacle(self, x: float, y: float):
        """Publish a virtual obstacle marker for visualization only."""
        self.virtual_obstacle_tracker.mark(x, y)

    def add_virtual_obstacle(self, x: float, y: float):
        """Add a virtual obstacle at the given position."""
        self.virtual_obstacle_tracker.add(x, y)

    def cleanup_virtual_obstacles(self):
        """Remove old virtual obstacles."""
        self.virtual_obstacle_tracker.cleanup()

    def get_virtual_obstacle_clearance(self, robot_x: float, robot_y: float, robot_heading: float) -> float:
        """Get minimum clearance to any virtual obstacle in front of the robot."""
        return self.virtual_obstacle_tracker.get_clearance(robot_x, robot_y, robot_heading)

    def fetch_imu_acceleration(self) -> Optional[float]:
        """Get current linear acceleration magnitude from IMU"""
        return get_imu_acceleration()

    def check_if_stuck(self, is_driving: bool, commanded_linear: float = 0.0) -> bool:
        """Check if robot is stuck using efficiency-based detection."""
        return self.stuck_detector.check(
            is_driving, commanded_linear, self.current_position, self.collision_verifier
        )

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

    def get_map_exploration_scores(self) -> List[float]:
        """
        Query Cartographer/SLAM map to find unexplored areas in each sector.

        Returns list of 12 scores (one per sector):
        - 1.0 = direction has lots of unknown cells (unexplored!)
        - 0.0 = direction is fully explored or blocked

        Uses occupancy grid: -1 = unknown, 0 = free, 100 = occupied
        Results are cached for _MAP_SCORES_CACHE_TTL seconds to reduce subprocess overhead.
        """
        # Return cached scores if still fresh
        if time.time() - self._map_scores_cache_time < self._MAP_SCORES_CACHE_TTL:
            return self._map_scores_cache

        try:
            result = subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
                 '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import tf2_ros
import math

rclpy.init()
node = rclpy.create_node('map_explorer')

# Get current robot pose from TF
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer, node)

# Wait for map
map_msg = None
def map_cb(m): global map_msg; map_msg = m
sub = node.create_subscription(OccupancyGrid, '/map', map_cb, 1)

for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.1)
    if map_msg: break

if not map_msg:
    print('NO_MAP')
    node.destroy_node()
    rclpy.shutdown()
    exit()

# Get robot position in map frame
try:
    trans = tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
    robot_x = trans.transform.translation.x
    robot_y = trans.transform.translation.y
    # Get yaw from quaternion
    q = trans.transform.rotation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    robot_yaw = math.atan2(siny_cosp, cosy_cosp)
except:
    print('NO_TF')
    node.destroy_node()
    rclpy.shutdown()
    exit()

# Map parameters
res = map_msg.info.resolution
origin_x = map_msg.info.origin.position.x
origin_y = map_msg.info.origin.position.y
width = map_msg.info.width
height = map_msg.info.height
data = map_msg.data

# Check 12 sectors (30 deg each) for unknown cells
# Look ahead 0.5m to 2m in each direction
scores = []
for sector in range(12):
    # Sector angle in robot frame (0 = front)
    sector_angle = sector * (math.pi / 6)
    # Convert to map frame
    map_angle = robot_yaw + sector_angle

    unknown_count = 0
    total_count = 0

    # Sample points along ray from 0.5m to 3.5m (increased from 2m to see unexplored rooms)
    for dist in [0.5, 0.8, 1.2, 1.6, 2.0, 2.5, 3.0, 3.5]:
        px = robot_x + dist * math.cos(map_angle)
        py = robot_y + dist * math.sin(map_angle)

        # Convert to grid cell
        gx = int((px - origin_x) / res)
        gy = int((py - origin_y) / res)

        if 0 <= gx < width and 0 <= gy < height:
            cell_val = data[gy * width + gx]
            total_count += 1
            if cell_val == -1:  # Unknown
                unknown_count += 1

    # Score: ratio of unknown cells
    if total_count > 0:
        scores.append(unknown_count / total_count)
    else:
        scores.append(0.0)

print(','.join([f'{s:.2f}' for s in scores]))
node.destroy_node()
rclpy.shutdown()
"'''],
                capture_output=True, text=True, timeout=5
            )

            if result.stdout.strip() and result.stdout.strip() not in ['NO_MAP', 'NO_TF']:
                scores = [float(x) for x in result.stdout.strip().split(',')]
                if len(scores) == NUM_SECTORS:
                    # Cache the successful result
                    self._map_scores_cache = scores
                    self._map_scores_cache_time = time.time()
                    return scores

            return self._map_scores_cache  # Return cached if map not available

        except Exception as e:
            return self._map_scores_cache  # Return cached on error

    def get_dead_end_sectors(self) -> List[bool]:
        """
        Detect dead-end sectors from the SLAM map.

        A sector is a dead-end if:
        - It's fully mapped (no unknown cells within 1.5m)
        - There are occupied cells (walls) blocking the path within 1.5m
        - The neighboring sectors are also blocked

        Returns list of 12 booleans: True = dead-end (avoid!), False = passable
        """
        # Return cached result if fresh
        if time.time() - self._dead_end_cache_time < self._DEAD_END_CACHE_TTL:
            return self._dead_end_cache

        try:
            result = subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
                 '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import math

rclpy.init()
node = rclpy.create_node('dead_end_detector')

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer, node)

# Wait for map
map_msg = None
def map_cb(m): global map_msg; map_msg = m
sub = node.create_subscription(OccupancyGrid, '/map', map_cb, 1)

for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.1)
    if map_msg: break

if not map_msg:
    print('NO_MAP')
    node.destroy_node()
    rclpy.shutdown()
    exit()

# Get robot position
try:
    trans = tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
    robot_x = trans.transform.translation.x
    robot_y = trans.transform.translation.y
    q = trans.transform.rotation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    robot_yaw = math.atan2(siny_cosp, cosy_cosp)
except:
    print('NO_TF')
    node.destroy_node()
    rclpy.shutdown()
    exit()

# Map parameters
res = map_msg.info.resolution
origin_x = map_msg.info.origin.position.x
origin_y = map_msg.info.origin.position.y
width = map_msg.info.width
height = map_msg.info.height
data = map_msg.data

# Check each sector for dead-ends
# Flag as dead-end when walls block the path ahead
dead_ends = []
CHECK_DIST = 2.0  # Increased from 1.2m to see past doorways

def find_wall_hit(angle, max_dist):
    # Ray-cast to find first wall hit. Returns (x, y, dist) or None
    for dist in [0.3, 0.5, 0.7, 0.9, 1.1, 1.4, 1.7, 2.0]:
        if dist > max_dist:
            break
        px = robot_x + dist * math.cos(angle)
        py = robot_y + dist * math.sin(angle)
        gx = int((px - origin_x) / res)
        gy = int((py - origin_y) / res)
        if 0 <= gx < width and 0 <= gy < height:
            cell_val = data[gy * width + gx]
            if cell_val >= 50:  # Wall
                return (px, py, dist)
            elif cell_val == -1:  # Unknown - NOT a dead-end (must explore!)
                return None
    return None

def trace_wall_continuous(hit1, hit2):
    # Trace between two wall hits and check if wall is CONTINUOUS
    if hit1 is None or hit2 is None:
        return False

    dx = hit2[0] - hit1[0]
    dy = hit2[1] - hit1[1]
    dist = math.sqrt(dx*dx + dy*dy)

    if dist < 0.05:
        return True

    # Check every ~3cm along the line (finer resolution)
    num_samples = max(3, int(dist / 0.03))
    wall_count = 0
    free_count = 0

    for i in range(num_samples + 1):
        t = i / num_samples
        px = hit1[0] + t * dx
        py = hit1[1] + t * dy

        gx = int((px - origin_x) / res)
        gy = int((py - origin_y) / res)

        if 0 <= gx < width and 0 <= gy < height:
            cell_val = data[gy * width + gx]
            if cell_val >= 50:
                wall_count += 1
            elif cell_val == -1:  # Unknown = MUST explore, not dead-end
                return False
            else:
                free_count += 1

    # Wall is continuous if mostly walls (70%+) with few gaps
    total = wall_count + free_count
    if total > 0 and wall_count >= total * 0.7:
        return True
    return False

for sector in range(12):
    sector_angle = sector * (math.pi / 6)
    map_angle = robot_yaw + sector_angle

    # Cast 5 rays to cover sector width
    angles = [
        map_angle - 0.20,
        map_angle - 0.10,
        map_angle,
        map_angle + 0.10,
        map_angle + 0.20,
    ]

    hits = [find_wall_hit(a, CHECK_DIST) for a in angles]

    # Dead-end if:
    # 1. At least 4 out of 5 rays hit walls within CHECK_DIST
    # 2. Center ray must hit a wall
    # 3. Walls form a mostly continuous barrier
    is_dead_end = False
    hit_count = sum(1 for h in hits if h is not None)
    center_hit = hits[2]  # Center ray

    if hit_count >= 4 and center_hit is not None:
        # Get valid hits for continuity check
        valid_hits = [h for h in hits if h is not None]

        # Check if walls are reasonably close (within 1.2m)
        all_close = all(h[2] <= 1.2 for h in valid_hits)

        if all_close and len(valid_hits) >= 3:
            # Check wall continuity between adjacent valid hits
            continuous_count = 0
            for i in range(len(valid_hits) - 1):
                if trace_wall_continuous(valid_hits[i], valid_hits[i+1]):
                    continuous_count += 1

            # Mostly continuous (at least half the segments)
            if continuous_count >= len(valid_hits) // 2:
                is_dead_end = True

    dead_ends.append('1' if is_dead_end else '0')

print(','.join(dead_ends))
node.destroy_node()
rclpy.shutdown()
"'''],
                capture_output=True, text=True, timeout=2
            )

            if result.stdout.strip() and result.stdout.strip() not in ['NO_MAP', 'NO_TF']:
                dead_ends = [x == '1' for x in result.stdout.strip().split(',')]
                if len(dead_ends) == NUM_SECTORS:
                    self._dead_end_cache = dead_ends
                    self._dead_end_cache_time = time.time()
                    return dead_ends

            return self._dead_end_cache

        except Exception as e:
            return self._dead_end_cache

    def _compute_sector_distances(self):
        """Compute minimum distance for each sector (in ROBOT frame, not LiDAR frame)."""
        if self.scan_ranges:
            self.sector_distances = compute_sector_distances(self.scan_ranges)

    def sector_to_angle(self, sector: int) -> float:
        """Convert sector number to angle in radians (0 = front, positive = left)
        NOTE: This is a wrapper around the global sector_to_angle() function."""
        return sector_to_angle(sector, NUM_SECTORS)

    def find_frontier_direction(self) -> Tuple[Optional[int], float]:
        """
        FRONTIER-BASED EXPLORATION: Find the direction with most unexplored space.

        Uses unified scoring system with "exploration" preset which combines:
        1. LiDAR distance: Farthest = more open space
        2. Map exploration: Unknown cells = unexplored frontier
        3. Space freedom: Avoid corners/constrained areas
        4. Visited tracking: Penalize revisiting areas
        5. Forward bias: Slight preference for forward movement

        Returns: (best_sector, frontier_score) or (None, 0) if all blocked
        """
        # Use unified scorer in exploration mode
        self.unified_scorer.set_mode("exploration")

        # Get map-based scores for obstacle density / exploration
        map_scores = self.get_map_exploration_scores()
        dead_ends = self.get_dead_end_sectors()

        # Use unified scoring
        best_sector, best_score, breakdown = self.unified_scorer.find_best_sector(
            sector_distances=self.sector_distances,
            min_distance=self.lidar_min_distance,
            vfh_controller=self.vfh,
            map_scores=map_scores,
            dead_ends=dead_ends,
            visited_tracker=self.visited_tracker,
            current_position=self.current_position,
            current_heading=self.current_heading,
        )

        return best_sector, best_score

    def get_frontier_visualization(self) -> str:
        """
        Create a visual representation of frontier scores for each sector.
        █ = unexplored (far), ▒ = partial, ░ = explored (close), X = blocked
        """
        viz = []
        for sector in range(NUM_SECTORS):
            dist = self.sector_distances[sector]
            if dist < 0.1:
                viz.append('?')  # Blind spot
            elif dist < self.lidar_min_distance:
                viz.append('X')  # Blocked
            elif dist > 2.0:
                viz.append('█')  # Far = unexplored frontier!
            elif dist > 1.0:
                viz.append('▒')  # Medium
            else:
                viz.append('░')  # Close = explored
        return ''.join(viz)

    def is_exploration_complete(self) -> bool:
        """
        Check if all reachable areas have been explored.

        Exploration is complete when:
        - No direction has a long LiDAR reading (>2m)
        - All directions are either blocked or show nearby walls

        This means we've seen everything we can reach.
        """
        for sector in range(NUM_SECTORS):
            dist = self.sector_distances[sector]
            # Skip blind spots
            if dist < 0.1:
                continue
            # If any direction has open space (>2m), keep exploring
            if dist > 2.0:
                return False
        return True

    def find_best_direction(self) -> Tuple[int, float]:
        """
        VFH-inspired algorithm: Find the best direction using unified scoring.

        Uses unified scorer with "safety" preset which emphasizes:
        - Clearance: Strong obstacle avoidance
        - Distance: Keep distance from obstacles
        - Freedom: Avoid tight spaces
        - Continuity: Smooth trajectory

        Returns (best_sector, best_score)
        """
        # Use unified scorer in safety mode
        self.unified_scorer.set_mode("safety")

        # Get map-based scores
        map_scores = self.get_map_exploration_scores()
        dead_ends = self.get_dead_end_sectors()

        # Use unified scoring
        best_sector, best_score, breakdown = self.unified_scorer.find_best_sector(
            sector_distances=self.sector_distances,
            min_distance=self.lidar_min_distance,
            vfh_controller=self.vfh,
            map_scores=map_scores,
            dead_ends=dead_ends,
            visited_tracker=self.visited_tracker,
            current_position=self.current_position,
            current_heading=self.current_heading,
        )

        # Default to sector 0 if all blocked
        if best_sector is None:
            best_sector = 0
            best_score = -999

        return best_sector, best_score

    def find_frontier_goal(self) -> Optional[Tuple[float, float]]:
        """
        Find the best frontier goal for path planning.

        Uses map exploration scores to identify the most promising
        unexplored area, then returns a goal position to navigate to.

        Returns:
            (x, y) goal position or None if no frontier found
        """
        if not self.current_position:
            return None

        x, y = self.current_position
        map_scores = self.get_map_exploration_scores()

        # Find direction with highest exploration potential
        best_sector = None
        best_score = 0.0

        for sector in range(NUM_SECTORS):
            dist = self.sector_distances[sector]
            if dist < self.lidar_min_distance:
                continue

            score = map_scores[sector]
            # Bonus for longer distances (more unexplored space)
            score += min(dist / 3.0, 1.0) * 0.5

            if score > best_score:
                best_score = score
                best_sector = sector

        if best_sector is None:
            return None

        # Calculate goal position in that direction
        dist = min(self.sector_distances[best_sector], 2.0)  # Max 2m goal
        # Use unified sector_to_angle function for consistency
        robot_angle = sector_to_angle(best_sector, NUM_SECTORS)

        # Account for robot heading if available
        heading = self.current_heading if self.current_heading else 0.0
        world_angle = heading + robot_angle

        goal_x = x + dist * math.cos(world_angle)
        goal_y = y + dist * math.sin(world_angle)

        return (goal_x, goal_y)

    def get_path_planned_direction(self) -> Optional[int]:
        """
        Use A* path planning to get optimal direction.

        Returns:
            Sector index (0-11) to move toward, or None if no path
        """
        if not self.use_path_planning:
            return None

        if not self.current_position or not self.current_heading:
            return None

        # Find or update frontier goal
        goal = self.find_frontier_goal()
        if goal is None:
            return None

        self.current_goal = goal

        # Get next waypoint from path planner
        waypoint = self.path_planner.get_next_waypoint(
            self.current_position, goal, lookahead=0.5
        )

        if waypoint is None:
            return None

        # Convert waypoint to sector direction
        sector = self.path_planner.get_direction_to_waypoint(
            self.current_position, self.current_heading, waypoint
        )

        return sector

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
        # INCREASED values - previous were too small to escape
        if obstacle_distance < 0.3:
            base_degrees = 90  # Very close - big turn
        elif obstacle_distance < 0.4:
            base_degrees = 75  # Close - moderate turn
        elif obstacle_distance < 0.5:
            base_degrees = 60  # Medium - decent turn
        else:
            base_degrees = 45  # Far - smaller turn

        # Adjust based on obstacle position (which sector)
        # Sectors 0, 11 = directly ahead, need bigger turn
        # Sectors 1, 10 = slightly off, need less turn
        # Sectors 2, 9 = more off-center, need even less
        if obstacle_sector in [0, 11]:  # Dead ahead
            base_degrees *= 1.3
        elif obstacle_sector in [1, 10]:  # Slightly off
            base_degrees *= 1.1
        elif obstacle_sector in [2, 9]:  # More off-center
            base_degrees *= 0.9

        return min(120, max(45, base_degrees))  # Clamp between 45-120°

    def transition_state(self, new_state: RobotState, force: bool = False) -> bool:
        """
        Transition to a new state with logging.
        Respects minimum state duration and committed action locking.
        Also tracks escape memory for learning from stuck situations.

        Args:
            new_state: Target state to transition to
            force: If True, bypass minimum duration check (not action lock)

        Returns:
            True if transition succeeded, False if blocked by lock or min duration
        """
        if new_state == self.robot_state:
            return True  # Already in this state - consider success

        # Check committed action locking (prevents race conditions)
        if not self.state_context.can_transition(new_state):
            lock_remaining = self.state_context.get_lock_remaining()
            # Only log if significant time remaining
            if lock_remaining > 0.1:
                pass  # Silently ignore to avoid log spam
            return False  # Transition blocked by lock

        time_in_state = time.time() - self.state_context.state_start_time
        min_duration = self.min_state_duration.get(self.robot_state, 0)

        if not force and time_in_state < min_duration:
            # Log blocked transitions for debugging (safety-critical info)
            print(f"\n[STATE-BLOCKED] {self.robot_state.name} -> {new_state.name} (min_duration={min_duration:.2f}s, elapsed={time_in_state:.2f}s)")
            return False  # Not enough time in current state - transition blocked

        old_state = self.robot_state
        self.robot_state = new_state
        self.state_context.state_start_time = time.time()

        # Clear action lock on state change (new state may set its own lock)
        self.state_context.clear_lock()

        print(f"\n[STATE] {old_state.name} -> {new_state.name}")

        # DIRECTION CHANGE PAUSE: Brief stop when changing from forward to backward
        # This prevents mechanical stress and gives robot time to settle
        if new_state == RobotState.BACKING_UP and old_state in [RobotState.FORWARD, RobotState.CORRIDOR, RobotState.AVOIDING]:
            # Stop the robot briefly before backing up
            self.send_cmd(0.0, 0.0)
            time.sleep(DIRECTION_CHANGE_PAUSE)
            print(f"[PAUSE] Brief stop before backup ({DIRECTION_CHANGE_PAUSE:.2f}s)")
            # Track backup start position to ensure minimum backup distance
            self.backup_start_position = self.current_position

        # Track escape memory for stuck situations
        if self.current_position:
            x, y = self.current_position

            # Entering BACKING_UP = robot is stuck
            if new_state == RobotState.BACKING_UP and old_state in [RobotState.FORWARD, RobotState.CORRIDOR, RobotState.AVOIDING]:
                front = self.sector_distances[0] if self.sector_distances else 1.0
                left = self.sector_distances[1] if self.sector_distances else 1.0
                right = self.sector_distances[11] if self.sector_distances else 1.0
                self.escape_memory.record_stuck(x, y, front, left, right)

            # Successful escape! TURNING -> FORWARD/CORRIDOR
            elif old_state == RobotState.TURNING and new_state in [RobotState.FORWARD, RobotState.CORRIDOR]:
                self.escape_memory.record_escape_success(x, y)
                print(f"[ESCAPE] Recorded successful escape at ({x:.1f}, {y:.1f})")

            # Also track STUCK_RECOVERY -> FORWARD as success
            elif old_state == RobotState.STUCK_RECOVERY and new_state == RobotState.FORWARD:
                self.escape_memory.record_escape_success(x, y)

        # Clear old failures periodically
        self.escape_memory.clear_old_failures()

        return True  # Transition succeeded

    def update_state_context(self, front_clearance: float, best_sector: int,
                            best_distance: float, stuck_detected: bool):
        """Update state context with current sensor data"""
        self.state_context.front_clearance = front_clearance
        self.state_context.best_sector = best_sector
        self.state_context.best_distance = best_distance
        self.state_context.stuck_detected = stuck_detected

    def compute_velocity_vfh(self) -> Tuple[float, float]:
        """
        VFH-based obstacle avoidance with state machine.
        Uses histogram smoothing and hysteresis for smoother behavior.

        Returns:
            (linear, angular) velocity command, or (None, None) if discrete maneuver was executed
        """
        # FIX: Track if safety-critical transition was requested but blocked
        # If blocked, we'll apply a velocity override at the end
        safety_transition_blocked = False
        blocked_target_state = None

        # Update VFH histogram with current sector distances
        # Use lidar_min_distance which accounts for LiDAR mounting offset
        self.vfh.update_histogram(self.sector_distances, self.lidar_min_distance)

        # Find best direction using VFH valley detection
        best_sector, confidence = self.vfh.find_best_valley(target_sector=0)  # Target = front

        # Get front clearance from VFH (smoothed)
        front_clearance = 1.0 - self.vfh.smoothed_histogram[0]  # Inverse: higher = more clear
        front_clear = self.vfh.is_path_clear(0)  # Binary: is front sector clear?

        # Also check raw distances for safety - use unified helper method
        front_arc_min = self.get_front_arc_min()

        # Detect corridor/narrow passage with hysteresis to prevent oscillation
        raw_corridor, raw_width, corridor_direction = self.vfh.detect_corridor(
            self.sector_distances, ROBOT_WIDTH
        )
        # Apply hysteresis to smooth corridor detection
        is_corridor, corridor_width = self.corridor_detector.update(
            raw_corridor, raw_width, ROBOT_WIDTH
        )
        self.state_context.in_corridor = is_corridor
        self.state_context.corridor_width = corridor_width

        # DYNAMIC THRESHOLD: Adjust min distance based on situation
        # Dynamic threshold based on environment context
        # In a corridor where robot fits, use much lower threshold - just physical clearance
        # In open space, use more conservative threshold

        # FIX: Get calibrated extra_margin for obstacle type (applies to both corridor and open space)
        avoidance_params, detected_obstacle = self.get_avoidance_params(front_arc_min)
        extra_margin = avoidance_params.get('extra_margin', 0.0)

        if is_corridor and corridor_width >= MIN_CORRIDOR_WIDTH:
            # CORRIDOR MODE: Tighter thresholds - robot fits, use physical clearance
            # LiDAR offset (37cm) + safety margin (10cm) = ~47cm
            dynamic_min_dist = LIDAR_FRONT_OFFSET + 0.10
            DANGER_DISTANCE = dynamic_min_dist + CORRIDOR_DANGER_MARGIN + extra_margin
        else:
            # OPEN SPACE: Use lidar threshold with standard margin
            dynamic_min_dist = self.lidar_min_distance
            DANGER_DISTANCE = self.lidar_min_distance + DANGER_DISTANCE_MARGIN + extra_margin

        # Update state context
        self.update_state_context(
            front_clearance=front_arc_min,
            best_sector=best_sector,
            best_distance=self.sector_distances[best_sector],
            stuck_detected=False  # Will be updated by stuck detection
        )

        # State machine logic
        current_time = time.time()
        time_in_state = current_time - self.state_context.state_start_time

        # Get dead-end info EARLY for state transitions (before checking in_dead_end)
        dead_ends = self.get_dead_end_sectors()
        map_front_dead_end = dead_ends[0] or (dead_ends[11] and dead_ends[1])
        map_left_dead_end = dead_ends[1] and dead_ends[2]
        map_right_dead_end = dead_ends[11] and dead_ends[10]

        # Check if robot is INSIDE a dead-end (walls close on front, left, AND right)
        # In this case, back out directly - rotating in tight space is difficult
        # Use larger margin (0.40m) to trigger backing out earlier with more room
        DEAD_END_MARGIN = 0.40  # Back out when walls are within this margin of min distance
        left_close = min(self.sector_distances[1], self.sector_distances[2]) < dynamic_min_dist + DEAD_END_MARGIN
        right_close = min(self.sector_distances[10], self.sector_distances[11]) < dynamic_min_dist + DEAD_END_MARGIN
        front_close = front_arc_min < dynamic_min_dist + DEAD_END_MARGIN
        rear_clear = min(self.sector_distances[5], self.sector_distances[6], self.sector_distances[7]) > dynamic_min_dist

        in_dead_end = front_close and left_close and right_close and rear_clear

        if in_dead_end and self.robot_state not in [RobotState.BACKING_UP, RobotState.TURNING]:
            # Robot is trapped in dead-end - back out directly
            print(f"\n[DEAD-END] Trapped! F={front_arc_min:.2f}m L={self.sector_distances[1]:.2f}m R={self.sector_distances[11]:.2f}m - backing out")
            self.avoidance_direction = None  # Will be set to go straight back
            # FIX: Check transition result - if blocked, flag for velocity override
            if not self.transition_state(RobotState.BACKING_UP):
                safety_transition_blocked = True
                blocked_target_state = RobotState.BACKING_UP

        # Evaluate state transitions based on current state
        # Use dynamic_min_dist which adapts to corridor vs open space
        elif self.robot_state == RobotState.STOPPED:
            # From STOPPED: transition based on front clearance
            if front_arc_min >= dynamic_min_dist:
                if is_corridor:
                    self.transition_state(RobotState.CORRIDOR)
                else:
                    self.transition_state(RobotState.FORWARD)
            elif front_arc_min < DANGER_DISTANCE:
                # Front blocked - need to back up
                self.transition_state(RobotState.BACKING_UP)
            else:
                # Can try to steer around
                self.transition_state(RobotState.AVOIDING)

        elif self.robot_state == RobotState.FORWARD:
            # From FORWARD: check for obstacles or corridor entry
            # Reset avoidance counter after sustained forward movement
            if time_in_state > 2.0 and self.state_context.consecutive_avoidances > 0:
                self.state_context.consecutive_avoidances = 0

            # PREDICTIVE dead-end check - avoid BEFORE getting trapped
            if self.predict_dead_end_ahead(look_ahead_distance=1.5):
                print(f"\n[PREDICT] Dead-end ahead! (front={front_arc_min:.2f}m) - avoiding early")
                self.transition_state(RobotState.AVOIDING)
            elif is_corridor and corridor_width >= MIN_CORRIDOR_WIDTH:
                # Entered a corridor - switch to careful mode
                print(f"\n[CORRIDOR] Detected passage (width={corridor_width:.2f}m, thresh={dynamic_min_dist:.2f}m)")
                self.transition_state(RobotState.CORRIDOR)
            elif front_arc_min < dynamic_min_dist:
                # Too close! Need to back up
                self.state_context.consecutive_avoidances += 1
                # FIX: Check transition result - if blocked, flag for velocity override
                if not self.transition_state(RobotState.BACKING_UP):
                    safety_transition_blocked = True
                    blocked_target_state = RobotState.BACKING_UP
            elif front_arc_min < DANGER_DISTANCE:
                # In caution zone - always try to steer around, don't wait for VFH
                # This prevents driving into walls that raw LiDAR can see
                self.transition_state(RobotState.AVOIDING)
            elif map_front_dead_end:
                # Map shows dead-end ahead - avoid BEFORE getting close
                # This is the key fix: don't wait for LiDAR to show obstacle
                print(f"\n[DEAD-END] Map shows dead-end ahead (front_dist={front_arc_min:.2f}m), avoiding early")
                self.transition_state(RobotState.AVOIDING)

        elif self.robot_state == RobotState.CORRIDOR:
            # From CORRIDOR: navigate carefully with dynamic threshold
            # Corridor mode uses lower threshold since we know robot fits
            if front_arc_min < dynamic_min_dist:
                # Front blocked - check if we can steer around before backing up
                left_clear = self.sector_distances[1] >= dynamic_min_dist
                right_clear = self.sector_distances[11] >= dynamic_min_dist
                if left_clear or right_clear:
                    # Can steer around obstacle
                    print(f"\n[CORRIDOR] Steering around obstacle (L={left_clear} R={right_clear})")
                    self.transition_state(RobotState.AVOIDING)
                else:
                    # No way around - must back up
                    print(f"\n[CORRIDOR] Path blocked (front={front_arc_min:.2f}m < {dynamic_min_dist:.2f}m)")
                    self.state_context.consecutive_avoidances += 1
                    # FIX: Check transition result - if blocked, flag for velocity override
                    if not self.transition_state(RobotState.BACKING_UP):
                        safety_transition_blocked = True
                        blocked_target_state = RobotState.BACKING_UP
            elif not is_corridor and front_arc_min >= self.lidar_min_distance:
                # Exited corridor into open space
                print(f"\n[CORRIDOR] Exited passage into open space")
                self.transition_state(RobotState.FORWARD)
            # Otherwise stay in CORRIDOR mode (handled below)

        elif self.robot_state == RobotState.AVOIDING:
            # From AVOIDING: check if we can go forward or need to back up
            if front_arc_min < DANGER_DISTANCE:
                # FIX: Check transition result - if blocked, flag for velocity override
                if not self.transition_state(RobotState.BACKING_UP):
                    safety_transition_blocked = True
                    blocked_target_state = RobotState.BACKING_UP
            elif front_clear and front_arc_min >= dynamic_min_dist:
                self.state_context.consecutive_avoidances = 0
                if is_corridor:
                    self.transition_state(RobotState.CORRIDOR)
                else:
                    self.transition_state(RobotState.FORWARD)

        elif self.robot_state == RobotState.BACKING_UP:
            # From BACKING_UP: back up until we have enough clearance to turn safely
            # Instead of fixed time, check front distance AND minimum backup distance

            # FIX: Get calibrated backup_mult for obstacle type
            # Round obstacles need MORE clearance before turning (backup_mult > 1.0)
            avoidance_params, detected_obstacle = self.get_avoidance_params(front_arc_min)
            backup_mult = avoidance_params.get('backup_mult', 1.0)

            # Check if we have enough clearance in front to turn safely
            # Scale required clearance by backup_mult for round/curved obstacles
            required_clearance = SAFE_TURN_CLEARANCE * backup_mult
            has_turn_clearance = front_arc_min >= required_clearance

            # Calculate how far we've backed up
            backup_distance = 0.0
            if self.backup_start_position and self.current_position:
                dx = self.current_position[0] - self.backup_start_position[0]
                dy = self.current_position[1] - self.backup_start_position[1]
                backup_distance = math.sqrt(dx*dx + dy*dy)

            # Minimum backup distance - scale by backup_mult AND consecutive failures
            # Each failed attempt increases required backup by 10cm
            failure_mult = 1.0 + (self.state_context.consecutive_avoidances * 0.5)
            required_min_distance = MIN_BACKUP_DISTANCE * backup_mult * failure_mult
            has_min_distance = backup_distance >= required_min_distance

            # Safety limit: don't backup forever
            exceeded_max_time = time_in_state >= MAX_BACKUP_TIME

            # Minimum backup time before checking clearance (let robot start moving first)
            MIN_BACKUP_TIME = 0.3
            past_minimum = time_in_state >= MIN_BACKUP_TIME

            if past_minimum and has_turn_clearance and has_min_distance:
                # We have enough clearance AND backed up far enough - start turning
                print(f"\n[BACKUP-DONE] Clearance: {front_arc_min:.2f}m >= {required_clearance:.2f}m, backed={backup_distance:.2f}m/{required_min_distance:.2f}m (attempt {self.state_context.consecutive_avoidances+1})")
                self.transition_state(RobotState.TURNING)
            elif past_minimum and has_turn_clearance and not has_min_distance:
                # Have clearance but haven't backed up enough - keep backing
                # This is the key fix: force minimum backup distance even if clearance looks OK
                pass  # Continue backing up
            elif exceeded_max_time:
                # Safety limit reached - stop backing even without full clearance
                print(f"\n[BACKUP-TIMEOUT] Max time {MAX_BACKUP_TIME:.1f}s reached, front={front_arc_min:.2f}m, dist={backup_distance:.2f}m")
                self.transition_state(RobotState.TURNING)

        elif self.robot_state == RobotState.TURNING:
            # From TURNING: check for clear path WHILE turning (not just after)
            # Stop early if we find a clear path - use dynamic threshold
            if front_arc_min >= dynamic_min_dist and time_in_state > 0.5:
                # SUCCESS! Clear path found - reset all avoidance state
                self.state_context.consecutive_avoidances = 0
                self.avoidance_direction = None  # Reset for next avoidance
                if is_corridor:
                    print(f"\n[SUCCESS] Found corridor ({front_arc_min:.2f}m >= {dynamic_min_dist:.2f}m)")
                    self.transition_state(RobotState.CORRIDOR)
                else:
                    print(f"\n[SUCCESS] Clear path found ({front_arc_min:.2f}m) during turn")
                    self.transition_state(RobotState.FORWARD)
            elif time_in_state >= self.state_context.maneuver_duration:
                # Turn completed but still blocked - try OPPOSITE direction
                # FIX: Don't reset direction! Alternate to avoid infinite loops
                if self.avoidance_direction == "left":
                    self.avoidance_direction = "right"
                elif self.avoidance_direction == "right":
                    self.avoidance_direction = "left"
                # else: keep None/straight, will be recalculated

                self.state_context.consecutive_avoidances += 1

                # If we've tried both directions multiple times, escalate to STUCK_RECOVERY
                if self.state_context.consecutive_avoidances > 4:
                    print(f"\n[STUCK] Multiple turn attempts failed ({self.state_context.consecutive_avoidances}x), entering STUCK_RECOVERY")
                    self.transition_state(RobotState.STUCK_RECOVERY)
                else:
                    self.transition_state(RobotState.BACKING_UP)

        elif self.robot_state == RobotState.STUCK_RECOVERY:
            # From STUCK_RECOVERY: aggressive recovery
            if time_in_state > 2.0 and front_arc_min >= dynamic_min_dist:
                self.transition_state(RobotState.FORWARD)

        # Execute current state
        linear, angular = 0.0, 0.0
        status = ""

        if self.robot_state == RobotState.FORWARD:
            # Go forward with auto-centering toward a NAVIGABLE path
            # Key: steer toward a direction that allows movement (above threshold)
            # Not necessarily the clearest - just any viable path

            # =================================================================
            # FRONTIER-BASED EXPLORATION: Go where LiDAR sees farthest!
            # Simple rule: Farthest = unexplored = go there!
            # =================================================================

            # Check if exploration is complete
            if self.is_exploration_complete():
                print(f"\n[COMPLETE] All reachable areas explored!")
                linear = 0.0
                angular = 0.0
                self.transition_state(RobotState.STOPPED)

            # Find the frontier (direction with most unexplored space)
            frontier_sector, frontier_score = self.find_frontier_direction()

            # CRITICAL: Check if front is blocked BEFORE following frontier
            # The frontier might be to the side, but we can't drive through obstacles!
            # Use DANGER_DISTANCE for consistency with state transition logic
            front_blocked = front_arc_min < DANGER_DISTANCE

            if frontier_sector is None:
                # All directions blocked - transition to BACKING_UP immediately
                print(f"\n[BLOCKED] No frontier found - all directions blocked!")
                self.state_context.consecutive_avoidances += 1
                self.state_context.state_start_time = 0
                self.transition_state(RobotState.BACKING_UP)
                # Return safe values and skip margin steering logic
                return self.linear_speed * 0.3, 0.0  # Slow forward while transitioning

            elif front_blocked:
                # FRONT IS BLOCKED - must steer away or stop, regardless of frontier!
                # This is the key fix: don't drive into obstacle while chasing frontier
                left_clear = self.sector_distances[1] >= DANGER_DISTANCE
                right_clear = self.sector_distances[11] >= DANGER_DISTANCE

                if left_clear and right_clear:
                    # Both sides clear - steer toward frontier if it's to the side
                    if frontier_sector in [1, 2, 3]:
                        linear = self.get_adaptive_speed(front_arc_min) * 0.5
                        angular = 0.4  # Turn left
                        print(f"\r[STEER-L] Front blocked ({front_arc_min:.2f}m), steering left toward frontier s{frontier_sector}", end="")
                    elif frontier_sector in [9, 10, 11]:
                        linear = self.get_adaptive_speed(front_arc_min) * 0.5
                        angular = -0.4  # Turn right
                        print(f"\r[STEER-R] Front blocked ({front_arc_min:.2f}m), steering right toward frontier s{frontier_sector}", end="")
                    else:
                        # Frontier is behind - turn in place
                        linear = 0.0
                        angular = 0.5 if frontier_sector <= 6 else -0.5
                        print(f"\r[TURN] Front blocked, turning toward frontier s{frontier_sector}", end="")
                elif left_clear:
                    # Only left is clear - steer left
                    linear = self.get_adaptive_speed(front_arc_min) * 0.4
                    angular = 0.5
                    print(f"\r[ESCAPE-L] Front blocked ({front_arc_min:.2f}m), only left clear", end="")
                elif right_clear:
                    # Only right is clear - steer right
                    linear = self.get_adaptive_speed(front_arc_min) * 0.4
                    angular = -0.5
                    print(f"\r[ESCAPE-R] Front blocked ({front_arc_min:.2f}m), only right clear", end="")
                else:
                    # All front directions blocked - transition to AVOIDING or BACKING_UP
                    print(f"\n[TRAPPED] Front arc blocked ({front_arc_min:.2f}m), transitioning to AVOIDING")
                    self.transition_state(RobotState.AVOIDING)
                    return 0.0, 0.0

            elif frontier_sector == 0:
                # Frontier is straight ahead AND front is clear - go forward!
                linear = self.get_adaptive_speed(front_arc_min)
                angular = 0.0
            elif frontier_sector in [1, 11]:
                # Frontier is slightly to the side - gentle curve
                linear = self.get_adaptive_speed(front_arc_min)
                angular = 0.15 if frontier_sector == 1 else -0.15
            elif frontier_sector in [2, 10]:
                # Frontier is more to the side - sharper curve
                linear = self.get_adaptive_speed(front_arc_min) * 0.8
                angular = 0.25 if frontier_sector == 2 else -0.25
            elif frontier_sector in [3, 9]:
                # Frontier is to the side - turn while moving slowly
                linear = self.get_adaptive_speed(front_arc_min) * 0.5
                angular = 0.35 if frontier_sector == 3 else -0.35
            else:
                # Frontier is behind (sectors 4-8) - need to turn around
                # Stop forward motion and turn toward frontier
                linear = 0.0
                if frontier_sector <= 6:
                    angular = 0.5  # Turn left
                else:
                    angular = -0.5  # Turn right

            # OBSTACLE MARGIN STEERING: Steer away from close obstacles on either side
            # If obstacle is closer on one side, add angular velocity to steer away
            left_dist = min(self.sector_distances[1], self.sector_distances[2])  # Front-left
            right_dist = min(self.sector_distances[10], self.sector_distances[11])  # Front-right
            front_left = self.sector_distances[1]
            front_right = self.sector_distances[11]

            # Only apply margin steering if obstacle is within danger zone
            MARGIN_THRESHOLD = 0.8  # Apply margin steering within 80cm
            MARGIN_GAIN = 0.3  # How much to steer away (rad/s per meter difference)

            if left_dist < MARGIN_THRESHOLD or right_dist < MARGIN_THRESHOLD:
                # Calculate asymmetry: positive = obstacle closer on left, negative = closer on right
                asymmetry = right_dist - left_dist  # Positive means left is closer

                # Scale by how close the nearest obstacle is
                closest = min(left_dist, right_dist)
                urgency = 1.0 - (closest / MARGIN_THRESHOLD)  # 0 to 1, higher = more urgent
                urgency = max(0.0, min(1.0, urgency))

                # Add steering away from the closer obstacle
                margin_angular = asymmetry * MARGIN_GAIN * urgency

                # Cap the margin steering
                margin_angular = max(-0.3, min(0.3, margin_angular))

                # Only apply if it doesn't conflict too much with frontier direction
                # (don't override strong turns toward frontier)
                if abs(angular) < 0.4:
                    angular += margin_angular

                if abs(margin_angular) > 0.05:
                    side = "L" if asymmetry > 0 else "R"
                    print(f"\r[MARGIN] {side} obstacle at {closest:.2f}m, steering {margin_angular:+.2f} rad/s", end="")

            self.emergency_maneuver = False

            # Show frontier visualization
            frontier_viz = self.get_frontier_visualization()
            front_dist = self.sector_distances[0]
            speed_pct = int(100 * linear / self.linear_speed) if self.linear_speed > 0 else 100
            target_str = f"s{frontier_sector}" if frontier_sector is not None else "NONE"
            status = f"[EXPLORE] f={front_dist:.2f}m [{frontier_viz}] target={target_str} spd={speed_pct}%"

            # Clear avoidance state
            if self.avoidance_direction is not None:
                self.avoidance_direction = None
                self.avoidance_intensity = 1.0

        elif self.robot_state == RobotState.AVOIDING:
            # Use DWA trajectory prediction to find best velocity
            # Assume robot is at origin facing forward (0,0,0) for local planning
            dwa_v, dwa_w, dwa_score = self.trajectory_scorer.compute_best_velocity(
                x=0.0, y=0.0, yaw=0.0,
                current_v=self.velocity_smoother.last_linear,
                current_w=self.velocity_smoother.last_angular,
                sector_distances=self.sector_distances,
                target_yaw=0.0  # Target is straight ahead
            )

            # ADAPTIVE SPEED: Use slower speed when avoiding obstacles
            adaptive_speed = self.get_adaptive_speed(front_arc_min)

            if dwa_score > float('-inf'):
                # Use DWA-computed velocity, but cap by adaptive speed
                linear = min(dwa_v, adaptive_speed)
                angular = dwa_w
                status = f"[AVOID-DWA] v={linear:.2f} w={dwa_w:.2f} score={dwa_score:.2f}"
            else:
                # DWA found no valid trajectory - check if front is also blocked
                # If front is too close, we need to back up, not steer into obstacle
                if front_arc_min < dynamic_min_dist:
                    # Can't go forward - transition to BACKING_UP
                    print(f"\n[AVOID-BLOCKED] DWA failed and front blocked ({front_arc_min:.2f}m), backing up")
                    self.transition_state(RobotState.BACKING_UP, force=True)
                    linear = 0.0
                    angular = 0.0
                    status = f"[AVOID->BACKUP] front={front_arc_min:.2f}m"
                else:
                    # Fallback to VFH sector steering if DWA finds no valid trajectory
                    sector_angle = self.sector_to_angle(best_sector)
                    turn_factor = abs(sector_angle) / math.pi
                    # Apply both turn factor reduction AND adaptive speed
                    linear = adaptive_speed * (1.0 - turn_factor * 0.4)
                    angular = sector_angle * 0.4
                    status = f"[AVOID-VFH] s{best_sector} ang={math.degrees(sector_angle):.0f}°"

            self.emergency_maneuver = False

        elif self.robot_state == RobotState.BACKING_UP:
            # Check if we're trapped in dead-end (both sides blocked)
            # In that case, back straight out - no room to turn
            # Use same DEAD_END_MARGIN as dead-end detection for consistency
            DEAD_END_MARGIN = 0.40
            trapped_in_dead_end = (
                min(self.sector_distances[1], self.sector_distances[2]) < dynamic_min_dist + DEAD_END_MARGIN and
                min(self.sector_distances[10], self.sector_distances[11]) < dynamic_min_dist + DEAD_END_MARGIN and
                min(self.sector_distances[5], self.sector_distances[6], self.sector_distances[7]) > dynamic_min_dist
            )

            if trapped_in_dead_end:
                # Back straight out - no turning in tight dead-end
                self.avoidance_direction = "straight"
                self.state_context.maneuver_duration = 0.0  # No turn after backing
                status = f"[BACKUP-STRAIGHT] trapped, backing out t={time_in_state:.1f}s"
                linear = -self.linear_speed
                angular = 0.0

            # Determine avoidance direction based on VFH (do this every iteration to ensure it's set)
            elif self.avoidance_direction is None or self.avoidance_direction == "straight":
                # Use VFH to find the nearest clear sector and calculate minimum turn
                # Start from front (sector 0) and scan outward in both directions
                best_left_sector = None
                best_right_sector = None

                # Get dead-end info to avoid turning toward dead-ends
                dead_ends = self.get_dead_end_sectors()
                map_scores = self.get_map_exploration_scores()

                # Scan left (sectors 1, 2, 3, 4, 5, 6) - includes back-left
                # Check both clearance AND if robot physically fits
                # Also check for dead-ends
                for sector in [1, 2, 3, 4, 5, 6]:
                    if (self.vfh.is_path_clear(sector) and
                        self.sector_distances[sector] >= self.lidar_min_distance and
                        self.vfh.robot_fits_in_direction(sector, self.sector_distances,
                                                         ROBOT_HALF_WIDTH, self.lidar_min_distance) and
                        not (dead_ends[sector] and map_scores[sector] < 0.1)):  # Skip dead-ends
                        best_left_sector = sector
                        break

                # Scan right (sectors 11, 10, 9, 8, 7, 6) - includes back-right
                for sector in [11, 10, 9, 8, 7, 6]:
                    if (self.vfh.is_path_clear(sector) and
                        self.sector_distances[sector] >= self.lidar_min_distance and
                        self.vfh.robot_fits_in_direction(sector, self.sector_distances,
                                                         ROBOT_HALF_WIDTH, self.lidar_min_distance) and
                        not (dead_ends[sector] and map_scores[sector] < 0.1)):  # Skip dead-ends
                        best_right_sector = sector
                        break

                # Calculate turn angles to each clear sector
                # Each sector is 30°, so sector 1 = 30°, sector 2 = 60°, etc.
                # REDUCED: Removed 15° margin for tighter turns
                left_turn_angle = (best_left_sector * 30) if best_left_sector else 180
                right_turn_angle = ((12 - best_right_sector) * 30) if best_right_sector else 180

                # Penalize dead-end directions by adding angle penalty
                # Consistent with FORWARD state: dead-end is dead-end regardless of exploration
                left_dead_end = dead_ends[1] and dead_ends[2]
                right_dead_end = dead_ends[11] and dead_ends[10]
                if left_dead_end:
                    left_turn_angle += 60  # Heavy penalty for dead-end
                if right_dead_end:
                    right_turn_angle += 60

                # Choose direction with smaller turn, plus small margin
                margin = 5  # REDUCED from 10° to 5° for tighter avoidance

                # FORCED ALTERNATION: Alternate direction after multiple failures to escape passages
                # Only trigger after 3+ consecutive avoidances to give same direction a fair chance
                # Was >= 1, which caused L-shaped corner traps
                force_alternate = self.state_context.consecutive_avoidances >= 3

                # Get calibrated avoidance params for current obstacle
                avoidance_params, detected_obstacle = self.get_avoidance_params(front_arc_min)
                calibrated_turn = avoidance_params.get('min_turn_deg', 45)
                backup_mult = avoidance_params.get('backup_mult', 1.0)  # FIX: Get backup multiplier

                # First, check escape memory for learned successful escapes
                escape_suggestion = None
                if self.current_position:
                    escape_suggestion = self.escape_memory.get_suggested_escape(*self.current_position)

                if escape_suggestion and escape_suggestion["confidence"] >= 2:
                    # We have strong history at this location
                    # BUT: only use memory if no specific obstacle detected, or obstacle type matches
                    use_memory = True
                    if detected_obstacle and detected_obstacle != escape_suggestion.get("obstacle_type"):
                        # Different obstacle type at this location - use calibration instead
                        use_memory = False
                        print(f"\n[ESCAPE-MEM] Ignoring memory (was {escape_suggestion.get('obstacle_type')}, now {detected_obstacle})")

                    if use_memory:
                        self.avoidance_direction = escape_suggestion["direction"]
                        turn_degrees = escape_suggestion.get("turn_angle", 60)
                        # Record this as an attempt
                        self.escape_memory.record_escape_attempt(
                            self.avoidance_direction,
                            escape_suggestion.get("duration", 1.0),
                            0.3,  # backup distance
                            turn_degrees
                        )
                        print(f"\n[ESCAPE-MEM] Using learned escape: {self.avoidance_direction} (conf={escape_suggestion['confidence']})")
                elif force_alternate:
                    # Force opposite direction from last time
                    # BUT: don't turn into a dead-end if other direction is viable
                    preferred = "right" if self.last_avoidance_direction == "left" else "left"
                    if preferred == "left" and left_dead_end and not right_dead_end:
                        preferred = "right"
                        print(f"\n[DEAD-END] Avoiding left dead-end, going right")
                    elif preferred == "right" and right_dead_end and not left_dead_end:
                        preferred = "left"
                        print(f"\n[DEAD-END] Avoiding right dead-end, going left")
                    self.avoidance_direction = preferred
                    turn_degrees = right_turn_angle + margin if self.avoidance_direction == "right" else left_turn_angle + margin
                    # Record attempt for escape memory
                    self.escape_memory.record_escape_attempt(self.avoidance_direction, 1.0, 0.3, abs(turn_degrees))
                    print(f"\n[ALTERNATE] Forcing {self.avoidance_direction} (avoiding passage trap)")
                elif left_turn_angle <= right_turn_angle:
                    self.avoidance_direction = "left"
                    turn_degrees = left_turn_angle + margin
                    # Record attempt for escape memory
                    self.escape_memory.record_escape_attempt(self.avoidance_direction, 1.0, 0.3, turn_degrees)
                else:
                    self.avoidance_direction = "right"
                    turn_degrees = right_turn_angle + margin
                    # Record attempt for escape memory
                    self.escape_memory.record_escape_attempt(self.avoidance_direction, 1.0, 0.3, turn_degrees)

                self.last_avoidance_direction = self.avoidance_direction

                # Clamp turn angle - use calibrated minimum for obstacle type
                # For round obstacles (pillow_chair), calibrated_turn might be 60°
                # For thin obstacles, might be 30°
                min_turn = max(20, calibrated_turn)  # Use calibrated min, floor at 20°
                turn_degrees = max(min_turn, min(120, turn_degrees))

                # Print detected obstacle for debugging
                if detected_obstacle:
                    print(f"        Detected: {detected_obstacle} (min_turn={calibrated_turn}°)")

                # Apply direction sign
                if self.avoidance_direction == "right":
                    turn_degrees = -turn_degrees

                # Store maneuver duration for turning state (use utility function)
                self.state_context.maneuver_duration = calc_turn_duration(abs(turn_degrees), 1.0)

                # FIX: Apply backup_mult to backup speed for round obstacles
                backup_speed = self.linear_speed * backup_mult

                # Lock the backup action to prevent race conditions
                # Lock for max backup duration - will be cleared early when clearance is reached
                self.state_context.lock_action(
                    action="backup",
                    duration=MAX_BACKUP_TIME,
                    direction=self.avoidance_direction,
                    velocity=(-backup_speed, 0.0)
                )

                dead_str = f"L={'D' if left_dead_end else 'ok'} R={'D' if right_dead_end else 'ok'}"
                print(f"\n[BACKUP] dir={self.avoidance_direction} turn={turn_degrees:.0f}° (L:{left_turn_angle}° R:{right_turn_angle}°) {dead_str} backup_mult={backup_mult:.1f}")

                # Send backup command with backup_mult applied
                linear = -backup_speed
                angular = 0.0
                status = f"[BACKUP] t={time_in_state:.1f}s front={front_arc_min:.2f}m/{SAFE_TURN_CLEARANCE:.2f}m dir={self.avoidance_direction}"

            else:
                # Direction already determined from previous iteration - continue backing up
                # FIX: Get backup_mult for consistent backup speed
                avoidance_params, _ = self.get_avoidance_params(front_arc_min)
                backup_mult = avoidance_params.get('backup_mult', 1.0)
                backup_speed = self.linear_speed * backup_mult
                linear = -backup_speed
                angular = 0.0
                status = f"[BACKUP] t={time_in_state:.1f}s front={front_arc_min:.2f}m/{SAFE_TURN_CLEARANCE:.2f}m dir={self.avoidance_direction}"

        elif self.robot_state == RobotState.TURNING:
            # Skip turning if we were backing straight out of dead-end
            if self.avoidance_direction == "straight":
                # No turn needed - go back to FORWARD to reassess
                self.avoidance_direction = None
                self.transition_state(RobotState.FORWARD)
                linear = 0.0
                angular = 0.0
                status = "[TURN->FWD] Exited dead-end"
            else:
                # Ensure direction is set (safety check)
                if self.avoidance_direction is None:
                    self.avoidance_direction = "left"  # Default

                # Lock the turn action if not already locked
                if not self.state_context.is_action_locked():
                    turn_speed = 1.0 if self.avoidance_direction == "left" else -1.0
                    self.state_context.lock_action(
                        action="turn",
                        duration=self.state_context.maneuver_duration,
                        direction=self.avoidance_direction,
                        velocity=(0.0, turn_speed)
                    )

                # Execute turn (use locked velocity if available)
                turn_speed = 1.0 if self.avoidance_direction == "left" else -1.0
                linear = 0.0
                angular = turn_speed
                status = f"[TURN] dir={self.avoidance_direction} t={time_in_state:.1f}s lock={self.state_context.get_lock_remaining():.1f}s"

        elif self.robot_state == RobotState.CORRIDOR:
            # CORRIDOR mode: Baby steps through narrow passage
            # Steer toward a navigable path, not just center
            corridor_width = self.state_context.corridor_width

            # Use consolidated helper for side clearances
            left_dist, right_dist = self.get_side_clearances()
            front_dist = self.sector_distances[0]

            # Check which directions are navigable (above corridor threshold)
            front_nav = front_dist >= dynamic_min_dist
            left_nav = left_dist >= dynamic_min_dist
            right_nav = right_dist >= dynamic_min_dist

            # Steer toward a navigable path
            max_correction = 0.35  # Slightly higher in corridor for quicker response

            if front_nav:
                # Front is clear - center between walls with margin from closer obstacle
                raw_balance = left_dist - right_dist

                # Apply exponential smoothing to balance to reduce oscillation from sensor noise
                # Higher alpha = faster response but more noise, lower = smoother but slower
                BALANCE_SMOOTHING = 0.3  # Smooth heavily to prevent oscillation
                if not hasattr(self, '_smoothed_balance'):
                    self._smoothed_balance = raw_balance
                self._smoothed_balance = BALANCE_SMOOTHING * raw_balance + (1 - BALANCE_SMOOTHING) * self._smoothed_balance
                balance = self._smoothed_balance

                # MARGIN STEERING: More aggressive steering away from close obstacles
                # If one side is much closer, steer away more aggressively
                CORRIDOR_MARGIN_THRESHOLD = 0.5  # 50cm threshold in corridors
                closest_side = min(left_dist, right_dist)

                # Hysteresis: use different thresholds for starting vs stopping correction
                # Start correcting at 10cm, stop correcting at 5cm
                BALANCE_START_THRESHOLD = 0.10  # Start correcting if 10cm+ off-center
                BALANCE_STOP_THRESHOLD = 0.05   # Stop correcting when within 5cm

                if closest_side < CORRIDOR_MARGIN_THRESHOLD:
                    # Close obstacle - steer away with urgency
                    urgency = 1.0 - (closest_side / CORRIDOR_MARGIN_THRESHOLD)
                    # Stronger correction when closer
                    angular = max(-max_correction, min(max_correction, balance * (0.5 + urgency * 0.5)))
                elif abs(balance) > BALANCE_START_THRESHOLD:
                    # Start centering correction
                    angular = max(-max_correction, min(max_correction, balance * 0.4))
                elif abs(balance) > BALANCE_STOP_THRESHOLD and abs(self.velocity_smoother.last_angular) > 0.01:
                    # Continue existing correction (hysteresis) until within stop threshold
                    angular = max(-max_correction, min(max_correction, balance * 0.3))
                else:
                    angular = 0.0
            elif left_nav and not right_nav:
                # Only left navigable - steer left
                angular = max_correction
            elif right_nav and not left_nav:
                # Only right navigable - steer right
                angular = -max_correction
            elif left_nav and right_nav:
                # Both sides navigable but not front - pick better one
                if left_dist > right_dist:
                    angular = max_correction
                else:
                    angular = -max_correction
            else:
                # Nothing navigable - go straight (state machine will backup)
                angular = 0.0

            # Use reduced speed in corridor (baby steps)
            corridor_speed = self.linear_speed * 0.5  # Half speed
            linear = corridor_speed

            nav_str = f"{'L' if left_nav else '.'}{'F' if front_nav else '.'}{'R' if right_nav else '.'}"
            status = f"[CORRIDOR] w={corridor_width:.2f}m L={left_dist:.2f} R={right_dist:.2f} nav={nav_str}"

        elif self.robot_state == RobotState.STUCK_RECOVERY:
            # Aggressive recovery: alternate backup and spin
            if time_in_state < 1.0:
                linear = -self.linear_speed * 1.5
                angular = 0.0
            else:
                linear = 0.0
                angular = 1.0 if self.avoidance_direction == "left" else -1.0
            status = f"[STUCK_REC] t={time_in_state:.1f}s"

        elif self.robot_state == RobotState.STOPPED:
            linear = 0.0
            angular = 0.0
            status = "[STOPPED]"

        # Print status (compact)
        if status:
            # Also show VFH binary histogram as a compact visualization
            hist_viz = ''.join(['█' if b else '░' for b in self.vfh.binary_histogram])
            print(f"\r{status} VFH:[{hist_viz}]", end="", flush=True)

        # FIX: Safety override - if a safety-critical transition was blocked,
        # apply safe velocity values to prevent collision regardless of current state
        if safety_transition_blocked:
            if blocked_target_state == RobotState.BACKING_UP:
                # Wanted to back up but couldn't - at least stop forward motion!
                # This prevents driving into obstacles when transition is blocked by min_duration
                print(f"\n[SAFETY] Transition to BACKING_UP blocked - stopping forward motion")
                linear = min(linear, 0.0)  # Stop forward, allow backward
                # If we had a positive linear (going forward), stop completely
                if linear >= 0:
                    linear = 0.0

        return linear, angular

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

        # Use consolidated helper method for front arc minimum distance
        # This replaces duplicated code and ensures consistent calculation
        front_arc_min = self.get_front_arc_min()

        # DANGER_DISTANCE: Backup when front ARC (sectors 11, 0, 1) is too close
        # Use front_arc_min which includes front-left and front-right, not just sector 0
        DANGER_DISTANCE = self.lidar_min_distance  # 0.82m (includes LiDAR offset)

        # Check if round obstacle detected - add extra margin
        # Round obstacles (pillow chair) need earlier avoidance due to curved surface
        extra_margin = 0.0
        if self._detect_round_obstacle():
            avoidance_params, _ = self.get_avoidance_params(front_arc_min)
            extra_margin = avoidance_params.get('extra_margin', 0.0)

        # Path is clear if front ARC is above threshold (including extra margin for round obstacles)
        # Uses front_arc_min which considers sectors 11, 0, 1 (front-right, front, front-left)
        path_clear = front_arc_min >= (self.lidar_min_distance + extra_margin)

        if front_arc_min < (DANGER_DISTANCE + extra_margin):
            # OBSTACLE TOO CLOSE! Execute discrete avoidance maneuver
            self.emergency_maneuver = True

            # ALWAYS re-evaluate direction based on current sensor readings
            # Don't keep old direction - it might point back to the obstacle!

            # Check if we're in the same area as last avoidance (within 0.5m)
            same_area = False
            if self.current_position and self.last_avoidance_position:
                dx = self.current_position[0] - self.last_avoidance_position[0]
                dy = self.current_position[1] - self.last_avoidance_position[1]
                dist_from_last = math.sqrt(dx*dx + dy*dy)
                same_area = dist_from_last < 0.5

            # Get clearance on both sides - use MIN to find CLOSEST obstacle on each side
            # LEFT side: sectors 1 (front-left), 2, 3, 4 (left)
            # RIGHT side: sectors 11 (front-right), 10, 9, 8 (right)
            left_dists = []
            for s in [1, 2, 3, 4]:
                d = self.sector_distances[s]
                thresh = BODY_THRESHOLD_FRONT if s == 1 else BODY_THRESHOLD_SIDE
                if d > thresh:
                    left_dists.append(d)
            # FIXED: Empty list means no obstacles detected = clear (10m), not blocked (0m)
            left_clearance = min(left_dists) if left_dists else 10.0

            right_dists = []
            for s in [11, 10, 9, 8]:
                d = self.sector_distances[s]
                thresh = BODY_THRESHOLD_FRONT if s == 11 else BODY_THRESHOLD_SIDE
                if d > thresh:
                    right_dists.append(d)
            # FIXED: Empty list means no obstacles detected = clear (10m), not blocked (0m)
            right_clearance = min(right_dists) if right_dists else 10.0

            # Get exploration scores to prefer unexplored directions
            map_scores = self.get_map_exploration_scores()
            left_explore = max(map_scores[1], map_scores[2], map_scores[3])  # Sectors 1-3 (left side)
            right_explore = max(map_scores[10], map_scores[11], map_scores[9])  # Sectors 9-11 (right side)

            # Combined score: clearance + exploration (exploration weighted higher)
            CLEARANCE_WEIGHT = 1.0
            EXPLORE_WEIGHT = 2.0  # Strongly prefer unexplored areas
            left_score = CLEARANCE_WEIGHT * left_clearance + EXPLORE_WEIGHT * left_explore
            right_score = CLEARANCE_WEIGHT * right_clearance + EXPLORE_WEIGHT * right_explore

            # Check if this looks like a passage (both sides have similar clearance)
            clearance_ratio = min(left_clearance, right_clearance) / max(left_clearance, right_clearance, 0.01)
            is_passage = clearance_ratio > 0.5 and left_clearance > 0.3 and right_clearance > 0.3

            # Track consecutive avoidance attempts FIRST (before updating avoidance_start_time)
            # Check time since LAST avoidance to determine if this is a repeated attempt
            time_since_last = time.time() - self.avoidance_start_time if self.avoidance_start_time > 0 else 999
            recent_avoidance = time_since_last < 8.0  # Within 8 seconds = same obstacle

            if same_area or recent_avoidance:
                self.avoidance_attempt_count += 1
            else:
                self.avoidance_attempt_count = 1  # First attempt at new location/time

            # DIRECTION SELECTION with forced alternation on repeated attempts
            # First attempt: turn toward side with MORE clearance
            # Repeated attempts: ALTERNATE direction to escape corners/traps
            if self.avoidance_attempt_count == 1:
                # First attempt - choose based on clearance
                if left_clearance > right_clearance:
                    self.avoidance_direction = "left"
                else:
                    self.avoidance_direction = "right"
            else:
                # Repeated attempt - ALTERNATE from last direction
                if self.last_avoidance_direction == "left":
                    self.avoidance_direction = "right"
                else:
                    self.avoidance_direction = "left"

            # Save state AFTER determining direction
            self.last_avoidance_direction = self.avoidance_direction
            self.last_avoidance_position = self.current_position
            self.avoidance_intensity = 1.0
            self.avoidance_start_time = time.time()  # Update AFTER using the old value

            # Show which side has more clearance and which direction was chosen
            obstacle_side = "LEFT" if left_clearance < right_clearance else "RIGHT"
            print(f"\n[AVOID] Obstacle on {obstacle_side}, turning {self.avoidance_direction.upper()}")
            print(f"        LEFT min:  {left_clearance:.2f}m from {left_dists}")
            print(f"        RIGHT min: {right_clearance:.2f}m from {right_dists}")

            # Get obstacle-specific avoidance parameters (based on shape: round/flat/thin)
            front_dist = self.sector_distances[0] if self.sector_distances else 1.0
            avoidance_params, detected_obstacle = self.get_avoidance_params(front_dist)
            base_turn = avoidance_params.get('min_turn_deg', 45)
            backup_mult = avoidance_params.get('backup_mult', 1.0)
            obstacle_shape = avoidance_params.get('shape', 'flat')

            # Print detected obstacle info for user
            if detected_obstacle:
                print(f"        Detected: {detected_obstacle} ({obstacle_shape} shape, turn {base_turn}°)")

            # ESCALATING turn angles based on attempt count
            # Attempt 1: base_turn (45°)
            # Attempt 2: 60° + alternate direction
            # Attempt 3: 90° + alternate direction
            # Attempt 4+: 120° + alternate direction
            if self.avoidance_attempt_count >= 4:
                turn_degrees = 120  # Large turn but not full 180° (which can put robot in same spot)
                print(f"        (Attempt #{self.avoidance_attempt_count} - LARGE TURN 120°, alternating)")
            elif self.avoidance_attempt_count >= 3:
                turn_degrees = 90
                print(f"        (Attempt #{self.avoidance_attempt_count} - ESCALATING to 90°, alternating)")
            elif self.avoidance_attempt_count >= 2:
                turn_degrees = 60
                print(f"        (Attempt #{self.avoidance_attempt_count} - 60° turn, alternating)")
            else:
                turn_degrees = base_turn

            # Apply direction sign
            if self.avoidance_direction == "right":
                turn_degrees = -turn_degrees

            # Increase backup distance with each failed attempt
            # Base: 0.5s, increases by 0.3s per attempt, max 2.0s
            backup_dur = min((0.5 + (self.avoidance_attempt_count - 1) * 0.3) * backup_mult, 2.0)

            # Quick backup + turn
            self.backup_then_turn(backup_duration=backup_dur, turn_degrees=turn_degrees)

            # Return None to indicate maneuver was executed
            return None, None

        elif path_clear:
            # PATH IS CLEAR - go straight
            linear = self.linear_speed
            angular = 0.0
            self.emergency_maneuver = False

            # If we were avoiding, check if we've traveled enough to return to original heading
            if self.avoidance_direction is not None:
                # Check distance traveled since avoidance started
                if self.last_avoidance_position is not None and self.current_position is not None:
                    dx = self.current_position[0] - self.last_avoidance_position[0]
                    dy = self.current_position[1] - self.last_avoidance_position[1]
                    dist_traveled = math.sqrt(dx*dx + dy*dy)

                    # Only return-to-heading after traveling at least 30cm past obstacle
                    if dist_traveled >= 0.30:
                        # Turn back toward original heading (opposite of avoidance turn)
                        return_degrees = -30 if self.avoidance_direction == "left" else 30
                        print(f"\n[RETURN] Passed obstacle ({dist_traveled:.2f}m), turning back {return_degrees}°")

                        turn_duration = calc_turn_duration(return_degrees, 1.0)
                        self.queue_maneuver("turn", 1.0 if return_degrees > 0 else -1.0, turn_duration)

                        # Clear avoidance state
                        self.avoidance_direction = None
                        self.avoidance_intensity = 1.0
                        self.avoidance_attempt_count = 0
                        self.avoidance_start_time = 0
                        self.last_avoidance_position = None

                        time.sleep(0.1)
                        return None, None
                    else:
                        # Still close to obstacle - keep going straight, don't turn back yet
                        status = f"[FWD] clear, bypassing ({dist_traveled:.2f}m)"
                else:
                    # No position data - just clear state
                    self.avoidance_direction = None
                    self.avoidance_start_time = 0
                    status = f"[FWD] clear={front_arc_min:.2f}m"
            else:
                status = f"[FWD] clear={front_arc_min:.2f}m - going straight"

        elif best_dist >= self.lidar_min_distance and best_sector not in [0, 11, 1]:
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

        elif best_dist >= self.lidar_min_distance:
            # Best sector is front (0, 1, or 11) - go straight!
            linear = self.linear_speed
            angular = 0.0
            self.emergency_maneuver = False

            # Reset ALL avoidance state when going forward
            if self.avoidance_direction is not None:
                print(f"\n[CLEAR] Front is best direction, resetting commitment")
            self.avoidance_direction = None
            self.avoidance_start_time = 0  # Reset so next avoidance is fresh
            self.avoidance_attempt_count = 0

            status = f"[FWD] front ok={front_arc_min:.2f}m s{best_sector}"

        else:
            # All directions have obstacles - quick backup and turn
            if self.avoidance_direction is None:
                self.avoidance_direction = "left" if best_sector <= 5 else "right"
                self.avoidance_start_time = time.time()

            turn_degrees = 30 if self.avoidance_direction == "left" else -30
            print(f"\n[BLOCKED] Backup and turn {turn_degrees}°")
            self.backup_then_turn(backup_duration=0.6, turn_degrees=turn_degrees)
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
                clear = "✓" if dist > self.lidar_min_distance else "✗"
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
        print(f"Linear speed:    {self.linear_speed} m/s (adaptive: {SPEED_SCALE_MIN_FACTOR*100:.0f}%-100%)")
        print(f"Min distance:    {self.min_distance} m (from robot front)")
        print(f"LiDAR threshold: {self.lidar_min_distance:.2f} m (min_dist + {LIDAR_FRONT_OFFSET:.2f}m offset)")
        print(f"Adaptive speed:  Slows from {SPEED_SCALE_FAR_DISTANCE}m to {SPEED_SCALE_NEAR_DISTANCE}m distance")
        print(f"Scan duration:   {'unlimited' if self.duration == 0 else f'{self.duration}s'}")
        print(f"Sectors:         {NUM_SECTORS} ({self.sector_degrees}° each)")
        print(f"LiDAR rotation:  90° (corrected in software)")
        print(f"Stuck detection: Enabled (uses odometry)")
        print(f"Collision verify: Enabled (tracks movement efficiency)")
        print(f"Motion smooth:   Continuous thread @30Hz + EMA(α={self.velocity_smoother.ema_alpha})")
        print(f"Algorithm:       {'VFH + State Machine' if self.use_vfh else 'Sector-based discrete'}")
        print(f"Dead-end detect: Enabled (skip 3+ blocked directions)")
        if self.log_sensors and self.sensor_logger:
            print(f"Sensor logging: Enabled (for offline mapping)")
        print("-"*60)
        print("Controls:")
        print("  SPACE/s  - Emergency stop")
        print("  p        - Pause/Resume")
        print("  r        - Resume after stop")
        print("  q        - Quit")
        print("="*60 + "\n")

        # Start sensor logging if enabled
        sensor_log_dir = None
        if self.log_sensors and self.sensor_logger:
            sensor_log_dir = self.sensor_logger.start()
            print(f"[LOGGER] Recording sensors to: {sensor_log_dir}")

        self.keyboard.start()
        # Initialize smoothing state to target speed for immediate start (no ramp-up)
        self.velocity_smoother.last_linear = self.linear_speed
        self.target_linear = self.linear_speed
        self.start_movement_thread()  # Start continuous movement for smooth motion
        last_status_time = time.time()
        last_rf2o_check = time.time()
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

                # Periodic RF2O health check (every 30s) - prevents EKF from going slow
                if time.time() - last_rf2o_check > 30:
                    check_rf2o_health()
                    ensure_slam_running()  # Also check SLAM is still running
                    last_rf2o_check = time.time()

                # Skip ALL processing if a maneuver is in progress
                # This prevents blocking subprocess calls during maneuvers
                with self.movement_lock:
                    maneuver_active = self.maneuver_mode is not None or len(self.maneuver_queue) > 0

                if maneuver_active:
                    # Let the maneuver complete without interference
                    # Don't call update_scan() or get_odometry() - they block!
                    time.sleep(0.03)  # ~30Hz polling to check when maneuver ends
                    continue

                # Update sensors (only when NOT in a maneuver)
                # OPTIMIZATION: Skip scan update every other iteration when far from obstacles
                # This reduces blocking subprocess calls from ~300ms to ~150ms average
                front_arc_min = min(self.sector_distances[0], self.sector_distances[1],
                                   self.sector_distances[11]) if self.sector_distances else 10.0

                # Check virtual obstacles - these are invisible obstacles the robot has learned about
                # Take minimum of lidar distance and virtual obstacle distance
                if self.current_position and self.current_heading is not None:
                    virtual_clearance = self.get_virtual_obstacle_clearance(
                        self.current_position[0], self.current_position[1], self.current_heading)
                    if virtual_clearance < front_arc_min:
                        print(f"\r[VIRTUAL-OBS] Reducing clearance from {front_arc_min:.2f}m to {virtual_clearance:.2f}m", end="")
                        front_arc_min = virtual_clearance

                iteration_count = getattr(self, '_iteration_count', 0)
                self._iteration_count = iteration_count + 1

                # Cleanup old virtual obstacles periodically (every 100 iterations = ~10 seconds)
                if iteration_count % 100 == 0:
                    self.cleanup_virtual_obstacles()

                skip_scan = False
                if front_arc_min > 1.0 and iteration_count % 2 == 1:
                    # Far from obstacles - can skip every other scan update
                    skip_scan = True

                if not skip_scan:
                    if not self.update_scan():
                        time.sleep(0.1)
                        continue

                # Update current position for visited tracking
                # Also skip odom every other iteration when far from obstacles
                pos = None
                if not skip_scan or iteration_count % 4 == 0:  # Always get odom at least every 4th iteration
                    pos = self.fetch_odometry()
                    if pos is None:
                        # Odometry failed - clear stale position to avoid wrong decisions
                        # Virtual obstacles, escape memory, visited tracking all use position
                        self.current_position = None
                        self.current_heading = None
                else:
                    pos = self.current_position  # Use cached position
                if pos:
                    self.current_position = pos
                    is_new = self.visited_tracker.mark_visited(pos[0], pos[1])
                    if is_new:
                        stats = self.visited_tracker.get_coverage_stats()
                        print(f"\n[EXPLORE] New cell! Total: {stats['cells_visited']} cells, {stats['area_covered']:.1f}m²")

                    # Log sensor data for offline mapping (reuses already-fetched data)
                    if self.log_sensors and self.sensor_logger:
                        self.sensor_logger.log_scan_sync(self.scan_ranges)
                        self.sensor_logger.log_odom_sync(pos[0], pos[1])

                    # Check for dead-end at current position
                    if self.is_dead_end():
                        if not self.is_in_dead_end(pos[0], pos[1]):
                            self.mark_dead_end(pos[0], pos[1])
                            dead_ends_found += 1
                            # Back out of dead-end
                            print(f"\n[DEAD-END] Backing out of dead-end area!")
                            self.backup_then_turn(backup_duration=1.0, turn_degrees=90)
                            continue

                # Compute velocity using selected algorithm
                if self.use_vfh:
                    # VFH with state machine (smoother, no discrete maneuvers)
                    linear, angular = self.compute_velocity_vfh()
                else:
                    # Original discrete maneuver approach
                    linear, angular = self.compute_velocity()

                # If compute_velocity returned None, a discrete maneuver was executed
                # Skip the rest of this iteration - DON'T reset velocity, let maneuver complete
                if linear is None:
                    continue

                # EARLY WARNING: Detect "approaching stuck" - front distance decreasing rapidly
                # This triggers BEFORE robot actually hits obstacle
                current_front = self.sector_distances[0] if self.sector_distances else 10.0
                is_driving = linear > 0.01  # Only forward motion can be "stuck"

                if self.prev_front_distance is not None and is_driving:
                    front_delta = self.prev_front_distance - current_front
                    # If closing at >5cm per cycle while driving forward, slow down preemptively
                    if front_delta > 0.05:
                        print(f"\r[WARN] Rapidly approaching obstacle! delta={front_delta:.2f}m", end="")
                        linear *= 0.5  # Cut speed in half
                    # If closing at >8cm per cycle, this is dangerous - nearly stop
                    elif front_delta > 0.08:
                        print(f"\n[DANGER] Very rapid approach! delta={front_delta:.2f}m - emergency slow")
                        linear *= 0.2  # Nearly stop
                self.prev_front_distance = current_front

                # COLLISION VERIFIER: Track movement efficiency to detect invisible obstacles
                # This runs BEFORE check_if_stuck to catch blockages earlier
                lidar_shows_clear = current_front > self.lidar_min_distance
                cv_result = self.collision_verifier.update(
                    current_position=self.current_position,
                    commanded_linear=linear,
                    front_lidar_dist=current_front,
                    lidar_shows_clear=lidar_shows_clear
                )

                # Apply collision verifier's recommended speed adjustment
                if cv_result['recommended_speed'] < 1.0:
                    linear *= cv_result['recommended_speed']
                    if cv_result['state'].name == 'PROBING':
                        # Probing - slow but still moving to verify
                        pass  # Already printed by CollisionVerifier
                    elif cv_result['is_blocked']:
                        # Blocked detected - create virtual obstacle and trigger recovery
                        print(f"\n[CV-BLOCKED] Invisible obstacle detected by CollisionVerifier!")
                        if cv_result['blocked_position']:
                            self.add_virtual_obstacle(cv_result['blocked_position'][0],
                                                     cv_result['blocked_position'][1])
                        # Trigger backup maneuver - longer backup for invisible obstacles
                        turn_degrees = 45 if self.last_avoidance_direction == "left" else -45
                        self.backup_then_turn(backup_duration=0.8, turn_degrees=turn_degrees)
                        self.collision_verifier.reset()
                        continue

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
                    self.stuck_detector.set_cooldown(2)  # Wait 2 iterations before checking again

                    # Add virtual obstacle at stuck position
                    # This helps robot remember invisible obstacles and avoid them in the future
                    if self.stuck_detector.stuck_position:
                        stuck_x, stuck_y = self.stuck_detector.stuck_position
                        # Check if lidar shows clear - if so, this is truly an invisible obstacle
                        if front_arc_min > self.lidar_min_distance + 0.1:
                            print(f"\n[INVISIBLE-OBS] LiDAR shows {front_arc_min:.2f}m clear but robot stuck!")
                            self.add_virtual_obstacle(stuck_x, stuck_y)
                        else:
                            # Visible obstacle - just publish marker for visualization
                            self.mark_virtual_obstacle(stuck_x, stuck_y)

                    # Use calibrated backup + turn maneuver
                    # Pick direction based on committed direction or default
                    if self.avoidance_direction is None:
                        self.avoidance_direction = "left"  # Default
                        self.avoidance_start_time = time.time()

                    turn_degrees = 50 if self.avoidance_direction == "left" else -50
                    self.backup_then_turn(backup_duration=1.0, turn_degrees=turn_degrees)

                    # Reset position tracking after recovery maneuver
                    self.last_position = None
                    continue

                # Set target velocity for the continuous movement thread
                # During emergency maneuvers, bypass smoothing for faster response
                if self.emergency_maneuver:
                    # Reset smoothing state and send direct command
                    self.velocity_smoother.last_linear = linear
                    self.velocity_smoother.last_angular = angular
                    self.send_cmd(linear, angular)
                else:
                    # Set target velocity - movement thread will smooth and send
                    self.set_target_velocity(linear, angular)

                # Print detailed status periodically
                if time.time() - last_status_time > 10:
                    self.print_sector_display()
                    last_status_time = time.time()

                # Control loop rate (~10 Hz) - for decision making
                # Movement thread runs at 30Hz for smooth motion
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n\nInterrupted (Ctrl+C)")
        finally:
            print("\n\nStopping...")
            self.set_target_velocity(0.0, 0.0)
            self.stop_movement_thread()
            self.emergency_stop()
            self.keyboard.stop()
            self.running = False

            # Stop sensor logging
            if self.log_sensors and self.sensor_logger:
                log_stats = self.sensor_logger.stop()

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
            if self.log_sensors and self.sensor_logger and sensor_log_dir:
                print("-"*60)
                print("SENSOR LOGGING:")
                print(f"  Output dir:       {sensor_log_dir}")
                print(f"  Scans recorded:   {log_stats.get('scan_count', 0)}")
                print(f"  Odom recorded:    {log_stats.get('odom_count', 0)}")
            print("="*60)


