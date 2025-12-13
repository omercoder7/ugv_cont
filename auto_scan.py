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
import heapq
from typing import List, Tuple, Optional, Set, Dict
from enum import Enum, auto
from dataclasses import dataclass
import json
import os

# Optional sensor logging for two-phase mapping
try:
    from mapping.robot.sensor_logger import SensorLogger
    HAS_SENSOR_LOGGER = True
except ImportError:
    HAS_SENSOR_LOGGER = False

# Try to import numpy for VFH smoothing, fall back to pure Python if not available
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    print("Note: numpy not available, using pure Python for histogram smoothing")

CONTAINER_NAME = "ugv_rpi_ros_humble"

# Visited locations grid settings
GRID_RESOLUTION = 0.5  # 50cm grid cells
VISITED_FILE = "/tmp/visited_locations.json"
NUM_SECTORS = 12  # 360° / 12 = 30° per sector

# Calibration: Robot actual turn rate at Z=1.0 command
# Measured with 30Hz movement thread: 5.07s for 90° rotation
ACTUAL_MAX_ANGULAR_VEL = 0.31  # rad/s at Z=1.0

# Linear velocity calibration (at 30Hz movement thread):
# Commanded 0.06 m/s -> Actual 0.14 m/s forward, 0.13 m/s backward
# Ratio: actual/commanded = 2.33 (forward), 2.17 (backward)
# Set to 1.0 to disable calibration - robot was moving too slow
LINEAR_VEL_RATIO = 1.0  # Set to 1.0 to disable velocity correction

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

# LiDAR position offset calibration:
# The LiDAR is mounted ~37cm behind the robot's front edge.
# When the robot front is 41cm from an obstacle, LiDAR reads 78cm.
# We must add this offset to min_distance when comparing with LiDAR readings.
LIDAR_FRONT_OFFSET = 0.37  # meters from robot front to LiDAR center

# Robot physical dimensions from ugv_beast.urdf:
# - Wheel-to-wheel width: 0.14m (left wheel y=+0.062, right wheel y=-0.062)
# - Wheelbase length: 0.155m (front wheel x=+0.083, rear wheel x=-0.072)
# - With pan-tilt and camera: ~0.17m total length
# Add safety margins for collision avoidance
ROBOT_WIDTH = 0.18  # meters (14cm + 4cm safety margin)
ROBOT_LENGTH = 0.22  # meters (17cm + 5cm safety margin)
ROBOT_HALF_WIDTH = ROBOT_WIDTH / 2  # 0.09m - for corridor width checking

# Minimum corridor width the robot can fit through
MIN_CORRIDOR_WIDTH = ROBOT_WIDTH + 0.10  # 28cm - robot width + 10cm clearance

# Safe turning clearance - minimum front distance needed to safely turn in place
# Robot needs enough space in front to turn without hitting obstacles
# This is used by BACKING_UP state to know when to stop backing and start turning
SAFE_TURN_CLEARANCE = 0.70  # 70cm - enough space to turn safely (increased from 55cm)
MAX_BACKUP_TIME = 5.0  # Maximum backup duration - increased to allow longer backups
MIN_BACKUP_DISTANCE = 0.20  # Minimum backup distance in meters (~20cm) before turning

# Brief stop duration between forward and backward movement
# This prevents mechanical stress and gives robot time to settle
DIRECTION_CHANGE_PAUSE = 0.15  # 150ms pause (feels like ~0.5s due to deceleration)

# Wheel encoder stuck detection constants
# If wheels should be moving but encoder delta is below threshold, robot is stuck
WHEEL_ENCODER_STUCK_THRESHOLD = 0.05  # Minimum encoder change expected when moving
WHEEL_ENCODER_STUCK_TIME = 0.2  # Seconds of no encoder change to trigger stuck (lowered for faster response)

# Adaptive speed control constants
# Robot slows down as it approaches obstacles for tighter maneuvering
SPEED_SCALE_FAR_DISTANCE = 1.5    # Distance (m) at which robot runs at full speed (increased for earlier slowdown)
SPEED_SCALE_NEAR_DISTANCE = 0.5   # Distance (m) at which robot runs at minimum speed
SPEED_SCALE_MIN_FACTOR = 0.4      # Minimum speed factor (40% of linear_speed)

# Body threshold for filtering invalid LiDAR readings
BODY_THRESHOLD_FRONT = 0.12  # Front sectors: real obstacles can be close
BODY_THRESHOLD_SIDE = 0.25   # Side/back sectors: may have body readings

# ============================================================================
# Utility Functions (consolidated to eliminate duplication)
# ============================================================================

def sector_to_angle(sector: int, num_sectors: int = NUM_SECTORS) -> float:
    """
    Convert sector number to angle in radians (0 = front, positive = left).

    This is the UNIFIED implementation - use this everywhere instead of
    inline calculations. Handles wrap-around correctly.

    Args:
        sector: Sector index (0 = front, increases counter-clockwise)
        num_sectors: Total number of sectors (default 12)

    Returns:
        Angle in radians, range [-π, π]
    """
    # Each sector is (2π / num_sectors) radians
    # Sector 0 = front (0 rad), sector 6 = back (π rad for 12 sectors)
    sector_width = 2 * math.pi / num_sectors
    if sector <= num_sectors // 2:
        return sector * sector_width  # 0 to π
    else:
        return (sector - num_sectors) * sector_width  # -π to 0


def pos_to_cell(x: float, y: float, resolution: float = GRID_RESOLUTION) -> Tuple[int, int]:
    """
    Convert world position to grid cell coordinates.

    UNIFIED implementation - use this everywhere instead of class methods.

    Args:
        x, y: World position in meters
        resolution: Grid cell size in meters

    Returns:
        (cell_x, cell_y) tuple
    """
    return (int(x / resolution), int(y / resolution))


def calculate_turn_duration(turn_degrees: float, turn_speed: float = 1.0) -> float:
    """
    Calculate how long a turn will take at given speed.

    Args:
        turn_degrees: Absolute turn angle in degrees
        turn_speed: Angular velocity factor (1.0 = max speed)

    Returns:
        Duration in seconds
    """
    actual_vel = ACTUAL_MAX_ANGULAR_VEL * abs(turn_speed)
    if actual_vel < 0.01:
        return 999.0  # Avoid division by zero
    return abs(turn_degrees) * (math.pi / 180) / actual_vel


# ============================================================================
# State Machine for Navigation
# ============================================================================

class RobotState(Enum):
    """Explicit states for navigation behavior"""
    FORWARD = auto()      # Moving forward, path is clear
    CORRIDOR = auto()     # Moving through narrow corridor (baby steps)
    TURNING = auto()      # Turning in place
    BACKING_UP = auto()   # Backing up from obstacle
    AVOIDING = auto()     # Active obstacle avoidance (curved path)
    STUCK_RECOVERY = auto()  # Recovering from stuck condition
    STOPPED = auto()      # Stationary (paused or waiting)


@dataclass
class StateContext:
    """Context data for state transitions with committed action locking"""
    front_clearance: float = 10.0
    best_sector: int = 0
    best_distance: float = 10.0
    stuck_detected: bool = False
    avoidance_direction: Optional[str] = None
    state_start_time: float = 0.0
    maneuver_duration: float = 0.0
    consecutive_avoidances: int = 0  # Track repeated avoidance attempts
    in_corridor: bool = False  # True if robot is in a narrow passage
    corridor_width: float = 10.0  # Estimated corridor width

    # Committed action locking - prevents race conditions
    committed_action: Optional[str] = None  # Current locked action type
    committed_until: float = 0.0  # Timestamp when lock expires
    committed_direction: Optional[str] = None  # Locked direction (left/right)
    committed_velocity: Tuple[float, float] = (0.0, 0.0)  # Locked (linear, angular)

    def lock_action(self, action: str, duration: float,
                    direction: Optional[str] = None,
                    velocity: Optional[Tuple[float, float]] = None):
        """
        Lock an action for a specified duration.

        Args:
            action: Action type (e.g., "backup", "turn", "avoid")
            duration: How long to lock (seconds)
            direction: Optional direction lock ("left" or "right")
            velocity: Optional velocity lock (linear, angular)
        """
        self.committed_action = action
        self.committed_until = time.time() + duration
        self.committed_direction = direction
        if velocity:
            self.committed_velocity = velocity

    def is_action_locked(self) -> bool:
        """Check if there's an active action lock."""
        if self.committed_action is None:
            return False
        if time.time() >= self.committed_until:
            self.clear_lock()
            return False
        return True

    def get_lock_remaining(self) -> float:
        """Get remaining lock time in seconds."""
        if not self.is_action_locked():
            return 0.0
        return max(0.0, self.committed_until - time.time())

    def clear_lock(self):
        """Clear the action lock."""
        self.committed_action = None
        self.committed_until = 0.0
        self.committed_direction = None
        self.committed_velocity = (0.0, 0.0)

    def can_transition(self, new_state: 'RobotState') -> bool:
        """
        Check if transition to new state is allowed.

        Some transitions are allowed even during lock:
        - Emergency transitions (STUCK_RECOVERY)
        - Same state (no-op)

        Returns True if transition is allowed.
        """
        if not self.is_action_locked():
            return True

        # Always allow emergency recovery transitions during lock
        if new_state == RobotState.STUCK_RECOVERY:
            return True

        return False


# ============================================================================
# Collision Verifier FSM - Detects invisible obstacles via movement analysis
# ============================================================================

class CollisionVerifierState(Enum):
    """States for the collision verification FSM"""
    NORMAL = auto()           # Normal operation, no suspicion
    PROBING = auto()          # Suspicious - driving slowly to verify
    BLOCKED_DETECTED = auto() # Confirmed invisible obstacle
    CALIBRATING = auto()      # Collecting calibration data


class CollisionVerifier:
    """
    Detects invisible obstacles by comparing expected vs actual movement.

    Key insight: When driving toward a clear path, odometry should match
    commanded velocity. When blocked by an invisible obstacle (glass, thin
    pole, etc.), LiDAR shows clear but odometry shows reduced/no movement.

    The verifier:
    1. Tracks movement efficiency (actual_distance / expected_distance)
    2. When efficiency drops below threshold, enters PROBING state
    3. In PROBING, drives slowly and monitors carefully
    4. If still blocked, confirms invisible obstacle and creates virtual marker

    PRE-CALIBRATION MODE:
    Run ./auto_scan.py --calibrate to collect baseline data.
    Drive the robot in:
    1. Clear paths (hallways, open rooms) - collects "clear" baseline
    2. Against known obstacles - collects "blocked" baseline
    The calibration saves thresholds to detect invisible obstacles.

    Calibration profiles:
    - clear_path: Robot moves freely, efficiency ~0.9-1.1
    - soft_block: Carpet edge, gentle resistance, efficiency ~0.4-0.6
    - hard_block: Wall, glass, immovable, efficiency ~0.0-0.2
    """

    # Calibration file path (persistent across runs)
    CALIBRATION_FILE = "/home/ws/ugv_cont/config/collision_verifier_calibration.json"

    def __init__(self, calibration_mode: bool = False):
        self.state = CollisionVerifierState.NORMAL
        self.state_start_time = time.time()
        self.calibration_mode = calibration_mode

        # Movement tracking
        self.last_position: Optional[Tuple[float, float]] = None
        self.last_position_time: float = 0
        self.commanded_velocity: float = 0.0

        # Movement efficiency history (rolling window)
        # efficiency = actual_distance / expected_distance
        self.efficiency_history: List[float] = []
        self.efficiency_window_size = 10  # Keep last 10 samples

        # PRE-CALIBRATION DATA STORAGE
        # Organized by scenario type for better thresholds
        self.calibration_data = {
            'clear_path': [],      # Efficiency samples when driving freely
            'soft_block': [],      # Efficiency samples with soft resistance (carpet, etc)
            'hard_block': [],      # Efficiency samples against walls/glass
            'metadata': {
                'robot_id': 'ugv_beast',
                'calibrated_at': None,
                'samples_collected': 0
            }
        }

        # LEARNED THRESHOLDS (loaded from pre-calibration)
        # These are the key values that distinguish clear vs blocked
        self.clear_efficiency_min = 0.70      # Below this = not moving freely
        self.suspicious_efficiency = 0.50     # Below this = possibly blocked
        self.blocked_efficiency = 0.25        # Below this = definitely blocked

        # Probing state parameters
        self.probing_speed_factor = 0.5  # Reduce speed to 50% when probing
        self.probing_duration = 0.8  # How long to probe before deciding (seconds)
        self.probing_start_time = 0
        self.probing_start_position: Optional[Tuple[float, float]] = None

        # Blocked detection output
        self.blocked_position: Optional[Tuple[float, float]] = None
        self.blocked_confidence = 0.0  # 0-1, how confident we are about block

        # Calibration collection mode state
        self.current_calibration_label: Optional[str] = None  # 'clear', 'soft', 'hard'
        self.calibration_samples_this_run: int = 0

        # Load pre-calibration if available
        self._load_calibration()

    def _load_calibration(self):
        """Load pre-calibration data from file if available"""
        try:
            if os.path.exists(self.CALIBRATION_FILE):
                with open(self.CALIBRATION_FILE, 'r') as f:
                    data = json.load(f)

                    # Load computed thresholds
                    thresholds = data.get('thresholds', {})
                    self.clear_efficiency_min = thresholds.get('clear_min', 0.70)
                    self.suspicious_efficiency = thresholds.get('suspicious', 0.50)
                    self.blocked_efficiency = thresholds.get('blocked', 0.25)

                    # Load raw data for potential re-calibration
                    self.calibration_data = data.get('raw_data', self.calibration_data)

                    metadata = data.get('metadata', {})
                    samples = metadata.get('samples_collected', 0)

                    print(f"[CollisionVerifier] Loaded calibration ({samples} samples):")
                    print(f"  Clear path efficiency: >= {self.clear_efficiency_min:.0%}")
                    print(f"  Suspicious (probe):    < {self.suspicious_efficiency:.0%}")
                    print(f"  Blocked (confirmed):   < {self.blocked_efficiency:.0%}")
            else:
                print(f"[CollisionVerifier] No calibration file found - using defaults")
                print(f"  Run './auto_scan.py --calibrate' to create calibration")
        except Exception as e:
            print(f"[CollisionVerifier] Failed to load calibration: {e}")
            print(f"  Using default thresholds")

    def _save_calibration(self):
        """Save calibration data and computed thresholds to file"""
        try:
            # Ensure config directory exists
            config_dir = os.path.dirname(self.CALIBRATION_FILE)
            if not os.path.exists(config_dir):
                os.makedirs(config_dir)

            data = {
                'thresholds': {
                    'clear_min': self.clear_efficiency_min,
                    'suspicious': self.suspicious_efficiency,
                    'blocked': self.blocked_efficiency
                },
                'raw_data': self.calibration_data,
                'metadata': {
                    'robot_id': 'ugv_beast',
                    'calibrated_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'samples_collected': (
                        len(self.calibration_data.get('clear_path', [])) +
                        len(self.calibration_data.get('soft_block', [])) +
                        len(self.calibration_data.get('hard_block', []))
                    )
                }
            }
            with open(self.CALIBRATION_FILE, 'w') as f:
                json.dump(data, f, indent=2)
            print(f"[CollisionVerifier] Calibration saved to {self.CALIBRATION_FILE}")
        except Exception as e:
            print(f"[CollisionVerifier] Failed to save calibration: {e}")

    def update(self, current_position: Optional[Tuple[float, float]],
               commanded_linear: float, front_lidar_dist: float,
               lidar_shows_clear: bool) -> dict:
        """
        Update the collision verifier with current sensor data.

        Args:
            current_position: (x, y) from odometry
            commanded_linear: Commanded forward velocity (m/s)
            front_lidar_dist: Front LiDAR distance reading
            lidar_shows_clear: True if LiDAR thinks path is clear

        Returns:
            dict with:
            - state: Current verifier state
            - is_blocked: True if invisible obstacle detected
            - recommended_speed: Suggested speed (reduced if probing/blocked)
            - blocked_position: (x, y) of detected block, or None
            - efficiency: Current movement efficiency
        """
        current_time = time.time()

        # Result dict
        result = {
            'state': self.state,
            'is_blocked': False,
            'recommended_speed': 1.0,  # Multiplier for commanded speed
            'blocked_position': None,
            'efficiency': 1.0
        }

        # Skip if not driving forward or no position data
        if commanded_linear <= 0.01 or current_position is None:
            self.last_position = current_position
            self.last_position_time = current_time
            return result

        # Calculate actual movement
        if self.last_position is not None and self.last_position_time > 0:
            dt = current_time - self.last_position_time

            if dt > 0.05:  # At least 50ms between samples
                # Actual distance moved
                dx = current_position[0] - self.last_position[0]
                dy = current_position[1] - self.last_position[1]
                actual_distance = math.hypot(dx, dy)

                # Expected distance based on commanded velocity
                expected_distance = commanded_linear * dt

                # Calculate efficiency
                if expected_distance > 0.005:  # Minimum expected movement
                    efficiency = actual_distance / expected_distance
                    efficiency = min(efficiency, 2.0)  # Cap at 200% (shouldn't happen)

                    result['efficiency'] = efficiency

                    # Add to history
                    self.efficiency_history.append(efficiency)
                    if len(self.efficiency_history) > self.efficiency_window_size:
                        self.efficiency_history.pop(0)

                    # State machine logic
                    result = self._process_state(
                        efficiency, current_position, front_lidar_dist,
                        lidar_shows_clear, current_time, result
                    )

                    # Collect calibration sample if path is clear and moving well
                    if lidar_shows_clear and efficiency > 0.7 and self.state == CollisionVerifierState.NORMAL:
                        self._add_calibration_sample(commanded_linear, efficiency, front_lidar_dist)

        # Update tracking
        self.last_position = current_position
        self.last_position_time = current_time
        self.commanded_velocity = commanded_linear

        return result

    def _process_state(self, efficiency: float, position: Tuple[float, float],
                       front_dist: float, lidar_clear: bool,
                       current_time: float, result: dict) -> dict:
        """Process FSM state transitions based on efficiency"""

        avg_efficiency = sum(self.efficiency_history) / len(self.efficiency_history) if self.efficiency_history else efficiency

        # In calibration mode, just collect data and don't trigger blocking
        if self.calibration_mode and self.current_calibration_label:
            self._collect_calibration_sample(avg_efficiency, front_dist)
            return result

        if self.state == CollisionVerifierState.NORMAL:
            # Check for suspicious movement
            if lidar_clear and avg_efficiency < self.suspicious_efficiency:
                # LiDAR says clear but we're not moving well - suspicious!
                print(f"[CollisionVerifier] SUSPICIOUS: LiDAR clear but efficiency={avg_efficiency:.0%}")
                self.state = CollisionVerifierState.PROBING
                self.state_start_time = current_time
                self.probing_start_time = current_time
                self.probing_start_position = position
                result['state'] = self.state
                result['recommended_speed'] = self.probing_speed_factor  # Slow down to probe

        elif self.state == CollisionVerifierState.PROBING:
            result['recommended_speed'] = self.probing_speed_factor  # Keep slow

            # Check if we've been probing long enough
            probing_time = current_time - self.probing_start_time

            if probing_time >= self.probing_duration:
                # Evaluate probing result
                if avg_efficiency < self.blocked_efficiency:
                    # Confirmed blocked!
                    print(f"[CollisionVerifier] BLOCKED CONFIRMED at ({position[0]:.2f}, {position[1]:.2f})")
                    print(f"  Avg efficiency: {avg_efficiency:.0%} (threshold: {self.blocked_efficiency:.0%})")
                    self.state = CollisionVerifierState.BLOCKED_DETECTED
                    self.blocked_position = position
                    self.blocked_confidence = 1.0 - avg_efficiency
                    result['is_blocked'] = True
                    result['blocked_position'] = position
                    result['recommended_speed'] = 0.0  # Stop!
                elif avg_efficiency < self.suspicious_efficiency:
                    # Still suspicious, keep probing
                    self.probing_start_time = current_time  # Reset probe timer
                else:
                    # False alarm, back to normal
                    print(f"[CollisionVerifier] False alarm, efficiency recovered to {avg_efficiency:.0%}")
                    self.state = CollisionVerifierState.NORMAL
                    result['recommended_speed'] = 1.0

            result['state'] = self.state

        elif self.state == CollisionVerifierState.BLOCKED_DETECTED:
            result['is_blocked'] = True
            result['blocked_position'] = self.blocked_position
            result['recommended_speed'] = 0.0

            # Stay blocked until external reset or we move away
            if self.probing_start_position:
                dist_from_block = math.hypot(
                    position[0] - self.probing_start_position[0],
                    position[1] - self.probing_start_position[1]
                )
                if dist_from_block > 0.15:  # Moved 15cm away from block point
                    print(f"[CollisionVerifier] Moved away from block, returning to NORMAL")
                    self.state = CollisionVerifierState.NORMAL
                    self.blocked_position = None
                    result['is_blocked'] = False
                    result['recommended_speed'] = 1.0

        return result

    def _collect_calibration_sample(self, efficiency: float, front_dist: float):
        """Collect a sample during calibration mode"""
        if self.current_calibration_label and self.current_calibration_label in self.calibration_data:
            self.calibration_data[self.current_calibration_label].append({
                'efficiency': efficiency,
                'front_dist': front_dist,
                'timestamp': time.time()
            })
            self.calibration_samples_this_run += 1

            # Print progress every 10 samples
            if self.calibration_samples_this_run % 10 == 0:
                print(f"[CALIBRATE] {self.current_calibration_label}: {self.calibration_samples_this_run} samples, "
                      f"last eff={efficiency:.0%}")

    def start_calibration(self, label: str):
        """
        Start collecting calibration data for a specific scenario.

        Args:
            label: One of 'clear_path', 'soft_block', 'hard_block'
        """
        if label not in ['clear_path', 'soft_block', 'hard_block']:
            print(f"[CALIBRATE] Invalid label '{label}'. Use: clear_path, soft_block, hard_block")
            return

        self.current_calibration_label = label
        self.calibration_samples_this_run = 0
        self.state = CollisionVerifierState.CALIBRATING
        print(f"\n[CALIBRATE] Started collecting '{label}' samples")
        print(f"  Drive the robot {'freely' if label == 'clear_path' else 'into obstacles'}")
        print(f"  Press 'n' when done to switch scenario or 'c' to compute thresholds")

    def stop_calibration(self):
        """Stop collecting calibration data"""
        if self.current_calibration_label:
            count = len(self.calibration_data.get(self.current_calibration_label, []))
            print(f"[CALIBRATE] Stopped '{self.current_calibration_label}' collection: {count} total samples")
        self.current_calibration_label = None
        self.state = CollisionVerifierState.NORMAL

    def compute_thresholds_from_calibration(self):
        """
        Compute detection thresholds from collected calibration data.

        Uses statistical analysis to find the boundary between clear and blocked.
        """
        clear_samples = self.calibration_data.get('clear_path', [])
        hard_samples = self.calibration_data.get('hard_block', [])
        soft_samples = self.calibration_data.get('soft_block', [])

        if len(clear_samples) < 10:
            print(f"[CALIBRATE] Need at least 10 clear_path samples (have {len(clear_samples)})")
            return False

        # Extract efficiency values
        clear_effs = [s['efficiency'] for s in clear_samples]
        hard_effs = [s['efficiency'] for s in hard_samples] if hard_samples else [0.1]
        soft_effs = [s['efficiency'] for s in soft_samples] if soft_samples else []

        # Statistics for clear path
        clear_mean = sum(clear_effs) / len(clear_effs)
        clear_min = min(clear_effs)
        clear_variance = sum((e - clear_mean) ** 2 for e in clear_effs) / len(clear_effs)
        clear_std = math.sqrt(clear_variance) if clear_variance > 0 else 0.1

        # Statistics for blocked
        if hard_effs:
            hard_mean = sum(hard_effs) / len(hard_effs)
            hard_max = max(hard_effs)
        else:
            hard_mean = 0.1
            hard_max = 0.2

        print(f"\n[CALIBRATE] Analysis Results:")
        print(f"  Clear path: mean={clear_mean:.0%}, min={clear_min:.0%}, std={clear_std:.0%}")
        print(f"  Hard block: mean={hard_mean:.0%}, max={hard_max:.0%}")
        if soft_effs:
            soft_mean = sum(soft_effs) / len(soft_effs)
            print(f"  Soft block: mean={soft_mean:.0%}")

        # Set thresholds
        # Clear minimum: 2 std below clear mean, but not below hard max
        self.clear_efficiency_min = max(hard_max + 0.10, clear_mean - 2 * clear_std)

        # Blocked threshold: midpoint between hard max and clear min
        self.blocked_efficiency = (hard_max + self.clear_efficiency_min) / 2 - 0.05

        # Suspicious threshold: between blocked and clear
        self.suspicious_efficiency = (self.blocked_efficiency + self.clear_efficiency_min) / 2

        # Ensure ordering makes sense
        self.blocked_efficiency = max(0.15, min(self.blocked_efficiency, 0.40))
        self.suspicious_efficiency = max(self.blocked_efficiency + 0.10, min(self.suspicious_efficiency, 0.60))
        self.clear_efficiency_min = max(self.suspicious_efficiency + 0.10, self.clear_efficiency_min)

        print(f"\n[CALIBRATE] Computed Thresholds:")
        print(f"  Clear path:  >= {self.clear_efficiency_min:.0%}")
        print(f"  Suspicious:  < {self.suspicious_efficiency:.0%}  (triggers probing)")
        print(f"  Blocked:     < {self.blocked_efficiency:.0%}  (confirms obstacle)")

        # Save calibration
        self._save_calibration()
        return True

    def reset(self):
        """Reset to NORMAL state (call after recovery maneuver)"""
        self.state = CollisionVerifierState.NORMAL
        self.efficiency_history.clear()
        self.blocked_position = None
        self.probing_start_position = None
        self.last_position = None
        self.last_position_time = 0

    def get_status_string(self) -> str:
        """Get human-readable status for display"""
        avg_eff = sum(self.efficiency_history) / len(self.efficiency_history) if self.efficiency_history else 1.0
        state_name = self.state.name
        return f"CV:{state_name[:3]}|eff:{avg_eff:.0%}"


# ============================================================================
# VFH Histogram Controller
# ============================================================================

class VFHController:
    """
    Vector Field Histogram (VFH) implementation for obstacle avoidance.
    Based on Borenstein & Koren paper with key features:
    - Polar histogram smoothing (reduces noise sensitivity)
    - Hysteresis thresholding (prevents oscillation)
    - Valley detection for finding gaps
    """

    def __init__(self, num_sectors: int = 12, smoothing_window: int = 3):
        self.num_sectors = num_sectors
        self.smoothing_window = smoothing_window

        # Polar Obstacle Density histogram
        self.polar_histogram = [0.0] * num_sectors
        self.smoothed_histogram = [0.0] * num_sectors

        # Binary histogram (blocked/free) with hysteresis
        self.binary_histogram = [False] * num_sectors
        self.prev_binary_histogram = [False] * num_sectors

        # Hysteresis thresholds (tunable)
        # High threshold: above this = definitely blocked
        # Low threshold: below this = definitely free
        # Between: keep previous state (hysteresis)
        self.threshold_high = 0.6
        self.threshold_low = 0.3

        # Previous selected direction for trajectory smoothing
        self.prev_direction = 0

    def update_histogram(self, sector_distances: List[float], min_distance: float):
        """
        Build polar histogram from sector distances.
        Uses distance-based thresholding with smooth transitions.

        Key insight: We want sectors with dist >= min_distance to be marked as CLEAR,
        and sectors with dist < min_distance to be marked as BLOCKED.
        """
        # Buffer zone around min_distance for smooth transition
        buffer_low = min_distance * 0.7   # Below this = definitely blocked
        buffer_high = min_distance * 1.2  # Above this = definitely clear

        for i in range(self.num_sectors):
            dist = sector_distances[i]

            if dist < 0.1:
                # Blind spot or invalid reading - neutral
                self.polar_histogram[i] = 0.5
            elif dist < buffer_low:
                # Definitely blocked (obstacle very close)
                self.polar_histogram[i] = 1.0
            elif dist > buffer_high:
                # Definitely clear (enough space)
                self.polar_histogram[i] = 0.0
            else:
                # Transition zone - linear interpolation
                # Map buffer_low -> 1.0, buffer_high -> 0.0
                ratio = (dist - buffer_low) / (buffer_high - buffer_low)
                self.polar_histogram[i] = 1.0 - ratio

        # Apply smoothing
        self._smooth_histogram()

        # Compute binary histogram with hysteresis
        self._compute_binary_histogram()

    def _smooth_histogram(self):
        """
        Apply triangular-weighted smoothing to reduce noise sensitivity.
        This is a key VFH feature that makes it robust to sensor errors.
        """
        half_window = self.smoothing_window // 2

        for k in range(self.num_sectors):
            weighted_sum = 0.0
            weight_total = 0.0

            for i in range(-half_window, half_window + 1):
                idx = (k + i) % self.num_sectors
                # Triangular weighting: center has highest weight
                weight = half_window + 1 - abs(i)
                weighted_sum += weight * self.polar_histogram[idx]
                weight_total += weight

            self.smoothed_histogram[k] = weighted_sum / weight_total if weight_total > 0 else 0

    def _compute_binary_histogram(self):
        """
        Convert smoothed histogram to binary using hysteresis thresholding.
        This prevents rapid oscillation between blocked/free states.
        """
        for k in range(self.num_sectors):
            val = self.smoothed_histogram[k]

            if val > self.threshold_high:
                # Definitely blocked
                self.binary_histogram[k] = True
            elif val < self.threshold_low:
                # Definitely free
                self.binary_histogram[k] = False
            else:
                # Uncertain - keep previous state (hysteresis)
                self.binary_histogram[k] = self.prev_binary_histogram[k]

        # Save for next iteration
        self.prev_binary_histogram = self.binary_histogram.copy()

    def find_best_valley(self, target_sector: int = 0) -> Tuple[int, float]:
        """
        Find the best valley (gap) in the binary histogram.
        Prefers valleys closest to target direction.

        Returns: (best_sector, confidence)
        """
        # Find all valleys (consecutive free sectors)
        valleys = []
        in_valley = False
        start = 0

        # Handle wrap-around by checking extended sequence
        extended = self.binary_histogram + self.binary_histogram

        for i in range(len(extended)):
            if not extended[i] and not in_valley:
                # Start of valley
                in_valley = True
                start = i
            elif extended[i] and in_valley:
                # End of valley
                in_valley = False
                end = i - 1
                # Only consider valleys that start in first half (avoid counting wrap-around twice)
                if start < self.num_sectors:
                    valley_width = end - start + 1
                    # Cap width to num_sectors (can't have valley wider than full circle)
                    valley_width = min(valley_width, self.num_sectors)
                    # FIX: Correct wrap-around center calculation
                    # For valley from sector 10 to 13 (i.e., 10, 11, 0, 1), center should be 0 not 11
                    valley_center = (start + valley_width // 2) % self.num_sectors
                    valleys.append((start % self.num_sectors, end % self.num_sectors, valley_center, valley_width))

        # Close valley if we ended inside one (all sectors clear at end)
        if in_valley and start < self.num_sectors:
            end = len(extended) - 1
            valley_width = min(end - start + 1, self.num_sectors)
            # FIX: Use same correct center calculation for wrap-around
            valley_center = (start + valley_width // 2) % self.num_sectors
            valleys.append((start % self.num_sectors, end % self.num_sectors, valley_center, valley_width))

        if not valleys:
            # No clear path - find the least blocked sector as fallback
            # This prevents returning a potentially blocked prev_direction
            min_blocked = 1.0
            best_fallback = target_sector
            for i in range(self.num_sectors):
                if self.smoothed_histogram[i] < min_blocked:
                    min_blocked = self.smoothed_histogram[i]
                    best_fallback = i
            return best_fallback, 0.0  # Zero confidence indicates all blocked

        # Score each valley based on:
        # 1. Alignment with target direction
        # 2. Valley width (prefer wider gaps)
        # 3. Alignment with previous direction (smooth trajectory)
        best_sector = target_sector
        best_score = -999

        for start, end, center, width in valleys:
            # Angle difference to target (0 = front)
            target_diff = self._angle_diff(center, target_sector)

            # Angle difference to previous direction
            prev_diff = self._angle_diff(center, self.prev_direction)

            # Combined score
            # - Prefer target direction (weight 3)
            # - Prefer wider valleys (weight 2)
            # - Prefer continuing previous direction (weight 1)
            score = (
                3.0 * (1.0 - abs(target_diff) / 6.0) +  # Max diff is 6 sectors (180°)
                2.0 * min(width / 4.0, 1.0) +           # Normalize width
                1.0 * (1.0 - abs(prev_diff) / 6.0)
            )

            if score > best_score:
                best_score = score
                best_sector = center

        # Update previous direction
        self.prev_direction = best_sector

        # Confidence based on score (normalize to 0-1)
        confidence = min(1.0, max(0.0, best_score / 6.0))

        return best_sector, confidence

    def _angle_diff(self, a: int, b: int) -> int:
        """Compute shortest angular difference between sectors"""
        diff = (a - b) % self.num_sectors
        if diff > self.num_sectors // 2:
            diff -= self.num_sectors
        return diff

    def is_path_clear(self, sector: int) -> bool:
        """Check if a specific sector is clear (not blocked)"""
        return not self.binary_histogram[sector]

    def get_clearance(self, sector: int) -> float:
        """Get inverse obstacle density for a sector (higher = more clear)"""
        return 1.0 - self.smoothed_histogram[sector]

    def calculate_space_freedom(self, sector: int, sector_distances: List[float]) -> float:
        """
        Calculate "space freedom" for a direction - how open/unconstrained the space is.

        Key insight: A direction with 2m to an obstacle in a corner is WORSE than
        a direction with 1.5m to an obstacle in open space, because the corner
        limits future movement options.

        Space Freedom Score considers:
        1. Distance in the target direction
        2. Distances in neighboring sectors (±1, ±2 sectors = ±30°, ±60°)
        3. Whether the path widens or narrows as we go forward

        Returns: 0.0 (dead-end/constrained) to 1.0 (wide open space)
        """
        # Get distances for target sector and neighbors
        target_dist = sector_distances[sector]

        # Skip if target is blocked
        if target_dist < 0.3:
            return 0.0

        # Get neighboring sector distances (±1 and ±2 sectors = ±30° and ±60°)
        left_1 = sector_distances[(sector + 1) % self.num_sectors]
        left_2 = sector_distances[(sector + 2) % self.num_sectors]
        right_1 = sector_distances[(sector - 1) % self.num_sectors]
        right_2 = sector_distances[(sector - 2) % self.num_sectors]

        # Calculate "angular width" of open space
        # Count how many adjacent sectors are also clear (distance > threshold)
        clear_threshold = 0.6  # Consider sector clear if > 60cm

        adjacent_clear = 0
        for offset in [-2, -1, 1, 2]:
            neighbor = (sector + offset) % self.num_sectors
            if sector_distances[neighbor] >= clear_threshold:
                adjacent_clear += 1

        # Score 1: How wide is the opening? (0-4 adjacent sectors clear)
        width_score = adjacent_clear / 4.0

        # Score 2: Minimum distance in the arc (the limiting factor)
        # If going into a corner, one side will be close
        arc_distances = [target_dist, left_1, right_1]
        min_arc_dist = min(arc_distances)
        max_arc_dist = max(arc_distances)

        # Normalize to 0-1 (cap at 3m for normalization)
        min_dist_score = min(min_arc_dist / 3.0, 1.0)

        # Score 3: Is the space widening or narrowing?
        # Compare inner arc (±30°) to outer arc (±60°)
        inner_avg = (left_1 + right_1) / 2.0
        outer_avg = (left_2 + right_2) / 2.0

        # Widening space (outer > inner) is better - indicates not a corner
        if outer_avg >= inner_avg:
            widening_score = 1.0  # Space is widening - good!
        else:
            # Narrowing - might be going into a corner
            ratio = outer_avg / max(inner_avg, 0.1)
            widening_score = max(0.2, ratio)  # 0.2 to 1.0

        # Score 4: Symmetry - a corridor is OK, but a corner is bad
        # Corner = one side much closer than the other
        left_min = min(left_1, left_2)
        right_min = min(right_1, right_2)

        if max(left_min, right_min) > 0.1:
            symmetry = min(left_min, right_min) / max(left_min, right_min)
        else:
            symmetry = 0.0  # Both sides blocked

        # High symmetry = corridor (OK) or open space (great)
        # Low symmetry = corner (bad)
        symmetry_score = symmetry

        # Combined space freedom score
        # Weight: min_dist matters most, then width, then widening, then symmetry
        freedom = (
            0.35 * min_dist_score +    # How far to closest obstacle in arc
            0.30 * width_score +       # How many adjacent sectors are clear
            0.20 * widening_score +    # Is space opening up or closing in
            0.15 * symmetry_score      # Is it symmetric (corridor/open) vs asymmetric (corner)
        )

        return freedom

    def detect_passage(self, sector_distances: List[float], robot_width: float) -> Dict[str, any]:
        """
        UNIFIED passage/corridor/gap detection.

        Detects narrow passages in ALL directions, not just forward.
        This handles:
        - Corridors (walls on both sides, clear ahead)
        - Gaps in walls (incomplete walls with openings)
        - Narrow openings between obstacles

        Args:
            sector_distances: Current distance readings for all sectors
            robot_width: Physical width of the robot

        Returns:
            Dict with:
            - is_passage: True if in any narrow passage
            - passage_width: Estimated width of narrowest point
            - passage_direction: Best sector to navigate through
            - passage_type: "corridor", "gap", "opening", or "open"
            - can_fit: True if robot width fits through passage
        """
        # Get distances for key sectors
        front = sector_distances[0]
        front_left = sector_distances[1]
        front_right = sector_distances[11]
        left = sector_distances[2]
        right = sector_distances[10]
        back_left = sector_distances[4]
        back_right = sector_distances[8]

        # Minimum width robot needs (robot width + safety buffer)
        min_fit_width = robot_width + 0.08  # 8cm total buffer (4cm each side)

        # Calculate passage widths in different directions
        # FORWARD PASSAGE: width between left and right obstacles
        left_clearance = min(left, front_left)
        right_clearance = min(right, front_right)
        forward_width = left_clearance + right_clearance

        # LEFT PASSAGE: width between front-left and back-left
        left_passage_width = min(front_left, front) + min(back_left, left)

        # RIGHT PASSAGE: width between front-right and back-right
        right_passage_width = min(front_right, front) + min(back_right, right)

        # Detect passage type
        max_side_dist = 1.2  # Walls must be within 1.2m to be "constraining"
        min_front_clearance = 0.4  # Need at least 40cm ahead

        # CORRIDOR: Both sides constrained, front clear
        is_corridor = (
            front > min_front_clearance and
            left_clearance < max_side_dist and
            right_clearance < max_side_dist and
            forward_width >= min_fit_width
        )

        # GAP DETECTION: Look for openings in walls
        # A gap is where one side is blocked but there's an opening
        # Check if front is blocked but sides have openings
        front_blocked = front < min_front_clearance

        # Left gap: front blocked, but front-left or left is clear
        has_left_gap = front_blocked and (front_left > front + 0.3 or left > front + 0.3)
        left_gap_width = min(front_left, left) if has_left_gap else 0

        # Right gap: front blocked, but front-right or right is clear
        has_right_gap = front_blocked and (front_right > front + 0.3 or right > front + 0.3)
        right_gap_width = min(front_right, right) if has_right_gap else 0

        # Determine passage type and best direction
        passage_type = "open"
        passage_width = forward_width
        passage_direction = 0
        is_passage = False

        if is_corridor:
            passage_type = "corridor"
            passage_width = forward_width
            is_passage = True
            # Center in corridor
            if abs(left_clearance - right_clearance) < 0.10:
                passage_direction = 0
            elif left_clearance > right_clearance:
                passage_direction = 1
            else:
                passage_direction = 11

        elif has_left_gap and left_gap_width >= min_fit_width:
            passage_type = "gap_left"
            passage_width = left_gap_width
            passage_direction = 1 if front_left > left else 2
            is_passage = True

        elif has_right_gap and right_gap_width >= min_fit_width:
            passage_type = "gap_right"
            passage_width = right_gap_width
            passage_direction = 11 if front_right > right else 10
            is_passage = True

        # Check if robot can fit through the detected passage
        can_fit = passage_width >= min_fit_width

        return {
            "is_passage": is_passage,
            "passage_width": passage_width,
            "passage_direction": passage_direction,
            "passage_type": passage_type,
            "can_fit": can_fit,
            "forward_width": forward_width,
            "left_clearance": left_clearance,
            "right_clearance": right_clearance,
        }

    def detect_corridor(self, sector_distances: List[float], robot_width: float) -> Tuple[bool, float, int]:
        """
        Detect if robot is in a corridor/narrow passage.
        This is a wrapper around detect_passage() for backward compatibility.

        Returns:
            (is_corridor, corridor_width, clear_direction)
        """
        passage = self.detect_passage(sector_distances, robot_width)

        # Consider both corridors and navigable gaps as "corridor mode"
        is_corridor = passage["is_passage"] and passage["can_fit"]
        corridor_width = passage["passage_width"]
        clear_direction = passage["passage_direction"]

        return is_corridor, corridor_width, clear_direction

    def robot_fits_in_direction(self, sector: int, sector_distances: List[float],
                                  robot_half_width: float, min_clearance: float) -> bool:
        """
        Check if robot physically fits when moving toward a sector.

        The robot needs clearance not just in the target direction,
        but also on both sides to avoid scraping walls.

        Args:
            sector: Target direction (sector index)
            sector_distances: Current distance readings
            robot_half_width: Half of robot's physical width
            min_clearance: Minimum required distance ahead

        Returns:
            True if robot can safely move in that direction
        """
        # Get the sector and its neighbors
        left_neighbor = (sector + 1) % self.num_sectors
        right_neighbor = (sector - 1) % self.num_sectors

        # Distance in target direction
        front_dist = sector_distances[sector]

        # Distances to sides (perpendicular to movement)
        # When moving forward (sector 0), sides are sectors 3 (left) and 9 (right)
        # When moving to sector 2 (left), sides are sectors 5 and 11
        side_offset = 3  # 90 degrees = 3 sectors
        left_side = (sector + side_offset) % self.num_sectors
        right_side = (sector - side_offset) % self.num_sectors

        left_dist = sector_distances[left_side]
        right_dist = sector_distances[right_side]

        # Check if there's enough clearance
        front_ok = front_dist >= min_clearance
        sides_ok = left_dist >= robot_half_width and right_dist >= robot_half_width

        return front_ok and sides_ok


# ============================================================================
# DWA-Style Trajectory Scorer
# ============================================================================

class TrajectoryScorer:
    """
    DWA-style trajectory generation and scoring.
    Predicts multiple trajectories and scores them based on:
    - Clearance: Distance from obstacles
    - Heading: Alignment with target direction
    - Velocity: Prefer higher speeds
    - Smoothness: Prefer straight paths

    This is used to select the best velocity command that avoids obstacles
    while making progress toward the goal direction.
    """

    def __init__(self, max_speed: float = 0.10, max_yaw_rate: float = 1.0):
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate

        # Velocity resolution for sampling
        self.v_resolution = 0.02  # 2cm/s steps
        self.yaw_resolution = 0.2  # 0.2 rad/s steps

        # Prediction parameters
        self.predict_time = 1.5  # seconds to predict ahead
        self.dt = 0.1  # timestep for prediction

        # Scoring weights (tunable)
        self.heading_weight = 0.3   # Alignment with goal
        self.clearance_weight = 0.4  # Distance from obstacles
        self.velocity_weight = 0.2   # Prefer higher speeds
        self.smoothness_weight = 0.1  # Prefer continuing current direction

        # Robot dimensions for collision checking
        self.robot_radius = 0.15  # 15cm radius

    def predict_trajectory(self, x: float, y: float, yaw: float,
                          v: float, w: float) -> List[Tuple[float, float, float]]:
        """
        Predict robot trajectory for given velocity command.
        Returns list of (x, y, yaw) poses along trajectory.
        """
        trajectory = [(x, y, yaw)]

        for _ in range(int(self.predict_time / self.dt)):
            # Simple motion model
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            trajectory.append((x, y, yaw))

        return trajectory

    def check_trajectory_collision(self, trajectory: List[Tuple[float, float, float]],
                                   sector_distances: List[float]) -> bool:
        """
        Check if trajectory collides with obstacles.
        Uses sector distances to build obstacle representation.

        Returns True if collision detected.
        """
        # Convert sector distances to obstacle points (polar to cartesian)
        # Uses unified sector_to_angle() function for consistency
        num_sectors = len(sector_distances)
        obstacle_points = []
        for sector, dist in enumerate(sector_distances):
            if 0.1 < dist < 2.0:  # Valid range
                # Use global sector_to_angle() for consistent angle calculation
                angle = sector_to_angle(sector, num_sectors)
                ox = dist * math.cos(angle)
                oy = dist * math.sin(angle)
                obstacle_points.append((ox, oy))

        # Check each trajectory point
        for tx, ty, _ in trajectory:
            for ox, oy in obstacle_points:
                dist = math.sqrt((tx - ox)**2 + (ty - oy)**2)
                if dist < self.robot_radius + 0.10:  # 10cm safety margin
                    return True

        return False

    def score_trajectory(self, trajectory: List[Tuple[float, float, float]],
                        sector_distances: List[float],
                        target_yaw: float = 0.0,
                        current_v: float = 0.0,
                        current_w: float = 0.0) -> float:
        """
        Score trajectory using multiple criteria.
        Higher score = better trajectory.
        """
        if not trajectory or len(trajectory) < 2:
            return float('-inf')

        final_x, final_y, final_yaw = trajectory[-1]

        # 1. Heading score: alignment with target direction
        heading_error = abs(self._normalize_angle(target_yaw - final_yaw))
        heading_score = 1.0 - (heading_error / math.pi)

        # 2. Clearance score: minimum distance to obstacles along trajectory
        min_clearance = float('inf')
        num_sectors = len(sector_distances)
        obstacle_points = []
        for sector, dist in enumerate(sector_distances):
            if 0.1 < dist < 3.0:
                # Use global sector_to_angle() for consistent angle calculation
                angle = sector_to_angle(sector, num_sectors)
                ox = dist * math.cos(angle)
                oy = dist * math.sin(angle)
                obstacle_points.append((ox, oy))

        for tx, ty, _ in trajectory:
            for ox, oy in obstacle_points:
                d = math.sqrt((tx - ox)**2 + (ty - oy)**2)
                min_clearance = min(min_clearance, d)

        clearance_score = min(1.0, min_clearance / 1.0) if min_clearance < float('inf') else 1.0

        # 3. Velocity score: prefer higher forward speeds
        v = math.sqrt((trajectory[-1][0] - trajectory[0][0])**2 +
                     (trajectory[-1][1] - trajectory[0][1])**2) / self.predict_time
        velocity_score = min(1.0, v / self.max_speed)

        # 4. Smoothness score: prefer continuing current motion
        first_yaw = trajectory[0][2]
        yaw_change = abs(self._normalize_angle(final_yaw - first_yaw))
        smoothness_score = 1.0 - min(1.0, yaw_change / math.pi)

        # Combined score
        total = (self.heading_weight * heading_score +
                self.clearance_weight * clearance_score +
                self.velocity_weight * velocity_score +
                self.smoothness_weight * smoothness_score)

        return total

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def compute_best_velocity(self, x: float, y: float, yaw: float,
                             current_v: float, current_w: float,
                             sector_distances: List[float],
                             target_yaw: float = 0.0,
                             max_accel: float = 0.2,
                             max_yaw_accel: float = 0.8) -> Tuple[float, float, float]:
        """
        Main DWA function: find best velocity command.

        Returns: (best_v, best_w, best_score)
        """
        # Calculate dynamic window (reachable velocities)
        min_v = max(0.0, current_v - max_accel * self.dt)
        max_v = min(self.max_speed, current_v + max_accel * self.dt)
        min_w = max(-self.max_yaw_rate, current_w - max_yaw_accel * self.dt)
        max_w = min(self.max_yaw_rate, current_w + max_yaw_accel * self.dt)

        best_v, best_w = 0.0, 0.0
        best_score = float('-inf')

        # Sample velocity space
        v = min_v
        while v <= max_v:
            w = min_w
            while w <= max_w:
                # Generate trajectory
                trajectory = self.predict_trajectory(x, y, yaw, v, w)

                # Check for collision
                if self.check_trajectory_collision(trajectory, sector_distances):
                    w += self.yaw_resolution
                    continue

                # Score trajectory
                score = self.score_trajectory(
                    trajectory, sector_distances, target_yaw, current_v, current_w
                )

                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w

                w += self.yaw_resolution
            v += self.v_resolution

        return best_v, best_w, best_score


# ============================================================================
# Unified Direction Scorer
# ============================================================================

class UnifiedDirectionScorer:
    """
    Unified scoring system that consolidates all direction evaluation criteria.

    Combines:
    - VFH valley detection (obstacle avoidance)
    - Frontier exploration (LiDAR distance + map exploration)
    - Space freedom (avoid corners/constrained areas)
    - Visited tracking (avoid revisiting)
    - Trajectory continuity (smooth movement)

    Use Cases:
    - FORWARD: exploration mode with map scores
    - AVOIDING: safety mode with clearance emphasis
    - CORRIDOR: tight space mode with centering
    """

    # Weight presets for different navigation modes
    PRESETS = {
        "exploration": {
            "clearance": 1.0,      # Basic obstacle safety
            "distance": 1.5,       # Prefer open directions
            "freedom": 1.5,        # Reduced: was 3.0, was avoiding doorways/passages
            "frontier": 4.0,       # Strongly prefer unexplored areas
            "visited": 2.0,        # Penalize revisiting
            "continuity": 0.5,     # Mild smoothing
            "forward_bias": 0.3,   # Slight forward preference
        },
        "safety": {
            "clearance": 4.0,      # Strong obstacle avoidance
            "distance": 2.0,       # Keep distance from obstacles
            "freedom": 2.0,        # Avoid tight spaces
            "frontier": 0.5,       # Exploration secondary
            "visited": 0.5,        # Don't worry about revisiting
            "continuity": 1.0,     # Smooth trajectory
            "forward_bias": 0.2,   # Less forward bias when avoiding
        },
        "corridor": {
            "clearance": 3.0,      # Stay clear of walls
            "distance": 1.0,       # Moderate distance weight
            "freedom": 1.0,        # Accept narrow spaces
            "frontier": 2.0,       # Still explore
            "visited": 1.0,        # Track visits
            "continuity": 2.0,     # Stay centered, smooth
            "forward_bias": 0.5,   # Prefer forward in corridors
        },
    }

    def __init__(self, num_sectors: int = 12):
        self.num_sectors = num_sectors
        self.previous_sector: Optional[int] = None
        self.weights = self.PRESETS["exploration"].copy()

    def set_mode(self, mode: str):
        """Set scoring weights based on navigation mode."""
        if mode in self.PRESETS:
            self.weights = self.PRESETS[mode].copy()

    def set_weights(self, **kwargs):
        """Override individual weights."""
        for key, value in kwargs.items():
            if key in self.weights:
                self.weights[key] = value

    def score_sector(
        self,
        sector: int,
        sector_distances: List[float],
        min_distance: float,
        vfh_controller: 'VFHController',
        map_scores: Optional[List[float]] = None,
        dead_ends: Optional[List[bool]] = None,
        visit_counts: Optional[List[int]] = None,
        current_position: Optional[Tuple[float, float]] = None,
    ) -> Tuple[float, Dict[str, float]]:
        """
        Score a single sector using unified criteria.

        Args:
            sector: Sector index (0 = front)
            sector_distances: LiDAR distances per sector
            min_distance: Minimum safe distance
            vfh_controller: VFH controller for histogram/freedom calculations
            map_scores: Optional map exploration scores per sector
            dead_ends: Optional dead-end flags per sector
            visit_counts: Optional visit count per sector
            current_position: Robot's current (x, y) position

        Returns:
            (total_score, score_breakdown) where breakdown shows each component
        """
        dist = sector_distances[sector]
        breakdown = {}

        # Skip invalid sectors
        # NOTE: Blind spots are marked as exactly 0.0
        # Readings 0.01-0.1m are REAL close obstacles, not blind spots!
        if dist == 0.0:  # Actual blind spot (no reading)
            return -1000.0, {"invalid": -1000.0}
        if dist < 0.1:  # Very close obstacle - extremely dangerous!
            return -800.0, {"critical_obstacle": -800.0}
        if dist < min_distance:  # Blocked
            return -500.0, {"blocked": -500.0}
        if dead_ends and dead_ends[sector]:  # Dead-end
            return -200.0, {"dead_end": -200.0}

        # 1. CLEARANCE SCORE: Is path clear? (binary from VFH)
        clearance = 1.0 if vfh_controller.is_path_clear(sector) else 0.3
        breakdown["clearance"] = clearance

        # 2. DISTANCE SCORE: How far can we see? (normalized 0-1)
        max_dist = 3.0  # Cap scoring at 3m
        distance_score = min(dist / max_dist, 1.0)
        breakdown["distance"] = distance_score

        # 3. FREEDOM SCORE: How unconstrained is this direction?
        freedom = vfh_controller.calculate_space_freedom(sector, sector_distances)
        breakdown["freedom"] = freedom

        # 4. FRONTIER SCORE: How unexplored is this direction?
        frontier = map_scores[sector] if map_scores else 0.5
        breakdown["frontier"] = frontier

        # 5. VISITED SCORE: Penalize revisiting (1.0 = unvisited, 0.0 = heavily visited)
        if visit_counts and visit_counts[sector] is not None:
            vc = visit_counts[sector]
            if vc == 0:
                visited_score = 1.0
            elif vc == 1:
                visited_score = 0.5
            elif vc == 2:
                visited_score = 0.25
            else:
                visited_score = 0.1
        else:
            visited_score = 0.7  # Default when unknown
        breakdown["visited"] = visited_score

        # 6. CONTINUITY SCORE: Preference for continuing previous direction
        if self.previous_sector is not None:
            angle_diff = abs(sector - self.previous_sector)
            if angle_diff > self.num_sectors // 2:
                angle_diff = self.num_sectors - angle_diff
            continuity = 1.0 - (angle_diff / (self.num_sectors // 2))
        else:
            continuity = 0.5  # Neutral for first call
        breakdown["continuity"] = continuity

        # 7. FORWARD BIAS: Slight preference for forward-facing sectors
        if sector in [0]:
            forward_bias = 1.0
        elif sector in [1, 11]:
            forward_bias = 0.8
        elif sector in [2, 10]:
            forward_bias = 0.5
        else:
            forward_bias = 0.2
        breakdown["forward_bias"] = forward_bias

        # 8. POCKET PENALTY: Penalize ONLY truly dangerous pockets (can't turn)
        # Reduced from previous aggressive values since freedom score handles
        # narrow passages already. This now only penalizes physical traps.
        pocket_penalty = 0.0
        if dist < 2.0:  # Only check for closer targets
            # Get side distances relative to this sector
            left_1 = sector_distances[(sector + 1) % self.num_sectors]
            left_2 = sector_distances[(sector + 2) % self.num_sectors]
            right_1 = sector_distances[(sector - 1) % self.num_sectors]
            right_2 = sector_distances[(sector - 2) % self.num_sectors]

            # Calculate estimated width at the target distance
            left_min = min(left_1, left_2)
            right_min = min(right_1, right_2)

            # If both sides are closer than the front, space is narrowing
            if left_min < dist and right_min < dist:
                estimated_width = left_min + right_min  # Rough width estimate
                MIN_TURN_WIDTH = 0.4  # Reduced from 0.5m - robot is 18cm wide

                if estimated_width < MIN_TURN_WIDTH:
                    # Only penalize truly impassable pockets (reduced from -3.0)
                    pocket_penalty = -1.0
                    breakdown["pocket"] = pocket_penalty
                # Removed penalties for 0.7m and 1.0m widths - let freedom score handle
                # Valid corridors (0.5-1.0m) should not be penalized

        # Calculate weighted total
        total = (
            self.weights["clearance"] * clearance +
            self.weights["distance"] * distance_score +
            self.weights["freedom"] * freedom +
            self.weights["frontier"] * frontier +
            self.weights["visited"] * visited_score +
            self.weights["continuity"] * continuity +
            self.weights["forward_bias"] * forward_bias +
            pocket_penalty  # Direct penalty, not weighted
        )

        return total, breakdown

    def find_best_sector(
        self,
        sector_distances: List[float],
        min_distance: float,
        vfh_controller: 'VFHController',
        map_scores: Optional[List[float]] = None,
        dead_ends: Optional[List[bool]] = None,
        visited_tracker: Optional['VisitedTracker'] = None,
        current_position: Optional[Tuple[float, float]] = None,
        current_heading: Optional[float] = None,
    ) -> Tuple[Optional[int], float, Dict[str, float]]:
        """
        Find the best sector to move toward.

        Returns:
            (best_sector, best_score, score_breakdown) or (None, 0, {}) if all blocked
        """
        best_sector = None
        best_score = float('-inf')
        best_breakdown = {}

        # Pre-calculate visit counts per sector if tracker provided
        visit_counts = None
        if visited_tracker and current_position:
            visit_counts = []
            # Use robot heading to convert sector angles to world frame
            heading = current_heading if current_heading is not None else 0.0
            for sector in range(self.num_sectors):
                dist = sector_distances[sector]
                if dist > 0.1:
                    # Use unified sector_to_angle function (in robot frame), convert to world frame
                    robot_angle = sector_to_angle(sector, self.num_sectors)
                    world_angle = heading + robot_angle
                    look_ahead = min(dist, 1.5)
                    target_x = current_position[0] + look_ahead * math.cos(world_angle)
                    target_y = current_position[1] + look_ahead * math.sin(world_angle)
                    visit_counts.append(visited_tracker.get_visit_count(target_x, target_y))
                else:
                    visit_counts.append(None)

        for sector in range(self.num_sectors):
            score, breakdown = self.score_sector(
                sector, sector_distances, min_distance, vfh_controller,
                map_scores, dead_ends, visit_counts, current_position
            )
            if score > best_score:
                best_score = score
                best_sector = sector
                best_breakdown = breakdown

        # Update previous sector for continuity
        if best_sector is not None:
            self.previous_sector = best_sector

        return best_sector, best_score, best_breakdown


# ============================================================================
# A* Path Planner - REMOVED (dead code, was never called)
# ============================================================================
# NOTE: AStarPlanner class was removed as it was never integrated.
# Path planning could be added in future using the SLAM map for
# smarter navigation instead of purely reactive obstacle avoidance.
# ============================================================================


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


class CorridorDetectorWithHysteresis:
    """
    Corridor detector with hysteresis to prevent oscillation.

    Uses exponential smoothing on corridor confidence and
    different thresholds for entering vs exiting corridor mode.
    """

    def __init__(self):
        self.in_corridor = False
        self.corridor_confidence = 0.0
        self.corridor_width_smooth = 1.0  # Smoothed width estimate

        # Hysteresis thresholds
        self.enter_confidence = 0.65  # Need 65% confidence to ENTER corridor mode
        self.exit_confidence = 0.35   # Need <35% confidence to EXIT corridor mode

        # Smoothing factor (0.0-1.0, lower = more smoothing)
        self.alpha = 0.3

    def update(self, is_corridor_raw: bool, corridor_width: float,
               robot_width: float) -> Tuple[bool, float]:
        """
        Update corridor detection with hysteresis.

        Args:
            is_corridor_raw: Raw corridor detection from VFH
            corridor_width: Measured corridor width
            robot_width: Robot's physical width

        Returns:
            (is_in_corridor, smoothed_width)
        """
        min_width = robot_width + 0.10  # Robot width + 10cm clearance

        if corridor_width < min_width:
            raw_confidence = 0.0  # Too narrow
        elif corridor_width > min_width * 2.5:
            raw_confidence = 0.0  # Too wide to be a corridor
        else:
            # Confidence peaks when width is ~1.5x robot width
            ideal_width = min_width * 1.5
            if corridor_width <= ideal_width:
                raw_confidence = (corridor_width - min_width) / (ideal_width - min_width)
            else:
                raw_confidence = 1.0 - (corridor_width - ideal_width) / (min_width * 1.0)
            raw_confidence = max(0.0, min(1.0, raw_confidence))

        # Apply raw detection
        if not is_corridor_raw:
            raw_confidence *= 0.5

        # Exponential smoothing
        self.corridor_confidence = (
            self.alpha * raw_confidence +
            (1 - self.alpha) * self.corridor_confidence
        )

        # Smooth width estimate
        self.corridor_width_smooth = (
            self.alpha * corridor_width +
            (1 - self.alpha) * self.corridor_width_smooth
        )

        # Hysteresis switching
        was_in_corridor = self.in_corridor
        if not self.in_corridor and self.corridor_confidence > self.enter_confidence:
            self.in_corridor = True
        elif self.in_corridor and self.corridor_confidence < self.exit_confidence:
            self.in_corridor = False

        return self.in_corridor, self.corridor_width_smooth


class EscapeMemory:
    """
    Tracks which escape maneuvers worked in different situations.

    When the robot gets stuck and successfully escapes, this class remembers:
    - The approximate location (grid cell)
    - What maneuver worked (direction, duration)
    - What the sensor readings looked like

    When stuck in a similar location/situation, suggests previously successful maneuvers.
    """

    def __init__(self, grid_resolution: float = 0.5):
        self.grid_resolution = grid_resolution
        # Key: (cell_x, cell_y), Value: list of successful escapes
        self.escape_history: Dict[Tuple[int, int], List[Dict]] = {}
        # Recent failed attempts at current location
        self.recent_failures: List[Dict] = []
        self.current_stuck_position: Optional[Tuple[float, float]] = None
        self.current_stuck_time: float = 0.0
        self.current_attempt = None  # Track current escape attempt

    def _pos_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world position to grid cell - uses global pos_to_cell()"""
        return pos_to_cell(x, y, self.grid_resolution)

    def record_stuck(self, x: float, y: float, front_dist: float,
                     left_dist: float, right_dist: float):
        """Record that robot is stuck at this position"""
        self.current_stuck_position = (x, y)
        self.current_stuck_time = time.time()
        self.recent_failures = []

    def record_escape_attempt(self, direction: str, duration: float,
                               backup_dist: float, turn_angle: float):
        """Record an escape attempt (before knowing if it worked)"""
        self.recent_failures.append({
            "direction": direction,
            "duration": duration,
            "backup_dist": backup_dist,
            "turn_angle": turn_angle,
            "timestamp": time.time()
        })

    def record_escape_success(self, x: float, y: float):
        """Robot escaped! Record what worked."""
        if self.current_stuck_position is None:
            return

        cell = self._pos_to_cell(*self.current_stuck_position)

        # The last attempt that was made before escaping
        if self.recent_failures:
            successful_maneuver = self.recent_failures[-1].copy()
            successful_maneuver["escape_time"] = time.time() - self.current_stuck_time

            if cell not in self.escape_history:
                self.escape_history[cell] = []

            # Keep only last 5 successful escapes per cell
            self.escape_history[cell].append(successful_maneuver)
            if len(self.escape_history[cell]) > 5:
                self.escape_history[cell].pop(0)

        # Clear current stuck state
        self.current_stuck_position = None
        self.recent_failures = []

    def get_suggested_escape(self, x: float, y: float) -> Optional[Dict]:
        """
        Get a suggested escape maneuver based on history at this location.

        Returns None if no history, or a dict with:
        - direction: "left" or "right"
        - duration: suggested turn duration
        - backup_dist: suggested backup distance
        - turn_angle: suggested turn angle in degrees
        - confidence: how many times this worked before
        """
        cell = self._pos_to_cell(x, y)

        # Check this cell and adjacent cells
        cells_to_check = [
            cell,
            (cell[0] + 1, cell[1]),
            (cell[0] - 1, cell[1]),
            (cell[0], cell[1] + 1),
            (cell[0], cell[1] - 1),
        ]

        all_escapes = []
        for c in cells_to_check:
            if c in self.escape_history:
                all_escapes.extend(self.escape_history[c])

        if not all_escapes:
            return None

        # Avoid directions that recently failed
        failed_directions = {f["direction"] for f in self.recent_failures}

        # Count successes by direction
        left_successes = [e for e in all_escapes if e["direction"] == "left"]
        right_successes = [e for e in all_escapes if e["direction"] == "right"]

        # Prefer direction that worked more often and hasn't recently failed
        if "left" not in failed_directions and len(left_successes) >= len(right_successes):
            best = left_successes
            direction = "left"
        elif "right" not in failed_directions and len(right_successes) > 0:
            best = right_successes
            direction = "right"
        elif left_successes:
            best = left_successes
            direction = "left"
        elif right_successes:
            best = right_successes
            direction = "right"
        else:
            return None

        # Average the successful maneuver parameters
        avg_duration = sum(e.get("duration", 1.0) for e in best) / len(best)
        avg_backup = sum(e.get("backup_dist", 0.3) for e in best) / len(best)
        avg_angle = sum(e.get("turn_angle", 45) for e in best) / len(best)

        return {
            "direction": direction,
            "duration": avg_duration,
            "backup_dist": avg_backup,
            "turn_angle": avg_angle,
            "confidence": len(best)
        }

    def get_alternative_direction(self) -> str:
        """Get a direction that hasn't been tried recently"""
        failed_directions = {f["direction"] for f in self.recent_failures}

        if "left" not in failed_directions:
            return "left"
        elif "right" not in failed_directions:
            return "right"
        else:
            # Both failed, try opposite of the most recent
            if self.recent_failures:
                last = self.recent_failures[-1]["direction"]
                return "right" if last == "left" else "left"
            return "left"

    def clear_old_failures(self, max_age_seconds: float = 30.0):
        """Clear failures older than max_age"""
        now = time.time()
        self.recent_failures = [
            f for f in self.recent_failures
            if now - f["timestamp"] < max_age_seconds
        ]


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
        """Convert world position to grid cell coordinates - uses global pos_to_cell()"""
        return pos_to_cell(x, y, self.resolution)

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
                 duration: float = 60.0, clear_visited: bool = False, use_vfh: bool = True,
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
        # Increased min distance floor to 0.45m for safer obstacle margins
        # Robot is ~17cm wide but needs extra buffer for reaction time
        self.min_distance = max(min_distance, 0.45)
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

        # Distance tracking for stuck detection
        self.prev_front_distance = None  # Previous front distance reading
        self.front_distance_unchanged_count = 0  # How many times front distance didn't change
        self.stuck_position = None  # Position where we got stuck (for virtual obstacle)

        # Virtual obstacle tracking - stores invisible obstacles the robot has encountered
        # Each entry is (x, y, timestamp, confidence)
        # These are used to adjust clearance calculations when lidar shows clear but robot knows there's something
        self.virtual_obstacles: List[Tuple[float, float, float, int]] = []
        self.virtual_obstacle_radius = 0.12  # Reduced from 25cm to 12cm - was blocking valid paths
        self.virtual_obstacle_max_age = 300.0  # Forget virtual obstacles after 5 minutes
        self.virtual_obstacle_max_count = 20  # Maximum stored virtual obstacles

        # Wheel encoder stuck detection
        self.prev_wheel_encoders = None  # Previous (left, right) encoder values
        self.wheel_encoder_stuck_start = None  # Time when encoder stopped changing

        # Derivative-based stuck detection using sensor history
        # If average readings don't change while driving, we're stuck at same place
        self.sensor_history = []  # List of (time, avg_front_distance, odom_x, odom_y)
        self.sensor_history_max_len = 10  # Keep last 10 samples (~0.33s at 30Hz) - faster detection
        self.derivative_stuck_threshold = 0.03  # If avg change < this, we're stuck
        self.derivative_stuck_ratio = 0.25  # Stuck if actual change < 25% of expected (slightly more sensitive)

        # Smoothing parameters (based on DWA/VFH research)
        # Exponential Moving Average for velocity smoothing
        # At 30Hz, we can use higher alpha for faster response while still being smooth
        self.ema_alpha = 0.7  # Higher = faster response (increased from 0.4 to reduce pauses)
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

        # NOTE: AStarPlanner was removed - dead code never called
        # Path planning integration could be added in future if needed

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
        self.avoidance_profiles = self._load_avoidance_profiles()

    def _load_avoidance_profiles(self) -> dict:
        """
        Load obstacle-specific avoidance profiles from calibration file.

        Profiles contain:
        - shape: 'round', 'flat', or 'thin'
        - extra_margin: Additional distance margin (meters)
        - min_turn_deg: Minimum turn angle for this obstacle
        - backup_mult: Multiplier for backup duration

        Round obstacles (like pillow chairs) need larger turns because
        the robot can slide along the curved surface.
        """
        default_profiles = {
            'default': {'shape': 'flat', 'extra_margin': 0.0, 'min_turn_deg': 45, 'backup_mult': 1.0}
        }

        try:
            calib_file = CollisionVerifier.CALIBRATION_FILE
            if os.path.exists(calib_file):
                with open(calib_file, 'r') as f:
                    data = json.load(f)
                    profiles = data.get('avoidance_profiles', {})
                    if profiles:
                        # Merge with defaults
                        default_profiles.update(profiles)
                        # Count non-default profiles
                        custom_count = len([k for k in profiles if k != 'default'])
                        if custom_count > 0:
                            print(f"[Avoidance] Loaded {custom_count} obstacle-specific avoidance profiles")
        except Exception as e:
            print(f"[Avoidance] Using default profiles: {e}")

        return default_profiles

    def get_avoidance_params(self, front_distance: float) -> Tuple[dict, Optional[str]]:
        """
        Get avoidance parameters based on current obstacle detection.

        Attempts to identify which calibrated obstacle we're facing
        based on LiDAR distance patterns and returns appropriate avoidance params.

        Returns:
            tuple: (params_dict, obstacle_name or None)

        For now, uses a simple distance-based heuristic:
        - Very close (< 0.4m): Check for round obstacles (pillow chair)
        - Close obstacles get larger margins
        """
        # Default params - ALWAYS works even without calibration
        default_params = {
            'shape': 'flat',
            'extra_margin': 0.0,
            'min_turn_deg': 45,
            'backup_mult': 1.0
        }
        params = self.avoidance_profiles.get('default', default_params).copy()
        detected_obstacle = None

        # Check for round obstacle pattern (like pillow_chair)
        if front_distance < 0.6 and self._detect_round_obstacle():
            # Look for any round-shape profile in calibration
            for name, profile in self.avoidance_profiles.items():
                if name == 'default':
                    continue
                if profile.get('shape') == 'round':
                    params = profile.copy()
                    detected_obstacle = name
                    break

            # If no round profile calibrated, use enhanced defaults for round obstacles
            if detected_obstacle is None:
                params = {
                    'shape': 'round',
                    'extra_margin': 0.10,  # Default extra margin for round obstacles
                    'min_turn_deg': 60,    # Larger turn for curved surfaces
                    'backup_mult': 1.3     # More backup for round obstacles
                }
                detected_obstacle = "round_obstacle (uncalibrated)"

        # Check for thin obstacle pattern (like chair legs)
        elif front_distance < 0.5 and self._detect_thin_obstacle():
            # Look for any thin-shape profile in calibration
            for name, profile in self.avoidance_profiles.items():
                if name == 'default':
                    continue
                if profile.get('shape') == 'thin':
                    params = profile.copy()
                    detected_obstacle = name
                    break

            # If no thin profile calibrated, use defaults for thin obstacles
            if detected_obstacle is None:
                params = {
                    'shape': 'thin',
                    'extra_margin': 0.0,
                    'min_turn_deg': 35,    # Smaller turn for thin obstacles
                    'backup_mult': 1.0
                }
                detected_obstacle = "thin_obstacle (uncalibrated)"

        return params, detected_obstacle

    def get_front_arc_min(self, body_threshold: float = BODY_THRESHOLD_FRONT) -> float:
        """
        Get minimum distance in front arc (sectors 11, 0, 1).

        This is the UNIFIED implementation - use this instead of inline calculations.
        Filters out body readings below threshold.

        Args:
            body_threshold: Minimum distance to consider valid (filters robot body readings)

        Returns:
            Minimum distance in front arc, or 0.3 if all invalid
        """
        raw_front = [d for d in [self.sector_distances[11], self.sector_distances[0],
                                 self.sector_distances[1]] if d > body_threshold]
        return min(raw_front) if raw_front else 0.3

    def get_side_clearances(self) -> Tuple[float, float]:
        """
        Get left and right clearance distances.

        This is the UNIFIED implementation - use this instead of inline calculations.

        Returns:
            (left_clearance, right_clearance) tuple in meters
        """
        left = min(self.sector_distances[1], self.sector_distances[2])
        right = min(self.sector_distances[10], self.sector_distances[11])
        return left, right

    def check_trapped_state(self, dynamic_min_dist: float, margin: float = 0.40) -> dict:
        """
        Check if robot is trapped in a dead-end or tight space.

        UNIFIED implementation - consolidates dead-end/trapped detection logic.

        Args:
            dynamic_min_dist: Current minimum distance threshold
            margin: Additional margin for triggering trapped detection

        Returns:
            Dict with keys: front_blocked, left_blocked, right_blocked, rear_clear, is_trapped
        """
        front_arc_min = self.get_front_arc_min()
        left_dist, right_dist = self.get_side_clearances()
        rear_dists = [self.sector_distances[5], self.sector_distances[6], self.sector_distances[7]]

        threshold = dynamic_min_dist + margin

        result = {
            'front_blocked': front_arc_min < threshold,
            'left_blocked': left_dist < threshold,
            'right_blocked': right_dist < threshold,
            'rear_clear': min(rear_dists) > dynamic_min_dist,
            'front_arc_min': front_arc_min,
            'left_dist': left_dist,
            'right_dist': right_dist,
        }
        result['is_trapped'] = (result['front_blocked'] and result['left_blocked'] and
                                result['right_blocked'] and result['rear_clear'])
        return result

    def _detect_thin_obstacle(self) -> bool:
        """
        Detect if current obstacle is thin (like chair leg, pole).

        Thin obstacles show a single sector blocked while adjacent sectors are clear.
        Must distinguish from doorways which have similar pattern but with open sides.
        """
        if len(self.sector_distances) < 12:
            return False

        front = self.sector_distances[0]
        front_left = self.sector_distances[1]
        front_right = self.sector_distances[11]

        # Also check further side sectors to distinguish from doorways
        left_side = self.sector_distances[2]
        right_side = self.sector_distances[10]

        # Thin obstacle: front is blocked but both sides are much clearer
        if front < 0.5:
            # Both adjacent sectors should be significantly further
            left_clear = front_left > front + 0.3
            right_clear = front_right > front + 0.3

            # DOORWAY CHECK: Doorways have BOTH sides very open (no walls nearby)
            # Thin obstacles have open adjacent sectors but walls nearby in side sectors
            # If both side sectors are very open, it's likely a doorway, not thin obstacle
            both_sides_very_open = left_side > 0.8 and right_side > 0.8
            if both_sides_very_open and left_clear and right_clear:
                return False  # Doorway - both sides wide open, not thin obstacle

            # If sides are blocked (door frame or wall), it's also not thin obstacle
            if left_side < 0.5 and right_side < 0.5:
                return False  # Doorway with door frame, not thin obstacle

            if left_clear and right_clear:
                return True

        return False

    def _detect_round_obstacle(self) -> bool:
        """
        Detect if current obstacle has round/curved shape.

        Round obstacles show gradually changing distances across sectors
        (like a curved surface), while flat obstacles show constant distance.

        IMPORTANT: Must distinguish from corners (two walls meeting) which
        show similar pattern but have walls extending to the sides.
        """
        if len(self.sector_distances) < 12:
            return False

        # Check front sectors (11, 0, 1) for round signature
        front = self.sector_distances[0]
        front_left = self.sector_distances[1]
        front_right = self.sector_distances[11]

        # Also check side sectors to distinguish from corners
        left_side = self.sector_distances[2]
        right_side = self.sector_distances[10]

        # Check back-side sectors for wall continuation (corners have walls extending back)
        left_back = self.sector_distances[3]
        right_back = self.sector_distances[9]

        # For a round obstacle, adjacent sectors are further than center
        if front < 0.6:  # Only when close
            # CORNER CHECK: Corners have walls extending to the sides
            # Round objects are isolated - sides should be clear
            if left_side < 0.6 or right_side < 0.6:
                return False  # It's a corner, not a round obstacle

            # NEW: Corners have walls extending backward
            # Round objects are isolated - back-side sectors should be clear
            if left_back < 0.8 or right_back < 0.8:
                return False  # Wall continues backward = corner, not round object

            # Check if we see the "curve" pattern
            # Both sides should be similar distance (symmetric curve)
            side_diff = abs(front_left - front_right)
            both_further = front_left > front and front_right > front

            # Round if: sides are symmetric AND both further than center
            if side_diff < 0.15 and both_further:
                return True

        return False

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

    def compute_adaptive_speed(self, front_distance: float) -> float:
        """
        Calculate adaptive speed based on distance to nearest obstacle.

        Robot slows down as it approaches obstacles, allowing for:
        - Tighter maneuvering in close quarters
        - More reaction time near obstacles
        - Full speed in open areas

        Args:
            front_distance: Distance to nearest obstacle in front arc (meters)

        Returns:
            Scaled linear speed (between min_factor * linear_speed and linear_speed)
        """
        # Subtract LiDAR offset to get actual distance from robot front
        actual_distance = front_distance - LIDAR_FRONT_OFFSET

        # Clamp to valid range
        if actual_distance >= SPEED_SCALE_FAR_DISTANCE:
            # Far from obstacles - full speed
            return self.linear_speed
        elif actual_distance <= SPEED_SCALE_NEAR_DISTANCE:
            # Very close to obstacle - minimum speed
            return self.linear_speed * SPEED_SCALE_MIN_FACTOR
        else:
            # Linear interpolation between min and max speed
            # speed_factor = min_factor + (1 - min_factor) * (dist - near) / (far - near)
            distance_range = SPEED_SCALE_FAR_DISTANCE - SPEED_SCALE_NEAR_DISTANCE
            distance_ratio = (actual_distance - SPEED_SCALE_NEAR_DISTANCE) / distance_range
            speed_factor = SPEED_SCALE_MIN_FACTOR + (1.0 - SPEED_SCALE_MIN_FACTOR) * distance_ratio
            return self.linear_speed * speed_factor

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

    def predict_dead_end_ahead(self, look_ahead_distance: float = 1.5) -> bool:
        """
        PREDICTIVE dead-end detection - warns BEFORE robot gets trapped.

        Returns True if continuing forward will likely lead to a dead-end:
        1. Front sector shows a wall within look_ahead_distance
        2. Map shows front is a dead-end
        3. Both side sectors are also constrained (no escape route)
        4. NEW: The space ahead is too small to maneuver (pocket detection)

        This is called in FORWARD state to avoid entering dead-ends.
        """
        front_dist = self.sector_distances[0]
        front_left_dist = self.sector_distances[1]
        front_right_dist = self.sector_distances[11]

        # If far from any obstacle, not a concern
        if front_dist > look_ahead_distance:
            return False

        # Get map-based dead-end info
        dead_ends = self.get_dead_end_sectors()

        # Front is dead-end according to map?
        if dead_ends[0]:
            return True

        # Check if both front-sides show dead-end
        if dead_ends[1] and dead_ends[11]:
            return True

        # NEW: SMALL POCKET DETECTION
        # If front is close AND both diagonal/side sectors show walls close,
        # we're heading into a small pocket - avoid it!
        # This catches small cells that aren't technically "dead-ends" but too small to turn in
        MIN_POCKET_SIZE = 0.6  # Minimum space needed to turn (60cm)
        if front_dist < look_ahead_distance:
            # Check if this is a narrow pocket by looking at diagonal distances
            left_diag = min(self.sector_distances[1], self.sector_distances[2])
            right_diag = min(self.sector_distances[10], self.sector_distances[11])

            # If both diagonals are closer than front + margin, it's a narrowing pocket
            if left_diag < front_dist + 0.2 and right_diag < front_dist + 0.2:
                # Estimate pocket width at the front distance
                # Using simple geometry: width ≈ 2 * front_dist * sin(30°) ≈ front_dist
                estimated_width = min(left_diag, right_diag) * 2
                if estimated_width < MIN_POCKET_SIZE:
                    print(f"\n[POCKET] Detected small pocket ahead (width~{estimated_width:.2f}m < {MIN_POCKET_SIZE:.2f}m)")
                    return True

        # Check if we'll be trapped - front is getting close AND sides constrained
        sides_constrained = (
            front_left_dist < self.lidar_min_distance + 0.15 or
            front_right_dist < self.lidar_min_distance + 0.15
        )
        front_close = front_dist < self.lidar_min_distance + 0.4

        if sides_constrained and front_close:
            # Check if there's an escape route via diagonal/side sectors
            escape_sectors = [2, 3, 9, 10]
            has_escape = any(
                self.sector_distances[s] >= self.lidar_min_distance and not dead_ends[s]
                for s in escape_sectors
            )
            if not has_escape:
                return True  # No escape route → dead-end ahead

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
                self.sensor_history = []
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
                    self.last_linear = self.linear_speed
                    self.last_angular = 0.0
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
        # Convert to robot's expected format: {"T":"13","X":linear,"Z":angular}
        # Scale: linear is in m/s, angular in rad/s
        # The robot expects values roughly in range -1 to 1
        # NOTE: Both linear AND angular are INVERTED due to motor wiring
        #
        # CALIBRATED: Robot moves 2.25x faster than commanded
        # To get desired velocity, we divide by the ratio to reduce the command
        # Example: Want 0.06 m/s, ratio is 2.25, so command 0.06/2.25 = 0.027 m/s equivalent
        calibrated_linear = linear_x / LINEAR_VEL_RATIO
        x_val = max(-1.0, min(1.0, -calibrated_linear / 0.3))  # INVERTED + Scale to -1 to 1
        z_val = max(-1.0, min(1.0, -angular_z / 1.0))  # INVERTED + Scale to -1 to 1

        # DEBUG: Print if sending zero when we shouldn't be
        if abs(linear_x) < 0.01 and self.maneuver_mode is None and not self.keyboard.emergency_stop:
            pass  # print(f"[DEBUG] Sending zero: lin={linear_x:.3f} ang={angular_z:.3f}")

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

    def backup_then_turn(self, backup_duration: float = 0.3, turn_degrees: float = 30):
        """
        Execute quick obstacle avoidance: brief backup then small turn.
        NON-BLOCKING - queues maneuvers and returns immediately.

        Args:
            backup_duration: How long to back up (default 0.3s = ~2cm at 0.06m/s)
            turn_degrees: How many degrees to turn (positive = left, negative = right)
        """
        # Use smaller, quicker turns (30° = ~1s turn time)
        turn_degrees = max(-45, min(45, turn_degrees))  # Cap at 45°

        # Calculate turn duration - use faster angular speed for quicker response
        turn_speed = 1.0
        actual_vel = ACTUAL_MAX_ANGULAR_VEL * turn_speed
        angle_rad = abs(turn_degrees) * (math.pi / 180)
        turn_duration = angle_rad / actual_vel

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
        self.sensor_history.clear()

        self.obstacles_avoided += 1
        self.total_rotations += 1

    def stop(self):
        """Stop the robot"""
        # Reset smoothing state for clean stop
        self.last_linear = 0.0
        self.last_angular = 0.0
        for _ in range(3):
            self.send_cmd(0.0, 0.0)
            time.sleep(0.05)

    def clear_tracking_data(self):
        """
        Clear all tracking data/caches to prevent stale data from affecting
        subsequent runs. This fixes the RViz erratic display bug where the
        robot appears to move crazily even when stationary.
        """
        # Clear sensor history (used for stuck detection)
        self.sensor_history.clear()

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

    def get_odometry(self) -> Optional[Tuple[float, float]]:
        """Get current position and heading from odometry. Also updates self.current_heading."""
        try:
            result = subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
                 '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
import math
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
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    # Extract yaw from quaternion
    q = msg.pose.pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    print(f'{x:.4f},{y:.4f},{yaw:.4f}')
node.destroy_node()
rclpy.shutdown()
"'''],
                capture_output=True, text=True, timeout=3
            )
            if result.stdout.strip():
                parts = result.stdout.strip().split(',')
                if len(parts) >= 3:
                    self.current_heading = float(parts[2])  # Update heading
                    return (float(parts[0]), float(parts[1]))
                elif len(parts) == 2:
                    # Fallback for old format
                    return (float(parts[0]), float(parts[1]))
            return None
        except:
            return None

    def get_wheel_encoders(self) -> Optional[Tuple[float, float]]:
        """Get raw wheel encoder values from /odom/odom_raw topic"""
        try:
            result = subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
                 '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from std_msgs.msg import Float32MultiArray
rclpy.init()
node = rclpy.create_node('encoder_reader')
msg = None
def cb(m): global msg; msg = m
sub = node.create_subscription(Float32MultiArray, '/odom/odom_raw', cb, 1)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.05)
    if msg: break
if msg and len(msg.data) >= 2:
    print(f'{msg.data[0]:.4f},{msg.data[1]:.4f}')
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

    def add_virtual_obstacle(self, x: float, y: float):
        """
        Add a virtual obstacle at the given position.
        Called when robot gets stuck but lidar shows clear path.
        """
        now = time.time()

        # Check if there's already a virtual obstacle nearby (avoid duplicates)
        for i, (ox, oy, ts, conf) in enumerate(self.virtual_obstacles):
            dist = math.sqrt((x - ox)**2 + (y - oy)**2)
            if dist < 0.3:  # Within 30cm - update existing
                # Increase confidence and update timestamp
                self.virtual_obstacles[i] = (ox, oy, now, min(conf + 1, 5))
                print(f"\n[VIRTUAL-OBS] Reinforced obstacle at ({ox:.2f}, {oy:.2f}), conf={conf+1}")
                return

        # Add new virtual obstacle
        self.virtual_obstacles.append((x, y, now, 1))
        print(f"\n[VIRTUAL-OBS] Added new obstacle at ({x:.2f}, {y:.2f})")

        # Limit total count
        if len(self.virtual_obstacles) > self.virtual_obstacle_max_count:
            # Remove oldest
            self.virtual_obstacles.sort(key=lambda o: o[2])  # Sort by timestamp
            self.virtual_obstacles = self.virtual_obstacles[-self.virtual_obstacle_max_count:]

        # Also publish for visualization
        self.publish_virtual_obstacle(x, y)

    def cleanup_virtual_obstacles(self):
        """Remove old virtual obstacles"""
        now = time.time()
        self.virtual_obstacles = [
            (x, y, ts, conf) for x, y, ts, conf in self.virtual_obstacles
            if now - ts < self.virtual_obstacle_max_age
        ]

    def get_virtual_obstacle_clearance(self, robot_x: float, robot_y: float, robot_heading: float) -> float:
        """
        Get the minimum clearance to any virtual obstacle in front of the robot.
        Returns the distance to the nearest virtual obstacle in the front arc,
        or a large value if no virtual obstacles are in the way.
        """
        if not self.virtual_obstacles or robot_x is None:
            return float('inf')

        min_clearance = float('inf')

        for ox, oy, ts, conf in self.virtual_obstacles:
            # Calculate relative position
            dx = ox - robot_x
            dy = oy - robot_y
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < 0.1:  # Too close to matter
                continue

            # Check if obstacle is in front of robot (within ±60 degrees)
            angle_to_obs = math.atan2(dy, dx)
            angle_diff = angle_to_obs - robot_heading

            # Normalize to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # Check if in front arc (±60 degrees = ±1.05 radians)
            if abs(angle_diff) < 1.05:
                # Virtual obstacle is in front - consider it
                # Subtract obstacle radius to get clearance
                clearance = dist - self.virtual_obstacle_radius
                if clearance < min_clearance:
                    min_clearance = clearance

        return min_clearance

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
        Check if robot is stuck using efficiency-based detection.

        Uses CollisionVerifier's efficiency history when available, with odometry
        fallback. Detects stuck when movement efficiency drops below 35%.

        Returns True if stuck is detected.
        """
        if not is_driving or commanded_linear <= 0.01:
            # Not driving forward, reset tracking
            self.last_position = None
            self.last_position_time = 0
            self.wheel_encoder_stuck_start = None
            self.sensor_history = []  # Clear derivative history
            return False

        if self.stuck_cooldown > 0:
            self.stuck_cooldown -= 1
            return False

        # Use cached position from main loop instead of calling get_odometry() again
        current_pos = self.current_position
        if current_pos is None:
            return False

        current_time = time.time()

        # --- METHOD 1: Use CollisionVerifier's efficiency data if available ---
        # CollisionVerifier already tracks movement efficiency precisely
        if self.collision_verifier and hasattr(self.collision_verifier, 'efficiency_history'):
            cv_efficiency = self.collision_verifier.efficiency_history
            if len(cv_efficiency) >= 5:
                avg_efficiency = sum(cv_efficiency[-5:]) / 5
                # Stuck if less than 35% of expected movement
                if avg_efficiency < 0.35:
                    print(f"\n[STUCK] CollisionVerifier efficiency={avg_efficiency:.0%}")
                    print(f"  Position: ({current_pos[0]:.2f}, {current_pos[1]:.2f})")
                    self.stuck_position = current_pos
                    return True

        # --- METHOD 2: Fallback to odometry-based efficiency check ---
        # Initialize tracking on first call
        if self.last_position is None:
            self.last_position = current_pos
            self.last_position_time = current_time
            self.sensor_history = [(current_time, current_pos[0], current_pos[1])]
            return False

        # Track history of positions
        self.sensor_history.append((current_time, current_pos[0], current_pos[1]))

        # Keep only recent samples
        while len(self.sensor_history) > self.sensor_history_max_len:
            self.sensor_history.pop(0)

        # Need enough samples for efficiency calculation
        if len(self.sensor_history) >= 5:
            oldest = self.sensor_history[0]
            newest = self.sensor_history[-1]
            dt = newest[0] - oldest[0]

            if dt > 0.25:  # Need at least 250ms of data
                odom_delta = math.hypot(newest[1] - oldest[1], newest[2] - oldest[2])
                expected_change = commanded_linear * dt

                if expected_change > 0.015:
                    efficiency = odom_delta / expected_change
                    # Stuck if less than 35% of expected movement
                    if efficiency < 0.35:
                        print(f"\n[STUCK] Efficiency={efficiency:.0%} (moved {odom_delta:.3f}m, expected {expected_change:.3f}m)")
                        print(f"  Position: ({current_pos[0]:.2f}, {current_pos[1]:.2f})")
                        self.stuck_position = current_pos
                        self.sensor_history = []
                        return True

        # Update tracking periodically
        time_elapsed = current_time - self.last_position_time
        if time_elapsed > 0.5:
            self.last_position = current_pos
            self.last_position_time = current_time

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

    # NOTE: find_frontier_direction_legacy() was removed - dead code never called
    # Use find_frontier_direction() via UnifiedDirectionScorer instead

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

    # NOTE: get_path_planned_direction() was removed - dead code never called
    # AStarPlanner integration is not currently used

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
            # CORRIDOR MODE: More cautious - robot needs physical clearance + margin
            # LiDAR offset (37cm) + safety margin (10cm) = ~47cm
            # This means: if LiDAR reads 47cm, robot front is 10cm from obstacle
            dynamic_min_dist = LIDAR_FRONT_OFFSET + 0.10  # ~0.47m - increased from 0.05
            DANGER_DISTANCE = dynamic_min_dist + 0.05 + extra_margin  # Include extra_margin for round obstacles
        else:
            # OPEN SPACE: Conservative threshold for safer margins
            dynamic_min_dist = self.lidar_min_distance
            DANGER_DISTANCE = self.lidar_min_distance + 0.35 + extra_margin  # Include extra_margin

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

            # Minimum backup distance - also scale by backup_mult
            required_min_distance = MIN_BACKUP_DISTANCE * backup_mult
            has_min_distance = backup_distance >= required_min_distance

            # Safety limit: don't backup forever
            exceeded_max_time = time_in_state >= MAX_BACKUP_TIME

            # Minimum backup time before checking clearance (let robot start moving first)
            MIN_BACKUP_TIME = 0.3
            past_minimum = time_in_state >= MIN_BACKUP_TIME

            if past_minimum and has_turn_clearance and has_min_distance:
                # We have enough clearance AND backed up far enough - start turning
                print(f"\n[BACKUP-DONE] Clearance: {front_arc_min:.2f}m >= {required_clearance:.2f}m, dist={backup_distance:.2f}m (took {time_in_state:.1f}s)")
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

                # If we've tried both directions multiple times, extend the turn
                if self.state_context.consecutive_avoidances > 4:
                    print(f"\n[STUCK] Multiple turn attempts failed, will try longer turn")
                    self.state_context.maneuver_duration = min(
                        self.state_context.maneuver_duration * 1.5, 3.0)

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
            FRONT_DANGER_DIST = self.lidar_min_distance + 0.10  # ~0.92m
            front_blocked = front_arc_min < FRONT_DANGER_DIST

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
                left_clear = self.sector_distances[1] >= FRONT_DANGER_DIST
                right_clear = self.sector_distances[11] >= FRONT_DANGER_DIST

                if left_clear and right_clear:
                    # Both sides clear - steer toward frontier if it's to the side
                    if frontier_sector in [1, 2, 3]:
                        linear = self.compute_adaptive_speed(front_arc_min) * 0.5
                        angular = 0.4  # Turn left
                        print(f"\r[STEER-L] Front blocked ({front_arc_min:.2f}m), steering left toward frontier s{frontier_sector}", end="")
                    elif frontier_sector in [9, 10, 11]:
                        linear = self.compute_adaptive_speed(front_arc_min) * 0.5
                        angular = -0.4  # Turn right
                        print(f"\r[STEER-R] Front blocked ({front_arc_min:.2f}m), steering right toward frontier s{frontier_sector}", end="")
                    else:
                        # Frontier is behind - turn in place
                        linear = 0.0
                        angular = 0.5 if frontier_sector <= 6 else -0.5
                        print(f"\r[TURN] Front blocked, turning toward frontier s{frontier_sector}", end="")
                elif left_clear:
                    # Only left is clear - steer left
                    linear = self.compute_adaptive_speed(front_arc_min) * 0.4
                    angular = 0.5
                    print(f"\r[ESCAPE-L] Front blocked ({front_arc_min:.2f}m), only left clear", end="")
                elif right_clear:
                    # Only right is clear - steer right
                    linear = self.compute_adaptive_speed(front_arc_min) * 0.4
                    angular = -0.5
                    print(f"\r[ESCAPE-R] Front blocked ({front_arc_min:.2f}m), only right clear", end="")
                else:
                    # All front directions blocked - transition to AVOIDING or BACKING_UP
                    print(f"\n[TRAPPED] Front arc blocked ({front_arc_min:.2f}m), transitioning to AVOIDING")
                    self.transition_state(RobotState.AVOIDING)
                    return 0.0, 0.0

            elif frontier_sector == 0:
                # Frontier is straight ahead AND front is clear - go forward!
                linear = self.compute_adaptive_speed(front_arc_min)
                angular = 0.0
            elif frontier_sector in [1, 11]:
                # Frontier is slightly to the side - gentle curve
                linear = self.compute_adaptive_speed(front_arc_min)
                angular = 0.15 if frontier_sector == 1 else -0.15
            elif frontier_sector in [2, 10]:
                # Frontier is more to the side - sharper curve
                linear = self.compute_adaptive_speed(front_arc_min) * 0.8
                angular = 0.25 if frontier_sector == 2 else -0.25
            elif frontier_sector in [3, 9]:
                # Frontier is to the side - turn while moving slowly
                linear = self.compute_adaptive_speed(front_arc_min) * 0.5
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
                current_v=self.last_linear,
                current_w=self.last_angular,
                sector_distances=self.sector_distances,
                target_yaw=0.0  # Target is straight ahead
            )

            # ADAPTIVE SPEED: Use slower speed when avoiding obstacles
            adaptive_speed = self.compute_adaptive_speed(front_arc_min)

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
                self.state_context.maneuver_duration = calculate_turn_duration(abs(turn_degrees), 1.0)

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
                elif abs(balance) > BALANCE_STOP_THRESHOLD and abs(self.last_angular) > 0.01:
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

                        turn_duration = abs(return_degrees) * (3.14159 / 180) / ACTUAL_MAX_ANGULAR_VEL
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
            print(f"\n[BLOCKED] Quick backup and turn {turn_degrees}°")
            self.backup_then_turn(backup_duration=0.3, turn_degrees=turn_degrees)
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
        print(f"Motion smooth:   Continuous thread @30Hz + EMA(α={self.ema_alpha})")
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
        self.last_linear = self.linear_speed
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
                    pos = self.get_odometry()
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
                            self.backup_then_turn(backup_duration=0.6, turn_degrees=90)
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
                        # Trigger backup maneuver
                        turn_degrees = 45 if self.last_avoidance_direction == "left" else -45
                        self.backup_then_turn(backup_duration=0.4, turn_degrees=turn_degrees)
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
                    self.stuck_cooldown = 2  # Wait 2 iterations before checking again (reduced)

                    # Add virtual obstacle at stuck position
                    # This helps robot remember invisible obstacles and avoid them in the future
                    if self.stuck_position:
                        stuck_x, stuck_y = self.stuck_position
                        # Check if lidar shows clear - if so, this is truly an invisible obstacle
                        if front_arc_min > self.lidar_min_distance + 0.1:
                            print(f"\n[INVISIBLE-OBS] LiDAR shows {front_arc_min:.2f}m clear but robot stuck!")
                            self.add_virtual_obstacle(stuck_x, stuck_y)
                        else:
                            # Visible obstacle - just publish marker for visualization
                            self.publish_virtual_obstacle(stuck_x, stuck_y)

                    # Use calibrated backup + turn maneuver
                    # Pick direction based on committed direction or default
                    if self.avoidance_direction is None:
                        self.avoidance_direction = "left"  # Default
                        self.avoidance_start_time = time.time()

                    turn_degrees = 50 if self.avoidance_direction == "left" else -50
                    self.backup_then_turn(backup_duration=0.5, turn_degrees=turn_degrees)

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


def ensure_slam_running():
    """Check if SLAM is running and publishing map, restart if needed"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ensure_slam_script = os.path.join(script_dir, 'ensure_slam.sh')

    if os.path.exists(ensure_slam_script):
        print("Checking SLAM status...")
        result = subprocess.run(
            [ensure_slam_script, '--check'],
            capture_output=True, text=True,
            timeout=30
        )
        print(result.stdout.strip())
        return result.returncode == 0
    else:
        # Fallback: basic check
        try:
            result = subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'pgrep', '-f', 'slam_toolbox'],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode != 0:
                print("SLAM not running, starting...")
                subprocess.Popen(
                    ['docker', 'exec', '-d', CONTAINER_NAME, 'bash', '-c',
                     'source /opt/ros/humble/setup.bash && '
                     'source /root/ugv_ws/install/setup.bash && '
                     'ros2 launch slam_toolbox online_async_launch.py > /tmp/slam.log 2>&1'],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                time.sleep(5)
            return True
        except Exception as e:
            print(f"SLAM check failed: {e}")
            return False


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


def check_rf2o_health():
    """
    Check if RF2O is running and publishing. Restart if dead.
    This prevents EKF from falling into prediction-only mode (1Hz).
    """
    # Check if RF2O process is running
    result = subprocess.run(
        ['docker', 'exec', CONTAINER_NAME, 'pgrep', '-f', 'rf2o_laser_odometry_node'],
        capture_output=True, text=True
    )

    if result.returncode != 0:
        print("\n[RF2O] Process dead! Restarting...")
        subprocess.Popen(
            ['docker', 'exec', '-d', CONTAINER_NAME, 'bash', '-c',
             'source /opt/ros/humble/setup.bash && '
             'source /root/ugv_ws/install/setup.bash && '
             'ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py > /tmp/rf2o.log 2>&1'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        time.sleep(3)
        # Verify it restarted
        result = subprocess.run(
            ['docker', 'exec', CONTAINER_NAME, 'pgrep', '-f', 'rf2o_laser_odometry_node'],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            print("[RF2O] Restarted successfully")
            return True
        else:
            print("[RF2O] FAILED TO RESTART - EKF will be slow!")
            return False
    return True


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
    parser.add_argument('--vfh', action='store_true',
                        help='Use VFH algorithm with state machine (smoother navigation)')
    parser.add_argument('--log-sensors', '-l', action='store_true',
                        help='Enable sensor logging for offline mapping')
    parser.add_argument('--log-dir', type=str, default='/tmp/exploration_data',
                        help='Output directory for sensor logs (default: /tmp/exploration_data)')
    args = parser.parse_args()

    if not check_prerequisites():
        sys.exit(1)

    # Start motor driver if needed
    start_driver()

    # Ensure SLAM is running and publishing map
    ensure_slam_running()

    avoider = SectorObstacleAvoider(
        linear_speed=args.speed,
        min_distance=args.min_dist,
        duration=args.duration,
        clear_visited=args.clear_visited,
        use_vfh=args.vfh,
        log_sensors=args.log_sensors,
        log_output_dir=args.log_dir
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
