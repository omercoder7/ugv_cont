"""
Collision verification FSM for detecting invisible obstacles.
"""

import os
import json
import time
import math
from enum import Enum, auto
from typing import List, Tuple, Optional


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
        self.efficiency_history: List[float] = []
        self.efficiency_window_size = 10

        # PRE-CALIBRATION DATA STORAGE
        self.calibration_data = {
            'clear_path': [],
            'soft_block': [],
            'hard_block': [],
            'metadata': {
                'robot_id': 'ugv_beast',
                'calibrated_at': None,
                'samples_collected': 0
            }
        }

        # LEARNED THRESHOLDS (loaded from pre-calibration)
        self.clear_efficiency_min = 0.70
        self.suspicious_efficiency = 0.50
        self.blocked_efficiency = 0.25

        # Probing state parameters
        self.probing_speed_factor = 0.5
        self.probing_duration = 0.8
        self.probing_start_time = 0
        self.probing_start_position: Optional[Tuple[float, float]] = None

        # Blocked detection output
        self.blocked_position: Optional[Tuple[float, float]] = None
        self.blocked_confidence = 0.0

        # Calibration collection mode state
        self.current_calibration_label: Optional[str] = None
        self.calibration_samples_this_run: int = 0

        # Load pre-calibration if available
        self._load_calibration()

    def _load_calibration(self):
        """Load pre-calibration data from file if available"""
        try:
            if os.path.exists(self.CALIBRATION_FILE):
                with open(self.CALIBRATION_FILE, 'r') as f:
                    data = json.load(f)

                    thresholds = data.get('thresholds', {})
                    self.clear_efficiency_min = thresholds.get('clear_min', 0.70)
                    self.suspicious_efficiency = thresholds.get('suspicious', 0.50)
                    self.blocked_efficiency = thresholds.get('blocked', 0.25)

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

        Returns:
            dict with state, is_blocked, recommended_speed, blocked_position, efficiency
        """
        current_time = time.time()

        result = {
            'state': self.state,
            'is_blocked': False,
            'recommended_speed': 1.0,
            'blocked_position': None,
            'efficiency': 1.0
        }

        if commanded_linear <= 0.01 or current_position is None:
            self.last_position = current_position
            self.last_position_time = current_time
            return result

        if self.last_position is not None and self.last_position_time > 0:
            dt = current_time - self.last_position_time

            if dt > 0.05:
                dx = current_position[0] - self.last_position[0]
                dy = current_position[1] - self.last_position[1]
                actual_distance = math.hypot(dx, dy)

                expected_distance = commanded_linear * dt

                if expected_distance > 0.005:
                    efficiency = actual_distance / expected_distance
                    efficiency = min(efficiency, 2.0)

                    result['efficiency'] = efficiency

                    self.efficiency_history.append(efficiency)
                    if len(self.efficiency_history) > self.efficiency_window_size:
                        self.efficiency_history.pop(0)

                    result = self._process_state(
                        efficiency, current_position, front_lidar_dist,
                        lidar_shows_clear, current_time, result
                    )

                    if lidar_shows_clear and efficiency > 0.7 and self.state == CollisionVerifierState.NORMAL:
                        self._add_calibration_sample(commanded_linear, efficiency, front_lidar_dist)

        self.last_position = current_position
        self.last_position_time = current_time
        self.commanded_velocity = commanded_linear

        return result

    def _add_calibration_sample(self, commanded_linear: float, efficiency: float, front_dist: float):
        """Add a calibration sample during normal operation (passive learning)"""
        # Only add if we have good movement data
        if commanded_linear > 0.03 and efficiency > 0.5:
            # This is passive collection - only during clear path operation
            pass  # Could implement passive learning here

    def _process_state(self, efficiency: float, position: Tuple[float, float],
                       front_dist: float, lidar_clear: bool,
                       current_time: float, result: dict) -> dict:
        """Process FSM state transitions based on efficiency"""

        avg_efficiency = sum(self.efficiency_history) / len(self.efficiency_history) if self.efficiency_history else efficiency

        if self.calibration_mode and self.current_calibration_label:
            self._collect_calibration_sample(avg_efficiency, front_dist)
            return result

        if self.state == CollisionVerifierState.NORMAL:
            if lidar_clear and avg_efficiency < self.suspicious_efficiency:
                print(f"[CollisionVerifier] SUSPICIOUS: LiDAR clear but efficiency={avg_efficiency:.0%}")
                self.state = CollisionVerifierState.PROBING
                self.state_start_time = current_time
                self.probing_start_time = current_time
                self.probing_start_position = position
                result['state'] = self.state
                result['recommended_speed'] = self.probing_speed_factor

        elif self.state == CollisionVerifierState.PROBING:
            result['recommended_speed'] = self.probing_speed_factor

            probing_time = current_time - self.probing_start_time

            if probing_time >= self.probing_duration:
                if avg_efficiency < self.blocked_efficiency:
                    print(f"[CollisionVerifier] BLOCKED CONFIRMED at ({position[0]:.2f}, {position[1]:.2f})")
                    print(f"  Avg efficiency: {avg_efficiency:.0%} (threshold: {self.blocked_efficiency:.0%})")
                    self.state = CollisionVerifierState.BLOCKED_DETECTED
                    self.blocked_position = position
                    self.blocked_confidence = 1.0 - avg_efficiency
                    result['is_blocked'] = True
                    result['blocked_position'] = position
                    result['recommended_speed'] = 0.0
                elif avg_efficiency < self.suspicious_efficiency:
                    self.probing_start_time = current_time
                else:
                    print(f"[CollisionVerifier] False alarm, efficiency recovered to {avg_efficiency:.0%}")
                    self.state = CollisionVerifierState.NORMAL
                    result['recommended_speed'] = 1.0

            result['state'] = self.state

        elif self.state == CollisionVerifierState.BLOCKED_DETECTED:
            result['is_blocked'] = True
            result['blocked_position'] = self.blocked_position
            result['recommended_speed'] = 0.0

            if self.probing_start_position:
                dist_from_block = math.hypot(
                    position[0] - self.probing_start_position[0],
                    position[1] - self.probing_start_position[1]
                )
                if dist_from_block > 0.15:
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

            if self.calibration_samples_this_run % 10 == 0:
                print(f"[CALIBRATE] {self.current_calibration_label}: {self.calibration_samples_this_run} samples, "
                      f"last eff={efficiency:.0%}")

    def start_calibration(self, label: str):
        """Start collecting calibration data for a specific scenario."""
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
        """Compute detection thresholds from collected calibration data."""
        clear_samples = self.calibration_data.get('clear_path', [])
        hard_samples = self.calibration_data.get('hard_block', [])
        soft_samples = self.calibration_data.get('soft_block', [])

        if len(clear_samples) < 10:
            print(f"[CALIBRATE] Need at least 10 clear_path samples (have {len(clear_samples)})")
            return False

        clear_effs = [s['efficiency'] for s in clear_samples]
        hard_effs = [s['efficiency'] for s in hard_samples] if hard_samples else [0.1]
        soft_effs = [s['efficiency'] for s in soft_samples] if soft_samples else []

        clear_mean = sum(clear_effs) / len(clear_effs)
        clear_min = min(clear_effs)
        clear_variance = sum((e - clear_mean) ** 2 for e in clear_effs) / len(clear_effs)
        clear_std = math.sqrt(clear_variance) if clear_variance > 0 else 0.1

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

        self.clear_efficiency_min = max(hard_max + 0.10, clear_mean - 2 * clear_std)
        self.blocked_efficiency = (hard_max + self.clear_efficiency_min) / 2 - 0.05
        self.suspicious_efficiency = (self.blocked_efficiency + self.clear_efficiency_min) / 2

        self.blocked_efficiency = max(0.15, min(self.blocked_efficiency, 0.40))
        self.suspicious_efficiency = max(self.blocked_efficiency + 0.10, min(self.suspicious_efficiency, 0.60))
        self.clear_efficiency_min = max(self.suspicious_efficiency + 0.10, self.clear_efficiency_min)

        print(f"\n[CALIBRATE] Computed Thresholds:")
        print(f"  Clear path:  >= {self.clear_efficiency_min:.0%}")
        print(f"  Suspicious:  < {self.suspicious_efficiency:.0%}  (triggers probing)")
        print(f"  Blocked:     < {self.blocked_efficiency:.0%}  (confirms obstacle)")

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
