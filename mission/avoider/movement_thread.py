"""
Continuous movement thread for smooth robot motion.
"""

import time
import threading
from typing import List, Tuple, Callable, Optional


class MovementThread:
    """
    Handles continuous velocity commands at 30Hz for smooth motion.

    Features:
    - Queued maneuvers (backup, turn, stop)
    - Velocity smoothing integration
    - Thread-safe target velocity updates
    """

    def __init__(self, send_cmd_callback: Callable[[float, float], None],
                 smooth_velocity_callback: Callable[[float, float], Tuple[float, float]],
                 loop_rate: float = 30.0):
        """
        Args:
            send_cmd_callback: Function to send velocity commands
            smooth_velocity_callback: Function to apply velocity smoothing
            loop_rate: Command rate in Hz
        """
        self.send_cmd = send_cmd_callback
        self.smooth_velocity = smooth_velocity_callback
        self.loop_period = 1.0 / loop_rate

        # Thread state
        self.thread: Optional[threading.Thread] = None
        self.running = False
        self.lock = threading.Lock()

        # Target velocities
        self.target_linear = 0.0
        self.target_angular = 0.0

        # Maneuver state
        self.maneuver_mode: Optional[str] = None  # "backup", "turn", "stop"
        self.maneuver_end_time = 0.0
        self.maneuver_speed = 0.0
        self.maneuver_queue: List[Tuple[str, float, float]] = []  # (mode, speed, duration)

    def start(self):
        """Start the movement thread."""
        if self.thread is not None and self.thread.is_alive():
            return

        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        """Stop the movement thread."""
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None

    def set_target_velocity(self, linear: float, angular: float):
        """Set target velocities (thread-safe)."""
        with self.lock:
            self.target_linear = linear
            self.target_angular = angular

    def queue_maneuver(self, mode: str, speed: float, duration: float):
        """Queue a maneuver to execute."""
        with self.lock:
            self.maneuver_queue.append((mode, speed, duration))

    def clear_maneuvers(self):
        """Clear all queued maneuvers."""
        with self.lock:
            self.maneuver_queue.clear()
            self.maneuver_mode = None

    def is_maneuvering(self) -> bool:
        """Check if a maneuver is in progress."""
        with self.lock:
            return self.maneuver_mode is not None or len(self.maneuver_queue) > 0

    def _loop(self):
        """Main movement loop running at 30Hz."""
        while self.running:
            loop_start = time.time()

            with self.lock:
                target_lin = self.target_linear
                target_ang = self.target_angular
                current_maneuver_mode = self.maneuver_mode
                current_maneuver_speed = self.maneuver_speed

            # Check if current maneuver has ended
            if current_maneuver_mode is not None and time.time() >= self.maneuver_end_time:
                with self.lock:
                    self.maneuver_mode = None
                    current_maneuver_mode = None

            # Start next queued maneuver if available
            if current_maneuver_mode is None:
                with self.lock:
                    if self.maneuver_queue:
                        mode, speed, duration = self.maneuver_queue.pop(0)
                        self.maneuver_mode = mode
                        self.maneuver_speed = speed
                        self.maneuver_end_time = time.time() + duration
                        current_maneuver_mode = mode
                        current_maneuver_speed = speed

            # Execute current maneuver or normal operation
            if current_maneuver_mode is not None:
                if current_maneuver_mode == "backup":
                    self.send_cmd(-abs(current_maneuver_speed), 0.0)
                elif current_maneuver_mode == "turn":
                    self.send_cmd(0.0, current_maneuver_speed)
                elif current_maneuver_mode == "stop":
                    self.send_cmd(0.0, 0.0)
            else:
                # Normal operation - apply smoothing
                smoothed_lin, smoothed_ang = self.smooth_velocity(target_lin, target_ang)
                self.send_cmd(smoothed_lin, smoothed_ang)

            # Maintain loop rate
            elapsed = time.time() - loop_start
            sleep_time = self.loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
