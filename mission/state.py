"""
State machine definitions for robot navigation.
"""

import time
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, Tuple


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
    committed_velocity: Tuple[float, float] = field(default_factory=lambda: (0.0, 0.0))

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
