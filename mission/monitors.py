"""
Keyboard monitor and corridor detection with hysteresis.
"""

import sys
import threading
import select
import termios
import tty
from typing import Tuple


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
    """

    def __init__(self):
        self.in_corridor = False
        self.corridor_confidence = 0.0
        self.corridor_width_smooth = 1.0

        self.enter_confidence = 0.65
        self.exit_confidence = 0.35

        self.alpha = 0.3

    def update(self, is_corridor_raw: bool, corridor_width: float,
               robot_width: float) -> Tuple[bool, float]:
        """
        Update corridor detection with hysteresis.

        Returns: (is_in_corridor, smoothed_width)
        """
        min_width = robot_width + 0.10

        if corridor_width < min_width:
            raw_confidence = 0.0
        elif corridor_width > min_width * 2.5:
            raw_confidence = 0.0
        else:
            ideal_width = min_width * 1.5
            if corridor_width <= ideal_width:
                raw_confidence = (corridor_width - min_width) / (ideal_width - min_width)
            else:
                raw_confidence = 1.0 - (corridor_width - ideal_width) / (min_width * 1.0)
            raw_confidence = max(0.0, min(1.0, raw_confidence))

        if not is_corridor_raw:
            raw_confidence *= 0.5

        self.corridor_confidence = (
            self.alpha * raw_confidence +
            (1 - self.alpha) * self.corridor_confidence
        )

        self.corridor_width_smooth = (
            self.alpha * corridor_width +
            (1 - self.alpha) * self.corridor_width_smooth
        )

        if not self.in_corridor and self.corridor_confidence > self.enter_confidence:
            self.in_corridor = True
        elif self.in_corridor and self.corridor_confidence < self.exit_confidence:
            self.in_corridor = False

        return self.in_corridor, self.corridor_width_smooth
