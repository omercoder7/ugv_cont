"""
Memory tracking for escape maneuvers and visited locations.
"""

import os
import json
import time
from typing import Dict, List, Set, Tuple, Optional

from .constants import GRID_RESOLUTION, VISITED_FILE
from .utils import pos_to_cell


class EscapeMemory:
    """
    Tracks which escape maneuvers worked in different situations.
    """

    def __init__(self, grid_resolution: float = 0.5):
        self.grid_resolution = grid_resolution
        self.escape_history: Dict[Tuple[int, int], List[Dict]] = {}
        self.recent_failures: List[Dict] = []
        self.current_stuck_position: Optional[Tuple[float, float]] = None
        self.current_stuck_time: float = 0.0
        self.current_attempt = None

    def _pos_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world position to grid cell"""
        return pos_to_cell(x, y, self.grid_resolution)

    def record_stuck(self, x: float, y: float, front_dist: float,
                     left_dist: float, right_dist: float):
        """Record that robot is stuck at this position"""
        self.current_stuck_position = (x, y)
        self.current_stuck_time = time.time()
        self.recent_failures = []

    def record_escape_attempt(self, direction: str, duration: float,
                               backup_dist: float, turn_angle: float):
        """Record an escape attempt"""
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

        if self.recent_failures:
            successful_maneuver = self.recent_failures[-1].copy()
            successful_maneuver["escape_time"] = time.time() - self.current_stuck_time

            if cell not in self.escape_history:
                self.escape_history[cell] = []

            self.escape_history[cell].append(successful_maneuver)
            if len(self.escape_history[cell]) > 5:
                self.escape_history[cell].pop(0)

        self.current_stuck_position = None
        self.recent_failures = []

    def get_suggested_escape(self, x: float, y: float) -> Optional[Dict]:
        """Get a suggested escape maneuver based on history."""
        cell = self._pos_to_cell(x, y)

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

        failed_directions = {f["direction"] for f in self.recent_failures}

        left_successes = [e for e in all_escapes if e["direction"] == "left"]
        right_successes = [e for e in all_escapes if e["direction"] == "right"]

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
    """

    def __init__(self, resolution: float = GRID_RESOLUTION):
        self.resolution = resolution
        self.visited_cells: Set[Tuple[int, int]] = set()
        self.visit_counts: Dict[Tuple[int, int], int] = {}
        self.last_cell: Optional[Tuple[int, int]] = None
        self.total_cells_visited = 0

        self.load()

    def pos_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world position to grid cell coordinates"""
        return pos_to_cell(x, y, self.resolution)

    def cell_to_pos(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        """Convert grid cell to world position (center of cell)"""
        x = (cell[0] + 0.5) * self.resolution
        y = (cell[1] + 0.5) * self.resolution
        return (x, y)

    def mark_visited(self, x: float, y: float) -> bool:
        """Mark a position as visited. Returns True if this is a new cell."""
        cell = self.pos_to_cell(x, y)

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
            "area_covered": unique_cells * (self.resolution ** 2),
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
