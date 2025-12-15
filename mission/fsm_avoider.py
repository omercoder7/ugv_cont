"""
Frontier-based obstacle avoidance with exploration efficiency.

Based on explore_lite and WFD (Wavefront Frontier Detector) algorithms.
Navigates toward maximum unexplored area using frontier cost function.

Features:
- Frontier-based exploration (like explore_lite)
- Direction commitment to prevent oscillation
- Visited cell tracking to avoid revisiting
- Stop-rotate-go pattern (no arc driving)
"""

import math
import time
from enum import Enum, auto
from typing import Optional, List, Tuple

from .pro_avoidance import ProObstacleAvoider
from .ros_interface import get_lidar_scan, send_velocity_cmd, get_odometry
from .avoider.lidar import compute_sector_distances
from .constants import (
    ROBOT_WIDTH, NUM_SECTORS,
    SECTORS_FRONT_ARC, SECTORS_LEFT, SECTORS_RIGHT,
    SECTORS_BACK, SECTORS_BACK_LEFT, SECTORS_BACK_RIGHT
)
from .exploration import ExplorationTracker
from .frontier import FrontierExplorer


class State(Enum):
    """Robot navigation states."""
    STOPPED = auto()
    FORWARD = auto()
    AVOIDING = auto()
    BACKING_UP = auto()


class FSMAvoider:
    """
    Frontier-based obstacle avoidance with exploration efficiency.

    Uses:
    - FrontierExplorer: frontier detection and exploration (like explore_lite)
    - ExplorationTracker: visited cells tracking
    - ProObstacleAvoider: TTC and speed limiting
    """

    def __init__(self, linear_speed: float = 0.08, min_distance: float = 0.35,
                 duration: float = 60.0):
        self.linear_speed = min(linear_speed, 0.12)
        self.min_distance = max(min_distance, 0.25)
        self.duration = duration
        self.running = False

        # Pro avoidance for TTC and environment detection
        self.pro = ProObstacleAvoider(
            num_sectors=NUM_SECTORS,
            robot_width=ROBOT_WIDTH,
            max_speed=self.linear_speed,
            max_yaw_rate=0.6
        )

        # Thresholds - tight margins (LiDAR distance)
        self.danger_threshold = 0.40  # ~3cm from robot front
        self.backup_threshold = 0.30  # Very close

        # Direction commitment to prevent oscillation
        self.committed_direction: Optional[str] = None
        self.commit_time: float = 0
        self.commit_duration: float = 1.5

        # Exploration modules
        self.explorer = ExplorationTracker(grid_resolution=0.5)
        self.frontier = FrontierExplorer(
            num_sectors=NUM_SECTORS,
            min_frontier_distance=0.5,
            potential_scale=1.0,  # Distance penalty
            gain_scale=2.0,  # Prefer larger frontiers with more info gain
            orientation_scale=0.3  # Small penalty for turning
        )

        # State tracking
        self.state = State.STOPPED
        self.current_pos: Optional[Tuple[float, float]] = None
        self.current_heading: Optional[float] = None
        self.last_debug_info: dict = {}  # Telemetry from ProObstacleAvoider
        self.last_sectors: List[float] = []  # Last sector distances for telemetry

        # Statistics
        self.start_time = 0.0
        self.iteration = 0
        self.obstacles_avoided = 0

    def run(self):
        """Main sense-compute-act loop."""
        print("=" * 55)
        print("FRONTIER-BASED EXPLORATION")
        print("(like explore_lite)")
        print("=" * 55)
        print(f"Speed: {self.linear_speed} m/s")
        print(f"Danger threshold: {self.danger_threshold:.2f}m (LiDAR)")
        print(f"Duration: {self.duration}s (0=unlimited)")
        print("Press Ctrl+C to stop\n")

        self.running = True
        self.start_time = time.time()
        self.state = State.FORWARD

        send_velocity_cmd(0.0, 0.0)
        time.sleep(0.2)

        try:
            while self.running:
                self.iteration += 1

                # Check duration
                if self.duration > 0:
                    if time.time() - self.start_time >= self.duration:
                        print(f"\n\nDuration {self.duration}s reached.")
                        break

                # 1. SENSE
                scan = get_lidar_scan()
                if not scan:
                    print("Waiting for LiDAR...")
                    time.sleep(0.5)
                    continue

                sectors = compute_sector_distances(scan)
                assert len(sectors) == NUM_SECTORS, f"Expected {NUM_SECTORS} sectors, got {len(sectors)}"
                self.last_sectors = sectors  # Store for telemetry

                # Check front arc for obstacles (uses SECTORS_FRONT_ARC from constants)
                front_arc = [sectors[s] for s in SECTORS_FRONT_ARC]
                valid_front = [d for d in front_arc if d > 0.05]
                if valid_front:
                    front_min = min(valid_front)
                else:
                    # All front sectors are blind spots - assume clear but warn
                    front_min = 1.0  # Assume clear, don't trigger backup on blind spots
                    if self.iteration % 10 == 0:
                        print(f"\n[WARN] All front sectors are blind spots")

                # Get position and heading
                odom = get_odometry()
                if odom:
                    self.current_pos = (odom[0], odom[1])
                    self.current_heading = odom[2]

                    # Record visit for exploration tracking
                    self.explorer.record_visit(odom[0], odom[1])

                # 2. COMPUTE - get base velocities
                safe_v, safe_w, self.last_debug_info = self.pro.compute_safe_velocity(
                    sector_distances=sectors,
                    min_distance=self.danger_threshold,
                    base_linear=self.linear_speed,
                    base_angular=0.0,
                    target_sector=0,
                    is_stuck=False
                )

                # 3. DECIDE
                if front_min < self.backup_threshold:
                    self.state = State.BACKING_UP
                    print(f"\n[BACKUP] Too close! front={front_min:.2f}m")
                    self._execute_backup(sectors)
                    self.obstacles_avoided += 1
                    self.explorer.reset_stuck()
                    self.state = State.FORWARD
                    continue

                elif front_min < self.danger_threshold:
                    self.state = State.AVOIDING

                    if self.committed_direction is None:
                        # Check side sectors to determine obstacle location
                        # SECTORS_LEFT: +30° to +90°, SECTORS_RIGHT: -90° to -30°
                        left_side_min = min(sectors[s] for s in SECTORS_LEFT)
                        right_side_min = min(sectors[s] for s in SECTORS_RIGHT)

                        # Steer AWAY from the side with obstacle (smaller distance)
                        if left_side_min < right_side_min:
                            # Obstacle on LEFT → steer RIGHT (negative angular)
                            self.committed_direction = "right"
                        else:
                            # Obstacle on RIGHT → steer LEFT (positive angular)
                            self.committed_direction = "left"

                        self.commit_time = time.time()
                        print(f"\n[AVOID] Obstacle at {front_min:.2f}m, L={left_side_min:.2f} R={right_side_min:.2f}, steering {self.committed_direction}")
                        self.obstacles_avoided += 1

                    # AVOIDING: Stop and rotate in place (no arc driving)
                    # Use committed direction to prevent oscillation
                    safe_v = 0.0  # Stop forward motion

                    if self.committed_direction == "left":
                        safe_w = 0.4   # Rotate left (positive angular)
                    elif self.committed_direction == "right":
                        safe_w = -0.4  # Rotate right (negative angular)
                    else:
                        # No direction committed - check sides and commit now
                        left_min = min(sectors[s] for s in SECTORS_LEFT)
                        right_min = min(sectors[s] for s in SECTORS_RIGHT)
                        # Steer away from obstacle side
                        if left_min < right_min:
                            self.committed_direction = "right"
                            safe_w = -0.4
                        else:
                            self.committed_direction = "left"
                            safe_w = 0.4
                        print(f"\n[AVOID] Late commit: L={left_min:.2f} R={right_min:.2f}, steering {self.committed_direction}")

                else:
                    self.state = State.FORWARD

                    # FORWARD STATE: Frontier is sole steering authority
                    # ProObstacleAvoider only provides speed limiting (TTC), not angular velocity
                    safe_w = 0.0  # Reset - don't use DWA/repulsive angular

                    # Clear avoidance commitment after timeout
                    if self.committed_direction:
                        if time.time() - self.commit_time > self.commit_duration:
                            print(f"\n[CLEAR] Path clear")
                            self.committed_direction = None

                    # Frontier-based exploration - navigate toward maximum unexplored area
                    # Frontier is the SOLE steering authority in FORWARD state
                    if self.current_pos and self.current_heading is not None:
                        best_frontier, no_frontiers = self.frontier.select_frontier(
                            sectors=sectors,
                            visited_cells=self.explorer.visited,
                            robot_x=self.current_pos[0],
                            robot_y=self.current_pos[1],
                            robot_heading=self.current_heading,
                            grid_res=self.explorer.grid_res
                        )

                        if no_frontiers:
                            # No good frontiers - turn around
                            print(f"\n[NO FRONTIERS] Turning around")
                            self._execute_backup(sectors)
                            self.frontier.reset()
                            continue

                        if best_frontier is not None:
                            # Stop-rotate-go: no arc driving toward frontiers
                            # If frontier angle is significant, stop and rotate first
                            angle_threshold = 0.25  # ~15 degrees

                            if abs(best_frontier.angle) > angle_threshold:
                                # Need to rotate - stop forward motion
                                safe_v = 0.0
                                safe_w = 0.4 if best_frontier.angle > 0 else -0.4
                                print(f"\n[ROTATE] Turning {'left' if best_frontier.angle > 0 else 'right'} toward frontier s{best_frontier.sector} ({math.degrees(best_frontier.angle):.0f}°)")
                            else:
                                # Aligned with frontier - go straight
                                safe_w = 0.0

                # 4. ACT
                send_velocity_cmd(safe_v, safe_w)

                # Status display with telemetry
                stats = self.explorer.get_stats()
                frontier_stats = self.frontier.get_stats()
                state_abbr = self.state.name[:3]
                odom_str = ""
                if self.current_pos and self.current_heading is not None:
                    odom_str = f" pos=({self.current_pos[0]:.2f},{self.current_pos[1]:.2f}) yaw={math.degrees(self.current_heading):.0f}°"

                # Print detailed telemetry every 10 iterations
                if self.iteration % 10 == 0:
                    telemetry = self._format_telemetry()
                    if telemetry:
                        print(f"\n--- Telemetry [{self.iteration}] ---")
                        print(telemetry)
                        print("-" * 40)

                # Compact status line
                status = (f"[{self.iteration:3d}] {state_abbr} front={front_min:.2f}m{odom_str} "
                         f"cells={stats['unique_cells']} fronts={frontier_stats['num_frontiers']}")
                print(f"{status}", flush=True)

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n\nStopping (Ctrl+C)...")
        finally:
            self.emergency_stop()
            self._print_stats()

    def _execute_backup(self, sectors: List[float]):
        """
        Execute backup maneuver with smart direction selection.

        Based on Nav2 recovery behaviors:
        1. Backup first
        2. Turn toward most open direction (back sectors only)
        """
        # Step 1: Backup
        print(f"\n[BACKUP] Reversing...")
        for _ in range(10):
            send_velocity_cmd(-0.05, 0.0)
            time.sleep(0.1)

        # Step 2: Determine best escape direction
        turn_dir = self._calculate_escape_direction(sectors)

        # Step 3: Turn toward escape
        turn_amount = 12 if abs(turn_dir) > 0.4 else 8  # More turn for bigger angle
        print(f"[BACKUP] Turning {'left' if turn_dir > 0 else 'right'}")
        for _ in range(turn_amount):
            send_velocity_cmd(0.0, turn_dir)
            time.sleep(0.1)

        # Clear committed direction
        self.committed_direction = None

    def _calculate_escape_direction(self, sectors: List[float]) -> float:
        """
        Calculate best direction to escape after backup.

        Key insight: We just backed away from a FRONT obstacle, so:
        1. AVOID front sectors (0, 1, 2, 10, 11) - that's where the obstacle is
        2. PREFER back sectors (4, 5, 6, 7, 8) - opposite of obstacle
        3. Use side clearance to break ties
        """
        # Check which side has more clearance (away from obstacle)
        # Uses SECTORS_BACK_LEFT and SECTORS_BACK_RIGHT from constants
        left_side_clear = min(sectors[s] for s in SECTORS_BACK_LEFT)
        right_side_clear = min(sectors[s] for s in SECTORS_BACK_RIGHT)

        # Find best BACK sector (avoid front entirely)
        # Uses SECTORS_BACK from constants
        best_back_sector = 6  # Default: straight back
        best_back_dist = sectors[6]

        for sector_idx in SECTORS_BACK:  # Only consider back sectors
            if sectors[sector_idx] > best_back_dist:
                best_back_dist = sectors[sector_idx]
                best_back_sector = sector_idx

        # Determine turn direction based on best back sector
        # Sectors 4, 5, 6 = back-left → turn left (positive)
        # Sectors 6, 7, 8 = back-right → turn right (negative)
        if best_back_sector <= 5:
            # Back-left is more open
            turn_dir = 0.5
            print(f"[ESCAPE] Turning left toward back-sector {best_back_sector} ({sectors[best_back_sector]:.2f}m)")
        elif best_back_sector >= 7:
            # Back-right is more open
            turn_dir = -0.5
            print(f"[ESCAPE] Turning right toward back-sector {best_back_sector} ({sectors[best_back_sector]:.2f}m)")
        else:
            # Sector 6 = straight back, use side clearance to decide
            if left_side_clear > right_side_clear:
                turn_dir = 0.5
                print(f"[ESCAPE] Back open, turning left (L={left_side_clear:.2f}m > R={right_side_clear:.2f}m)")
            else:
                turn_dir = -0.5
                print(f"[ESCAPE] Back open, turning right (R={right_side_clear:.2f}m >= L={left_side_clear:.2f}m)")

        return turn_dir

    def _format_telemetry(self) -> str:
        """Format telemetry data from ProObstacleAvoider and Frontier for display."""
        if not self.last_debug_info:
            return ""

        lines = []

        # TTC telemetry
        ttc = self.last_debug_info.get("ttc")
        if ttc:
            ttc_sectors = ttc.sector_ttcs
            critical = ttc.critical_sectors
            ttc_str = " ".join([f"s{s}:{t:.1f}" if t < 10 else f"s{s}:inf"
                               for s, t in sorted(ttc_sectors.items())])
            crit_str = f"crit:[{','.join(map(str, critical))}]" if critical else "crit:[]"
            lines.append(f"TTC: {ttc_str} {crit_str} rec:{ttc.recommended_speed:.2f}")

        # Gap analyzer telemetry
        gaps_full = self.last_debug_info.get("gaps_full", [])
        if gaps_full:
            gap_strs = []
            for g in gaps_full[:3]:  # Show top 3 gaps
                gap_strs.append(f"[s{g.start_sector}-s{g.end_sector}:w{g.width_sectors} "
                               f"d={g.min_distance:.1f}-{g.max_distance:.1f}m nav={g.navigability:.2f}]")
            lines.append(f"GAPS({len(gaps_full)}): {' '.join(gap_strs)}")

        # Sector distances
        if self.last_sectors:
            sect_str = " ".join([f"s{i}:{d:.2f}" for i, d in enumerate(self.last_sectors)])
            lines.append(f"SECT: {sect_str}")

        # Repulsive field telemetry
        repulsion = self.last_debug_info.get("repulsion", (0, 0, {}))
        if len(repulsion) >= 3:
            fx, fy, magnitudes = repulsion
            mag_str = " ".join([f"s{s}:{m:.1f}" for s, m in sorted(magnitudes.items()) if m > 0.1])
            lines.append(f"REPEL: fx={fx:.2f} fy={fy:.2f} | {mag_str if mag_str else 'none'}")

        # DWA telemetry
        dwa = self.last_debug_info.get("dwa", {})
        env = self.last_debug_info.get("environment", "?")
        if dwa:
            lines.append(f"DWA: env={env} clear={dwa.get('clearance', 0):.2f} "
                        f"vel={dwa.get('velocity', 0):.2f} head={dwa.get('heading', 0):.2f} "
                        f"goal={dwa.get('goal', 0):.2f} total={dwa.get('total', 0):.2f}")

        # Directional memory telemetry
        memory = self.last_debug_info.get("memory", {})
        if memory:
            failed = memory.get("failed", {})
            success = memory.get("successful", {})
            fail_str = " ".join([f"s{s}:{v:.1f}" for s, v in failed.items()]) if failed else "none"
            succ_str = " ".join([f"s{s}:{v:.1f}" for s, v in success.items()]) if success else "none"
            lines.append(f"MEM: fail=[{fail_str}] succ=[{succ_str}]")

        # Frontier telemetry
        frontier_costs = self.frontier.get_all_frontier_costs()
        if frontier_costs:
            cost_strs = [f"s{s}({ang:.0f}°):{c:.2f}" for s, ang, c in frontier_costs[:5]]
            best = frontier_costs[0] if frontier_costs else None
            best_str = f"BEST:s{best[0]}" if best else ""
            lines.append(f"FRONT: {' '.join(cost_strs)} {best_str}")

        return "\n".join(lines)

    def emergency_stop(self):
        """Stop the robot."""
        send_velocity_cmd(0.0, 0.0)
        self.running = False
        print("Robot stopped.")

    def _print_stats(self):
        """Print final statistics."""
        elapsed = time.time() - self.start_time
        stats = self.explorer.get_stats()
        print(f"\n{'=' * 55}")
        print(f"FRONTIER EXPLORATION COMPLETE")
        print(f"{'=' * 55}")
        print(f"Duration: {elapsed:.1f}s")
        print(f"Iterations: {self.iteration}")
        print(f"Unique cells: {stats['unique_cells']}")
        print(f"Total visits: {stats['total_visits']}")
        print(f"Revisit rate: {stats['revisit_pct']:.1f}%")
        print(f"Obstacles avoided: {self.obstacles_avoided}")
