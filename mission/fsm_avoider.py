"""
Frontier-based obstacle avoidance with exploration efficiency.

Based on explore_lite and WFD (Wavefront Frontier Detector) algorithms.
Navigates toward maximum unexplored area using frontier cost function.

Features:
- Frontier-based exploration (like explore_lite)
- Direction commitment to prevent oscillation
- Visited cell tracking to avoid revisiting
- Virtual obstacles for invisible obstacles
- Detailed telemetry from avoidance algorithms
"""

import math
import time
from enum import Enum, auto
from typing import Optional, List, Tuple

from .pro_avoidance import ProObstacleAvoider
from .ros_interface import get_lidar_scan, send_velocity_cmd, get_odometry
from .avoider.lidar import compute_sector_distances
from .constants import ROBOT_WIDTH, NUM_SECTORS, LIDAR_FRONT_OFFSET
from .exploration import ExplorationTracker
from .virtual_obstacles import VirtualObstacleManager
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
    - FrontierExplorer: frontier detection, dead end avoidance (like explore_lite)
    - ExplorationTracker: visited cells, stuck detection
    - VirtualObstacleManager: invisible obstacle tracking
    - ProObstacleAvoider: TTC and environment detection
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

        # Path memory - remember where we were heading before avoidance
        self.target_heading: Optional[float] = None
        self.avoid_start_time: float = 0
        self.max_avoid_duration: float = 5.0  # Dead end if avoiding longer than this

        # Exploration modules (separate files for easier debugging)
        self.explorer = ExplorationTracker(grid_resolution=0.5)
        self.virtual_obstacles = VirtualObstacleManager(default_radius=0.3, default_ttl=120.0)
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
        self.last_open_heading: Optional[float] = None  # Last heading when path was clear
        self.heading_error: float = 0.0  # For telemetry (heading correction disabled)
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
                self.last_sectors = sectors  # Store for telemetry
                front_arc = [sectors[11], sectors[0], sectors[1]]
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

                    # Record visit
                    self.explorer.record_visit(odom[0], odom[1])

                    # Check for stuck (invisible obstacle)
                    if self.state == State.FORWARD:
                        if self.explorer.check_stuck(odom[0], odom[1]):
                            new_obs = self.virtual_obstacles.add_obstacle(
                                odom[0], odom[1], odom[2])
                            if new_obs:
                                print(f"\n[VIRTUAL] Stuck! Added obstacle at ({new_obs.x:.2f}, {new_obs.y:.2f})")
                            front_min = 0.2  # Force backup

                    # Check virtual obstacles
                    virtual_dist = self.virtual_obstacles.get_nearest_in_front(
                        odom[0], odom[1], odom[2])
                    if virtual_dist < front_min:
                        front_min = virtual_dist

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
                        # Save current heading as target to return to after avoidance
                        if self.current_heading is not None:
                            self.target_heading = self.current_heading

                        # Check FULL side sectors to determine obstacle location
                        # Left side: sectors 1, 2, 3 (+30°, +60°, +90°)
                        # Right side: sectors 9, 10, 11 (-90°, -60°, -30°)
                        left_side_min = min(sectors[1], sectors[2], sectors[3])
                        right_side_min = min(sectors[9], sectors[10], sectors[11])

                        # Steer AWAY from the side with obstacle (smaller distance)
                        if left_side_min < right_side_min:
                            # Obstacle on LEFT → steer RIGHT (negative angular)
                            self.committed_direction = "right"
                        else:
                            # Obstacle on RIGHT → steer LEFT (positive angular)
                            self.committed_direction = "left"

                        self.commit_time = time.time()
                        self.avoid_start_time = time.time()
                        print(f"\n[AVOID] Obstacle at {front_min:.2f}m, L={left_side_min:.2f} R={right_side_min:.2f}, steering {self.committed_direction}")
                        self.obstacles_avoided += 1

                    # Dead end detection - if avoiding for too long, it's blocked
                    avoid_duration = time.time() - self.avoid_start_time
                    if avoid_duration > self.max_avoid_duration:
                        # Dead end - turn around completely
                        print(f"\n[DEAD END] Avoiding for {avoid_duration:.1f}s, turning around")
                        self.committed_direction = None
                        self.target_heading = None
                        # Execute backup immediately
                        self._execute_backup(sectors)
                        self.frontier.reset()
                        continue

                    # Keep moving while steering gently around obstacle
                    # Slow down based on how close we are
                    speed_factor = (front_min - self.backup_threshold) / (self.danger_threshold - self.backup_threshold)
                    speed_factor = max(0.2, min(1.0, speed_factor))  # 20-100% speed
                    safe_v = self.linear_speed * speed_factor

                    # Gentle steering - check committed_direction is set
                    if self.committed_direction == "left":
                        safe_w = 0.3   # Turn left (positive angular)
                    elif self.committed_direction == "right":
                        safe_w = -0.3  # Turn right (negative angular)
                    else:
                        # No direction committed - check full sides
                        # Left side: sectors 1, 2, 3 | Right side: sectors 9, 10, 11
                        left_min = min(sectors[1], sectors[2], sectors[3])
                        right_min = min(sectors[9], sectors[10], sectors[11])
                        # Steer away from obstacle
                        safe_w = -0.3 if left_min < right_min else 0.3

                else:
                    self.state = State.FORWARD

                    # FORWARD STATE: Frontier is sole steering authority
                    # ProObstacleAvoider only provides speed limiting (TTC), not angular velocity
                    # This prevents conflicting steering from DWA/repulsive field
                    safe_w = 0.0  # Reset - don't use DWA/repulsive angular

                    # Track last open heading - used for escape direction
                    if self.current_heading is not None and front_min > 0.8:
                        self.last_open_heading = self.current_heading

                    if self.committed_direction:
                        if time.time() - self.commit_time > self.commit_duration:
                            print(f"\n[CLEAR] Path clear")
                            self.committed_direction = None

                    # RETURN TO PATH: Log heading error for monitoring (correction disabled - causes rightward drift)
                    if self.target_heading is not None and self.current_heading is not None:
                        # Calculate heading error
                        heading_error = self.target_heading - self.current_heading

                        # Normalize to [-pi, pi]
                        while heading_error > math.pi:
                            heading_error -= 2 * math.pi
                        while heading_error < -math.pi:
                            heading_error += 2 * math.pi

                        # Log heading deviation for monitoring (no correction applied)
                        if abs(heading_error) > math.pi / 2:
                            # Odometry probably jumped - reset target
                            self.target_heading = None
                        elif abs(heading_error) < 0.1:  # ~6 degrees - close enough
                            self.target_heading = None
                        # Note: heading correction disabled, only tracking for telemetry
                        self.heading_error = heading_error  # Store for telemetry

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
                            # Steer toward best frontier (sole steering source)
                            safe_w = self.frontier.get_steering(best_frontier)

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

                # Debug: print all sector distances with angles
                # Sector angles: 0=front, positive=right, negative=left
                def sector_angle(s):
                    if s == 0: return 0
                    elif s <= 6: return s * 30    # Right side: +30 to +180
                    else: return -(12 - s) * 30   # Left side: -150 to -30
                sect_str = " ".join([f"{i}({sector_angle(i):+d}°):{d:.2f}" for i, d in enumerate(sectors)])
                print(f"\n[SECT] {sect_str}")

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
        2. Turn toward most open direction

        Note: Virtual obstacles are only placed by stuck detection, not here.
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
        Calculate best direction to escape - LIDAR ONLY (odometry unreliable).

        Simple rule: Turn toward the most open direction.
        Prefer back/side directions over front (we just hit something in front).
        """
        # Find the sector with maximum distance (most open)
        best_sector = 0
        best_distance = 0

        # Check all sectors, but give bonus to back/side directions
        for sector_idx in range(NUM_SECTORS):
            dist = sectors[sector_idx]

            # Give bonus to back sectors (we want to escape, not go forward)
            # Back sectors: 4, 5, 6, 7, 8
            if 4 <= sector_idx <= 8:
                dist *= 1.5  # 50% bonus for back directions

            if dist > best_distance:
                best_distance = dist
                best_sector = sector_idx

        # Convert sector to turn direction
        # Sectors 1-6 = LEFT side → turn left (positive angular)
        # Sectors 7-11 = RIGHT side → turn right (negative angular)
        if best_sector == 0:
            # Front is most open - check sides to decide, turn away from obstacle
            left_min = min(sectors[1], sectors[2], sectors[3])
            right_min = min(sectors[9], sectors[10], sectors[11])
            turn_dir = -0.5 if left_min < right_min else 0.5  # Turn away from closer side
            print(f"[ESCAPE] Front open, turning {'left' if turn_dir > 0 else 'right'} (away from obstacle)")
        elif best_sector <= 6:
            turn_dir = 0.5  # Turn left toward sectors 1-6
            print(f"[ESCAPE] Turning left toward sector {best_sector} ({sectors[best_sector]:.2f}m)")
        else:
            turn_dir = -0.5  # Turn right toward sectors 7-11
            print(f"[ESCAPE] Turning right toward sector {best_sector} ({sectors[best_sector]:.2f}m)")

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

        # Heading error (warning only, correction disabled)
        if abs(self.heading_error) > 0.15:  # ~8.5 degrees
            lines.append(f"WARN: heading_err={math.degrees(self.heading_error):.1f}° (correction disabled)")

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
        print(f"Virtual obstacles: {self.virtual_obstacles.count()}")
        print(f"Obstacles avoided: {self.obstacles_avoided}")
