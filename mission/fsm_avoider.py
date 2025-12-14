"""
Frontier-based obstacle avoidance with exploration efficiency.

Based on explore_lite and WFD (Wavefront Frontier Detector) algorithms.
Avoids dead ends proactively by steering toward frontiers (unexplored areas).

Features:
- Frontier-based exploration (like explore_lite)
- Direction commitment to prevent oscillation
- Visited cell tracking to avoid revisiting
- Virtual obstacles for invisible obstacles
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

        # Entry tracking - remember where we came from for escape
        self.entry_heading: Optional[float] = None  # Heading when entering new area
        self.entry_pos: Optional[Tuple[float, float]] = None
        self.last_open_heading: Optional[float] = None  # Last heading when path was clear

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
                front_arc = [sectors[11], sectors[0], sectors[1]]
                front_min = min(d for d in front_arc if d > 0.05) if any(d > 0.05 for d in front_arc) else 0

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

                # 2. COMPUTE
                safe_v, safe_w, debug = self.pro.compute_safe_velocity(
                    sector_distances=sectors,
                    min_distance=self.danger_threshold,
                    base_linear=self.linear_speed,
                    base_angular=0.0,
                    target_sector=0,
                    is_stuck=False
                )

                env = debug.get('environment', '?')
                ttc = debug.get('ttc')

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

                        # Choose direction based on which side has more space
                        if sectors[1] < sectors[11]:
                            self.committed_direction = "left"
                        else:
                            self.committed_direction = "right"
                        self.commit_time = time.time()
                        self.avoid_start_time = time.time()
                        print(f"\n[AVOID] Obstacle at {front_min:.2f}m, steering {self.committed_direction}")
                        self.obstacles_avoided += 1

                    # Dead end detection - if avoiding for too long, it's blocked
                    avoid_duration = time.time() - self.avoid_start_time
                    if avoid_duration > self.max_avoid_duration:
                        # Dead end - turn around completely
                        print(f"\n[DEAD END] Avoiding for {avoid_duration:.1f}s, turning around")
                        self.committed_direction = None
                        self.target_heading = None
                        # Force backup will handle turning around
                        front_min = 0.2  # Trigger backup

                    # Keep moving while steering gently around obstacle
                    # Slow down based on how close we are
                    speed_factor = (front_min - self.backup_threshold) / (self.danger_threshold - self.backup_threshold)
                    speed_factor = max(0.2, min(1.0, speed_factor))  # 20-100% speed
                    safe_v = self.linear_speed * speed_factor

                    # Gentle steering - just enough to go around
                    safe_w = 0.3 if self.committed_direction == "left" else -0.3

                else:
                    self.state = State.FORWARD

                    # Track last open heading - used for escape direction
                    if self.current_heading is not None and front_min > 0.8:
                        self.last_open_heading = self.current_heading

                    if self.committed_direction:
                        if time.time() - self.commit_time > self.commit_duration:
                            print(f"\n[CLEAR] Path clear")
                            self.committed_direction = None

                    # Frontier-based exploration (like explore_lite)
                    if self.current_pos and self.current_heading is not None:
                        # PROACTIVE dead-end detection - check BEFORE entering
                        is_dead_end_ahead, pattern = self.frontier.detect_dead_end_ahead(
                            sectors, threshold=0.5)

                        if is_dead_end_ahead:
                            # Don't enter the dead end - find escape route
                            print(f"\n[DEAD END AHEAD] Pattern: {pattern}, avoiding")
                            escape_sector = self.frontier.get_best_escape_direction(
                                sectors, self.explorer.visited,
                                self.current_pos[0], self.current_pos[1],
                                self.current_heading, self.explorer.grid_res
                            )
                            if escape_sector is not None:
                                escape_angle = self.frontier.sector_to_angle(escape_sector)
                                safe_w = max(-0.4, min(0.4, escape_angle * 0.6))
                            else:
                                # No escape, backup
                                self._execute_backup(sectors)
                                self.frontier.reset()
                                continue

                        else:
                            # Find best frontier to explore
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
                                # Steer toward best frontier
                                frontier_steer = self.frontier.get_steering(best_frontier)
                                safe_w += frontier_steer

                # 4. ACT
                send_velocity_cmd(safe_v, safe_w)

                # Status display
                stats = self.explorer.get_stats()
                frontier_stats = self.frontier.get_stats()
                state_abbr = self.state.name[:3]
                status = (f"[{self.iteration:3d}] {state_abbr} front={front_min:.2f}m "
                         f"cells={stats['unique_cells']} revisit={stats['revisit_pct']:.0f}% "
                         f"dead_ends={frontier_stats['dead_ends_detected']}")
                print(f"\r{status}", end="", flush=True)

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n\nStopping (Ctrl+C)...")
        finally:
            self.emergency_stop()
            self._print_stats()

    def _execute_backup(self, sectors: List[float]):
        """
        Execute backup maneuver with smart direction selection.

        Based on Nav2 recovery behaviors and Bug algorithm principles:
        1. Backup first
        2. Turn toward escape direction (where we came from / visited cells)
        3. Use virtual obstacle to prevent going wrong way
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

        # Step 4: Place virtual obstacle to prevent returning to dead end
        if self.current_pos and self.current_heading is not None:
            # Place virtual obstacle in front (where we were stuck)
            obs = self.virtual_obstacles.add_obstacle(
                self.current_pos[0], self.current_pos[1], self.current_heading)
            if obs:
                print(f"[BACKUP] Virtual obstacle placed to block dead end")

        # Clear path memory - we've changed direction significantly
        self.committed_direction = None
        self.target_heading = None

    def _calculate_escape_direction(self, sectors: List[float]) -> float:
        """
        Calculate best direction to escape dead end.

        Priority:
        1. Direction toward visited cells (where we came from)
        2. Direction toward last open heading
        3. Direction with more LiDAR space
        """
        # Method 1: Use visited cells to find way back
        if self.current_pos and self.current_heading is not None:
            best_sector = None
            best_visit_score = -1

            for sector_idx in range(NUM_SECTORS):
                if sectors[sector_idx] < 0.4:  # Skip blocked directions
                    continue

                # Calculate world angle for this sector
                sector_angle = sector_idx * 2 * math.pi / NUM_SECTORS
                if sector_angle > math.pi:
                    sector_angle -= 2 * math.pi
                world_angle = self.current_heading + sector_angle

                # Count visited cells in this direction (more = way back)
                visit_score = 0
                for dist in [0.3, 0.6, 0.9]:
                    x = self.current_pos[0] + dist * math.cos(world_angle)
                    y = self.current_pos[1] + dist * math.sin(world_angle)
                    cell = (int(x / self.explorer.grid_res),
                            int(y / self.explorer.grid_res))
                    visit_score += self.explorer.visited.get(cell, 0)

                if visit_score > best_visit_score:
                    best_visit_score = visit_score
                    best_sector = sector_idx

            # If found visited cells, turn toward them
            if best_sector is not None and best_visit_score > 0:
                sector_angle = best_sector * 2 * math.pi / NUM_SECTORS
                if sector_angle > math.pi:
                    sector_angle -= 2 * math.pi
                print(f"[ESCAPE] Turning toward visited cells (sector {best_sector})")
                return 0.5 if sector_angle > 0 else -0.5

        # Method 2: Turn toward last open heading (where we came from)
        if self.last_open_heading is not None and self.current_heading is not None:
            angle_diff = self.last_open_heading - self.current_heading
            # Normalize to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            if abs(angle_diff) > 0.3:  # Significant difference
                print(f"[ESCAPE] Turning toward last open heading")
                return 0.5 if angle_diff > 0 else -0.5

        # Method 3: Fallback - turn toward more open space
        # But prefer back directions (sectors 4-8) over front
        back_left = sectors[8] if len(sectors) > 8 else sectors[7]
        back_right = sectors[4] if len(sectors) > 4 else sectors[3]

        if back_left > back_right and back_left > 0.4:
            print(f"[ESCAPE] Turning left toward open back-left")
            return 0.5
        elif back_right > 0.4:
            print(f"[ESCAPE] Turning right toward open back-right")
            return -0.5

        # Final fallback - use side sectors
        if sectors[11] > sectors[1]:
            return 0.6
        else:
            return -0.6

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
