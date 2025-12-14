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
            min_frontier_distance=0.8,
            distance_weight=1.0,
            novelty_weight=2.0,  # Prefer unexplored areas
            turn_weight=0.3
        )

        # State tracking
        self.state = State.STOPPED
        self.current_pos: Optional[Tuple[float, float]] = None
        self.current_heading: Optional[float] = None

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

                    if self.committed_direction:
                        if time.time() - self.commit_time > self.commit_duration:
                            print(f"\n[CLEAR] Path clear")
                            self.committed_direction = None

                    # Frontier-based exploration (like explore_lite)
                    if self.current_pos and self.current_heading is not None:
                        # Find best frontier to explore
                        best_sector, is_dead_end = self.frontier.select_best_frontier(
                            sectors=sectors,
                            visited_cells=self.explorer.visited,
                            robot_x=self.current_pos[0],
                            robot_y=self.current_pos[1],
                            robot_heading=self.current_heading,
                            grid_resolution=self.explorer.grid_res
                        )

                        if is_dead_end:
                            # No good frontiers - turn around proactively
                            print(f"\n[DEAD END] No frontiers, turning around")
                            self._execute_backup(sectors)
                            self.frontier.reset()
                            continue

                        if best_sector is not None:
                            # Steer toward best frontier
                            frontier_steer = self.frontier.get_steering_toward_frontier(
                                best_sector, sectors)
                            safe_w += frontier_steer

                # 4. ACT
                send_velocity_cmd(safe_v, safe_w)

                # Status display
                stats = self.explorer.get_stats()
                frontier_stats = self.frontier.get_stats()
                state_abbr = self.state.name[:3]
                dead_ends = frontier_stats['consecutive_dead_ends']
                status = (f"[{self.iteration:3d}] {state_abbr} front={front_min:.2f}m "
                         f"cells={stats['unique_cells']} revisit={stats['revisit_pct']:.0f}% "
                         f"virt={self.virtual_obstacles.count()}")
                print(f"\r{status}", end="", flush=True)

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n\nStopping (Ctrl+C)...")
        finally:
            self.emergency_stop()
            self._print_stats()

    def _execute_backup(self, sectors: List[float]):
        """Execute backup maneuver."""
        for _ in range(10):
            send_velocity_cmd(-0.05, 0.0)
            time.sleep(0.1)

        turn_dir = -0.6 if sectors[1] < sectors[11] else 0.6
        for _ in range(8):
            send_velocity_cmd(0.0, turn_dir)
            time.sleep(0.1)

        # Clear path memory - we've changed direction significantly
        self.committed_direction = None
        self.target_heading = None

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
