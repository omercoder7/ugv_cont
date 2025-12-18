"""
Next-Best-View (NBV) navigation.

Selects goal POINTS (not directions) that maximize expected visibility gain.

Logic:
1. Select a goal point that:
   - Is in line-of-sight (visible via LiDAR)
   - Would give maximum new area to scan when reached
   - Is NOT at a "scan end" (wall/obstacle boundary)
2. Drive to that goal point
3. When reached (or blocked), select new goal

No step-by-step reevaluation - commits to goals.
"""

import math
import time
import threading
import subprocess
from typing import Optional, List, Tuple, Dict, Set

from .ros_interface import get_lidar_scan, send_velocity_cmd, get_odometry, publish_goal_marker
from .avoider.lidar import compute_sector_distances
from .constants import NUM_SECTORS, CONTAINER_NAME


class NBVNavigator:
    """
    Next-Best-View navigation.

    Selects goal points that maximize expected visibility gain,
    avoiding "scan ends" (walls, boundaries).
    """

    def __init__(self, linear_speed: float = 0.08, duration: float = 60.0,
                 backup_threshold: float = 0.25, blocked_margin: float = 0.05,
                 debug_marker: bool = False):
        self.linear_speed = linear_speed
        self.duration = duration
        self.running = False
        self.debug_marker = debug_marker
        self._marker_thread = None
        self._marker_proc = None

        # Thread lock for goal_point access (marker thread reads, main thread writes)
        self._goal_lock = threading.Lock()

        # Thresholds
        self.backup_threshold = backup_threshold
        self.blocked_margin = blocked_margin  # Margin for considering goal blocked (tighter = safer)
        self.goal_reached_dist = 0.20  # Consider goal reached if within 20cm
        self.min_goal_dist = 0.4       # Don't set goals closer than this
        self.max_goal_dist = 1.5       # Don't set goals further than this

        # Goal tracking
        self.goal_point: Optional[Tuple[float, float]] = None
        self.goal_set_time: float = 0
        self.goal_timeout: float = 30.0  # Much larger timeout - rotation takes time

        # Heading lock for drift correction
        # When aligned and driving straight, lock heading and continuously correct
        self.locked_heading: Optional[float] = None
        self.heading_lock_gain = 1.5  # Stronger correction for drift

        # Visited cells (for avoiding revisits) - tracks robot position history
        self.visited: Dict[Tuple[int, int], int] = {}
        self.grid_res = 0.3

        # Scanned cells - tracks what areas the LiDAR has observed
        # Key: cell, Value: timestamp of last scan
        self.scanned: Dict[Tuple[int, int], float] = {}

        # Scan boundaries - track where we've seen walls
        # Key: cell, Value: timestamp of when wall was last seen
        self.scan_ends: Dict[Tuple[int, int], float] = {}

        # Recent goals - to prevent circular patterns
        # Stores (cell, timestamp) of recent goal selections
        self.recent_goals: List[Tuple[Tuple[int, int], float]] = []
        self.recent_goal_memory = 60.0  # Remember goals for 60 seconds

        # Current state
        self.current_pos: Optional[Tuple[float, float]] = None
        self.current_heading: Optional[float] = None

        # Statistics
        self.iteration = 0
        self.start_time = 0.0
        self.goals_reached = 0
        self.backups = 0

    def _start_marker_thread(self):
        """Start persistent marker publisher that keeps the topic alive."""
        # Start persistent publisher in Docker that reads from stdin
        self._marker_proc = subprocess.Popen(
            ['docker', 'exec', '-i', CONTAINER_NAME, 'bash', '-c',
             '''source /opt/ros/humble/setup.bash && python3 -u -c "
import rclpy
from visualization_msgs.msg import Marker
import sys
import select
import time

rclpy.init()
node = rclpy.create_node('nav_goal_marker')
pub = node.create_publisher(Marker, '/nav_goal', 10)

x, y = 0.0, 0.0

while rclpy.ok():
    # Non-blocking read from stdin
    if select.select([sys.stdin], [], [], 0.01)[0]:
        try:
            line = sys.stdin.readline().strip()
            if line:
                parts = line.split(',')
                if len(parts) == 2:
                    x, y = float(parts[0]), float(parts[1])
        except:
            pass

    # Always publish current marker
    m = Marker()
    m.header.frame_id = 'odom'
    m.header.stamp = node.get_clock().now().to_msg()
    m.ns = 'goal'
    m.id = 0
    m.type = 2
    m.action = 0
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.15
    m.pose.orientation.w = 1.0
    m.scale.x = 0.25
    m.scale.y = 0.25
    m.scale.z = 0.25
    m.color.g = 1.0
    m.color.a = 1.0
    m.lifetime.sec = 1
    pub.publish(m)
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.15)

node.destroy_node()
rclpy.shutdown()
"'''],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        print("[MARKER] Persistent marker publisher started - add /nav_goal in RViz")
        time.sleep(0.5)  # Give it time to start

        # Thread to send coordinates via stdin when goal changes
        def update_loop():
            while self.running and self.debug_marker:
                with self._goal_lock:
                    goal = self.goal_point
                if goal:
                    x, y = goal
                    try:
                        self._marker_proc.stdin.write(f'{x},{y}\n'.encode())
                        self._marker_proc.stdin.flush()
                    except:
                        pass
                time.sleep(0.2)

        self._marker_thread = threading.Thread(target=update_loop, daemon=True)
        self._marker_thread.start()

    def _stop_marker_thread(self):
        """Stop the marker publisher subprocess."""
        if self._marker_proc:
            try:
                self._marker_proc.terminate()
                self._marker_proc.wait(timeout=2)
                print("[MARKER] Publisher stopped")
            except Exception:
                try:
                    self._marker_proc.kill()
                except Exception:
                    pass
            self._marker_proc = None

    def run(self):
        """Main loop: select goal -> drive to goal -> repeat."""
        print("=" * 55)
        print("NEXT-BEST-VIEW (NBV) NAVIGATION")
        print("=" * 55)
        print(f"Speed: {self.linear_speed} m/s")
        print(f"Goal distance: {self.min_goal_dist}-{self.max_goal_dist}m")
        print("Press Ctrl+C to stop\n")

        self.running = True
        self.start_time = time.time()

        # Start marker publishing thread if debug mode
        if self.debug_marker:
            print("[DEBUG] Marker publishing enabled - check /nav_goal in RViz")
            self._start_marker_thread()

        send_velocity_cmd(0.0, 0.0)
        time.sleep(0.2)

        try:
            while self.running:
                self.iteration += 1

                # Check duration
                if self.duration > 0 and time.time() - self.start_time >= self.duration:
                    print(f"\n\nDuration {self.duration}s reached.")
                    break

                # === 1. SENSE ===
                scan = get_lidar_scan()
                if not scan:
                    print("Waiting for LiDAR...")
                    time.sleep(0.5)
                    continue

                sectors = compute_sector_distances(scan)

                odom = get_odometry()
                if not odom:
                    time.sleep(0.1)
                    continue

                self.current_pos = (odom[0], odom[1])
                self.current_heading = odom[2]

                # Record visit
                cell = self._pos_to_cell(odom[0], odom[1])
                self.visited[cell] = self.visited.get(cell, 0) + 1

                # Record scan ends (where LiDAR hits walls)
                self._record_scan_coverage(sectors)

                # Front distance
                front_min = min(sectors[11], sectors[0], sectors[1])
                if front_min < 0.05:
                    front_min = 0.05  # Blind spot - treat as very close, trigger backup

                # === 2. CHECK IF NEED NEW GOAL ===
                need_new_goal = False

                if self.goal_point is None:
                    need_new_goal = True
                    reason = "no goal"
                elif self._goal_reached():
                    need_new_goal = True
                    reason = "reached"
                    self.goals_reached += 1
                elif self._goal_blocked(sectors):
                    need_new_goal = True
                    reason = "blocked"
                elif time.time() - self.goal_set_time > self.goal_timeout:
                    need_new_goal = True
                    reason = "timeout"

                if need_new_goal:
                    new_goal, goal_sector = self._select_goal_point(sectors)
                    if new_goal:
                        with self._goal_lock:
                            self.goal_point = new_goal
                        self.goal_set_time = time.time()
                        self.locked_heading = None  # Clear heading lock for new goal
                        dist = math.sqrt((new_goal[0] - self.current_pos[0])**2 +
                                        (new_goal[1] - self.current_pos[1])**2)

                        # Publish goal marker to RViz (green sphere)
                        publish_goal_marker(new_goal[0], new_goal[1])

                        # Print goal info (debug scoring is printed in _select_goal_point)
                        print(f"[GOAL] Selected: ({new_goal[0]:.2f}, {new_goal[1]:.2f}) "
                              f"dist={dist:.2f}m angle={goal_sector}° reason={reason}")
                    else:
                        # No valid goal found - backup
                        print(f"\n[NO GOAL] No valid viewpoint found, backing up")
                        self._backup(sectors)
                        self.backups += 1
                        continue

                # === 3. BACKUP if too close ===
                if front_min < self.backup_threshold:
                    print(f"\n[BACKUP] front={front_min:.2f}m")
                    self._backup(sectors)
                    self.backups += 1
                    with self._goal_lock:
                        self.goal_point = None  # Force replan
                    continue

                # === 4. DRIVE TO GOAL ===
                if self.goal_point:
                    v, w = self._compute_velocity_to_goal(front_min)
                    send_velocity_cmd(v, w)

                    # Status
                    goal_dist = math.sqrt((self.goal_point[0] - self.current_pos[0])**2 +
                                         (self.goal_point[1] - self.current_pos[1])**2)
                    pos_str = f"({self.current_pos[0]:.2f},{self.current_pos[1]:.2f})"
                    goal_str = f"({self.goal_point[0]:.2f},{self.goal_point[1]:.2f})"
                    status = f"[{self.iteration:3d}] DRIVE goal={goal_str} dist={goal_dist:.2f}m front={front_min:.2f}m cells={len(self.visited)}"
                    print(f"\r{status}", end="", flush=True)

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n\nStopping (Ctrl+C)...")
        finally:
            self.running = False
            send_velocity_cmd(0.0, 0.0)
            self._stop_marker_thread()
            self._print_stats()

    def _select_goal_point(self, sectors: List[float]) -> Tuple[Optional[Tuple[float, float]], Optional[int]]:
        """
        Select goal point that maximizes expected visibility gain.

        Criteria:
        1. Must be in line-of-sight (LiDAR can see it)
        2. Should NOT be at a "scan end" (wall boundary)
        3. Area around it should be open (good visibility from there)
        4. Prefer unscanned areas

        Returns: (goal_point, goal_sector) or (None, None)
        """
        if not self.current_pos or self.current_heading is None:
            return None, None

        # Clean up old goals from recent_goals list
        now = time.time()
        self.recent_goals = [(cell, t) for (cell, t) in self.recent_goals
                            if now - t < self.recent_goal_memory]

        best_point = None
        best_score = -999
        best_sector = None
        best_scores = {}  # For debug output

        # Collect all candidates with scores for debug
        candidates = []

        # Evaluate candidate points distributed across 300° arc (excluding back 60°)
        # 25 angles × 4 distances = 100 potential points
        # 25 angles over 300° = 12.5° per step (finer resolution to catch corridors)
        #
        # Angular range: -150° to +150° (300° total, excluding back blind spot)
        num_angles = 25
        num_distances = 4

        # Generate equally spaced angles from -150° to +150° (in radians)
        angle_start = -150.0 * math.pi / 180.0  # -150° = right-back
        angle_end = 150.0 * math.pi / 180.0     # +150° = left-back
        angle_step = (angle_end - angle_start) / (num_angles - 1)

        # Generate equally spaced distances from min to max goal distance
        dist_step = (self.max_goal_dist - self.min_goal_dist) / (num_distances - 1)
        candidate_distances = [self.min_goal_dist + i * dist_step for i in range(num_distances)]

        for angle_idx in range(num_angles):
            # Robot-frame angle for this candidate direction
            robot_angle = angle_start + angle_idx * angle_step
            world_angle = self.current_heading + robot_angle

            # Find which sector this angle falls into for obstacle checking
            sector_idx = self._angle_to_sector(robot_angle)
            sector_dist = sectors[sector_idx]

            if sector_dist < self.min_goal_dist + 0.1:
                # Direction too blocked
                continue

            # Don't place goal at the wall - leave margin
            max_dist = min(sector_dist - 0.3, self.max_goal_dist)

            for dist in candidate_distances:
                if dist > max_dist:
                    continue

                # Calculate world position of candidate point
                px = self.current_pos[0] + dist * math.cos(world_angle)
                py = self.current_pos[1] + dist * math.sin(world_angle)

                # FILTER: Skip points that are directly ON a known wall cell
                point_cell = self._pos_to_cell(px, py)
                if self.scan_ends.get(point_cell, 0) > 0:
                    continue  # Discard - this is a wall cell

                # Score this viewpoint with detailed breakdown
                score, breakdown = self._score_viewpoint_debug(px, py, sector_idx, dist, sectors)
                angle_deg = int(robot_angle * 180 / math.pi)
                candidates.append((score, angle_deg, dist, px, py, breakdown))

                if score > best_score:
                    best_score = score
                    best_point = (px, py)
                    best_sector = angle_deg  # Store angle in degrees for display
                    best_scores = breakdown

        # Debug output: show top candidates
        if candidates:
            candidates.sort(key=lambda x: x[0], reverse=True)
            print(f"\n[GOAL SELECT] Top {min(5, len(candidates))}/{len(candidates)} candidates "
                  f"(scanned={len(self.scanned)} walls={len(self.scan_ends)}):")
            for i, (score, ang, d, px, py, brk) in enumerate(candidates[:5]):
                marker = " <-- BEST" if i == 0 else ""
                print(f"  #{i+1} ang={ang:+4d}° d={d:.2f}m pos=({px:.2f},{py:.2f}) "
                      f"score={score:+.1f} [open={brk['open']:+.1f} unscan={brk['unscan']:+.1f} "
                      f"visit={brk['visited']:+.1f} wall={brk['wall']:+.1f}]{marker}")

        # Record this goal to prevent cycling back
        if best_point:
            goal_cell = self._pos_to_cell(best_point[0], best_point[1])
            self.recent_goals.append((goal_cell, now))

        return best_point, best_sector

    def _score_viewpoint_debug(self, px: float, py: float, sector_idx: int,
                               dist: float, sectors: List[float]) -> Tuple[float, dict]:
        """Score viewpoint and return breakdown for debugging."""
        point_cell = self._pos_to_cell(px, py)
        now = time.time()
        breakdown = {'open': 0.0, 'unscan': 0.0, 'recent': 0.0, 'wall': 0.0, 'visited': 0.0, 'dist': 0.0}

        # === Factor 1: OPENNESS (DOMINANT) ===
        openness_score = 0.0
        margin_to_wall = sectors[sector_idx] - dist
        if margin_to_wall < 0.2:
            openness_score -= 5.0
        else:
            openness_score += margin_to_wall * 2.0

        for adj in [-3, -2, -1, 1, 2, 3]:
            adj_sector = (sector_idx + adj) % NUM_SECTORS
            adj_dist = sectors[adj_sector]
            if adj_dist > 0.8:
                openness_score += min(adj_dist, 2.0)
            elif adj_dist < 0.4:
                openness_score -= 0.5

        breakdown['open'] = openness_score * 1.5

        # === Factor 2: UNSCANNED AREA ===
        unscanned_count = 0
        scan_age_bonus = 0.0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor = (point_cell[0] + dx, point_cell[1] + dy)
                last_scan_time = self.scanned.get(neighbor, 0)
                if last_scan_time == 0:
                    unscanned_count += 1
                else:
                    age = now - last_scan_time
                    if age > 30:
                        scan_age_bonus += min(age / 60.0, 1.0)

        breakdown['unscan'] = unscanned_count * 2.0 + scan_age_bonus * 0.5

        # === Factor 3: RECENT GOAL PENALTY ===
        recent_penalty = 0.0
        for (goal_cell, goal_time) in self.recent_goals:
            cell_dist = abs(point_cell[0] - goal_cell[0]) + abs(point_cell[1] - goal_cell[1])
            if cell_dist <= 2:
                recency = (self.recent_goal_memory - (now - goal_time)) / self.recent_goal_memory
                if recency > 0:
                    recent_penalty += recency * (3 - cell_dist) * 5.0
        breakdown['recent'] = -recent_penalty

        # === Factor 4: WALL AVOIDANCE ===
        # Points directly on walls are filtered in _select_goal_point
        # Here we penalize being adjacent to walls
        wall_penalty = 0.0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor = (point_cell[0] + dx, point_cell[1] + dy)
                if self.scan_ends.get(neighbor, 0) > 0:
                    if dx == 0 and dy == 0:
                        wall_penalty += 10.0  # On wall (shouldn't happen, filtered)
                    else:
                        wall_penalty += 2.0   # Adjacent to wall
        breakdown['wall'] = -wall_penalty

        # === Factor 5: VISITED PENALTY ===
        # Penalize revisiting, but with diminishing returns (log scale)
        # This prevents the penalty from growing unbounded
        visited_penalty = 0.0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor = (point_cell[0] + dx, point_cell[1] + dy)
                visit_count = self.visited.get(neighbor, 0)
                if visit_count > 0:
                    # Use log scale: penalty grows slowly after first few visits
                    # log(1)=0, log(2)=0.7, log(10)=2.3, log(100)=4.6
                    log_penalty = math.log(1 + visit_count)
                    if dx == 0 and dy == 0:
                        visited_penalty += log_penalty * 2.0  # Exact cell
                    else:
                        visited_penalty += log_penalty * 0.5  # Adjacent cells
        breakdown['visited'] = -visited_penalty

        # === Factor 6: DISTANCE EFFICIENCY ===
        breakdown['dist'] = dist * 0.3

        total = sum(breakdown.values())
        return total, breakdown

    def _record_scan_coverage(self, sectors: List[float]):
        """Record LiDAR scan coverage - both scanned areas and wall boundaries."""
        if not self.current_pos or self.current_heading is None:
            return

        now = time.time()

        for sector_idx in range(NUM_SECTORS):
            dist = sectors[sector_idx]

            if dist < 0.1:
                continue  # Invalid reading

            # Calculate direction
            angle = self._sector_to_angle(sector_idx)
            world_angle = self.current_heading + angle

            # Record all cells along the scan ray as "scanned"
            # Use adaptive step size based on grid resolution
            scan_dist = min(dist, 3.0)  # Cap at 3m
            step = self.grid_res * 0.8  # Slight overlap to avoid gaps
            num_steps = int(scan_dist / step)
            for i in range(1, num_steps + 1):
                d = i * step
                px = self.current_pos[0] + d * math.cos(world_angle)
                py = self.current_pos[1] + d * math.sin(world_angle)
                cell = self._pos_to_cell(px, py)
                self.scanned[cell] = now

            # Record wall position (scan end) if not max range
            # Use timestamp instead of count to avoid runaway accumulation
            if dist < 3.0:
                wall_x = self.current_pos[0] + dist * math.cos(world_angle)
                wall_y = self.current_pos[1] + dist * math.sin(world_angle)
                wall_cell = self._pos_to_cell(wall_x, wall_y)
                self.scan_ends[wall_cell] = now  # Store timestamp, not count

    def _goal_reached(self) -> bool:
        """Check if current goal is reached."""
        if not self.goal_point or not self.current_pos:
            return False

        dist = math.sqrt((self.goal_point[0] - self.current_pos[0])**2 +
                        (self.goal_point[1] - self.current_pos[1])**2)
        return dist < self.goal_reached_dist

    def _goal_blocked(self, sectors: List[float]) -> bool:
        """Check if path to goal is blocked."""
        if not self.goal_point or not self.current_pos or self.current_heading is None:
            return True

        # Calculate angle to goal
        dx = self.goal_point[0] - self.current_pos[0]
        dy = self.goal_point[1] - self.current_pos[1]
        goal_dist = math.sqrt(dx*dx + dy*dy)
        goal_world_angle = math.atan2(dy, dx)

        # Convert to robot frame
        goal_robot_angle = self._normalize_angle(goal_world_angle - self.current_heading)

        # Find which sector the goal is in
        goal_sector = self._angle_to_sector(goal_robot_angle)

        # Check if that sector is blocked before the goal
        sector_dist = sectors[goal_sector]
        if sector_dist < goal_dist - self.blocked_margin:
            return True  # Obstacle between us and goal

        return False

    def _compute_velocity_to_goal(self, front_min: float) -> Tuple[float, float]:
        """
        Compute velocity commands to drive toward goal.

        Uses heading lock for drift correction:
        - When aligned, lock in the target heading
        - Continuously correct any drift from locked heading
        - Stronger correction gain prevents sway
        """
        if not self.goal_point or not self.current_pos or self.current_heading is None:
            return 0.0, 0.0

        # Calculate angle to goal
        dx = self.goal_point[0] - self.current_pos[0]
        dy = self.goal_point[1] - self.current_pos[1]
        goal_world_angle = math.atan2(dy, dx)

        # Angle error (how much we need to turn to face goal)
        angle_error = self._normalize_angle(goal_world_angle - self.current_heading)

        # Proportional control with HEADING LOCK for drift correction
        if abs(angle_error) > 0.4:  # ~23 degrees - need to rotate
            # Clear heading lock - we're rotating
            self.locked_heading = None
            # Rotate in place - faster rotation
            v = 0.0
            w = 0.7 if angle_error > 0 else -0.7

        elif abs(angle_error) > 0.15:  # ~9-23 degrees - slow turn
            # Clear heading lock - still turning
            self.locked_heading = None
            v = self.linear_speed * 0.5
            w = angle_error * 1.2  # Proportional

        else:
            # Aligned (<9°) - LOCK HEADING and drive straight with drift correction
            if self.locked_heading is None:
                # Lock current goal heading (not current robot heading!)
                self.locked_heading = goal_world_angle

            # Calculate drift from locked heading
            drift_error = self._normalize_angle(self.locked_heading - self.current_heading)

            # Strong proportional correction to maintain locked heading
            # This prevents sway/drift by continuously correcting
            v = self.linear_speed
            w = drift_error * self.heading_lock_gain  # Stronger gain for drift

            # Limit correction to prevent oscillation
            w = max(-0.4, min(0.4, w))

        # Slow down if obstacle nearby
        if front_min < 0.5:
            speed_factor = (front_min - self.backup_threshold) / (0.5 - self.backup_threshold)
            speed_factor = max(0.3, min(1.0, speed_factor))
            v *= speed_factor

        return v, w

    def _backup(self, sectors: List[float]):
        """Backup maneuver."""
        print(f"\n[BACKUP] Reversing...")

        # Reverse
        for _ in range(10):
            send_velocity_cmd(-0.05, 0.0)
            time.sleep(0.1)

        # Turn toward more open side (check front-left vs front-right after backup)
        front_left = min(sectors[1], sectors[2], sectors[3])
        front_right = min(sectors[9], sectors[10], sectors[11])

        turn_w = 0.5 if front_left > front_right else -0.5
        print(f"[BACKUP] Turning {'left' if turn_w > 0 else 'right'} (L={front_left:.2f} R={front_right:.2f})")

        for _ in range(10):
            send_velocity_cmd(0.0, turn_w)
            time.sleep(0.1)

        send_velocity_cmd(0.0, 0.0)

        # Clear heading lock after backup
        self.locked_heading = None

    def _sector_to_angle(self, sector: int) -> float:
        """Convert sector index to angle in robot frame."""
        angle = sector * (2 * math.pi / NUM_SECTORS)
        if angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def _angle_to_sector(self, angle: float) -> int:
        """Convert robot-frame angle to sector index."""
        angle = self._normalize_angle(angle)
        if angle < 0:
            angle += 2 * math.pi
        sector = int(angle / (2 * math.pi / NUM_SECTORS)) % NUM_SECTORS
        return sector

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _pos_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Convert position to grid cell."""
        return (int(x / self.grid_res), int(y / self.grid_res))

    def __del__(self):
        """Cleanup subprocess on destruction."""
        if hasattr(self, '_marker_proc') and self._marker_proc:
            try:
                self._marker_proc.terminate()
                self._marker_proc.wait(timeout=1)
            except:
                pass

    def _print_stats(self):
        """Print final statistics."""
        elapsed = time.time() - self.start_time
        print(f"\n{'=' * 55}")
        print(f"NBV NAVIGATION COMPLETE")
        print(f"{'=' * 55}")
        print(f"Duration: {elapsed:.1f}s")
        print(f"Iterations: {self.iteration}")
        print(f"Goals reached: {self.goals_reached}")
        print(f"Backups: {self.backups}")
        print(f"Unique cells visited: {len(self.visited)}")
        print(f"Cells scanned by LiDAR: {len(self.scanned)}")
        print(f"Walls detected: {len(self.scan_ends)}")

        total = sum(self.visited.values())
        if total > 0:
            revisit_pct = 100 * (total - len(self.visited)) / total
            print(f"Revisit rate: {revisit_pct:.1f}%")


def run_simple_nav(speed: float = 0.08, duration: float = 60.0,
                   backup_threshold: float = 0.25, blocked_margin: float = 0.15,
                   debug_marker: bool = False):
    """Entry point for NBV navigation."""
    nav = NBVNavigator(linear_speed=speed, duration=duration,
                       backup_threshold=backup_threshold, blocked_margin=blocked_margin,
                       debug_marker=debug_marker)
    nav.run()


# Allow running directly: python -m mission.simple_cost_nav
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Next-Best-View (NBV) navigation")
    parser.add_argument("--speed", type=float, default=0.08, help="Linear speed (m/s)")
    parser.add_argument("--duration", type=float, default=60.0, help="Duration (seconds, 0=unlimited)")
    parser.add_argument("--backup-threshold", type=float, default=0.25, help="Backup if closer than this (m)")
    parser.add_argument("--blocked-margin", type=float, default=0.15, help="Margin for goal blocked detection (m)")
    parser.add_argument("--debug-marker", action="store_true", help="Continuously publish goal marker to RViz")
    args = parser.parse_args()

    run_simple_nav(speed=args.speed, duration=args.duration,
                   backup_threshold=args.backup_threshold, blocked_margin=args.blocked_margin,
                   debug_marker=args.debug_marker)
