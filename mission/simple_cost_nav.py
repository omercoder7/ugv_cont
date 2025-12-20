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
from .constants import NUM_SECTORS, CONTAINER_NAME, SECTORS_FRONT_ARC, SECTORS_LEFT, SECTORS_RIGHT


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
        self.goal_reached_dist = 0.40  # Consider goal reached if within 40cm (larger radius)
        self.min_goal_dist = 0.5       # Don't set goals closer than this
        self.max_goal_dist = 2.0       # Don't set goals further than this (prefer far points)

        # Goal tracking
        self.goal_point: Optional[Tuple[float, float]] = None
        self.goal_set_time: float = 0
        self.goal_initial_dist: float = 0.0  # Distance when goal was set (for timeout calc)
        self.goal_timeout_base: float = 15.0  # Base timeout per meter of distance (longer)
        self.goal_timeout_min: float = 20.0   # Minimum timeout regardless of distance
        self.goal_timeout_extended: bool = False  # Track if we've already extended once

        # Heading lock for drift correction
        # When aligned and driving straight, lock heading and continuously correct
        self.locked_heading: Optional[float] = None
        self.heading_lock_gain = 1.5  # Stronger correction for drift

        # Visited cells (for avoiding revisits) - tracks robot position history
        # Key: cell, Value: timestamp of last visit (for time-decay penalty)
        self.visited: Dict[Tuple[int, int], float] = {}
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
        self.start_pos = None  # Track starting position

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

                # Track starting position
                if self.start_pos is None:
                    self.start_pos = self.current_pos
                    print(f"[START] Initial position: ({self.start_pos[0]:.2f}, {self.start_pos[1]:.2f})")

                # Record visit with timestamp (for time-decay penalty)
                cell = self._pos_to_cell(odom[0], odom[1])
                self.visited[cell] = time.time()

                # Record scan coverage using raw LiDAR data (~500 rays for fine resolution)
                self._record_scan_coverage(scan)

                # Front distance (use SECTORS_FRONT_ARC from constants)
                front_min = min(sectors[s] for s in SECTORS_FRONT_ARC)
                if front_min < 0.05:
                    front_min = 0.05  # Blind spot - treat as very close, trigger backup

                # === 2. CHECK IF NEED NEW GOAL ===
                need_new_goal = False
                goal_blocked = False

                if self.goal_point is None:
                    need_new_goal = True
                    reason = "no goal"
                elif self._goal_reached():
                    need_new_goal = True
                    reason = "reached"
                    self.goals_reached += 1
                elif self._goal_blocked(sectors):
                    # Don't select new goal - keep current goal and try to navigate around
                    goal_blocked = True
                    self.locked_heading = None  # Clear heading lock to allow turning
                else:
                    # Check timeout - distance-based with possible extension
                    # Longer timeouts: min 20s + 15s per meter
                    timeout = max(self.goal_timeout_min,
                                  self.goal_initial_dist * self.goal_timeout_base + self.goal_timeout_min)
                    elapsed = time.time() - self.goal_set_time
                    if elapsed > timeout:
                        # Check if we're making progress (less than half distance remaining)
                        current_dist = math.sqrt((self.goal_point[0] - self.current_pos[0])**2 +
                                                (self.goal_point[1] - self.current_pos[1])**2)
                        if current_dist < self.goal_initial_dist * 0.5 and not self.goal_timeout_extended:
                            # We're close - extend timeout once more
                            self.goal_set_time = time.time()
                            self.goal_timeout_extended = True
                            elapsed_total = time.time() - self.start_time
                            print(f"\n[{elapsed_total:5.1f}s] [TIMEOUT] Extended - close to goal ({current_dist:.2f}m remaining)")
                        else:
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
                        self.goal_initial_dist = dist  # Store initial distance for timeout calc
                        self.goal_timeout_extended = False  # Reset extension flag

                        # Publish goal marker to RViz (green sphere)
                        publish_goal_marker(new_goal[0], new_goal[1])

                        # Calculate timeout for this goal
                        timeout = max(self.goal_timeout_min,
                                      dist * self.goal_timeout_base + self.goal_timeout_min)
                        # Print goal info with timestamp
                        elapsed_total = time.time() - self.start_time
                        print(f"[{elapsed_total:5.1f}s] [GOAL] Selected: ({new_goal[0]:.2f}, {new_goal[1]:.2f}) "
                              f"dist={dist:.2f}m timeout={timeout:.0f}s angle={goal_sector}° reason={reason}")
                    else:
                        # No valid goal found - backup
                        elapsed_total = time.time() - self.start_time
                        print(f"\n[{elapsed_total:5.1f}s] [NO GOAL] No valid viewpoint found, backing up")
                        self._backup(sectors)
                        self.backups += 1
                        continue

                # === 3. BACKUP if too close ===
                if front_min < self.backup_threshold:
                    elapsed_total = time.time() - self.start_time
                    print(f"\n[{elapsed_total:5.1f}s] [BACKUP] front={front_min:.2f}m")
                    self._backup(sectors)
                    self.backups += 1
                    with self._goal_lock:
                        self.goal_point = None  # Force replan
                    continue

                # === 4. DRIVE TO GOAL ===
                if self.goal_point:
                    if goal_blocked:
                        # Goal is blocked - turn toward goal direction to find a way around
                        v, w = self._compute_blocked_turn(sectors)
                        status_mode = "TURN"
                    else:
                        v, w = self._compute_velocity_to_goal(front_min)
                        status_mode = "DRIVE"
                    send_velocity_cmd(v, w)

                    # Status
                    goal_dist = math.sqrt((self.goal_point[0] - self.current_pos[0])**2 +
                                         (self.goal_point[1] - self.current_pos[1])**2)
                    pos_str = f"({self.current_pos[0]:.2f},{self.current_pos[1]:.2f})"
                    goal_str = f"({self.goal_point[0]:.2f},{self.goal_point[1]:.2f})"
                    status = f"[{self.iteration:3d}] {status_mode} goal={goal_str} dist={goal_dist:.2f}m front={front_min:.2f}m cells={len(self.visited)}"
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

        Uses adaptive angular refinement: if <5 candidates found, resample
        with 5x more angles. Repeat until ≥5 candidates or max refinement reached.

        Returns: (goal_point, goal_sector) or (None, None)
        """
        if not self.current_pos or self.current_heading is None:
            return None, None

        # Clean up old goals from recent_goals list
        now = time.time()
        self.recent_goals = [(cell, t) for (cell, t) in self.recent_goals
                            if now - t < self.recent_goal_memory]

        # Angular range: -150° to +150° (300° total, excluding back blind spot)
        angle_start = -150.0 * math.pi / 180.0  # -150° = right-back
        angle_end = 150.0 * math.pi / 180.0     # +150° = left-back

        # Generate equally spaced distances from min to max goal distance
        num_distances = 4
        dist_step = (self.max_goal_dist - self.min_goal_dist) / (num_distances - 1)
        candidate_distances = [self.min_goal_dist + i * dist_step for i in range(num_distances)]

        # Adaptive angular refinement parameters
        min_candidates = 5       # Minimum candidates we want
        base_num_angles = 25     # Starting number of angles
        refinement_factor = 5    # Multiply angles by this factor each iteration
        max_refinements = 3      # Max iterations (25 -> 125 -> 625 -> 3125)

        candidates = []
        num_angles = base_num_angles
        refinement_level = 0

        # Keep refining until we have enough candidates or hit max refinements
        while len(candidates) < min_candidates and refinement_level <= max_refinements:
            if refinement_level > 0:
                # Only show message if we're actually refining
                elapsed_total = time.time() - self.start_time if self.start_time > 0 else 0
                print(f"\n[{elapsed_total:5.1f}s] [REFINE] Only {len(candidates)} candidates, "
                      f"resampling with {num_angles} angles (level {refinement_level})")
                candidates = []  # Clear previous candidates for fresh sampling

            angle_step = (angle_end - angle_start) / (num_angles - 1)

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

                # Don't place goal at the wall - leave larger margin for safety
                max_dist = min(sector_dist - 0.5, self.max_goal_dist)

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

            # Prepare for next refinement if needed
            refinement_level += 1
            num_angles *= refinement_factor

        # Find best candidate
        best_point = None
        best_score = -999
        best_sector = None

        for score, angle_deg, dist, px, py, breakdown in candidates:
            if score > best_score:
                best_score = score
                best_point = (px, py)
                best_sector = angle_deg

        # Debug output: show top candidates
        if candidates:
            candidates.sort(key=lambda x: x[0], reverse=True)
            pos_str = f"({self.current_pos[0]:.2f},{self.current_pos[1]:.2f})" if self.current_pos else "?"
            elapsed_total = time.time() - self.start_time if self.start_time > 0 else 0
            refine_note = f" (refined {refinement_level - 1}x)" if refinement_level > 1 else ""
            print(f"\n[{elapsed_total:5.1f}s] [GOAL SELECT] robot@{pos_str} | {len(candidates)} candidates{refine_note} "
                  f"(scanned={len(self.scanned)} walls={len(self.scan_ends)} visited={len(self.visited)}):")
            for i, (score, ang, d, px, py, brk) in enumerate(candidates[:8]):
                marker = " <-- BEST" if i == 0 else ""
                print(f"  #{i+1} ang={ang:+4d}° d={d:.2f}m pos=({px:.2f},{py:.2f}) "
                      f"score={score:+.1f} [O={brk['open']:+.1f} U={brk['unscan']:+.1f} "
                      f"V={brk['visited']:+.1f} W={brk['wall']:+.1f} R={brk['recent']:+.1f} F={brk['fwd']:+.1f}]{marker}")

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
        breakdown = {'open': 0.0, 'unscan': 0.0, 'recent': 0.0, 'wall': 0.0, 'visited': 0.0, 'dist': 0.0, 'fwd': 0.0}

        # === Factor 1: OPENNESS (safety margin, not dominant) ===
        # Only penalize if we'd be dangerously close to walls ahead
        # Don't penalize narrow corridors on the sides - those are fine to traverse
        openness_score = 0.0
        margin_to_wall = sectors[sector_idx] - dist
        if margin_to_wall < 0.2:
            # Too close to wall ahead - dangerous
            openness_score -= 5.0
        elif margin_to_wall > 0.5:
            # Good clearance ahead - small bonus
            openness_score += min(margin_to_wall, 1.5)

        # Check FORWARD clearance (sectors near target direction) - this matters
        # Side clearance doesn't matter as much for corridors
        for adj in [-1, 1]:  # Only immediate neighbors, not full arc
            adj_sector = (sector_idx + adj) % NUM_SECTORS
            adj_dist = sectors[adj_sector]
            if adj_dist > 0.5:
                openness_score += 0.5  # Small bonus for forward clearance

        breakdown['open'] = openness_score

        # === Factor 2: UNSCANNED AREA with TIME DECAY ===
        # Look BEYOND the candidate point - what new areas would we see from there?
        # Old scans "fade" over time - areas not seen recently become attractive again
        # IMPORTANT: Recently scanned areas should be strongly penalized to prevent loops
        unscanned_score = 0.0
        scan_decay_rate = 0.01  # Slower decay - scans stay "fresh" longer (~70s half-life)

        # Direction from robot to candidate point
        dir_x = px - self.current_pos[0]
        dir_y = py - self.current_pos[1]
        dir_len = math.sqrt(dir_x*dir_x + dir_y*dir_y)
        if dir_len > 0.01:
            dir_x /= dir_len
            dir_y /= dir_len

        # Check cells BEYOND the candidate point (1-3m further in same direction)
        # These are cells we can't currently see but would see from the candidate
        for look_dist in [1.0, 1.5, 2.0, 2.5, 3.0]:
            look_x = px + dir_x * look_dist
            look_y = py + dir_y * look_dist
            look_cell = self._pos_to_cell(look_x, look_y)

            # Check scan freshness - older scans contribute more to "unscan" score
            scan_time = self.scanned.get(look_cell, 0)
            if scan_time == 0:
                unscanned_score += 1.0  # Never scanned - full bonus
            else:
                time_since_scan = now - scan_time
                # Freshness decays: recently scanned = low bonus, old scan = high bonus
                staleness = 1.0 - math.exp(-scan_decay_rate * time_since_scan)
                unscanned_score += staleness

            # Also check cells to the sides (fan out)
            for side_offset in [-0.5, 0.5]:
                side_x = look_x + (-dir_y) * side_offset  # Perpendicular
                side_y = look_y + dir_x * side_offset
                side_cell = self._pos_to_cell(side_x, side_y)
                scan_time = self.scanned.get(side_cell, 0)
                if scan_time == 0:
                    unscanned_score += 0.5
                else:
                    time_since_scan = now - scan_time
                    staleness = 1.0 - math.exp(-scan_decay_rate * time_since_scan)
                    unscanned_score += 0.5 * staleness

        # Also give bonus for cells immediately around candidate that are unscanned/stale
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor = (point_cell[0] + dx, point_cell[1] + dy)
                scan_time = self.scanned.get(neighbor, 0)
                if scan_time == 0:
                    unscanned_score += 0.3
                else:
                    time_since_scan = now - scan_time
                    staleness = 1.0 - math.exp(-scan_decay_rate * time_since_scan)
                    unscanned_score += 0.3 * staleness

        # Higher weight - unscanned areas are the PRIMARY exploration driver
        # This must dominate over other factors to prevent loops
        breakdown['unscan'] = unscanned_score * 4.0

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
        # Light penalty for being adjacent - corridors have walls nearby, that's OK
        wall_penalty = 0.0
        if self.scan_ends.get(point_cell, 0) > 0:
            wall_penalty = 10.0  # On wall (shouldn't happen, filtered)
        else:
            # Count adjacent walls - mild penalty, don't block corridor exploration
            adjacent_walls = 0
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    neighbor = (point_cell[0] + dx, point_cell[1] + dy)
                    if self.scan_ends.get(neighbor, 0) > 0:
                        adjacent_walls += 1
            # Only penalize if surrounded by walls (dead end), not for corridor sides
            if adjacent_walls >= 4:
                wall_penalty = 3.0  # Likely a dead end
            elif adjacent_walls >= 2:
                wall_penalty = 1.0  # Corridor - minor penalty
        breakdown['wall'] = -wall_penalty

        # === Factor 5: VISITED PENALTY with TIME DECAY ===
        # Pheromone-style decay: recent visits penalize more, old visits fade away
        # This allows re-exploration of areas after enough time has passed
        # Formula: penalty = base_penalty * e^(-λ * time_since_visit)
        # λ = decay_rate: higher = faster decay, 0.1 means ~10s half-life
        visited_penalty = 0.0
        visit_time = self.visited.get(point_cell, 0)
        if visit_time > 0:
            time_since_visit = now - visit_time
            decay_rate = 0.05  # Decay constant: penalty halves every ~14 seconds
            decay_factor = math.exp(-decay_rate * time_since_visit)
            # Base penalty of 8.0, decays over time
            visited_penalty = 8.0 * decay_factor
        breakdown['visited'] = -visited_penalty

        # === Factor 6: DISTANCE EFFICIENCY ===
        breakdown['dist'] = dist * 0.3

        # === Factor 7: FORWARD MOMENTUM ===
        # Prefer continuing in the direction we were heading, not backtracking
        # This prevents oscillating between areas
        fwd_bonus = 0.0
        if self.goal_point and self.start_pos:
            # Direction from start to current position (overall exploration direction)
            explore_dx = self.current_pos[0] - self.start_pos[0]
            explore_dy = self.current_pos[1] - self.start_pos[1]
            explore_len = math.sqrt(explore_dx*explore_dx + explore_dy*explore_dy)

            if explore_len > 0.3:  # Only if we've moved meaningfully from start
                explore_dx /= explore_len
                explore_dy /= explore_len

                # Direction to candidate
                cand_dx = px - self.current_pos[0]
                cand_dy = py - self.current_pos[1]
                cand_len = math.sqrt(cand_dx*cand_dx + cand_dy*cand_dy)
                if cand_len > 0.1:
                    cand_dx /= cand_len
                    cand_dy /= cand_len

                    # Dot product: +1 = same direction, -1 = opposite
                    dot = explore_dx * cand_dx + explore_dy * cand_dy

                    # Bonus for forward, penalty for backward
                    # dot=1 -> +5, dot=0 -> 0, dot=-1 -> -5
                    fwd_bonus = dot * 5.0

        breakdown['fwd'] = fwd_bonus

        total = sum(breakdown.values())
        return total, breakdown

    def _record_scan_coverage(self, raw_scan: List[float]):
        """
        Record LiDAR scan coverage using raw scan data for fine resolution.

        Uses all ~500 raw LiDAR rays instead of 12 sectors, giving ~0.7° resolution
        instead of 30° resolution. This ensures proper scan coverage tracking
        while keeping 12 sectors for navigation obstacle detection.
        """
        if not self.current_pos or self.current_heading is None:
            return

        if not raw_scan:
            return

        now = time.time()
        n_points = len(raw_scan)

        # LiDAR rotation offset: physical mounting puts LiDAR 0° at robot's back-left
        # Same offset as LIDAR_ROTATION_SECTORS but in radians
        # 3 sectors * (2π/12) = π/2 = 90°
        lidar_rotation_rad = 3 * (2 * math.pi / 12)  # 90° offset

        # Angle increment per LiDAR point
        angle_per_point = 2 * math.pi / n_points

        # Process each raw LiDAR ray
        for i, dist in enumerate(raw_scan):
            # Filter invalid readings (NaN, inf, out of range)
            if not (0.1 < dist < 10.0):
                continue

            # LiDAR-frame angle for this point
            lidar_angle = i * angle_per_point

            # Convert to robot frame (apply rotation offset)
            robot_angle = lidar_angle + lidar_rotation_rad

            # Convert to world frame
            world_angle = self.current_heading + robot_angle

            # Record all cells along the scan ray as "scanned"
            # Use smaller step for better coverage (half of grid resolution)
            scan_dist = min(dist, 3.0)  # Cap at 3m
            step = self.grid_res * 0.5  # Smaller step for better ray coverage
            num_steps = max(1, int(scan_dist / step))

            # Start from step 0 to include cells near robot
            for j in range(num_steps + 1):
                d = j * step
                if d < 0.1:
                    continue  # Skip cells too close to robot
                px = self.current_pos[0] + d * math.cos(world_angle)
                py = self.current_pos[1] + d * math.sin(world_angle)
                cell = self._pos_to_cell(px, py)
                self.scanned[cell] = now

            # Also mark the exact endpoint
            end_x = self.current_pos[0] + scan_dist * math.cos(world_angle)
            end_y = self.current_pos[1] + scan_dist * math.sin(world_angle)
            end_cell = self._pos_to_cell(end_x, end_y)
            self.scanned[end_cell] = now

            # Record wall position (scan end) if not max range
            if dist < 3.0:
                wall_x = self.current_pos[0] + dist * math.cos(world_angle)
                wall_y = self.current_pos[1] + dist * math.sin(world_angle)
                wall_cell = self._pos_to_cell(wall_x, wall_y)
                self.scan_ends[wall_cell] = now

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

    def _compute_blocked_turn(self, sectors: List[float]) -> Tuple[float, float]:
        """
        When goal is blocked, turn toward the goal direction while avoiding obstacles.

        Find the best open sector that is closest to the goal direction and turn toward it.
        This allows the robot to navigate around obstacles while keeping the goal in mind.
        """
        if not self.goal_point or not self.current_pos or self.current_heading is None:
            return 0.0, 0.0

        # Calculate angle to goal in robot frame
        dx = self.goal_point[0] - self.current_pos[0]
        dy = self.goal_point[1] - self.current_pos[1]
        goal_world_angle = math.atan2(dy, dx)
        goal_robot_angle = self._normalize_angle(goal_world_angle - self.current_heading)
        goal_sector = self._angle_to_sector(goal_robot_angle)

        # Find the best open sector near the goal direction
        # Check sectors in expanding order from goal direction
        best_sector = None
        best_dist = None
        min_clearance = 0.5  # Need at least 0.5m clearance

        for offset in range(NUM_SECTORS):
            for direction in [1, -1]:  # Check both sides
                check_sector = (goal_sector + offset * direction) % NUM_SECTORS
                if sectors[check_sector] > min_clearance:
                    if best_sector is None:
                        best_sector = check_sector
                        best_dist = sectors[check_sector]
                        break
            if best_sector is not None:
                break

        if best_sector is None:
            # No open sector found, just rotate toward goal
            w = 0.5 if goal_robot_angle > 0 else -0.5
            return 0.0, w

        # Turn toward the best open sector
        target_angle = self._sector_to_angle(best_sector)
        angle_error = self._normalize_angle(target_angle)

        # Proportional turn with no forward motion
        if abs(angle_error) > 0.1:
            w = 0.6 if angle_error > 0 else -0.6
        else:
            # Aligned with open sector, creep forward slowly
            w = angle_error * 1.0

        # Small forward motion if we have some clearance ahead
        front_min = min(sectors[11], sectors[0], sectors[1])
        v = 0.03 if front_min > 0.3 else 0.0

        return v, w

    def _backup(self, sectors: List[float]):
        """Backup maneuver."""
        print(f"\n[BACKUP] Reversing...")

        # Reverse
        for _ in range(10):
            send_velocity_cmd(-0.05, 0.0)
            time.sleep(0.1)

        # Turn toward more open side (use SECTORS_LEFT/RIGHT from constants)
        front_left = min(sectors[s] for s in SECTORS_LEFT)
        front_right = min(sectors[s] for s in SECTORS_RIGHT)

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

