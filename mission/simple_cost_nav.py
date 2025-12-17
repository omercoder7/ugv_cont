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
                 backup_threshold: float = 0.25, blocked_margin: float = 0.15,
                 debug_marker: bool = False):
        self.linear_speed = linear_speed
        self.duration = duration
        self.running = False
        self.debug_marker = debug_marker
        self._marker_thread = None

        # Thresholds
        self.backup_threshold = backup_threshold
        self.blocked_margin = blocked_margin  # Margin for considering goal blocked
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

        # Visited cells (for avoiding revisits)
        self.visited: Dict[Tuple[int, int], int] = {}
        self.grid_res = 0.3

        # Scan boundaries - track where we've seen walls
        # Key: cell, Value: number of times we saw a wall there
        self.scan_ends: Dict[Tuple[int, int], int] = {}

        # Current state
        self.current_pos: Optional[Tuple[float, float]] = None
        self.current_heading: Optional[float] = None

        # Statistics
        self.iteration = 0
        self.start_time = 0.0
        self.goals_reached = 0
        self.backups = 0

    def _start_marker_thread(self):
        """Start background thread that continuously publishes goal marker."""
        def publish_loop():
            # Start a persistent marker publisher inside Docker
            proc = subprocess.Popen(
                ['docker', 'exec', '-i', CONTAINER_NAME, 'bash', '-c',
                 '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from visualization_msgs.msg import Marker
import sys

rclpy.init()
node = rclpy.create_node('goal_marker_persistent')
pub = node.create_publisher(Marker, '/nav_goal', 10)

while True:
    line = sys.stdin.readline().strip()
    if not line:
        break
    try:
        x, y = map(float, line.split(','))
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = node.get_clock().now().to_msg()
        m.ns = 'goal'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.15
        m.pose.orientation.w = 1.0
        m.scale.x = 0.25
        m.scale.y = 0.25
        m.scale.z = 0.25
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.lifetime.sec = 5
        for _ in range(3):
            pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.01)
    except:
        pass

node.destroy_node()
rclpy.shutdown()
"'''],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            while self.running and self.debug_marker:
                if self.goal_point:
                    x, y = self.goal_point
                    try:
                        proc.stdin.write(f"{x},{y}\n".encode())
                        proc.stdin.flush()
                    except:
                        pass
                time.sleep(0.3)  # Publish every 0.3s

            proc.terminate()

        self._marker_thread = threading.Thread(target=publish_loop, daemon=True)
        self._marker_thread.start()

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
                self._record_scan_ends(sectors)

                # Front distance
                front_min = min(sectors[11], sectors[0], sectors[1])
                if front_min < 0.05:
                    front_min = 1.0  # Blind spot, assume clear

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
                        self.goal_point = new_goal
                        self.goal_set_time = time.time()
                        self.locked_heading = None  # Clear heading lock for new goal
                        dist = math.sqrt((new_goal[0] - self.current_pos[0])**2 +
                                        (new_goal[1] - self.current_pos[1])**2)

                        # Publish goal marker to RViz (green sphere)
                        publish_goal_marker(new_goal[0], new_goal[1])

                        # Print polar map with goal direction
                        print(f"\n{'='*40}")
                        print(f"[GOAL] New goal: ({new_goal[0]:.2f}, {new_goal[1]:.2f})")
                        print(f"       dist={dist:.2f}m, sector={goal_sector}, reason={reason}")
                        print(f"       yaw={math.degrees(self.current_heading):.0f}°")
                        print(f"{'='*40}")
                        print(self._draw_polar_map(sectors, goal_sector))
                        print(f"{'='*40}")
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
            send_velocity_cmd(0.0, 0.0)
            self._print_stats()

    def _select_goal_point(self, sectors: List[float]) -> Tuple[Optional[Tuple[float, float]], Optional[int]]:
        """
        Select goal point that maximizes expected visibility gain.

        Criteria:
        1. Must be in line-of-sight (LiDAR can see it)
        2. Should NOT be at a "scan end" (wall boundary)
        3. Area around it should be open (good visibility from there)
        4. Prefer unexplored areas

        Returns: (goal_point, goal_sector) or (None, None)
        """
        if not self.current_pos or self.current_heading is None:
            return None, None

        best_point = None
        best_score = -999
        best_sector = None

        # Evaluate candidate points in FRONT hemisphere only (±90° from front)
        # Front sectors: 0, 1, 2, 3 (left-front) and 9, 10, 11 (right-front)
        front_sectors = [0, 1, 2, 3, 9, 10, 11]

        for sector_idx in front_sectors:
            sector_dist = sectors[sector_idx]

            if sector_dist < self.min_goal_dist + 0.2:
                # Sector too blocked
                continue

            # Don't place goal at the wall - leave margin
            max_dist = min(sector_dist - 0.3, self.max_goal_dist)

            # Try points at different distances
            for dist in [0.5, 0.8, 1.0, 1.2, 1.5]:
                if dist < self.min_goal_dist or dist > max_dist:
                    continue

                # Calculate world position of candidate point
                angle = self._sector_to_angle(sector_idx)
                world_angle = self.current_heading + angle
                px = self.current_pos[0] + dist * math.cos(world_angle)
                py = self.current_pos[1] + dist * math.sin(world_angle)

                # Score this viewpoint
                score = self._score_viewpoint(px, py, sector_idx, dist, sectors)

                if score > best_score:
                    best_score = score
                    best_point = (px, py)
                    best_sector = sector_idx

        return best_point, best_sector

    def _score_viewpoint(self, px: float, py: float, sector_idx: int,
                         dist: float, sectors: List[float]) -> float:
        """
        Score a potential viewpoint.

        Higher score = better viewpoint (more new area visible from there)
        """
        score = 0.0

        # === Factor 1: Margin to wall (NOT at a scan end) ===
        # If close to wall, it's a poor viewpoint
        margin_to_wall = sectors[sector_idx] - dist
        if margin_to_wall < 0.2:
            score -= 3.0  # Heavy penalty for being at a wall
        elif margin_to_wall < 0.4:
            score -= 1.0
        else:
            score += margin_to_wall * 0.5  # Bonus for open space ahead

        # === Factor 2: Adjacent sectors open (not a corner/corridor end) ===
        # A good viewpoint has open space in multiple directions
        openness = 0
        for adj in [-2, -1, 1, 2]:
            adj_sector = (sector_idx + adj) % NUM_SECTORS
            adj_dist = sectors[adj_sector]
            if adj_dist > 0.5:
                openness += min(adj_dist, 1.5)
        score += openness * 0.3

        # === Factor 3: NOT a known scan end (previous wall boundary) ===
        point_cell = self._pos_to_cell(px, py)
        scan_end_count = self.scan_ends.get(point_cell, 0)
        if scan_end_count > 0:
            score -= scan_end_count * 1.0  # Penalize known boundaries

        # Check neighboring cells for scan ends too
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor = (point_cell[0] + dx, point_cell[1] + dy)
                score -= self.scan_ends.get(neighbor, 0) * 0.3

        # === Factor 4: Prefer unvisited areas (DOMINANT FACTOR) ===
        visit_count = self.visited.get(point_cell, 0)
        if visit_count == 0:
            score += 10.0  # Very strong bonus for unexplored
        else:
            score -= visit_count * 2.0  # Strong penalty for revisiting

        # Also check neighboring cells for unvisited areas
        unvisited_neighbors = 0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor = (point_cell[0] + dx, point_cell[1] + dy)
                if self.visited.get(neighbor, 0) == 0:
                    unvisited_neighbors += 1
        score += unvisited_neighbors * 1.0  # Bonus for being near unexplored

        # === Factor 5: Distance efficiency ===
        # Prefer further goals (more efficient exploration)
        score += dist * 0.5

        return score

    def _record_scan_ends(self, sectors: List[float]):
        """Record where LiDAR hits walls (scan boundaries)."""
        if not self.current_pos or self.current_heading is None:
            return

        for sector_idx in range(NUM_SECTORS):
            dist = sectors[sector_idx]

            # Only record actual walls (not max range)
            if dist < 0.1 or dist > 3.0:
                continue

            # Calculate wall position
            angle = self._sector_to_angle(sector_idx)
            world_angle = self.current_heading + angle
            wall_x = self.current_pos[0] + dist * math.cos(world_angle)
            wall_y = self.current_pos[1] + dist * math.sin(world_angle)

            # Record as scan end
            cell = self._pos_to_cell(wall_x, wall_y)
            self.scan_ends[cell] = self.scan_ends.get(cell, 0) + 1

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

        # Turn toward more open side
        back_left = min(sectors[3], sectors[4], sectors[5])
        back_right = min(sectors[7], sectors[8], sectors[9])

        turn_w = 0.5 if back_left > back_right else -0.5
        print(f"[BACKUP] Turning {'left' if turn_w > 0 else 'right'}")

        for _ in range(10):
            send_velocity_cmd(0.0, turn_w)
            time.sleep(0.1)

        send_velocity_cmd(0.0, 0.0)

        # Clear heading lock after backup
        self.locked_heading = None

    def _draw_polar_map(self, sectors: List[float], goal_sector: Optional[int] = None) -> str:
        """
        Draw a unicode polar map showing LiDAR readings and robot heading.

        Returns a string visualization of the robot's surroundings.
        """
        # Sector layout (12 sectors, 30° each):
        #          0 (front)
        #      11      1
        #    10          2
        #    9            3
        #      8      4
        #          6 (back)
        #        7   5

        lines = []

        # Convert distances to simple indicators
        def dist_char(d: float, is_goal: bool = False) -> str:
            if is_goal:
                return "◎"  # Goal direction
            if d < 0.3:
                return "█"  # Very close (wall)
            elif d < 0.5:
                return "▓"  # Close
            elif d < 0.8:
                return "▒"  # Medium
            elif d < 1.2:
                return "░"  # Far
            else:
                return "·"  # Very far / open

        def dist_str(d: float) -> str:
            if d > 9.9:
                return "9.9"
            return f"{d:.1f}"

        # Build the map
        s = sectors
        g = goal_sector

        # Row 1: Front
        c0 = dist_char(s[0], g == 0)
        lines.append(f"        {c0} {dist_str(s[0])}m")
        lines.append(f"       [0]")

        # Row 2: Front-left and front-right
        c11 = dist_char(s[11], g == 11)
        c1 = dist_char(s[1], g == 1)
        lines.append(f"    {c11}         {c1}")
        lines.append(f"   [11]{dist_str(s[11])}   {dist_str(s[1])}[1]")

        # Row 3: Left and right
        c10 = dist_char(s[10], g == 10)
        c2 = dist_char(s[2], g == 2)
        lines.append(f"  {c10}      ↑      {c2}")
        lines.append(f" [10]{dist_str(s[10])}   │   {dist_str(s[2])}[2]")

        # Row 4: Side
        c9 = dist_char(s[9], g == 9)
        c3 = dist_char(s[3], g == 3)
        lines.append(f"  {c9}      ●      {c3}")
        lines.append(f" [9] {dist_str(s[9])}       {dist_str(s[3])}[3]")

        # Row 5: Back-side
        c8 = dist_char(s[8], g == 8)
        c4 = dist_char(s[4], g == 4)
        lines.append(f"  {c8}             {c4}")
        lines.append(f" [8] {dist_str(s[8])}       {dist_str(s[4])}[4]")

        # Row 6: Back
        c7 = dist_char(s[7], g == 7)
        c5 = dist_char(s[5], g == 5)
        c6 = dist_char(s[6], g == 6)
        lines.append(f"    {c7}    {c6}    {c5}")
        lines.append(f"   [7]  [6]  [5]")
        lines.append(f"   {dist_str(s[7])} {dist_str(s[6])} {dist_str(s[5])}")

        # Legend
        lines.append(f"")
        lines.append(f"Legend: █<0.3m ▓<0.5m ▒<0.8m ░<1.2m ·>1.2m ◎=goal")

        return "\n".join(lines)

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
        print(f"Scan ends recorded: {len(self.scan_ends)}")

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
