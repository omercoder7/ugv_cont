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
import heapq
import os
import sys
from datetime import datetime
from enum import Enum
from typing import Optional, List, Tuple, Dict, Set, TextIO

from .ros_interface import get_lidar_scan, send_velocity_cmd, get_odometry, publish_goal_marker


class TeeLogger:
    """Write to both console and file simultaneously."""
    def __init__(self, log_file: TextIO):
        self.terminal = sys.stdout
        self.log_file = log_file

    def write(self, message):
        self.terminal.write(message)
        self.log_file.write(message)
        self.log_file.flush()

    def flush(self):
        self.terminal.flush()
        self.log_file.flush()


from .avoider.lidar import compute_sector_distances
from .constants import NUM_SECTORS, CONTAINER_NAME, SECTORS_FRONT_ARC, SECTORS_LEFT, SECTORS_RIGHT


class NavigationState(Enum):
    """Navigation state machine states."""
    EXPLORING = "exploring"  # Normal NBV exploration
    RETURNING = "returning"  # Returning to origin


class NBVNavigator:
    """
    Next-Best-View navigation.

    Selects goal points that maximize expected visibility gain,
    avoiding "scan ends" (walls, boundaries).
    """

    def __init__(self, linear_speed: float = 0.08, duration: float = 60.0,
                 backup_threshold: float = 0.15, blocked_margin: float = 0.05,
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

        # Velocity smoothing - track previous commands for ramping
        self.last_v = 0.0
        self.last_w = 0.0
        self.max_linear_accel = 0.03   # Max linear velocity change per 0.1s cycle
        self.max_angular_accel = 0.25  # Max angular velocity change per 0.1s cycle

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

        # Navigation state machine
        self.nav_state = NavigationState.EXPLORING
        self.return_trigger_ratio = 0.75  # Trigger return when 75% of time elapsed

        # Return navigation tracking
        self.return_start_time: float = 0.0  # When return phase started
        self.return_stuck_time: float = 0.0  # Time spent stuck during return
        self.return_stuck_threshold: float = 8.0  # Seconds with no movement/rotation before stuck
        self.return_last_progress_time: float = 0.0  # Last time we made progress toward origin
        self.return_last_dist: float = 0.0  # Last distance to origin (for progress detection)
        self.return_last_pos: Optional[Tuple[float, float]] = None  # Last position for movement detection
        self.return_last_heading: Optional[float] = None  # Last heading for rotation detection
        self.return_movement_threshold: float = 0.10  # Min movement (m) to count as "not stuck"

        # Area-based progress tracking for return
        # Triggers replan if robot stays in same area without getting closer to origin
        self.return_area_center: Optional[Tuple[float, float]] = None  # Center of current area
        self.return_area_time: float = 0.0  # When we entered this area
        self.return_area_radius: float = 0.5  # Radius to consider "same area" (0.5m)
        self.return_area_timeout: float = 15.0  # Replan if in same area for 15s without progress
        self.return_area_best_dist: float = float('inf')  # Best distance to origin while in this area
        self.return_rotation_threshold: float = 0.15  # Min rotation (rad, ~9°) to count as "not stuck"

        # Origin marker subprocess
        self._origin_marker_proc = None

        # A* path planning for return navigation
        self.return_path: List[Tuple[float, float]] = []  # Waypoints to follow
        self.return_waypoint_idx: int = 0  # Current waypoint index
        self.waypoint_reached_dist: float = 0.35  # Distance to consider waypoint reached
        self.origin_reached_dist: float = 0.15  # Tighter threshold for reaching origin (15cm)
        self.path_inflation_radius: int = 2  # Grid cells to inflate walls (2 cells = 0.6m clearance)
        self.goal_clearance_radius: int = 3  # Clear cells within this radius of goal from obstacles

        # Unreachable waypoints - cells that A* should treat as walls
        # When robot gets stuck trying to reach a waypoint, mark it as unreachable
        # Key: cell tuple, Value: timestamp when added (for debugging/expiry)
        self.unreachable_cells: Dict[Tuple[int, int], float] = {}

        # Consecutive goal selection failure tracking
        self.consecutive_no_goal: int = 0  # Count of consecutive "no valid goal" failures
        self.no_goal_turn_threshold: int = 3  # After this many failures, do a 180° turn

        # Turn mode stuck detection (for exploration)
        self.turn_mode_start_time: float = 0.0  # When we entered TURN mode
        self.turn_mode_timeout: float = 5.0  # Max time in TURN mode before forcing backup
        self.was_in_turn_mode: bool = False  # Track if we were in TURN mode last iteration

        # DRIVE mode stuck detection - sliding window on goal distance
        # Track recent goal distance readings to detect when robot isn't making progress
        self.drive_goal_dist_history: List[Tuple[float, float]] = []  # List of (timestamp, goal_distance)
        self.drive_stuck_window: float = 7.0  # Check for no progress in last 7 seconds
        self.drive_stuck_margin: float = 0.08  # Must change by 8cm to count as progress

        # Blocked sectors tracking - mark sectors as blocked for a duration
        # Key: sector index, Value: timestamp when block expires
        self.blocked_sectors: Dict[int, float] = {}

        # Output logging
        self.output_dir: Optional[str] = None
        self.log_file: Optional[TextIO] = None
        self._original_stdout = None

        # Drive accuracy tracking
        # Track deviation from ideal straight-line path to goal
        self.drive_error_sum_sq: float = 0.0  # Sum of squared cross-track errors
        self.drive_error_count: int = 0  # Number of measurements
        self.drive_total_distance: float = 0.0  # Total distance traveled
        self.drive_ideal_distance: float = 0.0  # Sum of straight-line distances to goals
        self.last_pos_for_distance: Optional[Tuple[float, float]] = None

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

    def _start_origin_marker(self, x: float, y: float):
        """Start a persistent blue marker at the origin position."""
        self._origin_marker_proc = subprocess.Popen(
            ['docker', 'exec', '-i', CONTAINER_NAME, 'bash', '-c',
             f'''source /opt/ros/humble/setup.bash && python3 -u -c "
import rclpy
from visualization_msgs.msg import Marker
import time

rclpy.init()
node = rclpy.create_node('origin_marker')
pub = node.create_publisher(Marker, '/nav_origin', 10)

while rclpy.ok():
    m = Marker()
    m.header.frame_id = 'odom'
    m.header.stamp = node.get_clock().now().to_msg()
    m.ns = 'origin'
    m.id = 0
    m.type = 2  # Sphere
    m.action = 0
    m.pose.position.x = {x}
    m.pose.position.y = {y}
    m.pose.position.z = 0.15
    m.pose.orientation.w = 1.0
    m.scale.x = 0.30
    m.scale.y = 0.30
    m.scale.z = 0.30
    m.color.b = 1.0  # Blue
    m.color.a = 1.0
    m.lifetime.sec = 2
    pub.publish(m)
    rclpy.spin_once(node, timeout_sec=0.1)
    time.sleep(0.5)

node.destroy_node()
rclpy.shutdown()
"'''],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        print(f"[MARKER] Origin marker (blue) started at ({x:.2f}, {y:.2f}) - add /nav_origin in RViz")

    def _stop_origin_marker(self):
        """Stop the origin marker subprocess."""
        if self._origin_marker_proc:
            try:
                self._origin_marker_proc.terminate()
                self._origin_marker_proc.wait(timeout=2)
            except Exception:
                try:
                    self._origin_marker_proc.kill()
                except Exception:
                    pass
            self._origin_marker_proc = None

    def _plan_path_astar(self, start: Tuple[float, float],
                         goal: Tuple[float, float],
                         inflation: int = None) -> List[Tuple[float, float]]:
        """
        Plan a path from start to goal using A* algorithm.

        Uses internal map data:
        - self.scan_ends: Wall cells (obstacles)
        - self.scanned: Traversable cells (known free space)
        - self.visited: Cells the robot has been in (definitely traversable)

        Inflates walls by inflation radius to ensure robot clearance.
        If path fails, automatically retries with reduced inflation.

        Returns: List of waypoints (world coordinates), or empty list if no path found.
        """
        if inflation is None:
            inflation = self.path_inflation_radius

        start_cell = self._pos_to_cell(start[0], start[1])
        goal_cell = self._pos_to_cell(goal[0], goal[1])

        print(f"[A* DEBUG] Planning from {start_cell} to {goal_cell}")
        print(f"[A* DEBUG] World coords: ({start[0]:.2f},{start[1]:.2f}) -> ({goal[0]:.2f},{goal[1]:.2f})")
        print(f"[A* DEBUG] Map stats: {len(self.scan_ends)} walls, {len(self.scanned)} scanned, {len(self.visited)} visited")

        # Build obstacle set with inflation for robot clearance
        obstacles: Set[Tuple[int, int]] = set()
        for wall_cell in self.scan_ends.keys():
            # Add the wall cell and inflated neighbors
            for dx in range(-inflation, inflation + 1):
                for dy in range(-inflation, inflation + 1):
                    # Use Chebyshev distance for rectangular inflation
                    if max(abs(dx), abs(dy)) <= inflation:
                        obstacles.add((wall_cell[0] + dx, wall_cell[1] + dy))

        # Add unreachable cells (waypoints we got stuck trying to reach)
        for unreachable_cell in self.unreachable_cells.keys():
            obstacles.add(unreachable_cell)
            # Also inflate unreachable cells to avoid routing close to them
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    obstacles.add((unreachable_cell[0] + dx, unreachable_cell[1] + dy))

        print(f"[A* DEBUG] Inflated obstacles: {len(obstacles)} cells (inflation={inflation}, unreachable={len(self.unreachable_cells)})")

        # IMPORTANT: Remove visited cells from obstacles - we've already been there!
        visited_cleared = 0
        for visited_cell in self.visited.keys():
            if visited_cell in obstacles:
                obstacles.discard(visited_cell)
                visited_cleared += 1
        if visited_cleared > 0:
            print(f"[A* DEBUG] Cleared {visited_cleared} visited cells from obstacles")

        # IMPORTANT: Clear cells around the goal to ensure we can approach it
        # This allows the final approach to origin even if walls are nearby
        goal_cleared = 0
        for dx in range(-self.goal_clearance_radius, self.goal_clearance_radius + 1):
            for dy in range(-self.goal_clearance_radius, self.goal_clearance_radius + 1):
                clear_cell = (goal_cell[0] + dx, goal_cell[1] + dy)
                if clear_cell in obstacles:
                    obstacles.discard(clear_cell)
                    goal_cleared += 1
        if goal_cleared > 0:
            print(f"[A* DEBUG] Cleared {goal_cleared} cells around goal (radius={self.goal_clearance_radius})")

        # Check if start or goal are in obstacles
        start_in_obs = start_cell in obstacles
        goal_in_obs = goal_cell in obstacles
        print(f"[A* DEBUG] Start in obstacles: {start_in_obs}, Goal in obstacles: {goal_in_obs}")

        # Don't block the start or goal cells
        obstacles.discard(start_cell)
        obstacles.discard(goal_cell)

        # A* algorithm
        def heuristic(cell: Tuple[int, int]) -> float:
            # Euclidean distance heuristic
            return math.sqrt((cell[0] - goal_cell[0])**2 + (cell[1] - goal_cell[1])**2)

        # Priority queue: (f_score, counter, cell)
        # Counter for tie-breaking to avoid comparing tuples
        counter = 0
        open_set = [(heuristic(start_cell), counter, start_cell)]
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {start_cell: 0}
        closed_set: Set[Tuple[int, int]] = set()

        # 8-directional neighbors (including diagonals)
        neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

        # Limit iterations to prevent infinite loops
        max_iterations = 10000
        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1
            _, _, current = heapq.heappop(open_set)

            # Skip if already processed
            if current in closed_set:
                continue
            closed_set.add(current)

            if current == goal_cell:
                # Reconstruct path
                path_cells = [current]
                while current in came_from:
                    current = came_from[current]
                    path_cells.append(current)
                path_cells.reverse()

                print(f"[A* DEBUG] Path found! {len(path_cells)} cells, {iterations} iterations")

                # Convert to world coordinates (center of each cell)
                path_world = []
                for cell in path_cells:
                    wx = (cell[0] + 0.5) * self.grid_res
                    wy = (cell[1] + 0.5) * self.grid_res
                    path_world.append((wx, wy))

                # Simplify path: remove intermediate waypoints on straight lines
                simplified = self._simplify_path(path_world)
                print(f"[A* DEBUG] Simplified to {len(simplified)} waypoints")
                return simplified

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)

                # Skip if already processed
                if neighbor in closed_set:
                    continue

                # Skip if obstacle
                if neighbor in obstacles:
                    continue

                # Diagonal moves cost sqrt(2), orthogonal cost 1
                move_cost = 1.414 if dx != 0 and dy != 0 else 1.0

                # Add wall proximity cost - prefer paths away from walls
                # Check distance to nearest wall and add penalty for being close
                wall_penalty = 0.0
                for wall_cell in self.scan_ends.keys():
                    wall_dx = wall_cell[0] - neighbor[0]
                    wall_dy = wall_cell[1] - neighbor[1]
                    # Quick bounding box check
                    if abs(wall_dx) <= 3 and abs(wall_dy) <= 3:
                        wall_dist = math.sqrt(wall_dx*wall_dx + wall_dy*wall_dy)
                        if wall_dist < 3:  # Within 3 cells (~0.9m)
                            # Exponential penalty: closer = much higher cost
                            # wall_dist=1 -> penalty=2.0, wall_dist=2 -> penalty=0.5
                            wall_penalty = max(wall_penalty, 2.0 / (wall_dist + 0.5))

                tentative_g = g_score[current] + move_cost + wall_penalty

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor)
                    counter += 1
                    heapq.heappush(open_set, (f_score, counter, neighbor))

        # No path found - provide debug info
        print(f"[A* DEBUG] NO PATH FOUND after {iterations} iterations")
        print(f"[A* DEBUG] Explored {len(closed_set)} cells, open set had {len(open_set)} remaining")

        # Find closest cell to goal that we could reach
        if closed_set:
            closest = min(closed_set, key=lambda c: heuristic(c))
            closest_dist = heuristic(closest)
            print(f"[A* DEBUG] Closest reachable cell: {closest}, dist to goal: {closest_dist:.1f} cells")

            # Check what's blocking around the goal
            blocked_neighbors = []
            for dx, dy in neighbors:
                neighbor = (goal_cell[0] + dx, goal_cell[1] + dy)
                if neighbor in obstacles:
                    blocked_neighbors.append(neighbor)
            print(f"[A* DEBUG] Goal neighbors blocked: {len(blocked_neighbors)}/8")

        # If path failed and we have inflation > 0, retry with no inflation
        if inflation > 0:
            print(f"[A* DEBUG] Retrying with inflation=0...")
            return self._plan_path_astar(start, goal, inflation=0)

        return []

    def _simplify_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Simplify path by removing intermediate waypoints where possible.

        Only removes a waypoint if:
        1. Direction change is small (< 11 degrees)
        2. There's clear line-of-sight between the previous and next waypoint

        This ensures the simplified path doesn't cut through walls.
        Also ensures the penultimate waypoint is in verified open space
        for a clear final approach to the goal.
        """
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        angle_threshold = 0.2  # ~11 degrees - keep waypoints with significant direction change

        for i in range(1, len(path) - 1):
            prev = simplified[-1]
            curr = path[i]
            next_pt = path[i + 1]

            # Calculate angles
            angle1 = math.atan2(curr[1] - prev[1], curr[0] - prev[0])
            angle2 = math.atan2(next_pt[1] - curr[1], next_pt[0] - curr[0])
            angle_diff = abs(self._normalize_angle(angle2 - angle1))

            # Keep this waypoint if:
            # 1. Direction changes significantly, OR
            # 2. No clear line-of-sight from prev to next (would cut through walls)
            if angle_diff > angle_threshold:
                simplified.append(curr)
            elif not self._has_line_of_sight_with_margin(prev[0], prev[1], next_pt[0], next_pt[1]):
                # Can't skip this waypoint - path would go through walls
                simplified.append(curr)

        simplified.append(path[-1])

        # Validate all waypoint connections have line-of-sight
        simplified = self._validate_path_connectivity(simplified, path)

        # Ensure penultimate waypoint is in open space for clear final approach
        simplified = self._ensure_penultimate_in_open_space(simplified, path)

        return simplified

    def _has_line_of_sight_with_margin(self, x1: float, y1: float, x2: float, y2: float,
                                        margin_cells: int = 1) -> bool:
        """
        Check line-of-sight with a safety margin around walls.

        Unlike _has_line_of_sight, this also checks adjacent cells to ensure
        the robot (not just a point) can pass through.
        """
        dx = x2 - x1
        dy = y2 - y1
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.01:
            return True

        # Normalize direction
        dx /= dist
        dy /= dist

        # Perpendicular direction for margin checks
        perp_x = -dy
        perp_y = dx

        # March along the line, checking for walls
        step = self.grid_res * 0.5
        num_steps = int(dist / step) + 1

        for i in range(num_steps):
            d = i * step
            px = x1 + dx * d
            py = y1 + dy * d

            # Check center and margins
            for m in range(-margin_cells, margin_cells + 1):
                check_x = px + perp_x * m * self.grid_res * 0.5
                check_y = py + perp_y * m * self.grid_res * 0.5
                cell = self._pos_to_cell(check_x, check_y)

                if cell in self.scan_ends:
                    return False  # Wall blocking path
                if cell in self.unreachable_cells:
                    return False  # Known unreachable cell

        return True

    def _validate_path_connectivity(self, simplified: List[Tuple[float, float]],
                                     original_path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Validate that all consecutive waypoints have clear line-of-sight.

        If any connection is blocked, insert intermediate waypoints from
        the original A* path to ensure connectivity.

        If too many segments are blocked (>50%), just return the original
        unsimplified path since simplification is causing more harm than good.
        """
        if len(simplified) <= 1:
            return simplified

        # First pass: count how many segments are blocked
        blocked_count = 0
        for i in range(1, len(simplified)):
            prev = simplified[i-1]
            curr = simplified[i]
            if not self._has_line_of_sight_with_margin(prev[0], prev[1], curr[0], curr[1]):
                blocked_count += 1

        # If more than 50% of segments are blocked, skip simplification entirely
        # The scan_ends map doesn't match reality - just use raw A* path
        if blocked_count > len(simplified) * 0.5:
            print(f"[A* DEBUG] {blocked_count}/{len(simplified)-1} segments blocked, using unsimplified A* path")
            return original_path

        validated = [simplified[0]]

        for i in range(1, len(simplified)):
            prev = validated[-1]
            curr = simplified[i]

            # Check if direct path is clear
            if self._has_line_of_sight_with_margin(prev[0], prev[1], curr[0], curr[1]):
                validated.append(curr)
            else:
                # Path blocked - find intermediate waypoints from original path
                print(f"[A* DEBUG] Path blocked between ({prev[0]:.2f},{prev[1]:.2f}) and ({curr[0]:.2f},{curr[1]:.2f})")

                # Find the segment in original path that corresponds to this gap
                prev_idx = self._find_closest_path_index(original_path, prev)
                curr_idx = self._find_closest_path_index(original_path, curr)

                if prev_idx < curr_idx:
                    # Add intermediate waypoints from original path
                    for j in range(prev_idx + 1, curr_idx):
                        intermediate = original_path[j]
                        # Only add if it helps (has LOS to previous)
                        if self._has_line_of_sight_with_margin(validated[-1][0], validated[-1][1],
                                                               intermediate[0], intermediate[1]):
                            validated.append(intermediate)

                validated.append(curr)

        return validated

    def _find_closest_path_index(self, path: List[Tuple[float, float]],
                                  point: Tuple[float, float]) -> int:
        """Find the index of the closest point in path to the given point."""
        best_idx = 0
        best_dist = float('inf')

        for i, p in enumerate(path):
            dist = math.sqrt((p[0] - point[0])**2 + (p[1] - point[1])**2)
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        return best_idx

    def _ensure_los_to_waypoint(self, from_pos: Tuple[float, float],
                                 waypoint_idx: int) -> Optional[Tuple[float, float]]:
        """
        Ensure clear line-of-sight from current position to the waypoint.

        If there's no clear line-of-sight, find an intermediate point between
        the current position and the waypoint that DOES have clear line-of-sight.

        Returns:
            The waypoint to drive to (either the original or an intermediate one).
            Returns None if no safe waypoint can be found.
        """
        if not self.return_path or waypoint_idx >= len(self.return_path):
            return None

        target_wp = self.return_path[waypoint_idx]

        # Check if direct line-of-sight exists
        if self._has_line_of_sight_with_margin(from_pos[0], from_pos[1],
                                                target_wp[0], target_wp[1]):
            return target_wp  # Direct path is clear

        # No direct line-of-sight - need to find intermediate point
        # Look for the furthest point along the path that has clear LOS
        elapsed_total = time.time() - self.start_time
        print(f"[{elapsed_total:5.1f}s] [A*] No clear LOS to WP{waypoint_idx} ({target_wp[0]:.2f},{target_wp[1]:.2f}), finding intermediate...")

        # Generate intermediate points between current position and waypoint
        dx = target_wp[0] - from_pos[0]
        dy = target_wp[1] - from_pos[1]
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.1:
            return target_wp  # Too close, just go there

        # Try points at decreasing distances from current position
        # Start at 90% of the way and work backwards in 10% increments
        for fraction in [0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2]:
            intermediate_x = from_pos[0] + dx * fraction
            intermediate_y = from_pos[1] + dy * fraction

            # Check if this intermediate point has clear LOS from current pos
            if self._has_line_of_sight_with_margin(from_pos[0], from_pos[1],
                                                    intermediate_x, intermediate_y):
                # Also ensure this intermediate point is not in a wall
                cell = self._pos_to_cell(intermediate_x, intermediate_y)
                if cell not in self.scan_ends and cell not in self.unreachable_cells:
                    print(f"[{elapsed_total:5.1f}s] [A*] Using intermediate point at {fraction*100:.0f}% ({intermediate_x:.2f},{intermediate_y:.2f})")
                    # Insert this intermediate point into the path
                    self.return_path.insert(waypoint_idx, (intermediate_x, intermediate_y))
                    return (intermediate_x, intermediate_y)

        # If no fraction works, try even smaller steps
        for fraction in [0.15, 0.1, 0.05]:
            intermediate_x = from_pos[0] + dx * fraction
            intermediate_y = from_pos[1] + dy * fraction

            if self._has_line_of_sight_with_margin(from_pos[0], from_pos[1],
                                                    intermediate_x, intermediate_y):
                cell = self._pos_to_cell(intermediate_x, intermediate_y)
                if cell not in self.scan_ends and cell not in self.unreachable_cells:
                    print(f"[{elapsed_total:5.1f}s] [A*] Using very close intermediate at {fraction*100:.0f}% ({intermediate_x:.2f},{intermediate_y:.2f})")
                    self.return_path.insert(waypoint_idx, (intermediate_x, intermediate_y))
                    return (intermediate_x, intermediate_y)

        # No clear path found - but A* validated this path, so trust it
        # The scan_ends map may have stale/inaccurate obstacles
        # Just use the original waypoint and let obstacle avoidance handle it
        print(f"[{elapsed_total:5.1f}s] [A*] No clear LOS, trusting A* path to WP{waypoint_idx}")
        return target_wp

    def _ensure_penultimate_in_open_space(self, simplified: List[Tuple[float, float]],
                                          original_path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Ensure the second-to-last waypoint provides a clear approach to the goal.

        Instead of picking from A* path (which hugs walls), find a point that:
        1. Has line-of-sight to the goal (no walls blocking)
        2. Is in a visited cell (known traversable)
        3. Maximizes clearance from walls along the approach path

        This gives a safe, open final approach to the origin.
        """
        if len(simplified) < 2:
            return simplified

        goal = simplified[-1]

        # Find the best approach point from visited cells
        approach_point = self._find_clear_approach_point(goal)

        if approach_point:
            # Replace path ending with: ... -> approach_point -> goal
            # Keep earlier waypoints that lead toward the approach point
            new_simplified = []

            # Find the waypoint closest to approach_point to connect to
            if len(simplified) > 2:
                # Keep waypoints until we're close to the approach point
                for i, wp in enumerate(simplified[:-1]):  # Exclude goal
                    dist_to_approach = math.sqrt((wp[0] - approach_point[0])**2 +
                                                 (wp[1] - approach_point[1])**2)
                    # Stop adding waypoints once we're within 1m of approach point
                    if dist_to_approach < 1.0:
                        break
                    new_simplified.append(wp)

            # If we have no waypoints yet, add the first one
            if not new_simplified and len(simplified) > 0:
                new_simplified.append(simplified[0])

            new_simplified.append(approach_point)
            new_simplified.append(goal)

            print(f"[A* DEBUG] Using clear approach point ({approach_point[0]:.2f}, {approach_point[1]:.2f})")
            return new_simplified

        return simplified

    def _find_clear_approach_point(self, goal: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """
        Find the best approach point for the final leg to the goal.

        Searches visited cells within 0.8-2.0m of goal that:
        1. Have line-of-sight to goal (no walls blocking)
        2. Maximize minimum clearance from walls along the approach path

        Returns: Best approach point, or None if no good point found.
        """
        goal_cell = self._pos_to_cell(goal[0], goal[1])

        # Search radius in cells (0.8m to 2.0m at 0.3m resolution = ~3-7 cells)
        min_dist_cells = 3  # ~0.9m
        max_dist_cells = 7  # ~2.1m

        best_point = None
        best_clearance = -1

        # Check all visited cells within range
        for cell, _ in self.visited.items():
            # Distance to goal in cells
            dx = cell[0] - goal_cell[0]
            dy = cell[1] - goal_cell[1]
            cell_dist = math.sqrt(dx*dx + dy*dy)

            if cell_dist < min_dist_cells or cell_dist > max_dist_cells:
                continue

            # Convert to world coordinates (center of cell)
            px = (cell[0] + 0.5) * self.grid_res
            py = (cell[1] + 0.5) * self.grid_res

            # Check line-of-sight to goal (no walls blocking)
            if not self._has_line_of_sight(px, py, goal[0], goal[1]):
                continue

            # Calculate minimum clearance along the approach path
            min_clearance = self._compute_path_clearance(px, py, goal[0], goal[1])

            if min_clearance > best_clearance:
                best_clearance = min_clearance
                best_point = (px, py)

        if best_point:
            print(f"[A* DEBUG] Found approach point with clearance={best_clearance:.2f}m")

        return best_point

    def _has_line_of_sight(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """
        Check if there's a clear line-of-sight between two points.

        Uses ray marching to check for wall cells along the path.
        """
        dx = x2 - x1
        dy = y2 - y1
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.01:
            return True

        # Normalize direction
        dx /= dist
        dy /= dist

        # March along the line, checking for walls
        step = self.grid_res * 0.5  # Half grid resolution for better accuracy
        num_steps = int(dist / step) + 1

        for i in range(num_steps):
            d = i * step
            px = x1 + dx * d
            py = y1 + dy * d
            cell = self._pos_to_cell(px, py)

            if cell in self.scan_ends:
                return False  # Wall blocking line of sight

        return True

    def _compute_path_clearance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """
        Compute the minimum clearance from walls along a path.

        Returns the minimum distance to any wall cell along the path.
        Higher values mean safer paths.
        """
        dx = x2 - x1
        dy = y2 - y1
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.01:
            return 5.0  # Max clearance for zero-length path

        # Normalize direction
        dx /= dist
        dy /= dist

        min_clearance = 5.0  # Start with max
        step = self.grid_res * 0.5
        num_steps = int(dist / step) + 1

        # Check points along the path
        for i in range(num_steps):
            d = i * step
            px = x1 + dx * d
            py = y1 + dy * d
            path_cell = self._pos_to_cell(px, py)

            # Find distance to nearest wall
            nearest_wall_dist = 5.0
            search_radius = 5  # Check within 5 cells (~1.5m)

            for wall_cell in self.scan_ends.keys():
                wall_dx = wall_cell[0] - path_cell[0]
                wall_dy = wall_cell[1] - path_cell[1]

                # Quick bounding box check
                if abs(wall_dx) > search_radius or abs(wall_dy) > search_radius:
                    continue

                wall_dist = math.sqrt(wall_dx*wall_dx + wall_dy*wall_dy) * self.grid_res
                if wall_dist < nearest_wall_dist:
                    nearest_wall_dist = wall_dist

            if nearest_wall_dist < min_clearance:
                min_clearance = nearest_wall_dist

        return min_clearance

    def run(self):
        """Main loop: select goal -> drive to goal -> repeat."""
        # Setup output logging first
        self._setup_output_logging()

        print("=" * 55)
        print("NEXT-BEST-VIEW (NBV) NAVIGATION")
        print("=" * 55)
        print(f"Speed: {self.linear_speed} m/s")
        print(f"Goal distance: {self.min_goal_dist}-{self.max_goal_dist}m")
        print("Press Ctrl+C to stop\n")

        self.running = True
        self.start_time = time.time()
        self.start_pos = None  # Track starting position
        self.goal_start_pos = None  # Track position when goal was set (for accuracy)

        # Start marker publishing thread if debug mode
        if self.debug_marker:
            print("[DEBUG] Marker publishing enabled - check /nav_goal in RViz")
            self._start_marker_thread()

        send_velocity_cmd(0.0, 0.0)
        time.sleep(0.2)

        try:
            while self.running:
                self.iteration += 1

                # Check duration - only applies during EXPLORING state
                # In RETURNING state, keep going until we reach origin
                if (self.nav_state == NavigationState.EXPLORING and
                    self.duration > 0 and
                    time.time() - self.start_time >= self.duration):
                    print(f"\n\nDuration {self.duration}s reached (exploration phase).")
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
                    # Start origin marker (blue sphere) if debug mode
                    if self.debug_marker:
                        self._start_origin_marker(self.start_pos[0], self.start_pos[1])

                # Record visit with timestamp (for time-decay penalty)
                cell = self._pos_to_cell(odom[0], odom[1])
                self.visited[cell] = time.time()

                # Record scan coverage using raw LiDAR data (~500 rays for fine resolution)
                self._record_scan_coverage(scan)

                # Front distance (use SECTORS_FRONT_ARC from constants)
                # Filter out blind spots (0.0) - these are sensor gaps, not obstacles
                front_sector_vals = [sectors[s] for s in SECTORS_FRONT_ARC if sectors[s] > 0.01]
                if front_sector_vals:
                    front_min = min(front_sector_vals)
                else:
                    # All sectors are blind - assume clear (will recheck next cycle)
                    front_min = 1.0

                # === CHECK FOR STATE TRANSITION TO RETURNING ===
                elapsed = time.time() - self.start_time
                if (self.nav_state == NavigationState.EXPLORING and
                    self.duration > 0 and
                    elapsed >= self.duration * self.return_trigger_ratio):
                    # Transition to RETURNING state
                    self.nav_state = NavigationState.RETURNING
                    self.return_start_time = time.time()
                    self.return_last_progress_time = time.time()
                    self.return_last_pos = self.current_pos  # Initialize position tracking
                    self.return_last_heading = self.current_heading  # Initialize heading tracking
                    if self.start_pos:
                        self.return_last_dist = math.sqrt(
                            (self.current_pos[0] - self.start_pos[0])**2 +
                            (self.current_pos[1] - self.start_pos[1])**2)
                    print(f"\n\n{'=' * 55}")
                    print(f"[{elapsed:.1f}s] RETURNING TO ORIGIN")
                    print(f"{'=' * 55}")
                    print(f"Time elapsed: {elapsed:.1f}s / {self.duration:.1f}s ({elapsed/self.duration*100:.0f}%)")
                    if self.start_pos:
                        print(f"Origin: ({self.start_pos[0]:.2f}, {self.start_pos[1]:.2f})")
                        print(f"Current: ({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f})")
                        print(f"Distance to origin: {self.return_last_dist:.2f}m")
                    print(f"{'=' * 55}\n")

                    # Plan path using A*
                    self.return_path = self._plan_path_astar(self.current_pos, self.start_pos)
                    self.return_waypoint_idx = 0
                    self.locked_heading = None

                    if self.return_path:
                        print(f"[A*] Planned path with {len(self.return_path)} waypoints")
                        for i, wp in enumerate(self.return_path):
                            print(f"  WP{i}: ({wp[0]:.2f}, {wp[1]:.2f})")
                        # Set first waypoint as goal (with LOS check)
                        safe_wp = self._ensure_los_to_waypoint(self.current_pos, 0)
                        if safe_wp:
                            with self._goal_lock:
                                self.goal_point = safe_wp
                        else:
                            # Fallback to first waypoint anyway
                            with self._goal_lock:
                                self.goal_point = self.return_path[0]
                        self.goal_set_time = time.time()
                    else:
                        print(f"[A*] No path found! Will use direct navigation")
                        # Fallback: go directly to origin
                        with self._goal_lock:
                            self.goal_point = self.start_pos
                        self.goal_set_time = time.time()

                # === HANDLE RETURNING STATE ===
                if self.nav_state == NavigationState.RETURNING:
                    if not self.start_pos:
                        time.sleep(0.1)
                        continue

                    dist_to_origin = math.sqrt(
                        (self.current_pos[0] - self.start_pos[0])**2 +
                        (self.current_pos[1] - self.start_pos[1])**2)

                    # Check if we've reached origin (tighter threshold than other waypoints)
                    if dist_to_origin < self.origin_reached_dist:
                        print(f"\n\n{'=' * 55}")
                        print(f"[{time.time() - self.start_time:.1f}s] MISSION COMPLETE!")
                        print(f"{'=' * 55}")
                        print(f"Successfully returned to origin!")
                        print(f"Final position: ({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f})")
                        print(f"Origin: ({self.start_pos[0]:.2f}, {self.start_pos[1]:.2f})")
                        print(f"Distance error: {dist_to_origin:.2f}m")
                        print(f"{'=' * 55}\n")
                        break

                    # Check if current waypoint reached
                    if self.return_path and self.return_waypoint_idx < len(self.return_path):
                        current_wp = self.return_path[self.return_waypoint_idx]
                        dist_to_wp = math.sqrt(
                            (self.current_pos[0] - current_wp[0])**2 +
                            (self.current_pos[1] - current_wp[1])**2)

                        if dist_to_wp < self.waypoint_reached_dist:
                            # Advance to next waypoint
                            self.return_waypoint_idx += 1
                            if self.return_waypoint_idx < len(self.return_path):
                                # Check if clear LOS to next waypoint, insert intermediate if needed
                                # _ensure_los_to_waypoint always returns a valid waypoint (original or intermediate)
                                safe_wp = self._ensure_los_to_waypoint(self.current_pos, self.return_waypoint_idx)
                                with self._goal_lock:
                                    self.goal_point = safe_wp
                                self.locked_heading = None
                                elapsed_total = time.time() - self.start_time
                                print(f"\n[{elapsed_total:5.1f}s] [A*] Reached WP{self.return_waypoint_idx - 1}, "
                                      f"heading to WP{self.return_waypoint_idx} ({safe_wp[0]:.2f}, {safe_wp[1]:.2f})")
                            else:
                                # All waypoints done, head to final origin
                                with self._goal_lock:
                                    self.goal_point = self.start_pos
                                self.locked_heading = None
                                elapsed_total = time.time() - self.start_time
                                print(f"\n[{elapsed_total:5.1f}s] [A*] All waypoints reached, heading to origin")

                    # Check for movement OR rotation (stuck detection)
                    # Robot is "active" if it moved position OR rotated significantly
                    is_active = False

                    if self.return_last_pos:
                        movement = math.sqrt(
                            (self.current_pos[0] - self.return_last_pos[0])**2 +
                            (self.current_pos[1] - self.return_last_pos[1])**2)
                        if movement > self.return_movement_threshold:
                            is_active = True
                            self.return_last_pos = self.current_pos

                    if self.return_last_heading is not None and self.current_heading is not None:
                        rotation = abs(self._normalize_angle(self.current_heading - self.return_last_heading))
                        if rotation > self.return_rotation_threshold:
                            is_active = True
                            self.return_last_heading = self.current_heading

                    if is_active:
                        self.return_last_progress_time = time.time()

                    # Also track progress toward origin
                    if dist_to_origin < self.return_last_dist - 0.1:
                        self.return_last_dist = dist_to_origin

                    # === Area-based progress tracking ===
                    # Check if robot is staying in same area without making progress toward origin
                    now = time.time()
                    if self.return_area_center is None:
                        # Initialize area tracking
                        self.return_area_center = self.current_pos
                        self.return_area_time = now
                        self.return_area_best_dist = dist_to_origin
                    else:
                        # Check if we've left the area
                        dist_from_area_center = math.sqrt(
                            (self.current_pos[0] - self.return_area_center[0])**2 +
                            (self.current_pos[1] - self.return_area_center[1])**2
                        )

                        if dist_from_area_center > self.return_area_radius:
                            # Left the area - reset tracking
                            self.return_area_center = self.current_pos
                            self.return_area_time = now
                            self.return_area_best_dist = dist_to_origin
                        else:
                            # Still in same area - track best distance
                            if dist_to_origin < self.return_area_best_dist - 0.1:
                                # Made progress toward origin
                                self.return_area_best_dist = dist_to_origin
                                self.return_area_time = now  # Reset timer on progress

                            # Check if we've been in this area too long without progress
                            time_in_area = now - self.return_area_time
                            if time_in_area > self.return_area_timeout:
                                elapsed_total = now - self.start_time
                                print(f"\n[{elapsed_total:5.1f}s] [A*] No progress in {time_in_area:.1f}s (stayed within {self.return_area_radius}m), replanning...")
                                print(f"[A*] Area center: ({self.return_area_center[0]:.2f},{self.return_area_center[1]:.2f}), "
                                      f"best_dist={self.return_area_best_dist:.2f}m, current_dist={dist_to_origin:.2f}m")

                                # Mark current waypoint and surrounding cells as unreachable
                                # If we've already failed at this cell, expand the blocked area
                                if self.return_path and self.return_waypoint_idx < len(self.return_path):
                                    failed_wp = self.return_path[self.return_waypoint_idx]
                                    failed_cell = self._pos_to_cell(failed_wp[0], failed_wp[1])

                                    # Check if we've already failed here before
                                    if failed_cell in self.unreachable_cells:
                                        # Expand blocked area - mark all adjacent cells too
                                        print(f"[A*] Cell {failed_cell} already unreachable, EXPANDING blocked area")
                                        for dx in range(-2, 3):
                                            for dy in range(-2, 3):
                                                expanded_cell = (failed_cell[0] + dx, failed_cell[1] + dy)
                                                self.unreachable_cells[expanded_cell] = now
                                        print(f"[A*] Blocked 5x5 area around ({failed_wp[0]:.2f},{failed_wp[1]:.2f})")
                                    else:
                                        self.unreachable_cells[failed_cell] = now
                                        print(f"[A*] Marked waypoint ({failed_wp[0]:.2f},{failed_wp[1]:.2f}) cell {failed_cell} as UNREACHABLE")

                                # Update obstacles and replan
                                self._update_obstacles_from_lidar(scan)
                                self._backup(sectors, scan)
                                self.backups += 1

                                # Reset area tracking
                                self.return_area_center = self.current_pos
                                self.return_area_time = now
                                self.return_area_best_dist = dist_to_origin

                                # Replan
                                self.return_path = self._plan_path_astar(self.current_pos, self.start_pos)
                                self.return_waypoint_idx = 0
                                self.return_last_progress_time = now
                                if self.return_path:
                                    print(f"[A*] Replanned with {len(self.return_path)} waypoints")
                                    safe_wp = self._ensure_los_to_waypoint(self.current_pos, 0)
                                    with self._goal_lock:
                                        self.goal_point = safe_wp if safe_wp else self.return_path[0]
                                else:
                                    print(f"[A*] Replan failed, using direct navigation")
                                    with self._goal_lock:
                                        self.goal_point = self.start_pos
                                continue

                    # Check if stuck (no significant movement OR rotation for too long)
                    time_since_progress = time.time() - self.return_last_progress_time
                    if time_since_progress > self.return_stuck_threshold:
                        elapsed_total = time.time() - self.start_time
                        print(f"\n[{elapsed_total:5.1f}s] [A*] Stuck for {time_since_progress:.1f}s (no movement/rotation), replanning...")

                        # Mark the current waypoint as unreachable so A* won't route through it again
                        # If already unreachable, expand the blocked area
                        if self.return_path and self.return_waypoint_idx < len(self.return_path):
                            failed_wp = self.return_path[self.return_waypoint_idx]
                            failed_cell = self._pos_to_cell(failed_wp[0], failed_wp[1])

                            if failed_cell in self.unreachable_cells:
                                # Already failed here - expand blocked area
                                print(f"[A*] Cell {failed_cell} already unreachable, EXPANDING blocked area")
                                for dx in range(-2, 3):
                                    for dy in range(-2, 3):
                                        expanded_cell = (failed_cell[0] + dx, failed_cell[1] + dy)
                                        self.unreachable_cells[expanded_cell] = time.time()
                                print(f"[A*] Blocked 5x5 area around ({failed_wp[0]:.2f},{failed_wp[1]:.2f})")
                            else:
                                self.unreachable_cells[failed_cell] = time.time()
                                print(f"[A*] Marked waypoint ({failed_wp[0]:.2f},{failed_wp[1]:.2f}) cell {failed_cell} as UNREACHABLE")

                        # Update scan_ends with current LiDAR obstacles
                        # This ensures A* knows about obstacles we're seeing now
                        self._update_obstacles_from_lidar(scan)

                        self._backup(sectors, scan)
                        self.backups += 1
                        self.return_last_progress_time = time.time()
                        self.return_last_pos = self.current_pos  # Reset position tracking after backup
                        self.return_last_heading = self.current_heading  # Reset heading tracking after backup

                        # Replan path from current position
                        self.return_path = self._plan_path_astar(self.current_pos, self.start_pos)
                        self.return_waypoint_idx = 0
                        if self.return_path:
                            print(f"[A*] Replanned with {len(self.return_path)} waypoints")
                            safe_wp = self._ensure_los_to_waypoint(self.current_pos, 0)
                            with self._goal_lock:
                                self.goal_point = safe_wp if safe_wp else self.return_path[0]
                        else:
                            print(f"[A*] Replan failed, using direct navigation")
                            with self._goal_lock:
                                self.goal_point = self.start_pos
                        continue

                    # Obstacle detection and navigation
                    goal_blocked = self._goal_blocked(sectors)

                    # BACKUP if too close
                    if front_min < self.backup_threshold and self.iteration > 5:
                        elapsed_total = time.time() - self.start_time
                        print(f"\n[{elapsed_total:5.1f}s] [BACKUP] front={front_min:.2f}m")
                        self._backup(sectors, scan)
                        self.backups += 1
                        # Replan after backup
                        self.return_path = self._plan_path_astar(self.current_pos, self.start_pos)
                        self.return_waypoint_idx = 0
                        if self.return_path:
                            safe_wp = self._ensure_los_to_waypoint(self.current_pos, 0)
                            with self._goal_lock:
                                self.goal_point = safe_wp if safe_wp else self.return_path[0]
                        else:
                            with self._goal_lock:
                                self.goal_point = self.start_pos
                        continue

                    # DRIVE toward current waypoint/goal
                    if self.goal_point:
                        if goal_blocked:
                            v, w = self._compute_blocked_turn(sectors)
                            status_mode = "TURN"
                        else:
                            v, w = self._compute_velocity_to_goal(front_min, sectors)
                            status_mode = "DRIVE"

                        v, w = self._ramp_velocity(v, w)
                        send_velocity_cmd(v, w)

                        # Status for RETURNING
                        wp_info = f"WP{self.return_waypoint_idx}/{len(self.return_path)}" if self.return_path else "direct"
                        goal_str = f"({self.goal_point[0]:.2f},{self.goal_point[1]:.2f})"
                        status = f"[{self.iteration:3d}] RETURN {status_mode} {wp_info} goal={goal_str} origin_dist={dist_to_origin:.2f}m front={front_min:.2f}m"
                        print(f"\r{status}", end="", flush=True)

                    time.sleep(0.1)
                    continue  # Skip normal exploration logic

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
                    new_goal, goal_sector = self._select_goal_point(sectors, scan)
                    if new_goal:
                        with self._goal_lock:
                            self.goal_point = new_goal
                        self.goal_set_time = time.time()
                        self.locked_heading = None  # Clear heading lock for new goal
                        dist = math.sqrt((new_goal[0] - self.current_pos[0])**2 +
                                        (new_goal[1] - self.current_pos[1])**2)
                        self.goal_initial_dist = dist  # Store initial distance for timeout calc
                        self.goal_timeout_extended = False  # Reset extension flag
                        self.consecutive_no_goal = 0  # Reset failure counter on success
                        self.drive_goal_dist_history.clear()  # Reset DRIVE stuck detection

                        # Track for drive accuracy
                        self.goal_start_pos = self.current_pos
                        self.drive_ideal_distance += dist

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
                        # No valid goal found
                        self.consecutive_no_goal += 1
                        elapsed_total = time.time() - self.start_time

                        if self.consecutive_no_goal >= self.no_goal_turn_threshold:
                            # Too many failures - do a 180° turn to look the other way
                            print(f"\n[{elapsed_total:5.1f}s] [NO GOAL] {self.consecutive_no_goal} consecutive failures, turning 180°")
                            self._turn_around()
                            self.consecutive_no_goal = 0  # Reset after turning
                        else:
                            # Normal backup
                            print(f"\n[{elapsed_total:5.1f}s] [NO GOAL] No valid viewpoint found ({self.consecutive_no_goal}/{self.no_goal_turn_threshold}), backing up")
                            self._backup(sectors, scan)
                        self.backups += 1
                        continue

                # === 3. BACKUP if too close ===
                # Skip backup check for first 5 iterations to let sensors stabilize
                if front_min < self.backup_threshold and self.iteration > 5:
                    elapsed_total = time.time() - self.start_time
                    print(f"\n[{elapsed_total:5.1f}s] [BACKUP] front={front_min:.2f}m")
                    self._backup(sectors, scan)
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

                        # Track time spent in TURN mode - if too long, force backup
                        if not self.was_in_turn_mode:
                            self.turn_mode_start_time = time.time()
                            self.turn_mode_positions = []  # Track positions during TURN mode
                            self.was_in_turn_mode = True
                            # Debug: Print why we're entering TURN mode
                            elapsed_total = time.time() - self.start_time
                            print(f"\n[{elapsed_total:5.1f}s] [TURN ENTER] Entering TURN mode")
                            self._goal_blocked(sectors, debug=True)
                            self._compute_blocked_turn(sectors, debug=True)
                        else:
                            # Track position during TURN mode
                            if hasattr(self, 'turn_mode_positions'):
                                self.turn_mode_positions.append((
                                    self.current_pos[0], self.current_pos[1],
                                    math.degrees(self.current_heading) if self.current_heading else 0
                                ))

                            turn_duration = time.time() - self.turn_mode_start_time
                            if turn_duration > self.turn_mode_timeout:
                                elapsed_total = time.time() - self.start_time
                                print(f"\n[{elapsed_total:5.1f}s] [TURN STUCK] In TURN mode for {turn_duration:.1f}s, backing up")

                                # Debug: Show position history during TURN mode
                                if hasattr(self, 'turn_mode_positions') and self.turn_mode_positions:
                                    positions = self.turn_mode_positions
                                    print(f"[TURN STUCK] Position history ({len(positions)} samples):")
                                    # Show first, middle, and last positions
                                    if len(positions) >= 3:
                                        first = positions[0]
                                        mid = positions[len(positions)//2]
                                        last = positions[-1]
                                        print(f"  First: ({first[0]:.2f}, {first[1]:.2f}) heading={first[2]:.1f}°")
                                        print(f"  Mid:   ({mid[0]:.2f}, {mid[1]:.2f}) heading={mid[2]:.1f}°")
                                        print(f"  Last:  ({last[0]:.2f}, {last[1]:.2f}) heading={last[2]:.1f}°")
                                        # Calculate movement
                                        pos_change = math.sqrt((last[0]-first[0])**2 + (last[1]-first[1])**2)
                                        heading_change = abs(last[2] - first[2])
                                        print(f"  Movement: {pos_change:.3f}m, heading change: {heading_change:.1f}°")
                                    else:
                                        for i, pos in enumerate(positions):
                                            print(f"  [{i}]: ({pos[0]:.2f}, {pos[1]:.2f}) heading={pos[2]:.1f}°")

                                # Debug: Show current sector readings
                                print(f"[TURN STUCK] Current front sectors: ", end="")
                                for s in SECTORS_FRONT_ARC:
                                    print(f"s{s}={sectors[s]:.2f} ", end="")
                                print()

                                self._backup_toward_clear_side(sectors)
                                self.backups += 1
                                self.was_in_turn_mode = False
                                with self._goal_lock:
                                    self.goal_point = None
                                continue
                    else:
                        v, w = self._compute_velocity_to_goal(front_min, sectors)
                        status_mode = "DRIVE"
                        self.was_in_turn_mode = False

                        # DRIVE mode stuck detection - sliding window on goal distance
                        # Only check when actually driving forward (v > 0), not rotating
                        now = time.time()
                        goal_dist = math.sqrt((self.goal_point[0] - self.current_pos[0])**2 +
                                             (self.goal_point[1] - self.current_pos[1])**2)

                        if v > 0.01:
                            # Add current reading to history
                            self.drive_goal_dist_history.append((now, goal_dist))

                            # Remove readings older than the window
                            cutoff = now - self.drive_stuck_window
                            self.drive_goal_dist_history = [(t, d) for t, d in self.drive_goal_dist_history if t >= cutoff]

                            # Check if we have enough history (at least 5 seconds of data)
                            if len(self.drive_goal_dist_history) >= 2:
                                oldest_time, oldest_dist = self.drive_goal_dist_history[0]
                                time_span = now - oldest_time

                                # Only check if we have at least 5 seconds of data
                                if time_span >= 5.0:
                                    # Check if goal distance changed significantly
                                    dist_change = abs(oldest_dist - goal_dist)

                                    if dist_change < self.drive_stuck_margin:
                                        # Goal distance not changing - robot is stuck
                                        elapsed_total = time.time() - self.start_time
                                        print(f"\n[{elapsed_total:5.1f}s] [DRIVE STUCK] Goal distance unchanged for {time_span:.1f}s")
                                        print(f"[DRIVE STUCK] dist_oldest={oldest_dist:.2f}m, "
                                              f"dist_now={goal_dist:.2f}m, change={dist_change:.2f}m < margin={self.drive_stuck_margin:.2f}m")
                                        print(f"[DRIVE STUCK] Front sectors: ", end="")
                                        for s in SECTORS_FRONT_ARC:
                                            print(f"s{s}={sectors[s]:.2f} ", end="")
                                        print()

                                        # Mark front sectors as blocked for 30 seconds
                                        block_expire = now + 30.0
                                        for s in SECTORS_FRONT_ARC:
                                            self.blocked_sectors[s] = block_expire
                                        print(f"[DRIVE STUCK] Marked front sectors {list(SECTORS_FRONT_ARC)} as blocked for 30s")

                                        # Backup toward clearer side
                                        self._backup_toward_clear_side(sectors)
                                        self.backups += 1

                                        with self._goal_lock:
                                            self.goal_point = None
                                        # Clear history
                                        self.drive_goal_dist_history.clear()
                                        continue
                        else:
                            # Not driving forward (rotating) - don't add to history but don't clear it
                            pass

                    # Apply velocity ramping for smooth acceleration (Fix 1)
                    v, w = self._ramp_velocity(v, w)
                    send_velocity_cmd(v, w)

                    # Update drive accuracy tracking
                    if self.goal_start_pos and self.goal_point:
                        self._update_drive_accuracy(self.goal_start_pos, self.goal_point)

                    # Status with drive accuracy
                    goal_dist = math.sqrt((self.goal_point[0] - self.current_pos[0])**2 +
                                         (self.goal_point[1] - self.current_pos[1])**2)
                    pos_str = f"({self.current_pos[0]:.2f},{self.current_pos[1]:.2f})"
                    goal_str = f"({self.goal_point[0]:.2f},{self.goal_point[1]:.2f})"
                    rms_cm = self._get_drive_rms_error() * 100
                    status = f"[{self.iteration:3d}] {status_mode} goal={goal_str} dist={goal_dist:.2f}m front={front_min:.2f}m cells={len(self.visited)} rms={rms_cm:.1f}cm"
                    print(f"\r{status}", end="", flush=True)

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n\nStopping (Ctrl+C)...")
        finally:
            self.running = False
            send_velocity_cmd(0.0, 0.0)
            self._stop_marker_thread()
            self._stop_origin_marker()
            self._print_stats()
            self._save_map()
            self._cleanup_logging()

    def _select_goal_point(self, sectors: List[float], raw_scan: List[float] = None) -> Tuple[Optional[Tuple[float, float]], Optional[int]]:
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
        min_best_score = 20.0    # Minimum acceptable best score before refining
        base_num_angles = 50     # Starting number of angles (finer: 300°/50 = 6° per step)
        refinement_factor = 3    # Multiply angles by this factor each iteration
        max_refinements = 3      # Max iterations (50 -> 150 -> 450 -> 1350)

        candidates = []
        num_angles = base_num_angles
        refinement_level = 0

        # Keep refining until we have enough good candidates or hit max refinements
        # Refine if: not enough candidates OR best score is too low
        def should_refine():
            if len(candidates) < min_candidates:
                return True
            # Check if best score is too low - might be missing better options
            if candidates:
                best = max(c[0] for c in candidates)
                if best < min_best_score:
                    return True
            return False

        while should_refine() and refinement_level <= max_refinements:
            if refinement_level > 0:
                # Only show message if we're actually refining
                elapsed_total = time.time() - self.start_time if self.start_time > 0 else 0
                best_so_far = max(c[0] for c in candidates) if candidates else -999
                reason = f"only {len(candidates)} candidates" if len(candidates) < min_candidates else f"best score {best_so_far:.1f} < {min_best_score}"
                print(f"\n[{elapsed_total:5.1f}s] [REFINE] {reason}, "
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

                    # FILTER: Check angular opening (not just single ray)
                    # Require 10° opening on each side to filter narrow slits and noise
                    if raw_scan and not self._check_angular_opening(raw_scan, robot_angle, dist + 0.2, min_opening_deg=10.0):
                        continue  # Discard - narrow slit or noise, robot can't fit

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
                blk = f" B={brk.get('blocked', 0):+.1f}" if brk.get('blocked', 0) != 0 else ""
                bkt = f" T={brk.get('backtrack', 0):+.1f}" if brk.get('backtrack', 0) != 0 else ""
                print(f"  #{i+1} ang={ang:+4d}° d={d:.2f}m pos=({px:.2f},{py:.2f}) "
                      f"score={score:+.1f} [O={brk['open']:+.1f} U={brk['unscan']:+.1f} "
                      f"V={brk['visited']:+.1f} W={brk['wall']:+.1f} R={brk['recent']:+.1f} F={brk['fwd']:+.1f}{blk}{bkt}]{marker}")

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
            decay_rate = 0.03  # SLOWER decay: penalty lasts longer
            decay_factor = math.exp(-decay_rate * time_since_visit)
            # INCREASED base penalty of 15.0 - strongly discourage revisiting
            visited_penalty = 15.0 * decay_factor
        # Also check neighboring cells - penalize being NEAR visited areas
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (point_cell[0] + dx, point_cell[1] + dy)
                neighbor_visit = self.visited.get(neighbor, 0)
                if neighbor_visit > 0:
                    time_since = now - neighbor_visit
                    decay = math.exp(-0.03 * time_since)
                    visited_penalty += 3.0 * decay  # Lighter penalty for neighbors
        breakdown['visited'] = -visited_penalty

        # === Factor 6: DISTANCE EFFICIENCY ===
        breakdown['dist'] = dist * 0.3

        # === Factor 7: FORWARD MOMENTUM ===
        # Prefer continuing in the direction we were heading, not backtracking
        # This prevents oscillating between areas
        fwd_bonus = 0.0
        if self.start_pos:
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

                    # REDUCED: Forward momentum should be a tiebreaker, not dominate
                    # The visited penalty already handles not revisiting areas
                    # dot=1 -> +8, dot=0 -> 0, dot=-1 -> -8
                    fwd_bonus = dot * 8.0

        breakdown['fwd'] = fwd_bonus

        # === Factor 7b: ALREADY VISITED AREA PENALTY ===
        # Strong penalty for selecting goals in areas we've recently been
        # This prevents going back to where we just came from
        backtrack_penalty = 0.0
        for visited_cell, visit_time in self.visited.items():
            # Check if candidate is close to a visited cell
            visited_wx = (visited_cell[0] + 0.5) * self.grid_res
            visited_wy = (visited_cell[1] + 0.5) * self.grid_res
            dist_to_visited = math.sqrt((px - visited_wx)**2 + (py - visited_wy)**2)

            # If candidate is within 0.5m of a visited cell, penalize based on recency
            if dist_to_visited < 0.5:
                time_since_visit = now - visit_time
                if time_since_visit < 60:  # Visited in last 60 seconds
                    # Recent visits get higher penalty
                    recency_factor = 1.0 - (time_since_visit / 60.0)
                    backtrack_penalty += recency_factor * 10.0

        breakdown['backtrack'] = -min(backtrack_penalty, 25.0)  # Cap at -25

        # === Factor 8: BLOCKED SECTOR PENALTY ===
        # Heavily penalize goals in sectors that are currently blocked
        # (marked after DRIVE stuck detection found an impassable obstacle)
        blocked_penalty = 0.0
        if sector_idx in self.blocked_sectors:
            if now < self.blocked_sectors[sector_idx]:
                # Sector still blocked - heavy penalty
                blocked_penalty = 50.0
            else:
                # Block expired - remove it
                del self.blocked_sectors[sector_idx]
        # Also check adjacent sectors (obstacles span multiple sectors)
        for adj in [-1, 1, -2, 2]:
            adj_sector = (sector_idx + adj) % NUM_SECTORS
            if adj_sector in self.blocked_sectors:
                if now < self.blocked_sectors[adj_sector]:
                    blocked_penalty += 25.0  # Lighter penalty for adjacent
                else:
                    del self.blocked_sectors[adj_sector]
        breakdown['blocked'] = -blocked_penalty

        total = sum(breakdown.values())
        return total, breakdown

    def _update_obstacles_from_lidar(self, raw_scan: List[float]):
        """
        Update scan_ends with obstacles detected by current LiDAR scan.

        Called when robot gets stuck during return - adds any close obstacles
        to the map so A* can route around them.
        """
        if not self.current_pos or self.current_heading is None:
            return

        if not raw_scan:
            return

        now = time.time()
        n_points = len(raw_scan)
        lidar_rotation_rad = 3 * (2 * math.pi / 12)  # 90° offset
        angle_per_point = 2 * math.pi / n_points

        new_walls = 0
        for i, dist in enumerate(raw_scan):
            # Only add close obstacles (< 1m) - these are what's blocking us
            if not (0.1 < dist < 1.0):
                continue

            lidar_angle = i * angle_per_point
            robot_angle = lidar_angle + lidar_rotation_rad
            world_angle = self.current_heading + robot_angle

            # Mark the obstacle cell
            wall_x = self.current_pos[0] + dist * math.cos(world_angle)
            wall_y = self.current_pos[1] + dist * math.sin(world_angle)
            wall_cell = self._pos_to_cell(wall_x, wall_y)

            if wall_cell not in self.scan_ends:
                self.scan_ends[wall_cell] = now
                new_walls += 1

        if new_walls > 0:
            print(f"[A*] Added {new_walls} new wall cells from current LiDAR")

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

    def _check_angular_opening(self, raw_scan: List[float], robot_angle: float,
                                required_dist: float, min_opening_deg: float = 10.0,
                                min_pass_ratio: float = 0.7) -> bool:
        """
        Check if there's sufficient angular opening around a direction.

        Verifies that not just the target direction is clear, but also adjacent
        rays within min_opening_deg on each side can reach at least required_dist.
        This filters out narrow slits and noise that the robot can't actually traverse.

        Args:
            raw_scan: Raw LiDAR scan data (~500 points)
            robot_angle: Target direction in robot frame (radians)
            required_dist: Minimum distance all rays must reach
            min_opening_deg: Required opening on each side (degrees)
            min_pass_ratio: Minimum fraction of rays that must pass (0.7 = 70%)

        Returns:
            True if opening is wide enough, False otherwise
        """
        if not raw_scan:
            return False

        n_points = len(raw_scan)
        angle_per_point = 2 * math.pi / n_points

        # LiDAR rotation offset (same as in _record_scan_coverage)
        lidar_rotation_rad = 3 * (2 * math.pi / 12)  # 90° offset

        # Convert robot_angle to LiDAR frame
        lidar_angle = robot_angle - lidar_rotation_rad
        # Normalize to [0, 2π)
        while lidar_angle < 0:
            lidar_angle += 2 * math.pi
        while lidar_angle >= 2 * math.pi:
            lidar_angle -= 2 * math.pi

        # Find the LiDAR point index for this angle
        center_idx = int(lidar_angle / angle_per_point) % n_points

        # Calculate how many points correspond to min_opening_deg
        opening_rad = min_opening_deg * math.pi / 180.0
        points_per_side = max(1, int(opening_rad / angle_per_point))

        # Count how many rays pass the distance check
        total_rays = 0
        passing_rays = 0

        for offset in range(-points_per_side, points_per_side + 1):
            idx = (center_idx + offset) % n_points
            ray_dist = raw_scan[idx]
            total_rays += 1

            # Valid reading that reaches far enough
            if 0.1 < ray_dist < 10.0 and ray_dist >= required_dist:
                passing_rays += 1

        # Require min_pass_ratio of rays to pass (robust to noise)
        if total_rays == 0:
            return False

        return (passing_rays / total_rays) >= min_pass_ratio

    def _goal_reached(self) -> bool:
        """Check if current goal is reached."""
        if not self.goal_point or not self.current_pos:
            return False

        dist = math.sqrt((self.goal_point[0] - self.current_pos[0])**2 +
                        (self.goal_point[1] - self.current_pos[1])**2)
        return dist < self.goal_reached_dist

    def _goal_blocked(self, sectors: List[float], debug: bool = False) -> bool:
        """Check if path to goal is blocked.

        Only considers the goal blocked if:
        1. Robot is roughly facing the goal (within ±45°)
        2. There's an obstacle IN THE DIRECTION OF THE GOAL blocking the path

        Key fix: Only check sectors within ±15° of goal direction, not entire front arc.
        This prevents false positives when obstacles are to the side but not blocking
        the actual path to the goal.

        If the goal is to the side or behind, return False - the robot
        needs to turn first, which is handled by normal velocity control.
        """
        if not self.goal_point or not self.current_pos or self.current_heading is None:
            if debug:
                print(f"\n[BLOCKED DEBUG] No goal/pos/heading - returning True")
            return True

        # Calculate angle to goal
        dx = self.goal_point[0] - self.current_pos[0]
        dy = self.goal_point[1] - self.current_pos[1]
        goal_dist = math.sqrt(dx*dx + dy*dy)
        goal_world_angle = math.atan2(dy, dx)

        # Convert to robot frame
        goal_robot_angle = self._normalize_angle(goal_world_angle - self.current_heading)

        # Only check for blocked if goal is in front arc (±45°)
        # If goal is to the side or behind, robot needs to turn first - not blocked
        if abs(goal_robot_angle) > 0.8:  # ~45 degrees
            if debug:
                print(f"\n[BLOCKED DEBUG] Goal angle {math.degrees(goal_robot_angle):.1f}° > 45° - returning False (need to turn first)")
            return False  # Goal not in front - let normal control handle turning

        # Determine which sectors are in the direction of the goal
        goal_sector = self._angle_to_sector(goal_robot_angle)

        # Calculate how many sectors to check based on robot width and goal distance
        # Robot is 18cm wide. Angular width = 2 * atan(0.09 / goal_dist)
        # At 0.5m: ~20° = 3 sectors; At 1m: ~10° = 2 sectors; At 2m: ~5° = 1 sector
        robot_half_width = 0.09  # 9cm = half of 18cm robot width
        angular_width_rad = 2.0 * math.atan(robot_half_width / max(goal_dist, 0.3))
        angular_width_deg = math.degrees(angular_width_rad)
        sector_width_deg = 6.0  # 6° per sector

        # Number of extra sectors on each side (0 = just goal sector, 1 = ±1, etc.)
        # Only add extra sectors if angular width exceeds current coverage
        extra_sectors = int((angular_width_deg - sector_width_deg) / (2 * sector_width_deg))
        extra_sectors = max(0, min(extra_sectors, 2))  # 0 to 2 extra sectors per side

        # Build check sectors list: always include goal sector, optionally add neighbors
        check_sectors = [goal_sector]
        for offset in range(1, extra_sectors + 1):
            check_sectors.append((goal_sector + offset) % NUM_SECTORS)
            check_sectors.append((goal_sector - offset) % NUM_SECTORS)

        # Filter out blind spots
        goal_direction_vals = [sectors[s] for s in check_sectors if sectors[s] > 0.01]
        if not goal_direction_vals:
            if debug:
                print(f"\n[BLOCKED DEBUG] All goal-direction sectors blind - returning False")
            return False  # All blind spots - assume clear

        # Use minimum distance in goal direction
        goal_dir_min = min(goal_direction_vals)

        # Blocked if obstacle in goal direction is closer than goal
        blocked = goal_dir_min < goal_dist - self.blocked_margin

        if debug:
            # Show readings for sectors we're checking
            goal_dir_readings = {s: sectors[s] for s in check_sectors}
            front_readings = {s: sectors[s] for s in SECTORS_FRONT_ARC}
            print(f"\n[BLOCKED DEBUG] Goal angle: {math.degrees(goal_robot_angle):.1f}° (sector {goal_sector}), "
                  f"goal_dist: {goal_dist:.2f}m, goal_dir_min: {goal_dir_min:.2f}m, "
                  f"margin: {self.blocked_margin:.2f}m")
            print(f"[BLOCKED DEBUG] Checking {len(check_sectors)} sectors (robot angular width at {goal_dist:.1f}m = {angular_width_deg:.1f}°): {goal_dir_readings}")
            print(f"[BLOCKED DEBUG] Check: {goal_dir_min:.2f} < {goal_dist:.2f} - {self.blocked_margin:.2f} = {goal_dist - self.blocked_margin:.2f}? {blocked}")
            print(f"[BLOCKED DEBUG] (Full front arc for reference: {front_readings})")

        if blocked:
            return True

        return False

    def _compute_velocity_to_goal(self, front_min: float,
                                    sectors: List[float] = None) -> Tuple[float, float]:
        """
        Compute velocity commands to drive toward goal.

        Uses smooth proportional control throughout (no bang-bang jumps).
        Heading lock for drift correction when well-aligned.
        Adds slight steering away from side walls for smoother navigation.
        """
        if not self.goal_point or not self.current_pos or self.current_heading is None:
            return 0.0, 0.0

        # Calculate angle to goal
        dx = self.goal_point[0] - self.current_pos[0]
        dy = self.goal_point[1] - self.current_pos[1]
        goal_world_angle = math.atan2(dy, dx)

        # Angle error (how much we need to turn to face goal)
        angle_error = self._normalize_angle(goal_world_angle - self.current_heading)

        # Smooth proportional angular velocity (Fix 3: no discrete jumps)
        # Use proportional control throughout, clamped to max
        max_w = 0.6  # Maximum angular velocity
        w = angle_error * 1.5  # Proportional gain
        w = max(-max_w, min(max_w, w))

        # Linear velocity based on alignment (smooth transitions)
        if abs(angle_error) > 0.4:  # ~23 degrees - rotate in place
            self.locked_heading = None
            # If obstacle too close while turning, back up slightly to get clearance
            if front_min < 0.4:
                v = -0.03  # Creep backward while turning
            else:
                v = 0.0
            # Ensure we're always turning when v=0
            if abs(w) < 0.15:
                w = 0.15 if angle_error >= 0 else -0.15
        elif abs(angle_error) > 0.15:  # ~9-23 degrees - slow forward while turning
            self.locked_heading = None
            # Smooth speed reduction based on angle error
            alignment_factor = 1.0 - (abs(angle_error) - 0.15) / 0.25
            v = self.linear_speed * (0.3 + 0.2 * alignment_factor)
        else:
            # Aligned (<9°) - LOCK HEADING and drive straight with drift correction
            if self.locked_heading is None:
                self.locked_heading = goal_world_angle

            # Use locked heading for drift correction
            drift_error = self._normalize_angle(self.locked_heading - self.current_heading)
            w = drift_error * self.heading_lock_gain
            w = max(-0.4, min(0.4, w))

            v = self.linear_speed

        # Slow down near front obstacles, but less aggressively
        if front_min < 0.35:
            speed_factor = (front_min - self.backup_threshold) / (0.35 - self.backup_threshold)
            speed_factor = max(0.6, min(1.0, speed_factor))  # Min 60% speed
            v *= speed_factor
            # Ensure minimum forward velocity when driving
            if v > 0 and v < 0.05:
                v = 0.05

        # Add slight steering away from side walls (only when moving forward)
        if sectors and v > 0.01:
            # Check left side (sectors 5-15, ~30-90 degrees left)
            left_vals = [sectors[s] for s in range(5, 16) if sectors[s] > 0.01]
            left_min = min(left_vals) if left_vals else 5.0
            # Check right side (sectors 45-55, ~30-90 degrees right)
            right_vals = [sectors[s] for s in range(45, 56) if sectors[s] > 0.01]
            right_min = min(right_vals) if right_vals else 5.0

            # If one side is significantly closer, steer slightly away
            side_threshold = 0.5  # Only react if wall within 0.5m
            steer_gain = 0.15  # Gentle steering correction

            if left_min < side_threshold and left_min < right_min - 0.1:
                # Wall on left, steer slightly right (negative w)
                wall_steer = -steer_gain * (side_threshold - left_min) / side_threshold
                w += wall_steer
            elif right_min < side_threshold and right_min < left_min - 0.1:
                # Wall on right, steer slightly left (positive w)
                wall_steer = steer_gain * (side_threshold - right_min) / side_threshold
                w += wall_steer

        return v, w

    def _compute_blocked_turn(self, sectors: List[float], debug: bool = False) -> Tuple[float, float]:
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
            # No open sector found - smooth proportional turn toward goal
            max_w = 0.5
            w = goal_robot_angle * 1.2
            w = max(-max_w, min(max_w, w))
            # Always maintain minimum angular velocity to avoid getting stuck
            if abs(w) < 0.15:
                w = 0.15 if w >= 0 else -0.15
            if debug:
                print(f"\n[BLOCKED TURN] No open sector found (need >{min_clearance:.1f}m), "
                      f"turning toward goal. goal_angle={math.degrees(goal_robot_angle):.1f}°, w={w:.2f}")
            return 0.0, w

        # Turn toward the best open sector - smooth proportional control
        target_angle = self._sector_to_angle(best_sector)
        angle_error = self._normalize_angle(target_angle)

        # Smooth proportional turn (no bang-bang)
        max_w = 0.5
        w = angle_error * 1.5
        w = max(-max_w, min(max_w, w))

        # Small forward motion if aligned and have clearance ahead
        # Filter out blind spots (same as main loop)
        front_vals = [sectors[s] for s in SECTORS_FRONT_ARC if sectors[s] > 0.01]
        front_min = min(front_vals) if front_vals else 1.0
        if abs(angle_error) < 0.15 and front_min > 0.2:
            v = 0.05
        else:
            v = 0.0
            # If not moving forward, ensure we're at least turning
            if abs(w) < 0.15:
                w = 0.15 if angle_error >= 0 else -0.15

        if debug:
            print(f"\n[BLOCKED TURN] goal_sector={goal_sector}, best_sector={best_sector} "
                  f"(dist={best_dist:.2f}m), target_angle={math.degrees(target_angle):.1f}°, "
                  f"angle_error={math.degrees(angle_error):.1f}°, v={v:.2f}, w={w:.2f}")

        return v, w

    def _ramp_velocity(self, target_v: float, target_w: float) -> Tuple[float, float]:
        """
        Apply acceleration limiting for smooth velocity transitions.

        Limits how fast v and w can change per control cycle to prevent
        jerky motion from sudden velocity jumps.
        """
        # Limit linear velocity change
        v_diff = target_v - self.last_v
        if abs(v_diff) > self.max_linear_accel:
            v = self.last_v + self.max_linear_accel * (1.0 if v_diff > 0 else -1.0)
        else:
            v = target_v

        # Limit angular velocity change
        w_diff = target_w - self.last_w
        if abs(w_diff) > self.max_angular_accel:
            w = self.last_w + self.max_angular_accel * (1.0 if w_diff > 0 else -1.0)
        else:
            w = target_w

        # Update state for next cycle
        self.last_v = v
        self.last_w = w

        return v, w

    def _backup(self, sectors: List[float], raw_scan: List[float] = None):
        """Backup maneuver with smart turn direction."""
        print(f"\n[BACKUP] Reversing...")

        # Reverse
        for _ in range(10):
            send_velocity_cmd(-0.05, 0.0)
            time.sleep(0.1)

        # Decide turn direction: prefer direction with valid angular openings
        # Robot convention: positive angles = LEFT, negative angles = RIGHT
        # Check left side (+30° to +90°) and right side (-90° to -30°) for traversable openings
        left_openings = 0
        right_openings = 0

        if raw_scan:
            # Check angles on LEFT side (POSITIVE angles in robot frame)
            for angle_deg in range(30, 100, 10):
                angle_rad = angle_deg * math.pi / 180.0
                if self._check_angular_opening(raw_scan, angle_rad, 0.6, min_opening_deg=10.0):
                    left_openings += 1

            # Check angles on RIGHT side (NEGATIVE angles in robot frame)
            for angle_deg in range(-90, -20, 10):
                angle_rad = angle_deg * math.pi / 180.0
                if self._check_angular_opening(raw_scan, angle_rad, 0.6, min_opening_deg=10.0):
                    right_openings += 1

        # Fallback to sector-based decision if no valid openings found
        if left_openings == 0 and right_openings == 0:
            front_left = min(sectors[s] for s in SECTORS_LEFT)
            front_right = min(sectors[s] for s in SECTORS_RIGHT)
            turn_w = 0.5 if front_left > front_right else -0.5
            print(f"[BACKUP] Turning {'left' if turn_w > 0 else 'right'} (L={front_left:.2f} R={front_right:.2f}) [sector fallback]")
        else:
            # Turn toward side with more valid openings
            turn_w = 0.5 if left_openings > right_openings else -0.5
            print(f"[BACKUP] Turning {'left' if turn_w > 0 else 'right'} (L_openings={left_openings} R_openings={right_openings})")

        for _ in range(10):
            send_velocity_cmd(0.0, turn_w)
            time.sleep(0.1)

        send_velocity_cmd(0.0, 0.0)

        # Clear heading lock and reset velocity state after backup
        self.locked_heading = None
        self.last_v = 0.0
        self.last_w = 0.0

    def _backup_toward_clear_side(self, sectors: List[float]):
        """
        Backup and turn toward the clearer side based on front-left vs front-right.

        Used when stuck in TURN mode - compares average clearance on front-left
        (sectors 1-5, ~6-30°) vs front-right (sectors 55-59, ~-30 to -6°) to
        decide which way to turn after backing up.
        """
        print(f"\n[BACKUP CLEAR] Reversing and turning toward clearer side...")

        # Calculate average clearance for front-left and front-right
        # Front-left: sectors 1-5 (~6° to 30° left)
        front_left_sectors = range(1, 6)
        front_left_vals = [sectors[s] for s in front_left_sectors if sectors[s] > 0.01]
        front_left_avg = sum(front_left_vals) / len(front_left_vals) if front_left_vals else 0.0

        # Front-right: sectors 55-59 (~-30° to -6° right)
        front_right_sectors = range(55, 60)
        front_right_vals = [sectors[s] for s in front_right_sectors if sectors[s] > 0.01]
        front_right_avg = sum(front_right_vals) / len(front_right_vals) if front_right_vals else 0.0

        # Reverse first
        for _ in range(12):
            send_velocity_cmd(-0.05, 0.0)
            time.sleep(0.1)

        # Turn toward the clearer side
        # Positive w = turn left, negative w = turn right
        if front_left_avg > front_right_avg + 0.1:
            turn_w = 0.5  # Turn left
            direction = "left"
        elif front_right_avg > front_left_avg + 0.1:
            turn_w = -0.5  # Turn right
            direction = "right"
        else:
            # Similar clearance - turn left by default
            turn_w = 0.5
            direction = "left (default)"

        print(f"[BACKUP CLEAR] Turning {direction} (L_avg={front_left_avg:.2f}m R_avg={front_right_avg:.2f}m)")

        # Turn for longer than normal backup to get a better angle
        for _ in range(15):
            send_velocity_cmd(0.0, turn_w)
            time.sleep(0.1)

        send_velocity_cmd(0.0, 0.0)

        # Clear heading lock and reset velocity state
        self.locked_heading = None
        self.last_v = 0.0
        self.last_w = 0.0

    def _turn_around(self):
        """
        Turn 180 degrees to face the opposite direction.

        Used when goal selection fails repeatedly, indicating the robot
        is stuck in a corner with no valid forward goals.
        """
        print(f"[TURN 180] Rotating to face opposite direction...")

        # First, back up a bit to get clearance
        for _ in range(8):
            send_velocity_cmd(-0.05, 0.0)
            time.sleep(0.1)

        # Turn 180 degrees (approximately 3-4 seconds at 0.5 rad/s)
        # Use positive angular velocity (turn left) - arbitrary choice
        turn_duration = 3.2  # ~180° at 0.5 rad/s (π / 0.5 ≈ 6.28, but with ramp-up use 3.2s)
        turn_w = 0.55

        start_time = time.time()
        while time.time() - start_time < turn_duration:
            send_velocity_cmd(0.0, turn_w)
            time.sleep(0.1)

        send_velocity_cmd(0.0, 0.0)

        # Clear heading lock and reset velocity state
        self.locked_heading = None
        self.last_v = 0.0
        self.last_w = 0.0

        print(f"[TURN 180] Complete")

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

    def _setup_output_logging(self):
        """Setup output directory and logging to file."""
        # Create output directory with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.output_dir = os.path.join(base_dir, "outputs", timestamp)

        # Create subdirectories
        logs_dir = os.path.join(self.output_dir, "logs")
        maps_dir = os.path.join(self.output_dir, "maps")
        os.makedirs(logs_dir, exist_ok=True)
        os.makedirs(maps_dir, exist_ok=True)

        # Setup logging to file
        log_path = os.path.join(logs_dir, "navigation.log")
        self.log_file = open(log_path, 'w')
        self._original_stdout = sys.stdout
        sys.stdout = TeeLogger(self.log_file)

        print(f"[OUTPUT] Logging to: {self.output_dir}")
        print(f"[OUTPUT] Log file: {log_path}")

    def _cleanup_logging(self):
        """Restore stdout and close log file."""
        if self._original_stdout:
            sys.stdout = self._original_stdout
        if self.log_file:
            self.log_file.close()

    def _save_map(self):
        """Save the current map to the output directory."""
        if not self.output_dir:
            return

        maps_dir = os.path.join(self.output_dir, "maps")
        try:
            # Save map using ROS map_saver
            result = subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
                 'source /opt/ros/humble/setup.bash && cd /tmp && '
                 'ros2 run nav2_map_server map_saver_cli -f map --ros-args -p save_map_timeout:=10.0'],
                capture_output=True, text=True, timeout=15
            )

            if result.returncode == 0:
                # Copy files from container
                subprocess.run(['docker', 'cp', f'{CONTAINER_NAME}:/tmp/map.pgm', maps_dir], check=True)
                subprocess.run(['docker', 'cp', f'{CONTAINER_NAME}:/tmp/map.yaml', maps_dir], check=True)

                # Convert to PNG
                try:
                    from PIL import Image
                    pgm_path = os.path.join(maps_dir, 'map.pgm')
                    png_path = os.path.join(maps_dir, 'map.png')
                    img = Image.open(pgm_path)
                    img.save(png_path)
                    print(f"[OUTPUT] Map saved to: {maps_dir}")
                except ImportError:
                    print(f"[OUTPUT] Map saved (PGM only, PIL not available for PNG conversion)")
            else:
                print(f"[OUTPUT] Map save failed: {result.stderr}")
        except Exception as e:
            print(f"[OUTPUT] Map save error: {e}")

    def _update_drive_accuracy(self, goal_start: Tuple[float, float], goal_end: Tuple[float, float]):
        """
        Update drive accuracy metrics.

        Calculates cross-track error (perpendicular distance from ideal line)
        and updates cumulative RMS error.
        """
        if not self.current_pos or not goal_start:
            return

        # Calculate cross-track error (perpendicular distance from ideal line)
        # Line from goal_start to goal_end
        dx = goal_end[0] - goal_start[0]
        dy = goal_end[1] - goal_start[1]
        line_len = math.sqrt(dx*dx + dy*dy)

        if line_len < 0.01:
            return

        # Perpendicular distance from current position to line
        # Using formula: |((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)| / sqrt((y2-y1)^2 + (x2-x1)^2)
        cross_track_error = abs(
            dy * self.current_pos[0] - dx * self.current_pos[1] +
            goal_end[0] * goal_start[1] - goal_end[1] * goal_start[0]
        ) / line_len

        self.drive_error_sum_sq += cross_track_error ** 2
        self.drive_error_count += 1

        # Track actual distance traveled
        if self.last_pos_for_distance:
            dist_moved = math.sqrt(
                (self.current_pos[0] - self.last_pos_for_distance[0])**2 +
                (self.current_pos[1] - self.last_pos_for_distance[1])**2
            )
            self.drive_total_distance += dist_moved
        self.last_pos_for_distance = self.current_pos

    def _get_drive_rms_error(self) -> float:
        """Get RMS cross-track error."""
        if self.drive_error_count == 0:
            return 0.0
        return math.sqrt(self.drive_error_sum_sq / self.drive_error_count)

    def _get_drive_efficiency(self) -> float:
        """Get drive efficiency (ideal distance / actual distance)."""
        if self.drive_total_distance < 0.01:
            return 1.0
        return self.drive_ideal_distance / self.drive_total_distance

    def _print_stats(self):
        """Print final statistics."""
        elapsed = time.time() - self.start_time
        print(f"\n{'=' * 55}")
        print(f"NBV NAVIGATION COMPLETE")
        print(f"{'=' * 55}")
        print(f"Final state: {self.nav_state.value}")
        print(f"Duration: {elapsed:.1f}s")
        print(f"Iterations: {self.iteration}")
        print(f"Goals reached: {self.goals_reached}")
        print(f"Backups: {self.backups}")
        print(f"Unique cells visited: {len(self.visited)}")
        print(f"Cells scanned by LiDAR: {len(self.scanned)}")
        print(f"Walls detected: {len(self.scan_ends)}")

        # Drive accuracy stats
        print(f"\n--- Drive Accuracy ---")
        rms_error = self._get_drive_rms_error()
        efficiency = self._get_drive_efficiency()
        print(f"Cross-track RMS error: {rms_error*100:.1f}cm")
        print(f"Total distance traveled: {self.drive_total_distance:.2f}m")
        print(f"Ideal distance (sum of goals): {self.drive_ideal_distance:.2f}m")
        print(f"Drive efficiency: {efficiency*100:.1f}%")

        # Return phase stats
        if self.return_start_time > 0:
            return_duration = time.time() - self.return_start_time
            print(f"\n--- Return Phase ---")
            print(f"Return phase duration: {return_duration:.1f}s")
            if self.start_pos and self.current_pos:
                final_dist = math.sqrt(
                    (self.current_pos[0] - self.start_pos[0])**2 +
                    (self.current_pos[1] - self.start_pos[1])**2)
                print(f"Final distance to origin: {final_dist:.2f}m")

        total = sum(self.visited.values())
        if total > 0:
            revisit_pct = 100 * (total - len(self.visited)) / total
            print(f"Revisit rate: {revisit_pct:.1f}%")

        # Map accuracy suggestions
        print(f"\n--- Map Accuracy Suggestions ---")
        print("To measure map accuracy, consider:")
        print("1. Ground truth comparison: Place markers at known positions,")
        print("   compare measured vs actual distances between them")
        print("2. Loop closure error: After returning to origin, check")
        print("   odometry drift (final_dist to origin should be ~0)")
        print("3. Feature alignment: Compare map walls to actual room layout")
        print("4. Consistency check: Run multiple times, overlay maps")
        print("5. Tape measure validation: Measure actual room dimensions,")
        print("   compare to map scale (resolution * pixels)")


def run_simple_nav(speed: float = 0.08, duration: float = 60.0,
                   backup_threshold: float = 0.15, blocked_margin: float = 0.15,
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
    parser.add_argument("--backup-threshold", type=float, default=0.15, help="Backup if closer than this (m)")
    parser.add_argument("--blocked-margin", type=float, default=0.15, help="Margin for goal blocked detection (m)")
    parser.add_argument("--debug-marker", action="store_true", help="Continuously publish goal marker to RViz")
    args = parser.parse_args()

    run_simple_nav(speed=args.speed, duration=args.duration,
                   backup_threshold=args.backup_threshold, blocked_margin=args.blocked_margin,
                   debug_marker=args.debug_marker)

