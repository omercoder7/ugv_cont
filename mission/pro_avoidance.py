"""
Professional obstacle avoidance algorithms.

Incorporates concepts from:
- Nav2 MPPI controller (time-to-collision, obstacle critics)
- VFH+ (look-ahead verification, gap analysis)
- DWA (adaptive weights, dynamic window)
- Gap-based navigation (gap quality scoring)
- Elastic bands (repulsive forces)

References:
- Fox et al. "The Dynamic Window Approach to Collision Avoidance"
- Borenstein & Koren "Vector Field Histogram - Fast Obstacle Avoidance"
- Nav2 MPPI documentation
"""

import math
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass, field


@dataclass
class GapInfo:
    """Information about a navigable gap between obstacles."""
    start_sector: int
    end_sector: int
    center_sector: int
    width_sectors: int
    min_distance: float  # Minimum distance within the gap
    max_distance: float  # Maximum distance (depth) within the gap
    navigability: float  # 0-1 score of how navigable
    is_dead_end: bool


@dataclass
class TTCResult:
    """Time-to-collision calculation result."""
    ttc: float  # Time to collision in seconds (inf if no collision)
    collision_distance: float  # Distance at which collision would occur
    collision_sector: int  # Which sector has the collision
    is_critical: bool  # True if TTC < critical threshold
    recommended_speed: float  # Recommended speed based on TTC


@dataclass
class DirectionScore:
    """Comprehensive direction score with breakdown."""
    sector: int
    total_score: float
    breakdown: Dict[str, float] = field(default_factory=dict)
    gap_info: Optional[GapInfo] = None
    ttc: Optional[TTCResult] = None


class TimeToCollision:
    """
    Calculates time-to-collision (TTC) for proactive speed control.

    Based on Nav2 MPPI obstacle critic concepts.
    """

    def __init__(self, robot_radius: float = 0.09,
                 critical_ttc: float = 1.5,
                 warning_ttc: float = 3.0):
        """
        Args:
            robot_radius: Robot body radius (half-width)
            critical_ttc: TTC threshold for emergency action (seconds)
            warning_ttc: TTC threshold for speed reduction (seconds)
        """
        self.robot_radius = robot_radius
        self.critical_ttc = critical_ttc
        self.warning_ttc = warning_ttc

        # Safety margin for collision detection
        self.safety_margin = 0.05  # 5cm extra margin

    def calculate(self, sector_distances: List[float],
                  current_linear: float, current_angular: float,
                  target_sector: int = 0) -> TTCResult:
        """
        Calculate time-to-collision in the direction of travel.

        Args:
            sector_distances: Current LiDAR sector distances
            current_linear: Current linear velocity (m/s)
            current_angular: Current angular velocity (rad/s)
            target_sector: Target direction sector (0 = front)

        Returns:
            TTCResult with TTC and recommendations
        """
        if current_linear <= 0.01:
            # Not moving forward, no forward collision possible
            return TTCResult(
                ttc=float('inf'),
                collision_distance=float('inf'),
                collision_sector=-1,
                is_critical=False,
                recommended_speed=1.0
            )

        num_sectors = len(sector_distances)

        # Check sectors in the direction of travel
        # When turning, we need to check sectors in the arc of travel
        sectors_to_check = self._get_travel_arc_sectors(
            target_sector, current_angular, num_sectors
        )

        min_ttc = float('inf')
        collision_sector = -1
        collision_distance = float('inf')

        for sector in sectors_to_check:
            dist = sector_distances[sector]

            # Skip blind spots and very far obstacles
            if dist <= 0.0 or dist > 5.0:
                continue

            # Effective distance accounting for robot radius and safety margin
            effective_dist = dist - self.robot_radius - self.safety_margin

            if effective_dist <= 0:
                # Already in collision zone!
                return TTCResult(
                    ttc=0.0,
                    collision_distance=dist,
                    collision_sector=sector,
                    is_critical=True,
                    recommended_speed=0.0
                )

            # Calculate TTC for this sector
            # Account for angular velocity - if turning away, TTC increases
            sector_angle = self._sector_to_angle(sector, num_sectors)
            effective_speed = self._project_velocity(
                current_linear, current_angular, sector_angle
            )

            if effective_speed > 0.01:
                ttc = effective_dist / effective_speed
                if ttc < min_ttc:
                    min_ttc = ttc
                    collision_sector = sector
                    collision_distance = dist

        # Determine criticality and recommended speed
        is_critical = min_ttc < self.critical_ttc

        if min_ttc < self.critical_ttc:
            # Critical zone - strong speed reduction or stop
            recommended_speed = max(0.0, min_ttc / self.critical_ttc * 0.3)
        elif min_ttc < self.warning_ttc:
            # Warning zone - gradual speed reduction
            ratio = (min_ttc - self.critical_ttc) / (self.warning_ttc - self.critical_ttc)
            recommended_speed = 0.3 + 0.7 * ratio
        else:
            # Safe zone
            recommended_speed = 1.0

        return TTCResult(
            ttc=min_ttc,
            collision_distance=collision_distance,
            collision_sector=collision_sector,
            is_critical=is_critical,
            recommended_speed=recommended_speed
        )

    def _get_travel_arc_sectors(self, target_sector: int, angular_vel: float,
                                 num_sectors: int) -> List[int]:
        """Get sectors in the arc of travel based on velocity."""
        sectors = [target_sector]

        # Width of arc depends on angular velocity
        # Higher angular velocity = check wider arc
        arc_width = 1 + int(abs(angular_vel) * 2)  # 1-3 sectors wide

        for offset in range(1, arc_width + 1):
            left = (target_sector + offset) % num_sectors
            right = (target_sector - offset) % num_sectors
            sectors.append(left)
            sectors.append(right)

        return sectors

    def _sector_to_angle(self, sector: int, num_sectors: int) -> float:
        """Convert sector to angle in radians."""
        sector_rad = 2 * math.pi / num_sectors
        angle = sector * sector_rad
        if angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def _project_velocity(self, linear: float, angular: float,
                          sector_angle: float) -> float:
        """Project velocity onto sector direction."""
        # Effective approach speed toward sector
        # If turning toward sector, speed increases; if away, decreases
        cos_angle = math.cos(sector_angle)

        # Simple projection for slow robot
        # For higher speeds, would need full trajectory prediction
        approach_speed = linear * max(0, cos_angle)

        return approach_speed


class GapAnalyzer:
    """
    Analyzes gaps in obstacle field for navigation.

    Based on VFH+ and gap-based navigation concepts.
    """

    def __init__(self, num_sectors: int = 12, robot_width: float = 0.18):
        self.num_sectors = num_sectors
        self.robot_width = robot_width
        self.min_gap_width = robot_width + 0.10  # Need 10cm clearance on sides

    def find_gaps(self, sector_distances: List[float],
                  min_distance: float) -> List[GapInfo]:
        """
        Find all navigable gaps in the obstacle field.

        A gap is a sequence of consecutive sectors where:
        - Distance > min_distance (not blocked)
        - Width is sufficient for robot to pass

        Args:
            sector_distances: Current LiDAR sector distances
            min_distance: Threshold for considering a sector blocked

        Returns:
            List of GapInfo objects for each gap found
        """
        gaps = []

        # Find contiguous free regions (valleys in VFH terminology)
        in_gap = False
        gap_start = 0

        # Extend array for wraparound detection
        extended = sector_distances + sector_distances

        for i in range(len(extended)):
            dist = extended[i % self.num_sectors]
            is_free = dist > min_distance and dist > 0.0

            if is_free and not in_gap:
                # Gap starts
                in_gap = True
                gap_start = i
            elif not is_free and in_gap:
                # Gap ends
                in_gap = False
                gap_end = i - 1

                # Only process gaps that start in first revolution
                if gap_start < self.num_sectors:
                    gap = self._analyze_gap(
                        gap_start % self.num_sectors,
                        gap_end % self.num_sectors,
                        sector_distances,
                        min_distance
                    )
                    if gap is not None:
                        gaps.append(gap)

        # Handle gap that wraps around
        if in_gap and gap_start < self.num_sectors:
            gap_end = len(extended) - 1
            gap = self._analyze_gap(
                gap_start % self.num_sectors,
                gap_end % self.num_sectors,
                sector_distances,
                min_distance
            )
            if gap is not None:
                gaps.append(gap)

        return gaps

    def _analyze_gap(self, start: int, end: int,
                     sector_distances: List[float],
                     min_distance: float) -> Optional[GapInfo]:
        """Analyze a single gap for navigability."""
        # Calculate gap width in sectors (handling wraparound)
        if end >= start:
            width = end - start + 1
        else:
            width = (self.num_sectors - start) + end + 1

        # Limit to max sectors
        width = min(width, self.num_sectors)

        # Get center sector
        center = (start + width // 2) % self.num_sectors

        # Calculate min/max distances in gap
        gap_distances = []
        for i in range(width):
            sector = (start + i) % self.num_sectors
            dist = sector_distances[sector]
            if dist > 0:  # Skip blind spots
                gap_distances.append(dist)

        if not gap_distances:
            return None

        min_dist = min(gap_distances)
        max_dist = max(gap_distances)

        # Estimate physical width at closest point
        # Approximate using sector angle span
        sector_angle = 2 * math.pi / self.num_sectors
        physical_width = 2 * min_dist * math.sin(width * sector_angle / 2)

        # Check if robot fits
        can_fit = physical_width >= self.min_gap_width

        if not can_fit:
            return None

        # Calculate navigability score
        navigability = self._calculate_navigability(
            width, min_dist, max_dist, physical_width, sector_distances, center
        )

        # Check for dead-end (front close, sides also close)
        is_dead_end = self._check_dead_end(center, sector_distances, min_distance)

        return GapInfo(
            start_sector=start,
            end_sector=end,
            center_sector=center,
            width_sectors=width,
            min_distance=min_dist,
            max_distance=max_dist,
            navigability=navigability,
            is_dead_end=is_dead_end
        )

    def _calculate_navigability(self, width: int, min_dist: float,
                                 max_dist: float, physical_width: float,
                                 sector_distances: List[float],
                                 center: int) -> float:
        """Calculate how navigable a gap is (0-1)."""
        scores = []

        # 1. Width score - wider gaps are better
        width_ratio = min(width / 4.0, 1.0)  # Full score at 4 sectors
        scores.append(("width", 0.25, width_ratio))

        # 2. Depth score - deeper gaps (farther obstacles) are better
        depth_ratio = min(max_dist / 2.0, 1.0)  # Full score at 2m
        scores.append(("depth", 0.25, depth_ratio))

        # 3. Clearance score - more clearance = safer
        clearance_ratio = min((physical_width - self.robot_width) / 0.3, 1.0)
        scores.append(("clearance", 0.20, max(0, clearance_ratio)))

        # 4. Expansion score - does the gap widen or narrow?
        left = sector_distances[(center + 1) % self.num_sectors]
        right = sector_distances[(center - 1) % self.num_sectors]
        left2 = sector_distances[(center + 2) % self.num_sectors]
        right2 = sector_distances[(center - 2) % self.num_sectors]

        inner_avg = (left + right) / 2
        outer_avg = (left2 + right2) / 2

        if inner_avg > 0.1:
            expansion = min(outer_avg / inner_avg, 1.5) / 1.5
        else:
            expansion = 0.5
        scores.append(("expansion", 0.15, expansion))

        # 5. Center alignment - prefer gaps aligned with front
        if center <= self.num_sectors // 4:
            alignment = 1.0 - (center / (self.num_sectors // 4)) * 0.5
        elif center >= 3 * self.num_sectors // 4:
            alignment = 1.0 - ((self.num_sectors - center) / (self.num_sectors // 4)) * 0.5
        else:
            alignment = 0.3
        scores.append(("alignment", 0.15, alignment))

        # Weighted sum
        total = sum(weight * score for _, weight, score in scores)
        return total

    def _check_dead_end(self, center: int, sector_distances: List[float],
                        min_distance: float) -> bool:
        """Check if gap leads to a dead-end."""
        # A dead-end is when the gap has limited depth and sides close in
        center_dist = sector_distances[center]

        if center_dist > 2.0:
            return False  # Far away, can't tell

        # Check if sides are closing in
        left = sector_distances[(center + 2) % self.num_sectors]
        right = sector_distances[(center - 2) % self.num_sectors]

        # Dead-end: front is close AND both sides are close
        is_dead_end = (
            center_dist < min_distance + 0.3 and
            left < min_distance + 0.2 and
            right < min_distance + 0.2
        )

        return is_dead_end

    def find_best_gap(self, gaps: List[GapInfo],
                      target_sector: int = 0,
                      previous_sector: Optional[int] = None) -> Optional[GapInfo]:
        """
        Find the best gap for navigation.

        Considers navigability, proximity to target, and continuity.
        """
        if not gaps:
            return None

        best_gap = None
        best_score = float('-inf')

        for gap in gaps:
            if gap.is_dead_end:
                continue  # Skip dead-ends

            score = gap.navigability

            # Bonus for being close to target direction
            angle_diff = self._sector_diff(gap.center_sector, target_sector)
            target_bonus = 1.0 - (angle_diff / (self.num_sectors // 2)) * 0.3
            score *= target_bonus

            # Bonus for continuity with previous direction
            if previous_sector is not None:
                continuity_diff = self._sector_diff(gap.center_sector, previous_sector)
                continuity_bonus = 1.0 - (continuity_diff / (self.num_sectors // 2)) * 0.2
                score *= continuity_bonus

            if score > best_score:
                best_score = score
                best_gap = gap

        return best_gap

    def _sector_diff(self, a: int, b: int) -> int:
        """Calculate minimum sector difference (handles wraparound)."""
        diff = abs(a - b)
        if diff > self.num_sectors // 2:
            diff = self.num_sectors - diff
        return diff


class RepulsiveField:
    """
    Calculates repulsive forces from obstacles (Elastic Band concept).

    Obstacles exert repulsive force that pushes the robot away.
    Force magnitude is inversely proportional to distance.
    """

    def __init__(self, influence_distance: float = 1.5,
                 robot_radius: float = 0.09):
        self.influence_distance = influence_distance
        self.robot_radius = robot_radius

    def calculate_repulsion(self, sector_distances: List[float]) -> Tuple[float, float]:
        """
        Calculate net repulsive force from all obstacles.

        Returns:
            (force_x, force_y) in robot frame (positive x = front)
        """
        num_sectors = len(sector_distances)
        total_fx = 0.0
        total_fy = 0.0

        for sector, dist in enumerate(sector_distances):
            if dist <= 0 or dist > self.influence_distance:
                continue

            # Angle of obstacle from robot
            angle = 2 * math.pi * sector / num_sectors
            if angle > math.pi:
                angle -= 2 * math.pi

            # Repulsive magnitude (inverse square law with saturation)
            effective_dist = max(dist - self.robot_radius, 0.05)
            magnitude = 1.0 / (effective_dist * effective_dist)

            # Cap magnitude to prevent extreme forces from very close obstacles
            magnitude = min(magnitude, 100.0)

            # Force points away from obstacle
            fx = -magnitude * math.cos(angle)
            fy = -magnitude * math.sin(angle)

            total_fx += fx
            total_fy += fy

        return total_fx, total_fy

    def get_velocity_adjustment(self, sector_distances: List[float],
                                base_linear: float,
                                base_angular: float) -> Tuple[float, float]:
        """
        Adjust velocity commands based on repulsive forces.

        Returns:
            (adjusted_linear, adjusted_angular)
        """
        fx, fy = self.calculate_repulsion(sector_distances)

        # Normalize force magnitude
        force_mag = math.sqrt(fx*fx + fy*fy)

        if force_mag < 0.1:
            return base_linear, base_angular

        # Reduce forward speed based on forward repulsion
        # If fx is negative (obstacles in front), slow down
        speed_factor = max(0.2, 1.0 + fx / force_mag * 0.5)
        adjusted_linear = base_linear * speed_factor

        # Add angular velocity to steer away from obstacles
        # fy > 0 means obstacles on left, steer right (negative angular)
        angular_adjustment = -fy / force_mag * 0.3
        adjusted_angular = base_angular + angular_adjustment

        return adjusted_linear, adjusted_angular


class AdaptiveDWA:
    """
    Dynamic Window Approach with adaptive weights.

    Weights change based on environment:
    - Corridor: Higher clearance weight, smoother turns
    - Open space: Balance between progress and safety
    - Near obstacles: Maximum safety focus
    """

    # Environment-based weight presets
    WEIGHT_PRESETS = {
        "open": {
            "clearance": 2.0,
            "velocity": 1.5,
            "heading": 1.0,
            "goal": 1.5,
        },
        "corridor": {
            "clearance": 4.0,
            "velocity": 1.0,
            "heading": 2.0,  # Prefer straight
            "goal": 1.0,
        },
        "danger": {
            "clearance": 6.0,
            "velocity": 0.5,
            "heading": 1.5,
            "goal": 0.5,
        },
        "stuck": {
            "clearance": 3.0,
            "velocity": 2.0,  # Need to move!
            "heading": 0.5,
            "goal": 0.5,
        }
    }

    def __init__(self, max_speed: float = 0.10, max_yaw_rate: float = 0.5,
                 robot_radius: float = 0.09):
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.robot_radius = robot_radius

        # Current weights (start with open space)
        self.weights = self.WEIGHT_PRESETS["open"].copy()
        self.current_mode = "open"

        # Trajectory simulation
        self.predict_time = 1.5
        self.dt = 0.1

        # Velocity samples
        self.linear_samples = 5
        self.angular_samples = 9

    def set_mode(self, mode: str):
        """Set weight preset based on environment."""
        if mode in self.WEIGHT_PRESETS:
            self.weights = self.WEIGHT_PRESETS[mode].copy()
            self.current_mode = mode

    def detect_environment(self, sector_distances: List[float],
                           min_distance: float) -> str:
        """Auto-detect environment type from sensor data."""
        front = sector_distances[0]
        left = sector_distances[3]
        right = sector_distances[9]

        num_close = sum(1 for d in sector_distances if 0 < d < min_distance + 0.1)
        avg_dist = sum(d for d in sector_distances if d > 0) / max(1, sum(1 for d in sector_distances if d > 0))

        # Detect danger zone
        if front < min_distance or num_close >= 4:
            return "danger"

        # Detect corridor
        if left < 1.0 and right < 1.0 and front > min_distance:
            return "corridor"

        # Open space
        if avg_dist > 1.5 and num_close <= 2:
            return "open"

        return "open"  # Default

    def compute_best_velocity(self, sector_distances: List[float],
                              target_angle: float = 0.0,
                              committed_direction: str = None) -> Tuple[float, float, Dict]:
        """
        Compute best velocity using adaptive DWA.

        Args:
            sector_distances: Current LiDAR distances
            target_angle: Target heading angle (radians)
            committed_direction: "left"/"right" to bias search

        Returns:
            (best_linear, best_angular, score_breakdown)
        """
        best_v = 0.0
        best_w = 0.0
        best_score = float('-inf')
        best_breakdown = {}

        # Generate linear velocity samples
        linear_values = [
            i * self.max_speed / (self.linear_samples - 1)
            for i in range(self.linear_samples)
        ]

        # Generate angular velocity samples
        angular_values = [
            -self.max_yaw_rate + i * 2 * self.max_yaw_rate / (self.angular_samples - 1)
            for i in range(self.angular_samples)
        ]

        for linear in linear_values:
            for angular in angular_values:
                # Apply direction commitment
                if committed_direction == "left" and angular < -0.1:
                    continue
                elif committed_direction == "right" and angular > 0.1:
                    continue

                # Simulate trajectory
                trajectory = self._simulate_trajectory(linear, angular)

                # Score trajectory
                score, breakdown = self._score_trajectory(
                    trajectory, sector_distances, linear, angular, target_angle
                )

                if score > best_score:
                    best_score = score
                    best_v = linear
                    best_w = angular
                    best_breakdown = breakdown

        return best_v, best_w, best_breakdown

    def _simulate_trajectory(self, linear: float, angular: float) -> List[Tuple[float, float, float]]:
        """Simulate trajectory from origin."""
        trajectory = [(0.0, 0.0, 0.0)]
        x, y, theta = 0.0, 0.0, 0.0

        steps = int(self.predict_time / self.dt)
        for _ in range(steps):
            theta += angular * self.dt
            x += linear * math.cos(theta) * self.dt
            y += linear * math.sin(theta) * self.dt
            trajectory.append((x, y, theta))

        return trajectory

    def _score_trajectory(self, trajectory: List[Tuple[float, float, float]],
                          sector_distances: List[float],
                          linear: float, angular: float,
                          target_angle: float) -> Tuple[float, Dict]:
        """Score a trajectory with adaptive weights."""
        if not trajectory:
            return float('-inf'), {"invalid": True}

        breakdown = {}

        # 1. Clearance score
        min_clearance = self._trajectory_clearance(trajectory, sector_distances)
        if min_clearance <= 0:
            return float('-inf'), {"collision": True}
        clearance_score = min(min_clearance / 1.0, 1.0)  # Normalize to 1m
        breakdown["clearance"] = clearance_score

        # 2. Velocity score
        velocity_score = linear / self.max_speed if self.max_speed > 0 else 0
        breakdown["velocity"] = velocity_score

        # 3. Heading score (prefer straight)
        heading_score = 1.0 - min(abs(angular) / self.max_yaw_rate, 1.0)
        breakdown["heading"] = heading_score

        # 4. Goal alignment score
        final_theta = trajectory[-1][2] if trajectory else 0
        angle_error = abs(final_theta - target_angle)
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        goal_score = 1.0 - min(abs(angle_error) / math.pi, 1.0)
        breakdown["goal"] = goal_score

        # Weighted sum
        total = (
            self.weights["clearance"] * clearance_score +
            self.weights["velocity"] * velocity_score +
            self.weights["heading"] * heading_score +
            self.weights["goal"] * goal_score
        )
        breakdown["total"] = total

        return total, breakdown

    def _trajectory_clearance(self, trajectory: List[Tuple[float, float, float]],
                               sector_distances: List[float]) -> float:
        """Calculate minimum clearance along trajectory."""
        num_sectors = len(sector_distances)
        min_clearance = float('inf')

        # Convert sector distances to obstacle points
        obstacles = []
        for sector, dist in enumerate(sector_distances):
            if 0.05 < dist < 2.0:
                angle = 2 * math.pi * sector / num_sectors
                ox = dist * math.cos(angle)
                oy = dist * math.sin(angle)
                obstacles.append((ox, oy))

        for tx, ty, _ in trajectory:
            for ox, oy in obstacles:
                d = math.sqrt((tx - ox)**2 + (ty - oy)**2)
                if d < self.robot_radius + 0.05:
                    return 0  # Collision
                min_clearance = min(min_clearance, d - self.robot_radius)

        return min_clearance


class DirectionalMemory:
    """
    Tracks which directions have failed during stuck recovery.

    Prevents repeating failed maneuvers and guides toward unexplored directions.
    """

    def __init__(self, num_sectors: int = 12, memory_duration: float = 60.0):
        self.num_sectors = num_sectors
        self.memory_duration = memory_duration

        # Track failed attempts: sector -> [(timestamp, severity), ...]
        self.failed_directions: Dict[int, List[Tuple[float, float]]] = {}

        # Track successful escapes
        self.successful_directions: Dict[int, List[float]] = {}

        # Current recovery attempt tracking
        self.current_attempt_sector: Optional[int] = None
        self.current_attempt_start: float = 0.0

    def start_attempt(self, sector: int):
        """Mark start of recovery attempt in a direction."""
        import time
        self.current_attempt_sector = sector
        self.current_attempt_start = time.time()

    def mark_failure(self, sector: int, severity: float = 1.0):
        """Mark a direction as failed."""
        import time
        current_time = time.time()

        if sector not in self.failed_directions:
            self.failed_directions[sector] = []

        self.failed_directions[sector].append((current_time, severity))

        # Clean old failures
        self._cleanup_old_entries()

        self.current_attempt_sector = None

    def mark_success(self, sector: int):
        """Mark a direction as successful."""
        import time
        current_time = time.time()

        if sector not in self.successful_directions:
            self.successful_directions[sector] = []

        self.successful_directions[sector].append(current_time)

        # Reduce failure weight for this sector
        if sector in self.failed_directions:
            self.failed_directions[sector] = [
                (t, s * 0.5) for t, s in self.failed_directions[sector]
            ]

        self.current_attempt_sector = None

    def get_failure_score(self, sector: int) -> float:
        """Get accumulated failure score for a sector (higher = worse)."""
        import time
        current_time = time.time()

        if sector not in self.failed_directions:
            return 0.0

        total = 0.0
        for timestamp, severity in self.failed_directions[sector]:
            age = current_time - timestamp
            if age < self.memory_duration:
                # Decay with time
                decay = 1.0 - (age / self.memory_duration)
                total += severity * decay

        return total

    def get_success_score(self, sector: int) -> float:
        """Get success score for a sector (higher = better)."""
        import time
        current_time = time.time()

        if sector not in self.successful_directions:
            return 0.0

        total = 0.0
        for timestamp in self.successful_directions[sector]:
            age = current_time - timestamp
            if age < self.memory_duration:
                decay = 1.0 - (age / self.memory_duration)
                total += decay

        return total

    def get_best_unexplored_direction(self, sector_distances: List[float],
                                       min_distance: float) -> Optional[int]:
        """Find the best direction that hasn't been tried recently."""
        best_sector = None
        best_score = float('-inf')

        for sector in range(self.num_sectors):
            dist = sector_distances[sector]

            # Must be passable
            if dist < min_distance or dist <= 0:
                continue

            # Calculate score
            failure_penalty = self.get_failure_score(sector) * 2.0
            success_bonus = self.get_success_score(sector) * 1.0
            distance_score = min(dist / 2.0, 1.0)

            # Prefer directions close to front
            front_bias = 1.0 - abs(self._sector_diff(sector, 0)) / (self.num_sectors // 2)

            score = distance_score + success_bonus - failure_penalty + front_bias * 0.3

            if score > best_score:
                best_score = score
                best_sector = sector

        return best_sector

    def _sector_diff(self, a: int, b: int) -> int:
        """Calculate minimum sector difference."""
        diff = abs(a - b)
        if diff > self.num_sectors // 2:
            diff = self.num_sectors - diff
        return diff

    def _cleanup_old_entries(self):
        """Remove entries older than memory_duration."""
        import time
        current_time = time.time()
        cutoff = current_time - self.memory_duration

        for sector in list(self.failed_directions.keys()):
            self.failed_directions[sector] = [
                (t, s) for t, s in self.failed_directions[sector]
                if t > cutoff
            ]
            if not self.failed_directions[sector]:
                del self.failed_directions[sector]

        for sector in list(self.successful_directions.keys()):
            self.successful_directions[sector] = [
                t for t in self.successful_directions[sector]
                if t > cutoff
            ]
            if not self.successful_directions[sector]:
                del self.successful_directions[sector]

    def clear(self):
        """Clear all memory."""
        self.failed_directions.clear()
        self.successful_directions.clear()
        self.current_attempt_sector = None


class ProObstacleAvoider:
    """
    Professional obstacle avoidance coordinator.

    Combines all advanced algorithms into a unified interface.
    """

    def __init__(self, num_sectors: int = 12, robot_width: float = 0.18,
                 max_speed: float = 0.10, max_yaw_rate: float = 0.5):
        self.num_sectors = num_sectors
        self.robot_width = robot_width
        self.robot_radius = robot_width / 2

        # Initialize components
        self.ttc = TimeToCollision(robot_radius=self.robot_radius)
        self.gap_analyzer = GapAnalyzer(num_sectors, robot_width)
        self.repulsive_field = RepulsiveField(robot_radius=self.robot_radius)
        self.adaptive_dwa = AdaptiveDWA(max_speed, max_yaw_rate, self.robot_radius)
        self.directional_memory = DirectionalMemory(num_sectors)

        # State
        self.previous_sector: Optional[int] = None
        self.current_gap: Optional[GapInfo] = None

    def compute_safe_velocity(self, sector_distances: List[float],
                               min_distance: float,
                               base_linear: float,
                               base_angular: float,
                               target_sector: int = 0,
                               is_stuck: bool = False,
                               committed_direction: str = None) -> Tuple[float, float, Dict]:
        """
        Compute safe velocity command using all avoidance algorithms.

        Args:
            sector_distances: Current LiDAR sector distances
            min_distance: Minimum safe distance threshold
            base_linear: Base linear velocity desired
            base_angular: Base angular velocity desired
            target_sector: Target direction (0 = front)
            is_stuck: Whether robot is currently stuck
            committed_direction: "left"/"right" direction commitment

        Returns:
            (safe_linear, safe_angular, debug_info)
        """
        debug_info = {
            "ttc": None,
            "gaps": [],
            "repulsion": (0, 0),
            "environment": "unknown",
            "mode": "normal"
        }

        # 1. Calculate time-to-collision
        ttc_result = self.ttc.calculate(
            sector_distances, base_linear, base_angular, target_sector
        )
        debug_info["ttc"] = ttc_result

        # 2. Detect environment and set DWA mode
        if is_stuck:
            env_mode = "stuck"
        elif ttc_result.is_critical:
            env_mode = "danger"
        else:
            env_mode = self.adaptive_dwa.detect_environment(sector_distances, min_distance)
        self.adaptive_dwa.set_mode(env_mode)
        debug_info["environment"] = env_mode

        # 3. Find navigable gaps
        gaps = self.gap_analyzer.find_gaps(sector_distances, min_distance)
        debug_info["gaps"] = [g.center_sector for g in gaps]

        # 4. Find best gap
        best_gap = self.gap_analyzer.find_best_gap(
            gaps, target_sector, self.previous_sector
        )
        self.current_gap = best_gap

        # 5. Get directional memory bias
        if is_stuck:
            # Use memory to find unexplored direction
            memory_sector = self.directional_memory.get_best_unexplored_direction(
                sector_distances, min_distance
            )
            if memory_sector is not None:
                target_sector = memory_sector
                debug_info["mode"] = "memory_guided"

        # 6. Compute DWA velocity
        if best_gap and not is_stuck:
            # Navigate toward gap center
            target_angle = 2 * math.pi * best_gap.center_sector / self.num_sectors
            if target_angle > math.pi:
                target_angle -= 2 * math.pi
        else:
            target_angle = 2 * math.pi * target_sector / self.num_sectors
            if target_angle > math.pi:
                target_angle -= 2 * math.pi

        dwa_linear, dwa_angular, dwa_breakdown = self.adaptive_dwa.compute_best_velocity(
            sector_distances, target_angle, committed_direction
        )
        debug_info["dwa"] = dwa_breakdown

        # 7. Apply repulsive field adjustment
        repel_linear, repel_angular = self.repulsive_field.get_velocity_adjustment(
            sector_distances, dwa_linear, dwa_angular
        )
        fx, fy = self.repulsive_field.calculate_repulsion(sector_distances)
        debug_info["repulsion"] = (fx, fy)

        # 8. Apply TTC speed limiting
        safe_linear = min(repel_linear, base_linear * ttc_result.recommended_speed)
        safe_angular = repel_angular

        # 9. Update tracking state
        if best_gap:
            self.previous_sector = best_gap.center_sector
        elif dwa_angular != 0:
            # Estimate sector from angular velocity
            if dwa_angular > 0.1:
                self.previous_sector = 1  # Turning left
            elif dwa_angular < -0.1:
                self.previous_sector = self.num_sectors - 1  # Turning right
            else:
                self.previous_sector = 0

        return safe_linear, safe_angular, debug_info

    def mark_stuck_direction(self, sector: int, severity: float = 1.0):
        """Mark a direction as having caused stuck situation."""
        self.directional_memory.mark_failure(sector, severity)

    def mark_escape_success(self, sector: int):
        """Mark a direction as having successfully escaped."""
        self.directional_memory.mark_success(sector)

    def get_ttc_status(self, sector_distances: List[float],
                       linear: float, angular: float) -> TTCResult:
        """Get current time-to-collision status."""
        return self.ttc.calculate(sector_distances, linear, angular)

    def reset(self):
        """Reset state for new navigation session."""
        self.previous_sector = None
        self.current_gap = None
        self.directional_memory.clear()
