"""
Vector Field Histogram (VFH) implementation for obstacle avoidance.
Based on Borenstein & Koren paper with key features:
- Polar histogram smoothing (reduces noise sensitivity)
- Hysteresis thresholding (prevents oscillation)
- Valley detection for finding gaps
"""

from typing import List, Tuple, Dict


class VFHController:
    """
    Vector Field Histogram (VFH) implementation for obstacle avoidance.
    """

    def __init__(self, num_sectors: int = 12, smoothing_window: int = 3):
        self.num_sectors = num_sectors
        self.smoothing_window = smoothing_window

        # Polar Obstacle Density histogram
        self.polar_histogram = [0.0] * num_sectors
        self.smoothed_histogram = [0.0] * num_sectors

        # Binary histogram (blocked/free) with hysteresis
        self.binary_histogram = [False] * num_sectors
        self.prev_binary_histogram = [False] * num_sectors

        # Hysteresis thresholds (tunable)
        self.threshold_high = 0.6
        self.threshold_low = 0.3

        # Previous selected direction for trajectory smoothing
        self.prev_direction = 0

    def update_histogram(self, sector_distances: List[float], min_distance: float):
        """
        Build polar histogram from sector distances.
        Uses distance-based thresholding with smooth transitions.
        """
        buffer_low = min_distance * 0.7
        buffer_high = min_distance * 1.2

        for i in range(self.num_sectors):
            dist = sector_distances[i]

            if dist < 0.1:
                self.polar_histogram[i] = 0.5
            elif dist < buffer_low:
                self.polar_histogram[i] = 1.0
            elif dist > buffer_high:
                self.polar_histogram[i] = 0.0
            else:
                ratio = (dist - buffer_low) / (buffer_high - buffer_low)
                self.polar_histogram[i] = 1.0 - ratio

        self._smooth_histogram()
        self._compute_binary_histogram()

    def _smooth_histogram(self):
        """Apply triangular-weighted smoothing to reduce noise sensitivity."""
        half_window = self.smoothing_window // 2

        for k in range(self.num_sectors):
            weighted_sum = 0.0
            weight_total = 0.0

            for i in range(-half_window, half_window + 1):
                idx = (k + i) % self.num_sectors
                weight = half_window + 1 - abs(i)
                weighted_sum += weight * self.polar_histogram[idx]
                weight_total += weight

            self.smoothed_histogram[k] = weighted_sum / weight_total if weight_total > 0 else 0

    def _compute_binary_histogram(self):
        """Convert smoothed histogram to binary using hysteresis thresholding."""
        for k in range(self.num_sectors):
            val = self.smoothed_histogram[k]

            if val > self.threshold_high:
                self.binary_histogram[k] = True
            elif val < self.threshold_low:
                self.binary_histogram[k] = False
            else:
                self.binary_histogram[k] = self.prev_binary_histogram[k]

        self.prev_binary_histogram = self.binary_histogram.copy()

    def find_best_valley(self, target_sector: int = 0) -> Tuple[int, float]:
        """
        Find the best valley (gap) in the binary histogram.
        Prefers valleys closest to target direction.

        Returns: (best_sector, confidence)
        """
        valleys = []
        in_valley = False
        start = 0

        extended = self.binary_histogram + self.binary_histogram

        for i in range(len(extended)):
            if not extended[i] and not in_valley:
                in_valley = True
                start = i
            elif extended[i] and in_valley:
                in_valley = False
                end = i - 1
                if start < self.num_sectors:
                    valley_width = end - start + 1
                    valley_width = min(valley_width, self.num_sectors)
                    valley_center = (start + valley_width // 2) % self.num_sectors
                    valleys.append((start % self.num_sectors, end % self.num_sectors, valley_center, valley_width))

        if in_valley and start < self.num_sectors:
            end = len(extended) - 1
            valley_width = min(end - start + 1, self.num_sectors)
            valley_center = (start + valley_width // 2) % self.num_sectors
            valleys.append((start % self.num_sectors, end % self.num_sectors, valley_center, valley_width))

        if not valleys:
            min_blocked = 1.0
            best_fallback = target_sector
            for i in range(self.num_sectors):
                if self.smoothed_histogram[i] < min_blocked:
                    min_blocked = self.smoothed_histogram[i]
                    best_fallback = i
            return best_fallback, 0.0

        best_sector = target_sector
        best_score = -999

        for start, end, center, width in valleys:
            target_diff = self._angle_diff(center, target_sector)
            prev_diff = self._angle_diff(center, self.prev_direction)

            score = (
                3.0 * (1.0 - abs(target_diff) / 6.0) +
                2.0 * min(width / 4.0, 1.0) +
                1.0 * (1.0 - abs(prev_diff) / 6.0)
            )

            if score > best_score:
                best_score = score
                best_sector = center

        self.prev_direction = best_sector
        confidence = min(1.0, max(0.0, best_score / 6.0))

        return best_sector, confidence

    def _angle_diff(self, a: int, b: int) -> int:
        """Compute shortest angular difference between sectors"""
        diff = (a - b) % self.num_sectors
        if diff > self.num_sectors // 2:
            diff -= self.num_sectors
        return diff

    def is_path_clear(self, sector: int) -> bool:
        """Check if a specific sector is clear (not blocked)"""
        return not self.binary_histogram[sector]

    def get_clearance(self, sector: int) -> float:
        """Get inverse obstacle density for a sector (higher = more clear)"""
        return 1.0 - self.smoothed_histogram[sector]

    def calculate_space_freedom(self, sector: int, sector_distances: List[float]) -> float:
        """
        Calculate "space freedom" for a direction - how open/unconstrained the space is.

        Returns: 0.0 (dead-end/constrained) to 1.0 (wide open space)
        """
        target_dist = sector_distances[sector]

        if target_dist < 0.3:
            return 0.0

        left_1 = sector_distances[(sector + 1) % self.num_sectors]
        left_2 = sector_distances[(sector + 2) % self.num_sectors]
        right_1 = sector_distances[(sector - 1) % self.num_sectors]
        right_2 = sector_distances[(sector - 2) % self.num_sectors]

        clear_threshold = 0.6

        adjacent_clear = 0
        for offset in [-2, -1, 1, 2]:
            neighbor = (sector + offset) % self.num_sectors
            if sector_distances[neighbor] >= clear_threshold:
                adjacent_clear += 1

        width_score = adjacent_clear / 4.0

        arc_distances = [target_dist, left_1, right_1]
        min_arc_dist = min(arc_distances)

        min_dist_score = min(min_arc_dist / 3.0, 1.0)

        inner_avg = (left_1 + right_1) / 2.0
        outer_avg = (left_2 + right_2) / 2.0

        if outer_avg >= inner_avg:
            widening_score = 1.0
        else:
            ratio = outer_avg / max(inner_avg, 0.1)
            widening_score = max(0.2, ratio)

        left_min = min(left_1, left_2)
        right_min = min(right_1, right_2)

        if max(left_min, right_min) > 0.1:
            symmetry = min(left_min, right_min) / max(left_min, right_min)
        else:
            symmetry = 0.0

        symmetry_score = symmetry

        freedom = (
            0.35 * min_dist_score +
            0.30 * width_score +
            0.20 * widening_score +
            0.15 * symmetry_score
        )

        return freedom

    def detect_passage(self, sector_distances: List[float], robot_width: float) -> Dict[str, any]:
        """
        UNIFIED passage/corridor/gap detection.

        Returns:
            Dict with is_passage, passage_width, passage_direction, passage_type, can_fit, etc.
        """
        front = sector_distances[0]
        front_left = sector_distances[1]
        front_right = sector_distances[11]
        left = sector_distances[2]
        right = sector_distances[10]
        back_left = sector_distances[4]
        back_right = sector_distances[8]

        min_fit_width = robot_width + 0.08

        left_clearance = min(left, front_left)
        right_clearance = min(right, front_right)
        forward_width = left_clearance + right_clearance

        left_passage_width = min(front_left, front) + min(back_left, left)
        right_passage_width = min(front_right, front) + min(back_right, right)

        max_side_dist = 1.2
        min_front_clearance = 0.4

        is_corridor = (
            front > min_front_clearance and
            left_clearance < max_side_dist and
            right_clearance < max_side_dist and
            forward_width >= min_fit_width
        )

        front_blocked = front < min_front_clearance

        has_left_gap = front_blocked and (front_left > front + 0.3 or left > front + 0.3)
        left_gap_width = min(front_left, left) if has_left_gap else 0

        has_right_gap = front_blocked and (front_right > front + 0.3 or right > front + 0.3)
        right_gap_width = min(front_right, right) if has_right_gap else 0

        passage_type = "open"
        passage_width = forward_width
        passage_direction = 0
        is_passage = False

        if is_corridor:
            passage_type = "corridor"
            passage_width = forward_width
            is_passage = True
            if abs(left_clearance - right_clearance) < 0.10:
                passage_direction = 0
            elif left_clearance > right_clearance:
                passage_direction = 1
            else:
                passage_direction = 11

        elif has_left_gap and left_gap_width >= min_fit_width:
            passage_type = "gap_left"
            passage_width = left_gap_width
            passage_direction = 1 if front_left > left else 2
            is_passage = True

        elif has_right_gap and right_gap_width >= min_fit_width:
            passage_type = "gap_right"
            passage_width = right_gap_width
            passage_direction = 11 if front_right > right else 10
            is_passage = True

        can_fit = passage_width >= min_fit_width

        return {
            "is_passage": is_passage,
            "passage_width": passage_width,
            "passage_direction": passage_direction,
            "passage_type": passage_type,
            "can_fit": can_fit,
            "forward_width": forward_width,
            "left_clearance": left_clearance,
            "right_clearance": right_clearance,
        }

    def detect_corridor(self, sector_distances: List[float], robot_width: float) -> Tuple[bool, float, int]:
        """
        Detect if robot is in a corridor/narrow passage.
        Wrapper around detect_passage() for backward compatibility.

        Returns: (is_corridor, corridor_width, clear_direction)
        """
        passage = self.detect_passage(sector_distances, robot_width)

        is_corridor = passage["is_passage"] and passage["can_fit"]
        corridor_width = passage["passage_width"]
        clear_direction = passage["passage_direction"]

        return is_corridor, corridor_width, clear_direction

    def robot_fits_in_direction(self, sector: int, sector_distances: List[float],
                                  robot_half_width: float, min_clearance: float) -> bool:
        """
        Check if robot physically fits when moving toward a sector.

        Returns: True if robot can safely move in that direction
        """
        front_dist = sector_distances[sector]

        side_offset = 3
        left_side = (sector + side_offset) % self.num_sectors
        right_side = (sector - side_offset) % self.num_sectors

        left_dist = sector_distances[left_side]
        right_dist = sector_distances[right_side]

        front_ok = front_dist >= min_clearance
        sides_ok = left_dist >= robot_half_width and right_dist >= robot_half_width

        return front_ok and sides_ok
