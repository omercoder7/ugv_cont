#!/usr/bin/env python3
"""
Offline Map Builder - Phase 2 of Two-Phase Mapping

Processes recorded sensor data to build high-quality occupancy grid maps.
Uses ICP for scan alignment and builds multi-resolution maps.

Usage:
    # Process recorded data
    python offline_mapper.py /path/to/exploration_data --output ./maps/room1

    # With visualization
    python offline_mapper.py ./data --output ./maps/room1 --visualize

Requirements:
    pip install numpy opencv-python scipy matplotlib pillow

Optional (for better ICP):
    pip install open3d
"""

import os
import sys
import json
import gzip
import math
import argparse
from datetime import datetime
from typing import List, Dict, Tuple, Optional, Any
from dataclasses import dataclass
import numpy as np

try:
    import cv2
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False
    print("Warning: OpenCV not available. Install with: pip install opencv-python")

try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    print("Note: Open3D not available. Using basic ICP. Install with: pip install open3d")


# Map building parameters
DEFAULT_RESOLUTION = 0.05  # 5cm per cell
DEFAULT_MAP_SIZE = 20.0    # 20m x 20m default
OCCUPIED_THRESHOLD = 0.65  # Probability to mark as occupied
FREE_THRESHOLD = 0.35      # Probability to mark as free
LOG_ODDS_OCCUPIED = 0.85   # Log-odds update for occupied
LOG_ODDS_FREE = -0.4       # Log-odds update for free


@dataclass
class Pose2D:
    """2D pose (x, y, theta)"""
    x: float
    y: float
    theta: float

    def to_matrix(self) -> np.ndarray:
        """Convert to 3x3 transformation matrix"""
        c, s = np.cos(self.theta), np.sin(self.theta)
        return np.array([
            [c, -s, self.x],
            [s, c, self.y],
            [0, 0, 1]
        ])

    @classmethod
    def from_quaternion(cls, x: float, y: float, qz: float, qw: float) -> 'Pose2D':
        """Create from position and quaternion (assuming 2D, qx=qy=0)"""
        theta = 2 * math.atan2(qz, qw)
        return cls(x=x, y=y, theta=theta)


@dataclass
class ScanData:
    """Processed scan data"""
    timestamp: float
    pose: Pose2D
    ranges: np.ndarray
    angle_min: float
    angle_max: float
    angle_increment: float

    def to_points(self, max_range: float = 10.0) -> np.ndarray:
        """Convert scan to 2D points in local frame"""
        points = []
        angle = self.angle_min

        for r in self.ranges:
            if 0.05 < r < max_range:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y])
            angle += self.angle_increment

        return np.array(points) if points else np.empty((0, 2))

    def to_world_points(self, max_range: float = 10.0) -> np.ndarray:
        """Convert scan to 2D points in world frame"""
        local_points = self.to_points(max_range)
        if len(local_points) == 0:
            return local_points

        # Transform to world frame
        c, s = np.cos(self.pose.theta), np.sin(self.pose.theta)
        rotation = np.array([[c, -s], [s, c]])
        translation = np.array([self.pose.x, self.pose.y])

        world_points = local_points @ rotation.T + translation
        return world_points


class OccupancyGridBuilder:
    """
    Builds occupancy grid from scan data using log-odds update.
    """

    def __init__(self, resolution: float = DEFAULT_RESOLUTION,
                 size: float = DEFAULT_MAP_SIZE,
                 origin: Tuple[float, float] = None):
        """
        Initialize occupancy grid.

        Args:
            resolution: Meters per cell
            size: Map size in meters (will be centered on origin)
            origin: Map origin (default: center at 0,0)
        """
        self.resolution = resolution
        self.size = size

        # Calculate grid dimensions
        self.width = int(size / resolution)
        self.height = int(size / resolution)

        # Origin in world coordinates (center of map)
        if origin is None:
            self.origin_x = -size / 2
            self.origin_y = -size / 2
        else:
            self.origin_x = origin[0]
            self.origin_y = origin[1]

        # Log-odds grid (0 = unknown)
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

        # Track which cells have been observed
        self.observed = np.zeros((self.height, self.width), dtype=bool)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid cell"""
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid cell to world coordinates (cell center)"""
        x = self.origin_x + (gx + 0.5) * self.resolution
        y = self.origin_y + (gy + 0.5) * self.resolution
        return x, y

    def is_valid_cell(self, gx: int, gy: int) -> bool:
        """Check if grid cell is within bounds"""
        return 0 <= gx < self.width and 0 <= gy < self.height

    def update_from_scan(self, scan: ScanData, max_range: float = 10.0):
        """
        Update occupancy grid from a single scan using ray casting.

        Args:
            scan: Scan data with pose and ranges
            max_range: Maximum range to consider
        """
        robot_gx, robot_gy = self.world_to_grid(scan.pose.x, scan.pose.y)

        angle = scan.angle_min + scan.pose.theta

        for r in scan.ranges:
            # Skip invalid ranges
            if not (0.05 < r < max_range):
                angle += scan.angle_increment
                continue

            # Calculate endpoint
            end_x = scan.pose.x + r * np.cos(angle)
            end_y = scan.pose.y + r * np.sin(angle)
            end_gx, end_gy = self.world_to_grid(end_x, end_y)

            # Ray cast from robot to endpoint
            self._ray_cast(robot_gx, robot_gy, end_gx, end_gy)

            angle += scan.angle_increment

    def _ray_cast(self, x0: int, y0: int, x1: int, y1: int):
        """
        Cast ray from (x0,y0) to (x1,y1) using Bresenham's algorithm.
        Mark cells along ray as free, endpoint as occupied.
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0

        while True:
            if self.is_valid_cell(x, y):
                if x == x1 and y == y1:
                    # Endpoint - occupied
                    self.log_odds[y, x] += LOG_ODDS_OCCUPIED
                    self.observed[y, x] = True
                else:
                    # Along ray - free
                    self.log_odds[y, x] += LOG_ODDS_FREE
                    self.observed[y, x] = True

                # Clamp log-odds to prevent saturation
                self.log_odds[y, x] = np.clip(self.log_odds[y, x], -10, 10)

            if x == x1 and y == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def get_occupancy_grid(self) -> np.ndarray:
        """
        Get occupancy grid as probability values.

        Returns:
            2D array with values:
            - 0.5: Unknown
            - 0.0-0.35: Free
            - 0.65-1.0: Occupied
        """
        # Convert log-odds to probability
        prob = 1.0 / (1.0 + np.exp(-self.log_odds))

        # Set unobserved cells to 0.5 (unknown)
        prob[~self.observed] = 0.5

        return prob

    def get_map_image(self) -> np.ndarray:
        """
        Get map as grayscale image (ROS convention).

        Returns:
            2D uint8 array:
            - 205: Unknown (gray)
            - 254: Free (white)
            - 0: Occupied (black)
        """
        prob = self.get_occupancy_grid()

        # Initialize as unknown
        img = np.full((self.height, self.width), 205, dtype=np.uint8)

        # Mark free cells
        img[prob < FREE_THRESHOLD] = 254

        # Mark occupied cells
        img[prob > OCCUPIED_THRESHOLD] = 0

        # Flip Y axis for image coordinates
        img = np.flipud(img)

        return img

    def save_map(self, output_dir: str, name: str = "map"):
        """
        Save map in ROS format (PGM + YAML) and PNG.

        Args:
            output_dir: Output directory
            name: Map name (without extension)
        """
        os.makedirs(output_dir, exist_ok=True)

        img = self.get_map_image()

        # Save PGM (ROS format)
        pgm_path = os.path.join(output_dir, f"{name}.pgm")
        self._save_pgm(pgm_path, img)

        # Save YAML (ROS map metadata)
        yaml_path = os.path.join(output_dir, f"{name}.yaml")
        self._save_yaml(yaml_path, name)

        # Save PNG for visualization
        png_path = os.path.join(output_dir, f"{name}.png")
        if HAS_PIL:
            Image.fromarray(img).save(png_path)
        elif HAS_OPENCV:
            cv2.imwrite(png_path, img)

        # Save metadata JSON
        meta_path = os.path.join(output_dir, f"{name}_metadata.json")
        self._save_metadata(meta_path)

        print(f"[MAP] Saved to {output_dir}/")
        print(f"  - {name}.pgm (ROS format)")
        print(f"  - {name}.yaml (ROS metadata)")
        print(f"  - {name}.png (visualization)")

    def _save_pgm(self, path: str, img: np.ndarray):
        """Save as PGM file"""
        with open(path, 'wb') as f:
            f.write(f"P5\n{self.width} {self.height}\n255\n".encode())
            f.write(img.tobytes())

    def _save_yaml(self, path: str, name: str):
        """Save ROS map YAML metadata"""
        yaml_content = f"""image: {name}.pgm
resolution: {self.resolution}
origin: [{self.origin_x}, {self.origin_y}, 0.0]
negate: 0
occupied_thresh: {OCCUPIED_THRESHOLD}
free_thresh: {FREE_THRESHOLD}
"""
        with open(path, 'w') as f:
            f.write(yaml_content)

    def _save_metadata(self, path: str):
        """Save processing metadata"""
        observed_count = np.sum(self.observed)
        total_cells = self.width * self.height
        coverage = observed_count / total_cells

        prob = self.get_occupancy_grid()
        free_count = np.sum(prob < FREE_THRESHOLD)
        occupied_count = np.sum(prob > OCCUPIED_THRESHOLD)

        metadata = {
            'resolution': self.resolution,
            'width': self.width,
            'height': self.height,
            'origin': [self.origin_x, self.origin_y],
            'coverage': float(coverage),
            'cells': {
                'total': int(total_cells),
                'observed': int(observed_count),
                'free': int(free_count),
                'occupied': int(occupied_count),
                'unknown': int(total_cells - observed_count)
            },
            'created': datetime.now().isoformat()
        }

        with open(path, 'w') as f:
            json.dump(metadata, f, indent=2)


class OfflineMapper:
    """
    Main offline mapping pipeline.
    Loads recorded data, aligns scans, and builds occupancy grid.
    """

    def __init__(self, resolution: float = DEFAULT_RESOLUTION,
                 use_icp: bool = True):
        """
        Initialize offline mapper.

        Args:
            resolution: Map resolution in meters
            use_icp: Whether to use ICP for scan refinement
        """
        self.resolution = resolution
        self.use_icp = use_icp and HAS_OPEN3D

        self.scans: List[ScanData] = []
        self.metadata: Dict[str, Any] = {}

    def load_data(self, data_dir: str) -> bool:
        """
        Load recorded sensor data from directory.

        Args:
            data_dir: Path to exploration_YYYYMMDD_HHMMSS directory

        Returns:
            True if data loaded successfully
        """
        print(f"[LOAD] Loading data from {data_dir}")

        # Load metadata
        meta_path = os.path.join(data_dir, "metadata.json")
        if os.path.exists(meta_path):
            with open(meta_path, 'r') as f:
                self.metadata = json.load(f)
            print(f"[LOAD] Session: {self.metadata.get('session_id', 'unknown')}")

        # Load scans
        scans_path = self._find_file(data_dir, "scans")
        if not scans_path:
            print("[ERROR] No scans file found")
            return False

        # Load odometry
        odom_path = self._find_file(data_dir, "odom")
        if not odom_path:
            print("[ERROR] No odom file found")
            return False

        # Parse files
        scans_raw = self._load_jsonl(scans_path)
        odom_raw = self._load_jsonl(odom_path)

        print(f"[LOAD] Found {len(scans_raw)} scans, {len(odom_raw)} odom readings")

        # Synchronize scans with odometry
        self.scans = self._synchronize_data(scans_raw, odom_raw)
        print(f"[LOAD] Synchronized {len(self.scans)} scan-pose pairs")

        return len(self.scans) > 0

    def _find_file(self, data_dir: str, prefix: str) -> Optional[str]:
        """Find data file (compressed or not)"""
        for ext in [".jsonl.gz", ".jsonl"]:
            path = os.path.join(data_dir, f"{prefix}{ext}")
            if os.path.exists(path):
                return path
        return None

    def _load_jsonl(self, path: str) -> List[Dict]:
        """Load JSON Lines file (compressed or not)"""
        open_func = gzip.open if path.endswith('.gz') else open
        mode = 'rt' if path.endswith('.gz') else 'r'

        data = []
        with open_func(path, mode) as f:
            for line in f:
                if line.strip():
                    data.append(json.loads(line))

        return data

    def _synchronize_data(self, scans: List[Dict], odoms: List[Dict]) -> List[ScanData]:
        """
        Synchronize scan and odometry data by timestamp.
        For each scan, find the closest odometry reading.
        """
        if not odoms:
            print("[WARN] No odometry data, using origin pose for all scans")
            return [
                ScanData(
                    timestamp=s['timestamp'],
                    pose=Pose2D(0, 0, 0),
                    ranges=np.array(s['ranges']),
                    angle_min=s['angle_min'],
                    angle_max=s['angle_max'],
                    angle_increment=s['angle_increment']
                )
                for s in scans
            ]

        # Sort by timestamp
        odoms = sorted(odoms, key=lambda x: x['timestamp'])
        odom_times = np.array([o['timestamp'] for o in odoms])

        synchronized = []
        for scan in scans:
            # Find closest odometry
            idx = np.argmin(np.abs(odom_times - scan['timestamp']))
            odom = odoms[idx]

            # Create pose from odometry
            pose = Pose2D.from_quaternion(
                x=odom['x'],
                y=odom['y'],
                qz=odom['qz'],
                qw=odom['qw']
            )

            synchronized.append(ScanData(
                timestamp=scan['timestamp'],
                pose=pose,
                ranges=np.array(scan['ranges']),
                angle_min=scan['angle_min'],
                angle_max=scan['angle_max'],
                angle_increment=scan['angle_increment']
            ))

        return synchronized

    def refine_poses_icp(self):
        """
        Refine poses using ICP scan matching.
        This improves alignment by matching consecutive scans.
        """
        if not self.use_icp or not HAS_OPEN3D:
            print("[ICP] Skipping ICP refinement (Open3D not available)")
            return

        print("[ICP] Refining poses with ICP...")

        # Convert scans to point clouds
        for i in range(1, len(self.scans)):
            prev_scan = self.scans[i - 1]
            curr_scan = self.scans[i]

            # Get points in local frame
            prev_points = prev_scan.to_points()
            curr_points = curr_scan.to_points()

            if len(prev_points) < 10 or len(curr_points) < 10:
                continue

            # Create Open3D point clouds
            prev_pcd = o3d.geometry.PointCloud()
            prev_pcd.points = o3d.utility.Vector3dVector(
                np.hstack([prev_points, np.zeros((len(prev_points), 1))])
            )

            curr_pcd = o3d.geometry.PointCloud()
            curr_pcd.points = o3d.utility.Vector3dVector(
                np.hstack([curr_points, np.zeros((len(curr_points), 1))])
            )

            # Initial guess from odometry
            delta_x = curr_scan.pose.x - prev_scan.pose.x
            delta_y = curr_scan.pose.y - prev_scan.pose.y
            delta_theta = curr_scan.pose.theta - prev_scan.pose.theta

            init_transform = np.eye(4)
            c, s = np.cos(delta_theta), np.sin(delta_theta)
            init_transform[:2, :2] = [[c, -s], [s, c]]
            init_transform[:2, 3] = [delta_x, delta_y]

            # Run ICP
            result = o3d.pipelines.registration.registration_icp(
                curr_pcd, prev_pcd, 0.5, init_transform,
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )

            # Update pose if ICP converged well
            if result.fitness > 0.5:
                # Extract refined transformation
                T = result.transformation
                refined_delta_x = T[0, 3]
                refined_delta_y = T[1, 3]
                refined_delta_theta = np.arctan2(T[1, 0], T[0, 0])

                # Update current pose
                curr_scan.pose.x = prev_scan.pose.x + refined_delta_x
                curr_scan.pose.y = prev_scan.pose.y + refined_delta_y
                curr_scan.pose.theta = prev_scan.pose.theta + refined_delta_theta

        print(f"[ICP] Refined {len(self.scans)} poses")

    def build_map(self, map_size: float = DEFAULT_MAP_SIZE) -> OccupancyGridBuilder:
        """
        Build occupancy grid from scans.

        Args:
            map_size: Map size in meters

        Returns:
            OccupancyGridBuilder with completed map
        """
        print(f"[MAP] Building {map_size}m x {map_size}m map at {self.resolution}m resolution")

        # Determine map bounds from scan poses
        if self.scans:
            xs = [s.pose.x for s in self.scans]
            ys = [s.pose.y for s in self.scans]
            center_x = (min(xs) + max(xs)) / 2
            center_y = (min(ys) + max(ys)) / 2

            # Adjust map size if needed
            span_x = max(xs) - min(xs) + 10  # Add 10m margin
            span_y = max(ys) - min(ys) + 10
            map_size = max(map_size, span_x, span_y)

            origin = (center_x - map_size / 2, center_y - map_size / 2)
        else:
            origin = None

        grid = OccupancyGridBuilder(
            resolution=self.resolution,
            size=map_size,
            origin=origin
        )

        # Process each scan
        for i, scan in enumerate(self.scans):
            grid.update_from_scan(scan)

            if (i + 1) % 100 == 0:
                print(f"[MAP] Processed {i + 1}/{len(self.scans)} scans")

        print(f"[MAP] Processed all {len(self.scans)} scans")
        return grid

    def process(self, data_dir: str, output_dir: str,
                map_size: float = DEFAULT_MAP_SIZE,
                visualize: bool = False) -> bool:
        """
        Full processing pipeline.

        Args:
            data_dir: Input data directory
            output_dir: Output directory for map
            map_size: Map size in meters
            visualize: Whether to show visualization

        Returns:
            True if successful
        """
        # Load data
        if not self.load_data(data_dir):
            return False

        # Refine poses with ICP
        if self.use_icp:
            self.refine_poses_icp()

        # Build map
        grid = self.build_map(map_size)

        # Save map
        grid.save_map(output_dir)

        # Visualize if requested
        if visualize and HAS_MATPLOTLIB:
            self._visualize(grid)

        return True

    def _visualize(self, grid: OccupancyGridBuilder):
        """Show map visualization"""
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))

        # Map
        img = grid.get_map_image()
        axes[0].imshow(img, cmap='gray', vmin=0, vmax=255)
        axes[0].set_title(f"Occupancy Grid ({grid.width}x{grid.height})")
        axes[0].set_xlabel("X (cells)")
        axes[0].set_ylabel("Y (cells)")

        # Trajectory
        if self.scans:
            xs = [s.pose.x for s in self.scans]
            ys = [s.pose.y for s in self.scans]
            axes[1].plot(xs, ys, 'b-', linewidth=0.5, alpha=0.7)
            axes[1].plot(xs[0], ys[0], 'go', markersize=10, label='Start')
            axes[1].plot(xs[-1], ys[-1], 'ro', markersize=10, label='End')
            axes[1].set_title("Robot Trajectory")
            axes[1].set_xlabel("X (m)")
            axes[1].set_ylabel("Y (m)")
            axes[1].legend()
            axes[1].axis('equal')
            axes[1].grid(True)

        plt.tight_layout()
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Build occupancy grid map from recorded sensor data',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Basic processing
    python offline_mapper.py ./exploration_20241215_143022 --output ./maps/room1

    # With visualization
    python offline_mapper.py ./data --output ./maps --visualize

    # Higher resolution
    python offline_mapper.py ./data --output ./maps --resolution 0.02

    # Larger map area
    python offline_mapper.py ./data --output ./maps --size 50
"""
    )

    parser.add_argument('data_dir', type=str,
                        help='Input data directory (exploration_YYYYMMDD_HHMMSS)')
    parser.add_argument('--output', '-o', type=str, default='./map_output',
                        help='Output directory (default: ./map_output)')
    parser.add_argument('--resolution', '-r', type=float, default=DEFAULT_RESOLUTION,
                        help=f'Map resolution in meters (default: {DEFAULT_RESOLUTION})')
    parser.add_argument('--size', '-s', type=float, default=DEFAULT_MAP_SIZE,
                        help=f'Map size in meters (default: {DEFAULT_MAP_SIZE})')
    parser.add_argument('--no-icp', action='store_true',
                        help='Disable ICP pose refinement')
    parser.add_argument('--visualize', '-v', action='store_true',
                        help='Show visualization after processing')

    args = parser.parse_args()

    if not os.path.exists(args.data_dir):
        print(f"Error: Data directory not found: {args.data_dir}")
        sys.exit(1)

    mapper = OfflineMapper(
        resolution=args.resolution,
        use_icp=not args.no_icp
    )

    success = mapper.process(
        data_dir=args.data_dir,
        output_dir=args.output,
        map_size=args.size,
        visualize=args.visualize
    )

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
