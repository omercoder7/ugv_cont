#!/usr/bin/env python3
"""
Sensor Logger for UGV Robot - Phase 1 of Two-Phase Mapping

Lightweight sensor recording during exploration.
Records LiDAR scans, odometry, and IMU data to JSON files for offline processing.

Usage:
    # Standalone recording
    ./sensor_logger.py --duration 300 --output /tmp/exploration_data

    # Or use as a module from auto_scan.py
    from mapping.robot.sensor_logger import SensorLogger
    logger = SensorLogger(output_dir="/tmp/exploration_data")
    logger.start()
    # ... exploration ...
    logger.stop()

Output format:
    exploration_YYYYMMDD_HHMMSS/
    ├── metadata.json       # Session info, sensor config
    ├── scans.jsonl         # LiDAR scans (JSON Lines for streaming)
    ├── odom.jsonl          # Odometry data
    └── imu.jsonl           # IMU data
"""

import os
import sys
import json
import time
import gzip
import threading
import subprocess
import signal
from datetime import datetime
from typing import Optional, Dict, List, Any
from dataclasses import dataclass, asdict
import argparse

# Container name for ROS2 access
CONTAINER_NAME = "ugv_rpi_ros_humble"

# Recording settings
DEFAULT_SCAN_RATE = 5.0   # Hz - subsample from ~10Hz LiDAR
DEFAULT_ODOM_RATE = 10.0  # Hz
DEFAULT_IMU_RATE = 10.0   # Hz

# LiDAR rotation offset (from constants.py)
# With 60 sectors (6° each): 15 sectors = 90° rotation
LIDAR_ROTATION_SECTORS = 15


@dataclass
class ScanFrame:
    """Single LiDAR scan frame"""
    timestamp: float
    ranges: List[float]
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float


@dataclass
class OdomFrame:
    """Single odometry frame"""
    timestamp: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    linear_x: float
    linear_y: float
    angular_z: float


@dataclass
class ImuFrame:
    """Single IMU frame"""
    timestamp: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float
    angular_velocity_x: float
    angular_velocity_y: float
    angular_velocity_z: float
    linear_acceleration_x: float
    linear_acceleration_y: float
    linear_acceleration_z: float


class SensorLogger:
    """
    Lightweight sensor logger for UGV robot.

    Records sensor data to JSON Lines files for efficient streaming writes.
    Designed to have minimal impact on robot navigation performance.
    """

    def __init__(self, output_dir: str = "/tmp/exploration_data",
                 scan_rate: float = DEFAULT_SCAN_RATE,
                 odom_rate: float = DEFAULT_ODOM_RATE,
                 imu_rate: float = DEFAULT_IMU_RATE,
                 compress: bool = True):
        """
        Initialize sensor logger.

        Args:
            output_dir: Base directory for recordings
            scan_rate: LiDAR recording rate (Hz)
            odom_rate: Odometry recording rate (Hz)
            imu_rate: IMU recording rate (Hz)
            compress: Whether to gzip output files
        """
        self.output_dir = output_dir
        self.scan_rate = scan_rate
        self.odom_rate = odom_rate
        self.imu_rate = imu_rate
        self.compress = compress

        self.session_dir: Optional[str] = None
        self.running = False
        self.threads: List[threading.Thread] = []

        # File handles
        self.scan_file = None
        self.odom_file = None
        self.imu_file = None

        # Statistics
        self.stats = {
            'scan_count': 0,
            'odom_count': 0,
            'imu_count': 0,
            'start_time': 0,
            'end_time': 0,
            'errors': []
        }

        # Rate limiting
        self.last_scan_time = 0
        self.last_odom_time = 0
        self.last_imu_time = 0

    def start(self) -> str:
        """
        Start recording session.

        Returns:
            Path to session directory
        """
        if self.running:
            raise RuntimeError("Logger already running")

        # Create session directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(self.output_dir, f"exploration_{timestamp}")
        os.makedirs(self.session_dir, exist_ok=True)

        # Open output files
        ext = ".jsonl.gz" if self.compress else ".jsonl"
        open_func = gzip.open if self.compress else open
        mode = "wt" if self.compress else "w"

        self.scan_file = open_func(os.path.join(self.session_dir, f"scans{ext}"), mode)
        self.odom_file = open_func(os.path.join(self.session_dir, f"odom{ext}"), mode)
        self.imu_file = open_func(os.path.join(self.session_dir, f"imu{ext}"), mode)

        # Write metadata
        self._write_metadata()

        # Start recording threads
        self.running = True
        self.stats['start_time'] = time.time()

        # Each sensor gets its own thread for parallel recording
        self.threads = [
            threading.Thread(target=self._record_scans, daemon=True),
            threading.Thread(target=self._record_odom, daemon=True),
            threading.Thread(target=self._record_imu, daemon=True),
        ]

        for t in self.threads:
            t.start()

        print(f"[LOGGER] Started recording to {self.session_dir}")
        return self.session_dir

    def stop(self) -> Dict[str, Any]:
        """
        Stop recording session.

        Returns:
            Recording statistics
        """
        if not self.running:
            return self.stats

        self.running = False
        self.stats['end_time'] = time.time()

        # Wait for threads to finish
        for t in self.threads:
            t.join(timeout=2.0)

        # Close files
        if self.scan_file:
            self.scan_file.close()
        if self.odom_file:
            self.odom_file.close()
        if self.imu_file:
            self.imu_file.close()

        # Update metadata with final stats
        self._write_metadata()

        duration = self.stats['end_time'] - self.stats['start_time']
        print(f"[LOGGER] Stopped recording. Duration: {duration:.1f}s")
        print(f"[LOGGER] Recorded: {self.stats['scan_count']} scans, "
              f"{self.stats['odom_count']} odom, {self.stats['imu_count']} imu")

        return self.stats

    def _write_metadata(self):
        """Write session metadata file"""
        metadata = {
            'session_id': os.path.basename(self.session_dir),
            'created': datetime.now().isoformat(),
            'config': {
                'scan_rate': self.scan_rate,
                'odom_rate': self.odom_rate,
                'imu_rate': self.imu_rate,
                'compressed': self.compress,
                'lidar_rotation_offset': LIDAR_ROTATION_SECTORS,
            },
            'stats': self.stats,
            'ros_topics': {
                'scan': '/scan',
                'odom': '/odom',
                'imu': '/imu/data',
            }
        }

        metadata_path = os.path.join(self.session_dir, "metadata.json")
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)

    def _record_scans(self):
        """Record LiDAR scans at specified rate"""
        interval = 1.0 / self.scan_rate

        while self.running:
            try:
                now = time.time()
                if now - self.last_scan_time < interval:
                    time.sleep(0.01)
                    continue

                scan = self._get_scan()
                if scan:
                    self.scan_file.write(json.dumps(asdict(scan)) + "\n")
                    self.scan_file.flush()
                    self.stats['scan_count'] += 1
                    self.last_scan_time = now

            except Exception as e:
                self.stats['errors'].append(f"scan: {str(e)}")
                time.sleep(0.1)

    def _record_odom(self):
        """Record odometry at specified rate"""
        interval = 1.0 / self.odom_rate

        while self.running:
            try:
                now = time.time()
                if now - self.last_odom_time < interval:
                    time.sleep(0.01)
                    continue

                odom = self._get_odom()
                if odom:
                    self.odom_file.write(json.dumps(asdict(odom)) + "\n")
                    self.odom_file.flush()
                    self.stats['odom_count'] += 1
                    self.last_odom_time = now

            except Exception as e:
                self.stats['errors'].append(f"odom: {str(e)}")
                time.sleep(0.1)

    def _record_imu(self):
        """Record IMU at specified rate"""
        interval = 1.0 / self.imu_rate

        while self.running:
            try:
                now = time.time()
                if now - self.last_imu_time < interval:
                    time.sleep(0.01)
                    continue

                imu = self._get_imu()
                if imu:
                    self.imu_file.write(json.dumps(asdict(imu)) + "\n")
                    self.imu_file.flush()
                    self.stats['imu_count'] += 1
                    self.last_imu_time = now

            except Exception as e:
                self.stats['errors'].append(f"imu: {str(e)}")
                time.sleep(0.1)

    def _get_scan(self) -> Optional[ScanFrame]:
        """Get single LiDAR scan from ROS"""
        try:
            result = subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
                 '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from sensor_msgs.msg import LaserScan
import json

rclpy.init()
node = rclpy.create_node('scan_reader')
msg = None
def cb(m): global msg; msg = m
sub = node.create_subscription(LaserScan, '/scan', cb, 1)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.05)
    if msg: break
if msg:
    data = {
        'ranges': [float(r) if r == r else 0.0 for r in msg.ranges],
        'angle_min': msg.angle_min,
        'angle_max': msg.angle_max,
        'angle_increment': msg.angle_increment,
        'range_min': msg.range_min,
        'range_max': msg.range_max
    }
    print(json.dumps(data))
node.destroy_node()
rclpy.shutdown()
"'''],
                capture_output=True, text=True, timeout=3
            )

            if result.stdout.strip():
                data = json.loads(result.stdout.strip())
                return ScanFrame(
                    timestamp=time.time(),
                    ranges=data['ranges'],
                    angle_min=data['angle_min'],
                    angle_max=data['angle_max'],
                    angle_increment=data['angle_increment'],
                    range_min=data['range_min'],
                    range_max=data['range_max']
                )
            return None

        except Exception:
            return None

    def _get_odom(self) -> Optional[OdomFrame]:
        """Get single odometry reading from ROS"""
        try:
            result = subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
                 '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from nav_msgs.msg import Odometry
import json

rclpy.init()
node = rclpy.create_node('odom_reader')
msg = None
def cb(m): global msg; msg = m
sub = node.create_subscription(Odometry, '/odom', cb, 1)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.05)
    if msg: break
if msg:
    data = {
        'x': msg.pose.pose.position.x,
        'y': msg.pose.pose.position.y,
        'z': msg.pose.pose.position.z,
        'qx': msg.pose.pose.orientation.x,
        'qy': msg.pose.pose.orientation.y,
        'qz': msg.pose.pose.orientation.z,
        'qw': msg.pose.pose.orientation.w,
        'linear_x': msg.twist.twist.linear.x,
        'linear_y': msg.twist.twist.linear.y,
        'angular_z': msg.twist.twist.angular.z
    }
    print(json.dumps(data))
node.destroy_node()
rclpy.shutdown()
"'''],
                capture_output=True, text=True, timeout=3
            )

            if result.stdout.strip():
                data = json.loads(result.stdout.strip())
                return OdomFrame(
                    timestamp=time.time(),
                    x=data['x'],
                    y=data['y'],
                    z=data['z'],
                    qx=data['qx'],
                    qy=data['qy'],
                    qz=data['qz'],
                    qw=data['qw'],
                    linear_x=data['linear_x'],
                    linear_y=data['linear_y'],
                    angular_z=data['angular_z']
                )
            return None

        except Exception:
            return None

    def _get_imu(self) -> Optional[ImuFrame]:
        """Get single IMU reading from ROS"""
        try:
            result = subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'bash', '-c',
                 '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from sensor_msgs.msg import Imu
import json

rclpy.init()
node = rclpy.create_node('imu_reader')
msg = None
def cb(m): global msg; msg = m
sub = node.create_subscription(Imu, '/imu/data', cb, 1)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.05)
    if msg: break
if msg:
    data = {
        'ox': msg.orientation.x,
        'oy': msg.orientation.y,
        'oz': msg.orientation.z,
        'ow': msg.orientation.w,
        'avx': msg.angular_velocity.x,
        'avy': msg.angular_velocity.y,
        'avz': msg.angular_velocity.z,
        'lax': msg.linear_acceleration.x,
        'lay': msg.linear_acceleration.y,
        'laz': msg.linear_acceleration.z
    }
    print(json.dumps(data))
node.destroy_node()
rclpy.shutdown()
"'''],
                capture_output=True, text=True, timeout=3
            )

            if result.stdout.strip():
                data = json.loads(result.stdout.strip())
                return ImuFrame(
                    timestamp=time.time(),
                    orientation_x=data['ox'],
                    orientation_y=data['oy'],
                    orientation_z=data['oz'],
                    orientation_w=data['ow'],
                    angular_velocity_x=data['avx'],
                    angular_velocity_y=data['avy'],
                    angular_velocity_z=data['avz'],
                    linear_acceleration_x=data['lax'],
                    linear_acceleration_y=data['lay'],
                    linear_acceleration_z=data['laz']
                )
            return None

        except Exception:
            return None

    # ----- Methods for integration with auto_scan.py -----

    def log_scan_sync(self, ranges: List[float], angle_min: float = -3.14159,
                      angle_max: float = 3.14159, angle_increment: float = 0.00436):
        """
        Synchronously log a scan (for use from auto_scan.py main loop).
        This avoids extra ROS calls by reusing data already fetched.
        """
        if not self.running or not self.scan_file:
            return

        now = time.time()
        if now - self.last_scan_time < (1.0 / self.scan_rate):
            return

        frame = ScanFrame(
            timestamp=now,
            ranges=ranges,
            angle_min=angle_min,
            angle_max=angle_max,
            angle_increment=angle_increment,
            range_min=0.0,
            range_max=12.0
        )

        try:
            self.scan_file.write(json.dumps(asdict(frame)) + "\n")
            self.stats['scan_count'] += 1
            self.last_scan_time = now
        except Exception:
            pass

    def log_odom_sync(self, x: float, y: float, theta: float = 0.0):
        """
        Synchronously log odometry (for use from auto_scan.py main loop).
        Simplified version that takes x, y, theta.
        """
        if not self.running or not self.odom_file:
            return

        now = time.time()
        if now - self.last_odom_time < (1.0 / self.odom_rate):
            return

        import math
        frame = OdomFrame(
            timestamp=now,
            x=x,
            y=y,
            z=0.0,
            qx=0.0,
            qy=0.0,
            qz=math.sin(theta / 2),
            qw=math.cos(theta / 2),
            linear_x=0.0,
            linear_y=0.0,
            angular_z=0.0
        )

        try:
            self.odom_file.write(json.dumps(asdict(frame)) + "\n")
            self.stats['odom_count'] += 1
            self.last_odom_time = now
        except Exception:
            pass


def main():
    """Standalone recording mode"""
    parser = argparse.ArgumentParser(
        description='Record sensor data for offline mapping',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Record for 5 minutes
    ./sensor_logger.py --duration 300

    # Record with custom rates
    ./sensor_logger.py --scan-rate 10 --odom-rate 20 --duration 120

    # Record to specific directory
    ./sensor_logger.py --output ~/my_recordings --duration 60
"""
    )

    parser.add_argument('--duration', '-d', type=float, default=60,
                        help='Recording duration in seconds (default: 60, 0=unlimited)')
    parser.add_argument('--output', '-o', type=str, default='/tmp/exploration_data',
                        help='Output directory (default: /tmp/exploration_data)')
    parser.add_argument('--scan-rate', type=float, default=DEFAULT_SCAN_RATE,
                        help=f'LiDAR scan rate Hz (default: {DEFAULT_SCAN_RATE})')
    parser.add_argument('--odom-rate', type=float, default=DEFAULT_ODOM_RATE,
                        help=f'Odometry rate Hz (default: {DEFAULT_ODOM_RATE})')
    parser.add_argument('--imu-rate', type=float, default=DEFAULT_IMU_RATE,
                        help=f'IMU rate Hz (default: {DEFAULT_IMU_RATE})')
    parser.add_argument('--no-compress', action='store_true',
                        help='Disable gzip compression')

    args = parser.parse_args()

    logger = SensorLogger(
        output_dir=args.output,
        scan_rate=args.scan_rate,
        odom_rate=args.odom_rate,
        imu_rate=args.imu_rate,
        compress=not args.no_compress
    )

    def signal_handler(sig, frame):
        print("\n[LOGGER] Stopping...")
        logger.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print(f"[LOGGER] Starting recording for {args.duration if args.duration > 0 else 'unlimited'}s")
    print(f"[LOGGER] Rates: scan={args.scan_rate}Hz, odom={args.odom_rate}Hz, imu={args.imu_rate}Hz")
    print("[LOGGER] Press Ctrl+C to stop")

    session_dir = logger.start()

    if args.duration > 0:
        time.sleep(args.duration)
        logger.stop()
    else:
        # Run until interrupted
        while logger.running:
            time.sleep(1)
            elapsed = time.time() - logger.stats['start_time']
            print(f"\r[LOGGER] Recording... {elapsed:.0f}s - "
                  f"scans:{logger.stats['scan_count']} odom:{logger.stats['odom_count']} "
                  f"imu:{logger.stats['imu_count']}", end="", flush=True)


if __name__ == '__main__':
    main()
