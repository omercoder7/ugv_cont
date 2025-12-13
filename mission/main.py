"""
Main entry point and launcher functions for autonomous scanning.
"""

import os
import sys
import time
import signal
import argparse
import subprocess

from .constants import CONTAINER_NAME


def ensure_slam_running():
    """Check if SLAM is running and publishing map, restart if needed"""
    script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ensure_slam_script = os.path.join(script_dir, 'ensure_slam.sh')

    if os.path.exists(ensure_slam_script):
        print("Checking SLAM status...")
        result = subprocess.run(
            [ensure_slam_script, '--check'],
            capture_output=True, text=True,
            timeout=30
        )
        print(result.stdout.strip())
        return result.returncode == 0
    else:
        try:
            result = subprocess.run(
                ['docker', 'exec', CONTAINER_NAME, 'pgrep', '-f', 'slam_toolbox'],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode != 0:
                print("SLAM not running, starting...")
                subprocess.Popen(
                    ['docker', 'exec', '-d', CONTAINER_NAME, 'bash', '-c',
                     'source /opt/ros/humble/setup.bash && '
                     'source /root/ugv_ws/install/setup.bash && '
                     'ros2 launch slam_toolbox online_async_launch.py > /tmp/slam.log 2>&1'],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                time.sleep(5)
            return True
        except Exception as e:
            print(f"SLAM check failed: {e}")
            return False


def check_prerequisites():
    """Ensure container and bringup are running using helper script"""
    script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ensure_script = os.path.join(script_dir, 'ensure_bringup.sh')

    if not os.path.exists(ensure_script):
        print(f"Warning: {ensure_script} not found, falling back to manual check")
        result = subprocess.run(
            ['docker', 'ps', '--filter', f'name={CONTAINER_NAME}', '--format', '{{.Names}}'],
            capture_output=True, text=True
        )
        if CONTAINER_NAME not in result.stdout:
            print(f"Error: Container '{CONTAINER_NAME}' is not running.")
            print("Start it with: ./start_ugv_service.sh")
            return False
        return True

    print("Checking robot bringup...")
    result = subprocess.run(
        [ensure_script, '--wait'],
        capture_output=True, text=True,
        timeout=30
    )

    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr)
        return False

    return True


def start_driver():
    """Start ugv_driver for motor control if not already running"""
    result = subprocess.run(
        ['docker', 'exec', CONTAINER_NAME, 'pgrep', '-f', 'ugv_driver'],
        capture_output=True, text=True
    )
    if result.returncode == 0:
        print("ugv_driver already running")
        return True

    print("Starting ugv_driver for motor control...")
    subprocess.Popen(
        ['docker', 'exec', '-d', CONTAINER_NAME, 'bash', '-c',
         'source /opt/ros/humble/setup.bash && '
         'source /root/ugv_ws/install/setup.bash && '
         'ros2 run ugv_bringup ugv_driver'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(2)

    result = subprocess.run(
        ['docker', 'exec', CONTAINER_NAME, 'pgrep', '-f', 'ugv_driver'],
        capture_output=True, text=True
    )
    if result.returncode == 0:
        print("ugv_driver started successfully")
        return True
    else:
        print("Warning: ugv_driver may not have started properly")
        return False


def check_rf2o_health():
    """Check if RF2O is running and publishing. Restart if dead."""
    result = subprocess.run(
        ['docker', 'exec', CONTAINER_NAME, 'pgrep', '-f', 'rf2o_laser_odometry_node'],
        capture_output=True, text=True
    )

    if result.returncode != 0:
        print("\n[RF2O] Process dead! Restarting...")
        subprocess.Popen(
            ['docker', 'exec', '-d', CONTAINER_NAME, 'bash', '-c',
             'source /opt/ros/humble/setup.bash && '
             'source /root/ugv_ws/install/setup.bash && '
             'ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py > /tmp/rf2o.log 2>&1'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        time.sleep(3)
        result = subprocess.run(
            ['docker', 'exec', CONTAINER_NAME, 'pgrep', '-f', 'rf2o_laser_odometry_node'],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            print("[RF2O] Restarted successfully")
            return True
        else:
            print("[RF2O] FAILED TO RESTART - EKF will be slow!")
            return False
    return True


def main():
    """Main entry point for autonomous scanning."""
    # Import here to avoid circular imports
    from .avoider import SectorObstacleAvoider

    parser = argparse.ArgumentParser(
        description='Autonomous scanning with sector-based obstacle avoidance',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ./auto_scan.py                      # Default: 60s at 0.06 m/s
  ./auto_scan.py --duration 120       # 2 minute scan
  ./auto_scan.py --duration 0         # Unlimited
  ./auto_scan.py --speed 0.08         # Slower scanning
  ./auto_scan.py --min-dist 0.7       # More cautious

Algorithm:
  Divides 360° LiDAR scan into 12 sectors (30° each).
  Finds clearest path and navigates with minimal rotation.
  Based on: github.com/Rad-hi/Obstacle-Avoidance-ROS
        """
    )
    parser.add_argument('--speed', '-s', type=float, default=0.06,
                        help='Linear speed m/s (default: 0.06, max: 0.08)')
    parser.add_argument('--min-dist', '-m', type=float, default=0.35,
                        help='Min obstacle distance m (default: 0.35)')
    parser.add_argument('--duration', '-d', type=float, default=60,
                        help='Scan duration seconds, 0=unlimited (default: 60)')
    parser.add_argument('--clear-visited', '-c', action='store_true',
                        help='Clear visited locations history before starting')
    parser.add_argument('--test-front', action='store_true',
                        help='Test front sector calibration (place obstacle in front)')
    parser.add_argument('--vfh', action='store_true',
                        help='Use VFH algorithm with state machine (smoother navigation)')
    parser.add_argument('--log-sensors', '-l', action='store_true',
                        help='Enable sensor logging for offline mapping')
    parser.add_argument('--log-dir', type=str, default='/tmp/exploration_data',
                        help='Output directory for sensor logs (default: /tmp/exploration_data)')
    args = parser.parse_args()

    if not check_prerequisites():
        sys.exit(1)

    start_driver()
    ensure_slam_running()

    avoider = SectorObstacleAvoider(
        linear_speed=args.speed,
        min_distance=args.min_dist,
        duration=args.duration,
        clear_visited=args.clear_visited,
        use_vfh=args.vfh,
        log_sensors=args.log_sensors,
        log_output_dir=args.log_dir
    )

    if args.test_front:
        avoider.test_front_sector()
        sys.exit(0)

    def signal_handler(sig, frame):
        avoider.running = False
        avoider.emergency_stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    avoider.run()


if __name__ == '__main__':
    main()
