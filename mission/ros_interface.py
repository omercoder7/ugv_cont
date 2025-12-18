"""
ROS2 interface functions for communication with the robot.

All communication happens via docker exec to the ROS container.
These functions are standalone and can be called from any module.

Includes PersistentROSBridge for efficient repeated queries using a single node.
"""

import subprocess
import threading
import time
import math
from typing import Optional, Tuple, List

from .constants import CONTAINER_NAME, LINEAR_VEL_RATIO


class PersistentROSBridge:
    """
    A persistent ROS2 bridge that maintains a single node for all readings.
    Avoids creating/destroying nodes on each call, reducing overhead and
    preventing node accumulation issues.
    """
    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self._initialized = True
        self._proc = None
        self._proc_lock = threading.Lock()
        self._last_scan = None
        self._last_odom = None
        self._scan_time = 0
        self._odom_time = 0
        self._cache_timeout = 0.15  # Cache readings for 150ms

    def _ensure_bridge(self):
        """Ensure the bridge process is running."""
        with self._proc_lock:
            if self._proc is not None and self._proc.poll() is None:
                return True  # Already running

            # Start the persistent bridge process
            bridge_script = '''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import sys
import select
import math
import json

class BridgeNode(Node):
    def __init__(self):
        super().__init__('persistent_ros_bridge')
        self.scan_data = None
        self.odom_data = None
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 1)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 1)

    def scan_cb(self, msg):
        self.scan_data = [round(r, 3) for r in msg.ranges]

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # 180 degree offset correction
        yaw = yaw + math.pi
        if yaw > math.pi: yaw -= 2*math.pi
        self.odom_data = (round(x, 4), round(y, 4), round(yaw, 4))

rclpy.init()
node = BridgeNode()

while rclpy.ok():
    # Process ROS callbacks
    rclpy.spin_once(node, timeout_sec=0.02)

    # Check for commands from stdin (non-blocking)
    if select.select([sys.stdin], [], [], 0.01)[0]:
        try:
            line = sys.stdin.readline().strip()
            if not line:
                break
            if line == 'SCAN':
                if node.scan_data:
                    print('SCAN:' + ','.join(map(str, node.scan_data)), flush=True)
                else:
                    print('SCAN:NONE', flush=True)
            elif line == 'ODOM':
                if node.odom_data:
                    print(f'ODOM:{node.odom_data[0]},{node.odom_data[1]},{node.odom_data[2]}', flush=True)
                else:
                    print('ODOM:NONE', flush=True)
        except:
            pass

node.destroy_node()
rclpy.shutdown()
'''
            try:
                self._proc = subprocess.Popen(
                    ['docker', 'exec', '-i', CONTAINER_NAME, 'bash', '-c',
                     f'source /opt/ros/humble/setup.bash && python3 -u -c "{bridge_script}"'],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    bufsize=1,
                    universal_newlines=True
                )
                time.sleep(0.3)  # Give it time to start
                return True
            except:
                self._proc = None
                return False

    def _query(self, cmd: str, timeout: float = 1.0) -> Optional[str]:
        """Send a command and get response."""
        if not self._ensure_bridge():
            return None

        with self._proc_lock:
            try:
                self._proc.stdin.write(cmd + '\n')
                self._proc.stdin.flush()

                # Read response with timeout
                import select as sel
                ready, _, _ = sel.select([self._proc.stdout], [], [], timeout)
                if ready:
                    response = self._proc.stdout.readline().strip()
                    return response
            except:
                # Bridge died, mark for restart
                self._proc = None
        return None

    def get_scan(self) -> Optional[List[float]]:
        """Get LiDAR scan with caching."""
        now = time.time()
        if self._last_scan and (now - self._scan_time) < self._cache_timeout:
            return self._last_scan

        response = self._query('SCAN')
        if response and response.startswith('SCAN:') and response != 'SCAN:NONE':
            try:
                data = response[5:]  # Remove 'SCAN:' prefix
                self._last_scan = [float(x) for x in data.split(',')]
                self._scan_time = now
                return self._last_scan
            except:
                pass
        return None

    def get_odom(self) -> Optional[Tuple[float, float, float]]:
        """Get odometry with caching."""
        now = time.time()
        if self._last_odom and (now - self._odom_time) < self._cache_timeout:
            return self._last_odom

        response = self._query('ODOM')
        if response and response.startswith('ODOM:') and response != 'ODOM:NONE':
            try:
                data = response[5:]  # Remove 'ODOM:' prefix
                parts = data.split(',')
                self._last_odom = (float(parts[0]), float(parts[1]), float(parts[2]))
                self._odom_time = now
                return self._last_odom
            except:
                pass
        return None

    def shutdown(self):
        """Shutdown the bridge."""
        with self._proc_lock:
            if self._proc:
                try:
                    self._proc.terminate()
                    self._proc.wait(timeout=1)
                except:
                    pass
                self._proc = None


# Global bridge instance (lazy-initialized)
_ros_bridge: Optional[PersistentROSBridge] = None
_use_persistent_bridge = True  # Set to False to use legacy per-call nodes


def _get_bridge() -> Optional[PersistentROSBridge]:
    """Get or create the global bridge instance."""
    global _ros_bridge
    if _ros_bridge is None:
        _ros_bridge = PersistentROSBridge()
    return _ros_bridge


def send_velocity_cmd(linear_x: float, angular_z: float, container: str = CONTAINER_NAME):
    """
    Send velocity command directly to serial port (bypasses ROS2).

    Args:
        linear_x: Linear velocity in m/s (positive = forward)
        angular_z: Angular velocity in rad/s (positive = left/CCW)
        container: Docker container name
    """
    # Apply velocity calibration
    calibrated_linear = linear_x / LINEAR_VEL_RATIO
    x_val = max(-1.0, min(1.0, -calibrated_linear / 0.3))  # INVERTED + Scale
    z_val = max(-1.0, min(1.0, -angular_z / 1.0))  # INVERTED for rotation

    serial_cmd = f'{{"T":"13","X":{x_val:.2f},"Z":{z_val:.2f}}}'
    try:
        subprocess.run(
            ['docker', 'exec', container, 'bash', '-c',
             f"echo '{serial_cmd}' > /dev/ttyAMA0"],
            timeout=1,
            capture_output=True
        )
    except:
        pass


def get_odometry(container: str = CONTAINER_NAME) -> Optional[Tuple[float, float, float]]:
    """
    Get current position and heading from odometry.

    Returns:
        (x, y, yaw) tuple or None if unavailable
    """
    # Try persistent bridge first (avoids node accumulation)
    if _use_persistent_bridge:
        bridge = _get_bridge()
        if bridge:
            result = bridge.get_odom()
            if result:
                return result
            # Bridge failed, fall through to legacy method

    # Legacy per-call method (creates/destroys node each call)
    try:
        result = subprocess.run(
            ['docker', 'exec', container, 'bash', '-c',
             '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
import math
from nav_msgs.msg import Odometry
rclpy.init()
node = rclpy.create_node('odom_reader')
msg = None
def cb(m): global msg; msg = m
sub = node.create_subscription(Odometry, '/odom', cb, 1)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.05)
    if msg: break
if msg:
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    # Correct for 180 degree offset between odometry frame and robot physical front
    yaw = yaw + math.pi
    if yaw > math.pi: yaw -= 2*math.pi
    print(f'{x:.4f},{y:.4f},{yaw:.4f}')
node.destroy_node()
rclpy.shutdown()
"'''],
            capture_output=True, text=True, timeout=3
        )
        if result.stdout.strip():
            parts = result.stdout.strip().split(',')
            if len(parts) >= 3:
                return (float(parts[0]), float(parts[1]), float(parts[2]))
        return None
    except:
        return None


def get_wheel_encoders(container: str = CONTAINER_NAME) -> Optional[Tuple[float, float]]:
    """
    Get raw wheel encoder values from /odom/odom_raw topic.

    Returns:
        (left, right) encoder values or None if unavailable
    """
    try:
        result = subprocess.run(
            ['docker', 'exec', container, 'bash', '-c',
             '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from std_msgs.msg import Float32MultiArray
rclpy.init()
node = rclpy.create_node('encoder_reader')
msg = None
def cb(m): global msg; msg = m
sub = node.create_subscription(Float32MultiArray, '/odom/odom_raw', cb, 1)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.05)
    if msg: break
if msg and len(msg.data) >= 2:
    print(f'{msg.data[0]:.4f},{msg.data[1]:.4f}')
node.destroy_node()
rclpy.shutdown()
"'''],
            capture_output=True, text=True, timeout=3
        )
        if result.stdout.strip():
            parts = result.stdout.strip().split(',')
            return (float(parts[0]), float(parts[1]))
        return None
    except:
        return None


def get_imu_acceleration(container: str = CONTAINER_NAME) -> Optional[float]:
    """
    Get current linear acceleration magnitude from IMU.

    Returns:
        Acceleration magnitude in m/s^2 or None if unavailable
    """
    try:
        result = subprocess.run(
            ['docker', 'exec', container, 'bash', '-c',
             '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from sensor_msgs.msg import Imu
rclpy.init()
node = rclpy.create_node('imu_reader')
msg = None
def cb(m): global msg; msg = m
sub = node.create_subscription(Imu, '/imu/data', cb, 1)
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=0.05)
    if msg: break
if msg:
    ax, ay = msg.linear_acceleration.x, msg.linear_acceleration.y
    import math
    print(f'{math.sqrt(ax*ax + ay*ay):.4f}')
node.destroy_node()
rclpy.shutdown()
"'''],
            capture_output=True, text=True, timeout=2
        )
        if result.stdout.strip():
            return float(result.stdout.strip())
        return None
    except:
        return None


def get_lidar_scan(container: str = CONTAINER_NAME) -> Optional[List[float]]:
    """
    Fetch latest LiDAR scan ranges.

    Returns:
        List of range values or None if unavailable
    """
    # Try persistent bridge first (avoids node accumulation)
    if _use_persistent_bridge:
        bridge = _get_bridge()
        if bridge:
            result = bridge.get_scan()
            if result:
                return result
            # Bridge failed, fall through to legacy method

    # Legacy per-call method (creates/destroys node each call)
    try:
        result = subprocess.run(
            ['docker', 'exec', container, 'bash', '-c',
             '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from sensor_msgs.msg import LaserScan
rclpy.init()
node = rclpy.create_node('scan_reader')
msg = None
def cb(m): global msg; msg = m
sub = node.create_subscription(LaserScan, '/scan', cb, 1)
for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.1)
    if msg: break
if msg:
    print(','.join([f'{r:.3f}' for r in msg.ranges]))
node.destroy_node()
rclpy.shutdown()
"'''],
            capture_output=True, text=True, timeout=5
        )
        if result.stdout.strip():
            return [float(x) for x in result.stdout.strip().split(',')]
        return None
    except:
        return None


def publish_virtual_obstacle(x: float, y: float, container: str = CONTAINER_NAME):
    """
    Publish a virtual obstacle marker at the given position.
    Visible in RViz and helps mark invisible obstacles.

    Args:
        x, y: World coordinates of the obstacle
        container: Docker container name
    """
    try:
        subprocess.run(
            ['docker', 'exec', container, 'bash', '-c',
             f'''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from visualization_msgs.msg import Marker

rclpy.init()
node = rclpy.create_node('virtual_obstacle_pub')
marker_pub = node.create_publisher(Marker, '/virtual_obstacles', 10)

marker = Marker()
marker.header.frame_id = 'odom'
marker.header.stamp = node.get_clock().now().to_msg()
marker.ns = 'stuck_obstacles'
marker.id = int({x}*1000 + {y}*100) % 10000
marker.type = Marker.CYLINDER
marker.action = Marker.ADD
marker.pose.position.x = {x}
marker.pose.position.y = {y}
marker.pose.position.z = 0.1
marker.pose.orientation.w = 1.0
marker.scale.x = 0.3
marker.scale.y = 0.3
marker.scale.z = 0.2
marker.color.r = 1.0
marker.color.g = 0.0
marker.color.b = 0.0
marker.color.a = 0.8
marker.lifetime.sec = 300

for _ in range(3):
    marker_pub.publish(marker)
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
"'''],
            capture_output=True, text=True, timeout=3
        )
    except:
        pass


def publish_goal_marker(x: float, y: float, container: str = CONTAINER_NAME):
    """
    Publish a goal point marker (non-blocking).
    Uses a Marker on /nav_goal topic for reliable visualization in RViz.

    Args:
        x, y: World coordinates of the goal point
        container: Docker container name
    """
    # Run in background to avoid blocking the main loop
    # Use Python script for reliable immediate publishing
    cmd = f'''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from visualization_msgs.msg import Marker
rclpy.init()
node = rclpy.create_node('goal_marker')
pub = node.create_publisher(Marker, '/nav_goal', 10)
m = Marker()
m.header.frame_id = 'odom'
m.header.stamp = node.get_clock().now().to_msg()
m.ns = 'goal'
m.id = 0
m.type = Marker.SPHERE
m.action = Marker.ADD
m.pose.position.x = {x}
m.pose.position.y = {y}
m.pose.position.z = 0.1
m.pose.orientation.w = 1.0
m.scale.x = 0.2
m.scale.y = 0.2
m.scale.z = 0.2
m.color.r = 0.0
m.color.g = 1.0
m.color.b = 0.0
m.color.a = 1.0
m.lifetime.sec = 60
for _ in range(3):
    pub.publish(m)
    rclpy.spin_once(node, timeout_sec=0.02)
node.destroy_node()
rclpy.shutdown()
"'''
    subprocess.Popen(
        ['docker', 'exec', container, 'bash', '-c', cmd],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )


def get_map_data(container: str = CONTAINER_NAME) -> Optional[dict]:
    """
    Get occupancy grid map data from SLAM.

    Returns:
        Dict with 'data', 'width', 'height', 'resolution', 'origin_x', 'origin_y'
        or None if unavailable
    """
    try:
        result = subprocess.run(
            ['docker', 'exec', container, 'bash', '-c',
             '''source /opt/ros/humble/setup.bash && python3 -c "
import rclpy
from nav_msgs.msg import OccupancyGrid
import json
rclpy.init()
node = rclpy.create_node('map_reader')
msg = None
def cb(m): global msg; msg = m
sub = node.create_subscription(OccupancyGrid, '/map', cb, 1)
for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.1)
    if msg: break
if msg:
    data = {
        'width': msg.info.width,
        'height': msg.info.height,
        'resolution': msg.info.resolution,
        'origin_x': msg.info.origin.position.x,
        'origin_y': msg.info.origin.position.y,
        'data': list(msg.data)
    }
    print(json.dumps(data))
node.destroy_node()
rclpy.shutdown()
"'''],
            capture_output=True, text=True, timeout=5
        )
        if result.stdout.strip():
            import json
            return json.loads(result.stdout.strip())
        return None
    except:
        return None


def check_rf2o_health(container: str = CONTAINER_NAME) -> bool:
    """Check if RF2O is running and publishing. Restart if dead."""
    import time

    result = subprocess.run(
        ['docker', 'exec', container, 'pgrep', '-f', 'rf2o_laser_odometry_node'],
        capture_output=True, text=True
    )

    if result.returncode != 0:
        print("\n[RF2O] Process dead! Restarting...")
        subprocess.Popen(
            ['docker', 'exec', '-d', container, 'bash', '-c',
             'source /opt/ros/humble/setup.bash && '
             'source /root/ugv_ws/install/setup.bash && '
             'ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py > /tmp/rf2o.log 2>&1'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        time.sleep(3)
        result = subprocess.run(
            ['docker', 'exec', container, 'pgrep', '-f', 'rf2o_laser_odometry_node'],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            print("[RF2O] Restarted successfully")
            return True
        else:
            print("[RF2O] FAILED TO RESTART - EKF will be slow!")
            return False
    return True


def ensure_slam_running(container: str = CONTAINER_NAME) -> bool:
    """Check if SLAM is running and publishing map, restart if needed."""
    import os
    import time

    script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ensure_slam_script = os.path.join(script_dir, 'ensure_slam.sh')

    if os.path.exists(ensure_slam_script):
        result = subprocess.run(
            [ensure_slam_script, '--check'],
            capture_output=True, text=True,
            timeout=30
        )
        return result.returncode == 0
    else:
        try:
            result = subprocess.run(
                ['docker', 'exec', container, 'pgrep', '-f', 'slam_toolbox'],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode != 0:
                print("SLAM not running, starting...")
                subprocess.Popen(
                    ['docker', 'exec', '-d', container, 'bash', '-c',
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


def shutdown_ros_bridge():
    """Shutdown the persistent ROS bridge if running."""
    global _ros_bridge
    if _ros_bridge is not None:
        _ros_bridge.shutdown()
        _ros_bridge = None


# Register cleanup on exit
import atexit
atexit.register(shutdown_ros_bridge)
