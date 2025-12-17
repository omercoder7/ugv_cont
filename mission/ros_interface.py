"""
ROS2 interface functions for communication with the robot.

All communication happens via docker exec to the ROS container.
These functions are standalone and can be called from any module.
"""

import subprocess
from typing import Optional, Tuple, List

from .constants import CONTAINER_NAME, LINEAR_VEL_RATIO


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
    x_val = max(-1.0, min(1.0, calibrated_linear / 0.3))  # Scale to [-1, 1]
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
