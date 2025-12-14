"""
Constants for autonomous scanning and navigation.
"""

# Docker container name for ROS2 commands
CONTAINER_NAME = "ugv_rpi_ros_humble"

# LiDAR configuration
NUM_SECTORS = 12  # 360 / 12 = 30 per sector

# LiDAR orientation calibration:
# Physical mounting: LiDAR 0 is offset from robot chassis front by ~270
# Robot FRONT = LiDAR sector 9 (270-300)
LIDAR_ROTATION_SECTORS = 3  # Calibrated: LiDAR 270 = Robot FRONT

# LiDAR position offset calibration:
# The LiDAR is mounted ~37cm behind the robot's front edge.
LIDAR_FRONT_OFFSET = 0.37  # meters from robot front to LiDAR center

# Linear velocity calibration
# Set to 1.0 to disable calibration
LINEAR_VEL_RATIO = 1.0

# Robot physical dimensions from ugv_beast.urdf
ROBOT_WIDTH = 0.18  # meters (14cm + 4cm safety margin)

# Recovery maneuver parameters
SAFE_TURN_CLEARANCE = 0.70  # 70cm - enough space to turn safely
MIN_BACKUP_DISTANCE = 0.35  # Minimum backup distance before turning
