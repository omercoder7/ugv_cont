"""
Constants for autonomous scanning and navigation.
"""

# Docker container name for ROS2 commands
CONTAINER_NAME = "ugv_rpi_ros_humble"

# =============================================================================
# SECTOR DEFINITIONS (Robot Frame)
# =============================================================================
# 60 sectors, 6° each, numbered 0-59
#
# Convention:
#   - Sector 0 = FRONT (0°)
#   - Sectors 1-30 = LEFT side (positive angles: +6° to +180°)
#   - Sectors 31-59 = RIGHT side (negative angles: -174° to -6°)
#   - Positive angular velocity = turn LEFT
#   - Negative angular velocity = turn RIGHT
# =============================================================================

NUM_SECTORS = 60  # 360 / 60 = 6° per sector

# Sector groups for obstacle detection
# With 60 sectors, each sector = 6°
SECTOR_FRONT = 0
SECTOR_BACK = 30                               # Straight back (180°)
SECTORS_FRONT_ARC = tuple(range(55, 60)) + tuple(range(0, 6))  # Front ±30° (10 sectors)
SECTORS_LEFT = tuple(range(1, 16))              # Left side: +6° to +90° (15 sectors)
SECTORS_RIGHT = tuple(range(46, 60))            # Right side: -84° to -6° (14 sectors)
SECTORS_BACK = tuple(range(16, 46))             # Back: +96° to -90° (30 sectors)
SECTORS_BACK_LEFT = tuple(range(16, 26))        # Back-left: +96° to +150° (10 sectors)
SECTORS_BACK_RIGHT = tuple(range(36, 46))       # Back-right: -144° to -90° (10 sectors)

# LiDAR orientation calibration:
# Physical mounting: LiDAR 0 is offset from robot chassis front by ~270°
# Robot FRONT = LiDAR ~270° region
# With 60 sectors (6° each): 15 sectors = 90° rotation
LIDAR_ROTATION_SECTORS = 15  # Calibrated: LiDAR 270° = Robot FRONT

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
