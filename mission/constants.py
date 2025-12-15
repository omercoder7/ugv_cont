"""
Constants for autonomous scanning and navigation.
"""

# Docker container name for ROS2 commands
CONTAINER_NAME = "ugv_rpi_ros_humble"

# =============================================================================
# SECTOR DEFINITIONS (Robot Frame)
# =============================================================================
# 12 sectors, 30° each, numbered 0-11
#
#                    FRONT
#                      0 (0°)
#               11 (-30°)  1 (+30°)
#           10 (-60°)          2 (+60°)
#         9 (-90°)                3 (+90°)
#           8 (-120°)          4 (+120°)
#               7 (-150°)  5 (+150°)
#                      6 (±180°)
#                     BACK
#
# Convention:
#   - Sector 0 = FRONT (0°)
#   - Sectors 1-6 = LEFT side (positive angles: +30° to +180°)
#   - Sectors 7-11 = RIGHT side (negative angles: -150° to -30°)
#   - Positive angular velocity = turn LEFT
#   - Negative angular velocity = turn RIGHT
# =============================================================================

NUM_SECTORS = 12  # 360 / 12 = 30° per sector

# Sector groups for obstacle detection
SECTOR_FRONT = 0
SECTORS_FRONT_ARC = (11, 0, 1)       # Front ±30° (right-front, front, left-front)
SECTORS_LEFT = (1, 2, 3)             # Left side: +30° to +90°
SECTORS_RIGHT = (9, 10, 11)          # Right side: -90° to -30°
SECTORS_BACK = (4, 5, 6, 7, 8)       # Back: +120° to -120°
SECTORS_BACK_LEFT = (3, 4, 5)        # Back-left: +90° to +150°
SECTORS_BACK_RIGHT = (7, 8, 9)       # Back-right: -150° to -90°

# LiDAR orientation calibration:
# Physical mounting: LiDAR 0 is offset from robot chassis front by ~270°
# Robot FRONT = LiDAR sector 9 (270-300°)
LIDAR_ROTATION_SECTORS = 3  # Calibrated: LiDAR 270° = Robot FRONT

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
