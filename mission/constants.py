"""
Constants for autonomous scanning and navigation.
"""

# Docker container name for ROS2 commands
CONTAINER_NAME = "ugv_rpi_ros_humble"

# Visited locations grid settings
GRID_RESOLUTION = 0.5  # 50cm grid cells
VISITED_FILE = "/tmp/visited_locations.json"
NUM_SECTORS = 12  # 360° / 12 = 30° per sector

# Calibration: Robot actual turn rate at Z=1.0 command
# Measured with 30Hz movement thread: 5.07s for 90° rotation
ACTUAL_MAX_ANGULAR_VEL = 0.31  # rad/s at Z=1.0

# Linear velocity calibration (at 30Hz movement thread):
# Commanded 0.06 m/s -> Actual 0.14 m/s forward, 0.13 m/s backward
# Ratio: actual/commanded = 2.33 (forward), 2.17 (backward)
# Set to 1.0 to disable calibration - robot was moving too slow
LINEAR_VEL_RATIO = 1.0  # Set to 1.0 to disable velocity correction

# LiDAR orientation calibration:
# Run calibrate_lidar.py with robot FRONT facing clear space to determine this value.
# Physical mounting: LiDAR 0° is offset from robot chassis front by ~270°
# Calibration shows: Robot FRONT = LiDAR sector 9 (270°-300°)
# Formula: robot_sector = (lidar_sector + offset) % 12
#          lidar_sector = (robot_sector - offset) % 12
# With offset=3: Robot sector 0 (FRONT) <- LiDAR sector 9
LIDAR_ROTATION_SECTORS = 3  # Calibrated: LiDAR 270° = Robot FRONT

# LiDAR position offset calibration:
# The LiDAR is mounted ~37cm behind the robot's front edge.
# When the robot front is 41cm from an obstacle, LiDAR reads 78cm.
# We must add this offset to min_distance when comparing with LiDAR readings.
LIDAR_FRONT_OFFSET = 0.37  # meters from robot front to LiDAR center

# Robot physical dimensions from ugv_beast.urdf:
# - Wheel-to-wheel width: 0.14m (left wheel y=+0.062, right wheel y=-0.062)
# - Wheelbase length: 0.155m (front wheel x=+0.083, rear wheel x=-0.072)
# - With pan-tilt and camera: ~0.17m total length
# Add safety margins for collision avoidance
ROBOT_WIDTH = 0.18  # meters (14cm + 4cm safety margin)
ROBOT_LENGTH = 0.22  # meters (17cm + 5cm safety margin)
ROBOT_HALF_WIDTH = ROBOT_WIDTH / 2  # 0.09m - for corridor width checking

# Minimum corridor width the robot can fit through
MIN_CORRIDOR_WIDTH = ROBOT_WIDTH + 0.10  # 28cm - robot width + 10cm clearance

# Safe turning clearance - minimum front distance needed to safely turn in place
# Robot needs enough space in front to turn without hitting obstacles
# This is used by BACKING_UP state to know when to stop backing and start turning
SAFE_TURN_CLEARANCE = 0.70  # 70cm - enough space to turn safely
MAX_BACKUP_TIME = 5.0  # Maximum backup duration
MIN_BACKUP_DISTANCE = 0.35  # Minimum backup distance in meters (~35cm) before turning

# Danger distance margin - extra buffer added to min_distance for triggering avoidance
# Lower = more aggressive (closer approach before avoiding)
# Higher = more conservative (starts avoiding earlier)
DANGER_DISTANCE_MARGIN = 0.15  # 15cm margin - triggers avoidance at min_distance + this
CORRIDOR_DANGER_MARGIN = 0.05  # 5cm margin in corridors - tighter spaces need tighter control

# Brief stop duration between forward and backward movement
# This prevents mechanical stress and gives robot time to settle
DIRECTION_CHANGE_PAUSE = 0.15  # 150ms pause

# Wheel encoder stuck detection constants
# If wheels should be moving but encoder delta is below threshold, robot is stuck
WHEEL_ENCODER_STUCK_THRESHOLD = 0.05  # Minimum encoder change expected when moving
WHEEL_ENCODER_STUCK_TIME = 0.2  # Seconds of no encoder change to trigger stuck

# Adaptive speed control constants
# Robot slows down as it approaches obstacles for tighter maneuvering
SPEED_SCALE_FAR_DISTANCE = 1.5    # Distance (m) at which robot runs at full speed
SPEED_SCALE_NEAR_DISTANCE = 0.5   # Distance (m) at which robot runs at minimum speed
SPEED_SCALE_MIN_FACTOR = 0.4      # Minimum speed factor (40% of linear_speed)

# Body threshold for filtering invalid LiDAR readings
BODY_THRESHOLD_FRONT = 0.12  # Front sectors: real obstacles can be close
BODY_THRESHOLD_SIDE = 0.25   # Side/back sectors: may have body readings
