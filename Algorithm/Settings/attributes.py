from Settings.config import *


# Robot Attributes
ROBOT_LENGTH = 20 * SCALING_FACTOR  # Recommended robot footprint is 30cm by 30cm
ROBOT_TURN_RADIUS = 30 * SCALING_FACTOR
ROBOT_SPEED_PER_SECOND = 30 * SCALING_FACTOR
ROBOT_S_FACTOR = ROBOT_LENGTH / ROBOT_TURN_RADIUS  
ROBOT_SAFETY_DISTANCE = 15 * SCALING_FACTOR
ROBOT_SCAN_TIME = 0.25  # Time provided for scanning an obstacle image in seconds.

# Grid Attributes
GRID_LENGTH = 200 * SCALING_FACTOR  # Movement area is 200cm by 200cm
GRID_CELL_LENGTH = 10 * SCALING_FACTOR  # Grid cell is 10cm by 10cm
GRID_START_BOX_LENGTH = 30 * SCALING_FACTOR  # Recommended starting area is 40cm by 40cm
GRID_NUM_GRIDS = GRID_LENGTH // GRID_CELL_LENGTH  # Number of grid cells

# Obstacle Attributes
OBSTACLE_LENGTH = 10 * SCALING_FACTOR  # Obstacle is 10cm by 10cm
OBSTACLE_SAFETY_WIDTH = ROBOT_SAFETY_DISTANCE // 3 * 4  # With respect to the center of the obstacle

# Path Finding Attributes
PATH_TURN_COST = 999 * ROBOT_SPEED_PER_SECOND * ROBOT_TURN_RADIUS
# NOTE: Higher number == Lower Granularity == Faster Checking.
# Must be an integer more than 0! Number higher than 3 not recommended.
PATH_TURN_CHECK_GRANULARITY = 1