import math
from .classes.core.pose import Pose
from .classes.core.size import Size

# size of the map
max_width = 10
max_height = 5

# number of entities
robot_count = 2
n_wall_obstacles = 10

# constants
horizontal_yaw = math.pi / 2
wall_thickness = 0.1
atomic_gap = 0.0001
wall_gap = wall_thickness + atomic_gap
