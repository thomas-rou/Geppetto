import math
from .classes.core.pose import Pose
from .classes.core.size import Size
from .classes.obstacles.wall import Wall
from .classes.robots.robot import Robot


max_width = 10
max_height = 5

wall_thickness = 0.1

robot_count = 2
n_wall_obstacles = 10

horizontal_yaw = math.pi / 2

# fmt: off
boundary_walls = [
    Wall(pose=Pose(y= max_width/2),               size=Size(x=max_width)), # west wall
    Wall(pose=Pose(y=-max_width/2),               size=Size(x=max_width)), # east wall
    Wall(pose=Pose(x= max_width/2 , yaw=horizontal_yaw),  size=Size(x=max_width)), # north wall
    Wall(pose=Pose(x=-max_width/2 , yaw=horizontal_yaw),  size=Size(x=max_width)), # south wall
]

robots = [
    Robot(name="pino", size=Size(x=0.20, y=0.20, z=0.1)),
    Robot(name="chio", pose=Pose(y=1), size=Size(x=0.20, y=0.20, z=0.1)),
]
# fmt: on
