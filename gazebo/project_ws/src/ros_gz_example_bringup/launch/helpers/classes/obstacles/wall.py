import os
import sys
import random
from .obstacle import Obstacle
from ..core.pose import Pose
from ..core.size import Size
from ament_index_python.packages import get_package_share_directory

# import helpers directory
current_dir = os.path.dirname(__file__)
sys.path.append(current_dir)

helpers_dir = os.path.join(
    get_package_share_directory("ros_gz_example_bringup"),
    "helpers",
)
sys.path.append(helpers_dir)

from helpers.config import *


class Wall(Obstacle):
    def __init__(
        self, pose: Pose = None, size: Size = None, starter_wall: bool = False
    ):
        self.name = "wall"
        self.build_entity(pose, size)

        if starter_wall and not Wall.check_spawn_kill(self):
            self.spawn_wall()

    def spawn_wall(self):
        self.index = Obstacle.get_id()
        self.spawn_obstacle(self.name, self.index)

    @staticmethod
    def generate_random_wall_obstacles(n_walls: int):
        n_border_walls = random.randint(0, n_walls)
        n_inner_walls = n_walls - n_border_walls

        border_walls = [
            Wall._generate_random_border_wall() for _ in range(n_border_walls)
        ]
        inner_walls = [Wall._generate_random_inner_wall() for _ in range(n_inner_walls)]

        walls = border_walls + inner_walls
        for wall in walls:
            wall.spawn_wall()

    # fmt: off
    @staticmethod
    def _generate_random_border_wall():
        while True:
            start = random.randint(0, 3) # 4 sides
            size=Size(x=random.uniform(0.5, max_width/2))
            match(start):
                case 0 : # north
                    wall = Wall(
                        pose=Pose(y=random.uniform(-max_width / 2 + wall_gap, max_width / 2 - wall_gap), x=max_width / 2 - size.x / 2 - wall_gap), size=size)
                case 1 : # south
                    wall = Wall(
                        pose=Pose(y=random.uniform(-max_width / 2 + wall_gap, max_width / 2 - wall_gap), x=-max_width / 2 + size.x / 2 + wall_gap), size=size)
                case 2 : # west
                    wall = Wall(
                        pose=Pose(x=random.uniform(-max_width / 2 + wall_gap, max_width / 2 - wall_gap), y=max_width / 2 - size.x / 2  - wall_thickness / 2, yaw=horizontal_yaw), size=size)
                case 3 : # east
                    wall = Wall(
                        pose=Pose(x=random.uniform(-max_width / 2 + wall_gap, max_width / 2 - wall_gap), y=-max_width / 2 + size.x / 2 + wall_thickness / 2, yaw=horizontal_yaw), size=size)
            if not Wall.check_spawn_kill(wall):
                break

        return wall
    
    @staticmethod
    def _generate_random_inner_wall():
        while True:
            wall = Wall(
                        pose=Pose(x=random.uniform(-max_height / 2 + wall_thickness, max_height / 2 - wall_thickness), y=random.uniform(-max_width  / 2 + wall_thickness, max_width / 2 - wall_thickness), yaw=random.uniform(0, math.pi)), 
                        size=Size(x=random.uniform(0.5, max_width/2)))
            
            if not Wall.check_spawn_kill(wall):
                break

        return wall
    # fmt: on
    @classmethod
    def check_spawn_kill(cls, wall):
        return super().check_spawn_kill(wall)
