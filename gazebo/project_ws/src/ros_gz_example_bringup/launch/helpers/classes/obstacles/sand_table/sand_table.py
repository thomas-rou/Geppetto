import os
import sys
import random
from ..obstacle import Obstacle
from ...core.pose import Pose
from ...core.size import Size
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
from helpers.classes.core.direction import Direction


class Sand_Table(Obstacle):
    def __init__(
        self,
        pose: Pose = Pose(z=-WALL_THICKNESS / N_BOUNDARY_WALLS),
        size: Size = None,
    ) -> None:
        self.name = "sand_table"
        self.build_entity(pose, size)
        self.index = Obstacle.get_id()
        self.spawn_obstacle(self.name, self.index)
