from  .  obstacle import Obstacle
from .. core.pose import Pose
from .. core.size import Size

class Wall(Obstacle):
    def __init__(self, pose: Pose = None, size: Size = None):
        self.name = "wall"
        self.build_entity(pose, size)