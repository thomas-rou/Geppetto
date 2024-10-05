from .. core.entity import Entity
from .. core.pose import Pose
from .. core.size import Size


class Robot(Entity):
    def __init__(self, name="robot", pose: Pose = None, size: Size = None):
        self.name = name
        self.build_entity(pose, size)
