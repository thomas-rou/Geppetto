from abc import ABC, abstractmethod
from . pose import Pose
from . size import Size


# pure abstract class
class Entity(ABC):
    pose: Pose
    size: Size
    name: str

    @abstractmethod
    def __init__(self):
        pass

    def build_entity(self, pose: Pose = None, size: Size = None):
        # Initialize pose with provided values or default values
        if pose is None:
            self.pose = Pose()
        else:
            self.pose = Pose(
                x=pose.x if hasattr(pose, "x") else 0.0,
                y=pose.y if hasattr(pose, "y") else 0.0,
                z=pose.z if hasattr(pose, "z") else 0.0,
                roll=pose.roll if hasattr(pose, "roll") else 0.0,
                pitch=pose.pitch if hasattr(pose, "pitch") else 0.0,
                yaw=pose.yaw if hasattr(pose, "yaw") else 0.0,
            )

        # Initialize size with provided values or default values
        if size is None:
            self.size = Size()
        else:
            self.size = Size(
                x=size.x if hasattr(size, "x") else 0.0,
                y=size.y if hasattr(size, "y") else 0.0,
                z=size.z if hasattr(size, "z") else 0.0,
            )