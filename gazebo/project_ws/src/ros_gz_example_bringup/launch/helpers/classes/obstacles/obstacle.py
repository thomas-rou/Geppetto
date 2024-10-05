from .. core.entity import Entity
from abc import abstractmethod

# pure abstract class
class Obstacle(Entity):
    _id = 0

    @abstractmethod
    def __init__(self, name: str):
        pass

    def get_id():
        current_id = Obstacle._id
        Obstacle._id += 1
        return current_id
