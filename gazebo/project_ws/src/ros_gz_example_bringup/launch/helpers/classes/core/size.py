import numpy as np

class Size:
    def __init__(self, x=0.0, y=0.1, z=0.2):
        self.x = x
        self.y = y
        self.z = z

    def to_array(self):
        return np.array([self.x, self.y, self.z])
