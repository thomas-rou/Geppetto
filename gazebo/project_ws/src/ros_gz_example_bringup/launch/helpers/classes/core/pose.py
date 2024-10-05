import numpy as np

class Pose:
    def __init__(self, x=0.0, y=0.0, z=0.1, roll=0.0, pitch=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def to_array(self):
        return np.array([self.x, self.y, self.z])
