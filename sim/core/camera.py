import math
from OpenGL.GLU import gluLookAt


class OrbitCamera:
    def __init__(self):
        self.target = [0.0, 0.0, 0.0]
        self.distance = 700.0
        self.yaw = 230.0
        self.pitch = 30.0

    def apply(self):
        yr = math.radians(self.yaw)
        pr = math.radians(self.pitch)
        cx = self.target[0] + self.distance * math.cos(pr) * math.cos(yr)
        cy = self.target[1] + self.distance * math.cos(pr) * math.sin(yr)
        cz = self.target[2] + self.distance * math.sin(pr)
        gluLookAt(cx, cy, cz, self.target[0], self.target[1], self.target[2], 0, 0, 1)
