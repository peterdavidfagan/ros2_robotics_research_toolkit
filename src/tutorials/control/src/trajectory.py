"""Sample trajectory for the lite6"""
from abc import ABC, abstractmethod
import numpy as np
from geometry_msgs.msg import Point, Quaternion

class Trajectory(ABC):

    def __init__(self):
        pass

    @abstractmethod
    def sample_position(self, t) -> Point:
        """Samples a position from a trajectory."""
        pass

    @abstractmethod
    def sample_orientation(self, t) -> Quaternion:
        """Samples an orientation from a trajectory."""
        pass

    def sample_point(self, t):
        """Samples waypoints from a trajectory."""
        position = self.sample_position(t)
        orientation = self.sample_orientation(t)
        return position, orientation

    def sample_points(self, num_points):
        """Samples points from a trajectory."""
        positions = []
        orientations = []
        for t in np.linspace(0, 1, num_points):
            position, orientation = self.sample_point(t*20)
            positions.append(position)
            orientations.append(orientation)

        return positions, orientations

class Spiral(Trajectory):

    def __init__(self, radius, height):
        self.radius = radius
        self.height = height

    def sample_position(self, t):
        p = Point()
        #p.x = self.radius * ((np.cos(2 * np.pi * t) + 1 / 2)) + 0.1
        #p.y = self.radius * np.sin(2 * np.pi * t)
        #p.z = (self.height * t) + 0.1
        p.x = ((np.sin(t) + 1) / 10) + 0.1
        p.y = ((np.cos(t) + 1) / 10)
        p.z = (t / 50) + 0.1
        return p

    def sample_orientation(self, t):
        q = Quaternion()
        q.x = 0.924
        q.y = -0.382
        q.z = 0.0
        q.w = 0.0
        return q
