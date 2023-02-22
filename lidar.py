from __future__ import annotations
import numpy as np
import math

from laser import Laser
from obstacle import Obstacle


class Lidar:
    def __init__(self, max_range=5, position: np.array = np.array([0.0, 0.0]), angular_resolution=0.25):
        self.position = position
        self.max_range = max_range
        self.angular_resolution = angular_resolution
        self.lasers = [Laser(angle=i, max_range=max_range, origin=position) for i in range(0, 360, math.floor(1/angular_resolution))]

    def draw(self):
        for laser in self.lasers:
            laser.draw()

    def merge(self, others: list[Lidar]):
        for i, laser in enumerate(self.lasers):
            distances = []
            for lidar in others:
                distances.append(lidar.lasers[i].detected_distance)

            min_distance = min(distances)

            laser.detected_distance = min_distance

    def update(self, obstacles: list[Obstacle]):
        for i, laser in enumerate(self.lasers):
            laser.detect_obstacles(obstacles)
