import dearpygui.dearpygui as dpg
from typing import Optional

import utils
import numpy as np
from obstacle import Obstacle
import shapely.geometry


class Laser:
    def __init__(self,
                 origin: np.array = np.array([0.0, 0.0]),
                 max_range=10,
                 angle=0,
                 detected_distance: Optional[float] = None):
        self.origin = origin
        self.angle = angle
        self.direction_vector = np.array([np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))])

        self.max_range = max_range
        self.max_end = self.direction_vector * self.max_range + self.origin

        self.detected_distance = self.max_range

        if detected_distance:
            self.detected_distance = detected_distance

    def detect_obstacles(self, obstacles: list[Obstacle]):
        min_dist = self.max_range
        for obstacle in obstacles:
            circle = shapely.Point(obstacle.position).buffer(obstacle.radius).boundary
            line = shapely.LineString([self.origin, self.max_end])

            i = circle.intersection(line)

            if isinstance(i, shapely.MultiPoint):
                p1: shapely.Point = i.geoms[0]
                p2 = i.geoms[1]

                p1_dist = p1.distance(shapely.Point(self.origin))
                p2_dist = p2.distance(shapely.Point(self.origin))

                if p1_dist < min_dist:
                    min_dist = p1_dist

                if p2_dist < min_dist:
                    min_dist = p2_dist

        self.detected_distance = min_dist

        return min_dist

    def draw(self):
        if self.detected_distance is not self.max_range:
            dpg.draw_line(p1=utils.world_to_screen(self.origin), p2=utils.world_to_screen(self.direction_vector * self.detected_distance + self.origin), color=[255, 50, 50])
