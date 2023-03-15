from __future__ import annotations
from abc import ABC, abstractmethod
from math import atan2

import utils
from obstacle import Obstacle
from lidar import Lidar
import numpy as np
from dearpygui import dearpygui as dpg


class AbstractObstacleTracker(ABC):
    @abstractmethod
    def __init__(self, horizon: int, position: np.array = np.array([0.0, 0.0])):
        self.position = position
        self.horizon = horizon

    @abstractmethod
    def update(self, obstacles: list[Obstacle], position: np.array = np.array([0.0, 0.0])):
        pass

    @abstractmethod
    def draw(self):
        pass


class MergedLaserScanTracker(AbstractObstacleTracker):

    def __init__(self, horizon: int, position: np.array = np.array([0.0, 0.0])):
        super().__init__(horizon, position)
        self.current_lidar = Lidar(position=position, angular_resolution=0.5)
        self.predicted_lidars = [Lidar(position=position, angular_resolution=0.5) for _ in range(0, horizon)]
        self.merged_lidar = Lidar(position=position, angular_resolution=0.5)

    def update(self, obstacles: list[Obstacle], position: np.array = np.array([0.0, 0.0])):
        self.current_lidar.position = position
        self.current_lidar.update(obstacles)
        for i, predicted_lidar in enumerate(self.predicted_lidars):
            predicted_lidar.position = position
            predicted_lidar.update([obstacle.get_predicted_obstacle(i) for obstacle in obstacles])

        self.merged_lidar.position = position
        self.merged_lidar.merge([self.current_lidar, *self.predicted_lidars])

    def draw(self):
        self.merged_lidar.draw()


class PursuitGuidanceCurve:

    def __init__(self, target_obstacle: Obstacle, start_position: np.array = np.array([0.0, 0.0]), horizon: int = 30,
                 agent_velocity: int = 0.1):
        self.start_position = start_position

        self.target_obstacle = target_obstacle
        self.horizon = horizon
        self.agent_velocity = agent_velocity

        self.points: list[np.array] = [self.start_position]

        self.generate()

    def generate(self):
        for i in range(0, self.horizon):
            predicted_obstacle = self.target_obstacle.get_predicted_obstacle(i)

            direction_vector = predicted_obstacle.position - self.points[i]
            direction_vector = direction_vector / np.linalg.norm(direction_vector)

            next_curve_point = self.points[i] + (direction_vector * self.agent_velocity)

            self.points.append(next_curve_point)

    def start_direction_vector(self) -> np.array:
        return self.points[1] - self.points[0]

    def end_direction_vector(self) -> np.array:
        return self.points[-1] - self.points[0]

    def draw(self, opacity):
        for point in self.points:
            dpg.draw_circle(radius=utils.meters_to_pixels(0.025),
                            center=utils.world_to_screen(point), fill=[255, 0, 0, opacity])

    @staticmethod
    def sort(curve: PursuitGuidanceCurve):
        return atan2(curve.end_direction_vector()[1], curve.end_direction_vector()[0]) - atan2(0, 1)

    def get_start_angle_to_x(self):
        return atan2(self.start_direction_vector()[1], self.start_direction_vector()[0]) - atan2(0, 1)

    def get_end_angle_to_x(self):
        return atan2(self.end_direction_vector()[1], self.end_direction_vector()[0]) - atan2(0, 1)


class PursuitGuidanceCurvePair:
    def __init__(self, a: PursuitGuidanceCurve, b: PursuitGuidanceCurve):
        self.a = a
        self.b = b

        self.a_start_vector = self.a.start_direction_vector()
        self.b_start_vector = self.b.start_direction_vector()

        self.a_end_vector = self.a.end_direction_vector()
        self.b_end_vector = self.b.end_direction_vector()

        self.connecting_vector = self.a.points[-1] - self.b.points[-1]

        self.mid_point_vector = self.a.points[-1] - self.b.points[-1]
        self.mid_point_vector = self.mid_point_vector / 2

        self.is_valid = (a.get_start_angle_to_x() < b.get_start_angle_to_x() and a.get_end_angle_to_x() < b.get_end_angle_to_x()) or (a.get_start_angle_to_x() > b.get_start_angle_to_x() and a.get_end_angle_to_x() > b.get_end_angle_to_x())

    def draw(self):
        dpg.draw_line(utils.world_to_screen(self.a.points[0]),
                      utils.world_to_screen(self.a_end_vector + self.a.points[0]), color=(0, 0, 255, 255))
        dpg.draw_line(utils.world_to_screen(self.b.points[0]),
                      utils.world_to_screen(self.b_end_vector + self.b.points[0]), color=(0, 0, 255, 255))

        dpg.draw_line(utils.world_to_screen(self.a.points[0]),
                      utils.world_to_screen(self.a_start_vector * 5 + self.a.points[0]), thickness=3,
                      color=(0, 0, 255, 255))

        dpg.draw_line(utils.world_to_screen(self.b.points[0]),
                      utils.world_to_screen(self.b_start_vector * 5 + self.b.points[0]), thickness=1,
                      color=(0, 0, 255, 255))

        dpg.draw_line(utils.world_to_screen(self.b.points[-1]),
                      utils.world_to_screen(self.connecting_vector + self.b.points[-1]), thickness=1,
                      color=(0, 0, 255, 255))

        if self.is_valid:
            dpg.draw_circle(
                          center=utils.world_to_screen(self.mid_point_vector + self.b.points[-1]), radius=5,
                          fill=(0, 255, 0, 255))


class PursuitGuidanceTracker(AbstractObstacleTracker):

    def __init__(self, horizon: int, position: np.array = np.array([0.0, 0.0])):
        super().__init__(horizon, position)
        self.pursuit_guidance_curves: list[PursuitGuidanceCurve] = []
        self.pursuit_guidance_curve_pairs: list[PursuitGuidanceCurvePair] = []

    def update(self, obstacles: list[Obstacle], position: np.array = np.array([0.0, 0.0])):
        self.position = position
        self.pursuit_guidance_curves = [PursuitGuidanceCurve(obstacle, position, horizon=self.horizon) for obstacle in
                                        obstacles]
        self.pursuit_guidance_curve_pairs = []

        self.pursuit_guidance_curves.sort(key=PursuitGuidanceCurve.sort)

        for i in range(0, len(self.pursuit_guidance_curves)):
            if i is not len(self.pursuit_guidance_curves) - 1:
                self.pursuit_guidance_curve_pairs.append(
                    PursuitGuidanceCurvePair(self.pursuit_guidance_curves[i], self.pursuit_guidance_curves[i + 1]))
            else:
                self.pursuit_guidance_curve_pairs.append(
                    PursuitGuidanceCurvePair(self.pursuit_guidance_curves[i], self.pursuit_guidance_curves[0]))

    def draw(self):
        for i, pursuit_guidance_curve in enumerate(self.pursuit_guidance_curves):
            pursuit_guidance_curve.draw(opacity=255 / (i + 1))

        for pair in self.pursuit_guidance_curve_pairs:
            pair.draw()
