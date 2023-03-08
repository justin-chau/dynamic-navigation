from abc import ABC, abstractmethod

import utils
from obstacle import Obstacle
from lidar import Lidar
import numpy as np
from dearpygui import dearpygui as dpg
from config import config

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

        self.merged_lidar.merge([self.current_lidar, *self.predicted_lidars])

    def draw(self):
        self.merged_lidar.draw()


class PursuitGuidanceCurve:

    def __init__(self, target_obstacle: Obstacle, start_position: np.array = np.array([0.0, 0.0]), horizon: int = 30, agent_velocity: int = 0.1):
        self.start_position = start_position
        self.target_obstacle = target_obstacle
        self.horizon = horizon
        self.agent_velocity = agent_velocity

        self.curve_points: list[np.array] = [self.start_position]

        self.generate()

    def generate(self):
        for i in range(0, self.horizon):
            predicted_obstacle = self.target_obstacle.get_predicted_obstacle(i)

            direction_vector = predicted_obstacle.position - self.curve_points[i]
            direction_vector = direction_vector / np.linalg.norm(direction_vector)

            next_curve_point = self.curve_points[i] + (direction_vector * self.agent_velocity)
            print(next_curve_point)
            self.curve_points.append(next_curve_point)

    def draw(self):
        for curve_point in self.curve_points:
            dpg.draw_circle(radius=utils.meters_to_pixels(0.025), center=utils.world_to_screen(curve_point), fill=[255, 0, 0])


class PursuitGuidanceTracker(AbstractObstacleTracker):

    def __init__(self, horizon: int, position: np.array = np.array([0.0, 0.0])):
        super().__init__(horizon, position)
        self.pursuit_guidance_curves: list[PursuitGuidanceCurve] = []

    def update(self, obstacles: list[Obstacle], position: np.array = np.array([0.0, 0.0])):
        self.pursuit_guidance_curves = [PursuitGuidanceCurve(obstacle, position) for obstacle in obstacles]

    def draw(self):
        for pursuit_guidance_curve in self.pursuit_guidance_curves:
            pursuit_guidance_curve.draw()