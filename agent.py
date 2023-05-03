import math

import dearpygui.dearpygui as dpg
import numpy as np

from config import config
import utils
from obstacle import Obstacle
from obstacle_tracker import AbstractObstacleTracker, PursuitGuidanceTracker


class Agent:
    def __init__(self, radius=config['AGENT_RADIUS'], position: np.array = np.array([0.0, 0.0]),
                 global_goal=np.array([0.0, 0.0])):
        self.radius = radius
        self.position = position
        self.velocity = np.array([0.0, 0.0])
        self.global_goal = global_goal
        self.closest_local_goal = self.position
        self.obstacle_tracker: AbstractObstacleTracker = PursuitGuidanceTracker(horizon=30, position=self.position)

    def update(self, obstacles: list[Obstacle]):
        goals = self.obstacle_tracker.update(obstacles, self.position)
        self.closest_local_goal = self.position
        closest_distance = math.inf

        for goal in goals:
            distance = np.linalg.norm(goal - self.global_goal)
            if distance < closest_distance:
                closest_distance = distance
                self.closest_local_goal = goal

        velocity_direction = (self.closest_local_goal - self.position)
        velocity_direction = velocity_direction / np.linalg.norm(velocity_direction)

        self.velocity = velocity_direction * 0.1

        self.position = self.position + (self.velocity * config['TIMESTEP'])

    def draw(self):
        self.obstacle_tracker.draw()
        dpg.draw_circle(radius=utils.meters_to_pixels(self.radius),
                        center=utils.world_to_screen(self.position), fill=[255, 255, 255])

        dpg.draw_circle(radius=utils.meters_to_pixels(self.radius),
                        center=utils.world_to_screen(self.closest_local_goal), fill=[255, 255, 255])

        dpg.draw_circle(radius=utils.meters_to_pixels(self.radius),
                        center=utils.world_to_screen(self.global_goal), fill=[255, 255, 255])
