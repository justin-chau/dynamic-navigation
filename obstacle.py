import dearpygui.dearpygui as dpg
import numpy as np
from config import config
import utils
import random


class Obstacle:
    def __init__(self, radius=config['OBSTACLE_RADIUS'],
                 position: np.array = np.array([0.0, 0.0]),
                 velocity: np.array = np.array([0.0, 0.0]), randomize: bool = False):
        self.velocity = velocity
        self.position = position

        if randomize:
            self.velocity = np.array([random.uniform(config['MIN_OBSTACLE_VELOCITY'][0],
                                                     config['MAX_OBSTACLE_VELOCITY'][0]),
                                      random.uniform(config['MIN_OBSTACLE_VELOCITY'][1],
                                                     config['MAX_OBSTACLE_VELOCITY'][1])])

            self.position = np.array([random.uniform(-config['ENVIRONMENT_WIDTH'] / 2,
                                                     config['ENVIRONMENT_WIDTH'] / 2),
                                      random.uniform(-config['ENVIRONMENT_HEIGHT'] / 2,
                                                     config['ENVIRONMENT_HEIGHT'] / 2)])

        self.radius = radius

    def get_predicted_obstacle(self, n: int = 1):
        predicted_position = self.position + (self.velocity * config['TIMESTEP'] * n)

        return Obstacle(radius=config['OBSTACLE_RADIUS'], position=predicted_position, velocity=self.velocity)

    def update(self, n: int = 1):
        self.position = self.position + (self.velocity * config['TIMESTEP'] * n)

    def draw(self):
        dpg.draw_circle(radius=utils.meters_to_pixels(self.radius),
                        center=utils.world_to_screen(self.position))
