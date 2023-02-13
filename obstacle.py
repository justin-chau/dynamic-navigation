import random
import dearpygui.dearpygui as dpg

from config import config
import utils


class Obstacle:
    def __init__(self, radius=config['OBSTACLE_RADIUS']):
        velocity_x = random.uniform(config['MIN_OBSTACLE_VELOCITY'][0], config['MAX_OBSTACLE_VELOCITY'][0])
        velocity_y = random.uniform(config['MIN_OBSTACLE_VELOCITY'][1], config['MAX_OBSTACLE_VELOCITY'][1])

        self.velocity = (velocity_x, velocity_y)

        position_x = random.uniform(0, config['ENVIRONMENT_WIDTH'])
        position_y = random.uniform(0, config['ENVIRONMENT_HEIGHT'])

        self.position = (position_x, position_y)

        self.radius = radius

    def step(self):
        position_x = self.position[0] + self.velocity[0] * config['TIMESTEP']
        position_y = self.position[1] + self.velocity[1] * config['TIMESTEP']
        self.position = (position_x, position_y)

    def draw(self):
        dpg.draw_circle(radius=utils.world_to_screen(self.radius),
                        center=(utils.world_to_screen(self.position[0]),
                                utils.world_to_screen(self.position[1])))
