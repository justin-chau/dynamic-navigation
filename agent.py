import dearpygui.dearpygui as dpg
import numpy as np

from config import config
import utils
from lidar import Lidar
from obstacle import Obstacle


class Agent:
    def __init__(self, radius=config['AGENT_RADIUS'], position: np.array = np.array([0.0, 0.0]), horizon: int = 30):
        self.radius = radius
        self.position = position
        self.horizon = horizon

        self.current_lidar = Lidar(position=position, angular_resolution=0.5)
        self.predicted_lidars = [Lidar(position=position, angular_resolution=0.5) for _ in range(0, horizon)]

        self.merged_lidar = Lidar(position=position, angular_resolution=0.5)
        print(len(self.merged_lidar.lasers))

    def update(self, obstacles: list[Obstacle]):
        self.current_lidar.update(obstacles)

        for i, predicted_lidar in enumerate(self.predicted_lidars):
            predicted_lidar.update([obstacle.get_predicted_obstacle(i) for obstacle in obstacles])

        self.merged_lidar.merge([self.current_lidar, *self.predicted_lidars])

    def draw(self):
        self.merged_lidar.draw()

        dpg.draw_circle(radius=utils.meters_to_pixels(self.radius),
                        center=utils.world_to_screen(self.position), fill=[255, 255, 255])
