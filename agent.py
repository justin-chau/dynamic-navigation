import dearpygui.dearpygui as dpg
import numpy as np

from config import config
import utils
from obstacle import Obstacle
from obstacle_tracker import AbstractObstacleTracker, PursuitGuidanceTracker, MergedLaserScanTracker


class Agent:
    def __init__(self, radius=config['AGENT_RADIUS'], position: np.array = np.array([0.0, 0.0]), velocity: np.array = np.array([0.0, 0.0])):
        self.radius = radius
        self.position = position
        self.velocity = velocity
        self.obstacle_tracker: AbstractObstacleTracker = PursuitGuidanceTracker(horizon=50, position=self.position)
        # self.obstacle_tracker_2: AbstractObstacleTracker = MergedLaserScanTracker(horizon=5, position=self.position)

    def update(self, obstacles: list[Obstacle]):
        self.position = self.position + (self.velocity * config['TIMESTEP'])
        self.obstacle_tracker.update(obstacles, self.position)
        # self.obstacle_tracker_2.update(obstacles, self.position)

    def draw(self):
        self.obstacle_tracker.draw()
        # self.obstacle_tracker_2.draw()
        dpg.draw_circle(radius=utils.meters_to_pixels(self.radius),
                        center=utils.world_to_screen(self.position), fill=[255, 255, 255])
