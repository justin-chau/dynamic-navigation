import time

import dearpygui.dearpygui as dpg
import math

from config import config
from agent import Agent
from obstacle import Obstacle
import numpy as np
import utils


class Simulation:
    def __init__(self, height: int = config['ENVIRONMENT_HEIGHT'], width: int = config['ENVIRONMENT_WIDTH']):
        self.height = height
        self.width = width
        self.is_running = True

        self.agent = Agent(position=np.array([0.5, 0.5]), global_goal=np.array([4.5, 4.5]))

        self.obstacles = [Obstacle(position=np.array([3.0, -1.0]), velocity=np.array([0.0, 0.1])),
                          Obstacle(position=np.array([1.0, 3.0]), velocity=np.array([-0.1, 0.0])),
                          Obstacle(position=np.array([-1, -1]), velocity=np.array([0.15, -0.15])),
                          Obstacle(position=np.array([3.0, -4.0]), velocity=np.array([0.1, 0.25]))]

        dpg.create_context()

        viewport_height = math.floor(utils.meters_to_pixels(self.height))
        viewport_width = math.floor(utils.meters_to_pixels(self.width))

        with dpg.handler_registry():
            dpg.add_key_press_handler(callback=self.toggle_is_running)

        dpg.create_viewport(title="Dynamic Navigation Exploration", width=viewport_width, height=viewport_height)
        dpg.setup_dearpygui()
        dpg.show_viewport()

    def toggle_is_running(self):
        self.is_running = not self.is_running

    def run(self):
        while dpg.is_dearpygui_running():
            if self.is_running:
                if dpg.does_item_exist('canvas'):
                    dpg.delete_item('canvas')

                with dpg.viewport_drawlist(tag='canvas'):
                    for obstacle in self.obstacles:
                        obstacle.update()
                        obstacle.draw()

                    self.agent.update(self.obstacles)
                    self.agent.draw()

                time.sleep(config['TIMESTEP'])
            dpg.render_dearpygui_frame()

    def teardown(self):
        dpg.destroy_context()
