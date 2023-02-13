import time

import dearpygui.dearpygui as dpg
import math

from config import config
from agent import Agent
from obstacle import Obstacle
import utils


class Simulation:
    def __init__(self, height: int = config['ENVIRONMENT_HEIGHT'], width: int = config['ENVIRONMENT_WIDTH']):
        self.height = height
        self.width = width

        self.agent = Agent(position=(self.height / 2, self.width / 2))

        self.obstacles = [Obstacle() for x in range(config['OBSTACLE_COUNT'])]

        dpg.create_context()

        viewport_height = math.floor(utils.world_to_screen(self.height))
        viewport_width = math.floor(utils.world_to_screen(self.width))

        dpg.create_viewport(title="Dynamic Navigation Exploration", width=viewport_width, height=viewport_height)
        dpg.setup_dearpygui()
        dpg.show_viewport()

    def run(self):
        while dpg.is_dearpygui_running():

            with dpg.viewport_drawlist():
                for obstacle in self.obstacles:
                    obstacle.step()
                    obstacle.draw()

                self.agent.draw()

            time.sleep(config['TIMESTEP'])
            dpg.render_dearpygui_frame()

    def teardown(self):
        dpg.destroy_context()
