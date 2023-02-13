import dearpygui.dearpygui as dpg

from config import config
import utils


class Agent:
    def __init__(self, radius=config['AGENT_RADIUS'], position=(0, 0)):
        self.radius = radius
        self.position = position

    def draw(self):
        dpg.draw_circle(radius=utils.world_to_screen(self.radius),
                        center=(utils.world_to_screen(self.position[0]),
                                utils.world_to_screen(self.position[1])))
