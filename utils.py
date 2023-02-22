from __future__ import annotations

import numpy as np

from config import config


def meters_to_pixels(meters: int | float):
    return meters * config['PIXELS_PER_METER']


def world_to_screen(position: np.array):
    return meters_to_pixels(position[0] + config['ENVIRONMENT_WIDTH'] / 2), meters_to_pixels(position[1] * -1 + config['ENVIRONMENT_HEIGHT'] / 2)
