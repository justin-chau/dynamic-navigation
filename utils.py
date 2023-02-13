from __future__ import annotations
from config import config


def world_to_screen(meters: int | float):
    return meters * config['PIXELS_PER_METER']