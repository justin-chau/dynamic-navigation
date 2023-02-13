from typing import TypedDict


class ConfigDict(TypedDict):
    PIXELS_PER_METER: int
    ENVIRONMENT_WIDTH: int
    ENVIRONMENT_HEIGHT: int
    AGENT_RADIUS: float
    OBSTACLE_RADIUS: float
    TIMESTEP: float
    MAX_OBSTACLE_VELOCITY: tuple[float, float]
    MIN_OBSTACLE_VELOCITY: tuple[float, float]
    OBSTACLE_COUNT: int


config: ConfigDict = {
    'PIXELS_PER_METER': 50,
    'ENVIRONMENT_WIDTH': 10,
    'ENVIRONMENT_HEIGHT': 10,
    'AGENT_RADIUS': 0.1,
    'OBSTACLE_RADIUS': 0.1,
    'TIMESTEP': 0.1,
    'MAX_OBSTACLE_VELOCITY': (0.3, 0.3),
    'MIN_OBSTACLE_VELOCITY': (-0.3, -0.3),
    'OBSTACLE_COUNT': 10
}
