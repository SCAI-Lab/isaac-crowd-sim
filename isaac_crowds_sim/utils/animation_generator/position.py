from random import random

import numpy as np


class Position:
    def __init__(self, x: float, y: float, queue: bool = False, error: float = 5.0) -> None:
        self.position = np.array([x, y])
        self.error = error
        self.queue = queue

    def get_close_position(self) -> np.ndarray:
        x = self.position[0] + random() * self.error - 0.5 * self.error
        y = self.position[1] + random() * self.error - 0.5 * self.error

        return np.array([x, y])
