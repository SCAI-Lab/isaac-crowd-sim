from typing import Tuple

import numpy as np
from pxr import Gf


def quaternion_array_to_gf_element(quaternion: np.ndarray) -> Gf.Quatd:
    return Gf.Quatd(quaternion[0], quaternion[1], quaternion[2], quaternion[3])


class ImageConverter:
    def __init__(self, img_dim: Tuple[float, float], obj_dim: Tuple[float, float]) -> None:
        self.X_dim, self.Y_dim = obj_dim  # size x,y of actual airport [m]
        self.x_dim, self.y_dim, _ = img_dim

    def meter_to_pixel(self, x_m: float, y_m: float) -> Tuple[int, int]:
        center_x = self.x_dim // 2
        center_y = self.y_dim // 2

        x_px = int(x_m * self.x_dim / self.X_dim + center_x)
        y_px = int(y_m * self.y_dim / self.Y_dim + center_y)

        return np.clip(x_px, 0, self.x_dim - 1), np.clip(y_px, 0, self.y_dim - 1)

    # Scale image and create transformation such that the center of the image is at coordinates (0,0)
    def pixel_to_meter(self, x_px: int, y_px: int) -> Tuple[float, float]:
        center_x = self.x_dim // 2
        center_y = self.y_dim // 2

        x_m = (x_px - center_x) * self.X_dim / self.x_dim
        y_m = (y_px - center_y) * self.Y_dim / self.y_dim

        return x_m, y_m
