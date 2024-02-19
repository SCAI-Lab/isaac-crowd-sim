from typing import Dict, List, Optional, Set

import numpy as np
import open3d as o3d


class Obstacle:
    def __init__(self, name: str) -> None:
        self.name = name
        self.position = np.array([])
        self.orientation: float = None
        self.velocity = np.array([])
        self.is_2d: bool = False
        self.is_character: bool = True

        self.length = 0.8
        self.width = 0.6
        self.height = 1.85

    def get_number_of_pnts_in_bb(self, output_file: str, bb_width: float) -> int:
        pcd = o3d.io.read_point_cloud(output_file)
        pcd_np = np.asarray(pcd.points)

        dist_from_centre = np.abs(pcd_np - self.position[0:3])
        max_distance = np.array([self.length / 2, bb_width / 2, self.height / 2])
        pnt_idx = np.where(np.all(dist_from_centre <= max_distance, axis=1))[0]
        return len(pnt_idx)

    def get_bounding_box(self, pcd_file: int, output_dir: str) -> Optional[Dict[str, float]]:
        output_file = f"{str(pcd_file).zfill(6)}.pcd"
        bb_width = self.width + float(np.linalg.norm(self.velocity)) * 0.3
        self.position[2] = 0 if self.is_2d else self.height / 2

        num_pnts = self.get_number_of_pnts_in_bb(
            f"{output_dir}/../pointcloud/{output_file}", bb_width
        )
        if num_pnts == 0:
            return None

        self.velocity[2] = 0.0

        if self.is_2d:
            bounding_box = {
                "cx": float(self.position[0]),
                "cy": float(self.position[1]),
                "l": self.length,
                "w": bb_width,
                "rot_z": self.orientation,
            }
        else:
            bounding_box = {
                "cx": float(self.position[0]),
                "cy": float(self.position[1]),
                "cz": float(self.position[2]),
                "l": self.length,
                "w": bb_width,
                "h": self.height,
                "rot_z": self.orientation,
            }
        label_id = f"Character:{self.name}" if self.is_character else f"Obstacle:{self.name}"
        bb_data = {
            "box": bounding_box,
            "file_id": output_file,
            "label_id": label_id,
            "attributes": {
                "position": list(self.position),
                "velocity": list(self.velocity),
                "bb_pnts": num_pnts,
            },
        }

        return bb_data
