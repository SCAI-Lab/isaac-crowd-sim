import json
from typing import List

import numpy as np
from utils.rosbag_exporter.obstacle import Obstacle


class Label:
    def __init__(self, timestamp: str) -> None:
        self.timestamp = timestamp
        self.obstacles: List[Obstacle] = []
        self.scan_index: int = None
        self.lin_velocity: List[float] = []
        self.ang_velocity: List[float] = []
        self.translation: List[float] = []
        self.quaternion: List[float] = []
        self.label_data_set: List[bool] = [
            False,
            False,
            False,
        ]  # [label_index, obstacles, wheelchair_position]

    def get_obstacle(self, name: str) -> int:
        for idx, obstacle in enumerate(self.obstacles):
            if obstacle.name == name:
                return idx

        obstacle_id = len(self.obstacles)
        obstacle = Obstacle(name)
        self.obstacles.append(obstacle)

        return obstacle_id

    def set_label_index(self, index: int) -> bool:
        self.scan_index = index
        self.label_data_set[0] = True
        return np.all(self.label_data_set)

    def set_label_obstacles_added(self) -> bool:
        self.label_data_set[1] = True
        return np.all(self.label_data_set)

    def set_label_position_added(self) -> bool:
        self.label_data_set[2] = True
        return np.all(self.label_data_set)

    def generate_label_file(self, output_dir: str) -> None:
        if self.scan_index is None:
            return

        detections = []
        for obstacle in self.obstacles:
            data = obstacle.get_bounding_box(self.scan_index, output_dir)
            if data is not None:
                detections.append(data)

        data_dict = {
            "detections": {f"{str(self.scan_index).zfill(6)}.pcd": detections},
            "robot": {
                "linear_velocity": list(self.lin_velocity),
                "angular_velocity": list(self.ang_velocity),
                "translation": list(self.translation),
                "quaternion": list(self.quaternion),
            },
        }

        json_object = json.dumps(data_dict, indent=4)
        # with open(f"{output_dir}/{self.timestamp}.json", "w") as outfile:
        #     outfile.write(json_object)
        with open(f"{output_dir}/{str(self.scan_index).zfill(6)}.json", "w") as outfile:
            outfile.write(json_object)
