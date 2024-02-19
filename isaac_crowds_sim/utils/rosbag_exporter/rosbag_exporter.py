import argparse
import ast
import json
import os
import sys
from typing import Dict, List, Optional

import cv2
import numpy as np
import open3d as o3d
from cv_bridge import CvBridge
from rosbags.interfaces import Connection
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))
import csv
import logging
import os

from utils.rosbag_exporter.label import Label
from utils.transformations import euler_from_quaternion, rotation_matrix_from_euler_angle

logging.basicConfig(level=logging.INFO)


class Exporter:
    def __init__(self, input: str, output: str) -> None:
        self.input_file = input
        self.output_file = output
        self.lidar_2d_msg = "/pointcloud_2d_lidar"
        self.lidar_3d_msg = "/pointcloud_3d_lidar"
        self.camera_msg = "/camera_rgb"
        self.camera_sim_msg = "/camera_sim"
        self.camera_label_msg = "/camera_label"
        self.camera_segmented_msg = "/camera_segmented"
        self.wheelchair_pos_msg = "/wheelchair_pos"
        self.possible_sensor = [self.lidar_2d_msg, self.lidar_3d_msg, self.camera_msg]

        local_asset_path = os.path.join(os.path.dirname(__file__), "../..")
        self.obstacle_pos_path = local_asset_path + "/config/airport_models.csv"
        self.obstacle_size_path = local_asset_path + "/config/asset_size.csv"

        self.lidar_subdirs = ["labels", "pointcloud"]
        self.camera_subdirs = ["images", "labels", "segments", "simulator"]

        self.static_obstacles = False

        if self.static_obstacles:
            self._setup_obstacle_size()

    def _setup_obstacle_size(self) -> None:
        self.obstacle_size: Dict[str, np.ndarray] = {}
        with open(self.obstacle_size_path) as csvfile:
            csvreader = csv.reader(csvfile, delimiter=",")
            for row in csvreader:
                self.obstacle_size[row[0]] = np.array(
                    [
                        float(row[1]),
                        float(row[2]),
                        float(row[3]),
                    ]
                )

    def export(self) -> None:
        with Reader(self.input_file) as reader:
            # topic and msgtype information is available on .connections list
            available_sensors = []
            for connection in reader.connections:
                if connection.topic in self.possible_sensor:
                    available_sensors.append(connection.topic)

                    if "lidar" in connection.topic:
                        dirname = "lidar_2d" if "2d" in connection.topic else "lidar_3d"
                        subdirs = self.lidar_subdirs

                    if "camera" in connection.topic:
                        dirname = "camera"
                        subdirs = self.camera_subdirs

                    for subdir in subdirs:
                        dir = f"{self.output_file}/{dirname}/{subdir}"
                        os.makedirs(dir, exist_ok=True)

            if self.lidar_2d_msg in available_sensors:
                self.pointcloud_to_pcd(reader, self.lidar_2d_msg)
                logging.info("2D LiDAR pointclouds pcd files saved")
                self.generate_pointcloud_labels(reader, self.lidar_2d_msg)
                logging.info("2D LiDAR labels generated")

            if self.lidar_3d_msg in available_sensors:
                self.pointcloud_to_pcd(reader, self.lidar_3d_msg)
                logging.info("3D LiDAR pointclouds pcd files saved")
                self.generate_pointcloud_labels(reader, self.lidar_3d_msg)
                logging.info("3D LiDAR labels generated")

            if self.camera_msg in available_sensors:
                self.generate_camera_labels(reader)
                logging.info("Camera data generated. Finishing.")

    def generate_camera_labels(self, reader: Reader) -> None:
        index = 0
        self.label_data: List[Label] = []
        bridge = CvBridge()
        output_file_path = f"{self.output_file}/camera"

        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == self.camera_msg:
                time = self.get_message_timestamp(rawdata, connection)
                label_idx = self.find_label_with_timestamp(time)
                self.label_data[label_idx].scan_index = index
                index += 1

                msg = deserialize_cdr(rawdata, connection.msgtype)
                cv2_img = bridge.imgmsg_to_cv2(msg, "rgb8")
                cv2.imwrite(
                    f"{output_file_path}/images/{self._get_current_index(label_idx)}.jpg",
                    cv2_img,
                )

            elif connection.topic == self.camera_label_msg:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                data = msg.data.split("|")
                time = data[0]
                label_idx = self.find_label_with_timestamp(time)

                data_dict = ast.literal_eval(data[1])
                json_object = json.dumps(data_dict, indent=4)

                with open(
                    f"{output_file_path}/labels/{self._get_current_index(label_idx)}.json",
                    "w",
                ) as outfile:
                    outfile.write(json_object)

            elif connection.topic == self.camera_segmented_msg:
                time = self.get_message_timestamp(rawdata, connection)
                label_idx = self.find_label_with_timestamp(time)

                msg = deserialize_cdr(rawdata, connection.msgtype)
                cv2_img = bridge.imgmsg_to_cv2(msg, "rgb8")
                cv2.imwrite(
                    f"{output_file_path}/segments/{self._get_current_index(label_idx)}.jpg",
                    cv2_img,
                )

            elif connection.topic == self.camera_sim_msg:
                time = self.get_message_timestamp(rawdata, connection)
                label_idx = self.find_label_with_timestamp(time)

                msg = deserialize_cdr(rawdata, connection.msgtype)
                cv2_img = bridge.imgmsg_to_cv2(msg, "rgb8")
                cv2.imwrite(
                    f"{output_file_path}/simulator/{self._get_current_index(label_idx)}.png",
                    cv2_img,
                )

    def _get_current_index(self, label_idx: str) -> int:
        try:
            idx = self.label_data[label_idx].scan_index
        except Exception:
            idx = self._get_current_index(self, label_idx - 1) + 1

        return str(idx).zfill(6)

    def generate_pointcloud_labels(self, reader: Reader, ros_message: str) -> None:
        index = 0
        self.label_data: List[Label] = []

        sensor_directory = "lidar_2d" if ros_message == self.lidar_2d_msg else "lidar_3d"
        output_file_path = f"{self.output_file}/{sensor_directory}/labels"

        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == ros_message:
                time = self.get_message_timestamp(rawdata, connection)
                label_idx = self.find_label_with_timestamp(time)

                remove_label = self.label_data[label_idx].set_label_index(index=index)
                if remove_label:
                    self.label_data[label_idx].generate_label_file(output_dir=output_file_path)
                    removed_label = self.label_data.pop(label_idx)
                    logging.debug(
                        f"Removed pointcloud index: {removed_label.scan_index}. New label_data length: {len(self.label_data)}"
                    )

                index += 1

            if connection.topic == "/obstacle_pose":
                time = self.get_message_timestamp(rawdata, connection)
                label_idx = self.find_label_with_timestamp(time)

                msg = deserialize_cdr(rawdata, connection.msgtype)

                for idx, obs in enumerate(msg.poses):
                    obstacle_name = str(idx).zfill(6)

                    obstacle_id = self.label_data[label_idx].get_obstacle(obstacle_name)

                    self.label_data[label_idx].obstacles[obstacle_id].position = np.array(
                        [obs.position.x, obs.position.y, 0]
                    )
                    self.label_data[label_idx].obstacles[obstacle_id].orientation = obs.position.z
                    self.label_data[label_idx].obstacles[obstacle_id].is_2d = (
                        True if ros_message == self.lidar_2d_msg else False
                    )

                    self.label_data[label_idx].obstacles[obstacle_id].velocity = np.array(
                        [obs.orientation.x, obs.orientation.y, obs.orientation.z]
                    )

                remove_label = self.label_data[label_idx].set_label_obstacles_added()
                if remove_label:
                    self.label_data[label_idx].generate_label_file(output_dir=output_file_path)
                    removed_label = self.label_data.pop(label_idx)
                    logging.debug(
                        f"Removed pointcloud index: {removed_label.scan_index}. New label_data length: {len(self.label_data)}"
                    )

            # if connection.topic == "/odom":
            #     time = self.get_message_timestamp(rawdata=rawdata, connection=connection)
            #     label_idx = self.find_label_with_timestamp(timestamp=time)

            #     msg = deserialize_cdr(rawdata, connection.msgtype)

            if connection.topic == self.wheelchair_pos_msg:
                time = self.get_message_timestamp(rawdata, connection)
                label_idx = self.find_label_with_timestamp(time)

                msg = deserialize_cdr(rawdata, connection.msgtype)

                self.wheelchair_tf = msg.transform

                translation = [
                    self.wheelchair_tf.translation.x,
                    self.wheelchair_tf.translation.y,
                    self.wheelchair_tf.translation.z,
                ]
                quat = [
                    self.wheelchair_tf.rotation.x,
                    self.wheelchair_tf.rotation.y,
                    self.wheelchair_tf.rotation.z,
                    self.wheelchair_tf.rotation.w,
                ]
                self.label_data[label_idx].translation = translation
                self.label_data[label_idx].quaternion = quat
                translation.append(1.0)
                transformation_matrix = np.zeros((4, 4))
                translation_vector = np.array(translation)
                transformation_matrix[:, 3] = translation_vector
                roll, pitch, yaw = euler_from_quaternion(
                    self.wheelchair_tf.rotation.x,
                    self.wheelchair_tf.rotation.y,
                    self.wheelchair_tf.rotation.z,
                    self.wheelchair_tf.rotation.w,
                )
                rotation_matrix = rotation_matrix_from_euler_angle(np.array([roll, pitch, yaw]))
                transformation_matrix[0:3, 0:3] = rotation_matrix

                transformation_matrix_inv = np.linalg.inv(transformation_matrix)

                if self.static_obstacles:
                    object_dict: Dict[str, int] = {}
                    with open(self.obstacle_pos_path) as obs_pos:
                        csvreader = csv.reader(obs_pos, delimiter=",")
                        for line in csvreader:
                            object_name = line[0]
                            pos = np.array(
                                [
                                    float(line[1]) - 300,
                                    float(line[1]) + 300,
                                    float(line[2]),
                                    1.0,
                                ]
                            )

                            rotation = float(line[6])

                            pos_tf = np.matmul(transformation_matrix_inv, pos)
                            rotation_tf = rotation + yaw
                            if object_name in object_dict.keys():
                                object_dict[object_name] += 1
                            else:
                                object_dict[object_name] = 0

                            name = f"{object_name}_{object_dict[object_name]}"

                            obstacle_id = self.label_data[label_idx].get_obstacle(name)

                            length, width, height = self.obstacle_size[object_name]

                            self.label_data[label_idx].obstacles[obstacle_id].position = pos_tf
                            self.label_data[label_idx].obstacles[
                                obstacle_id
                            ].orientation = rotation_tf
                            self.label_data[label_idx].obstacles[obstacle_id].is_2d = (
                                True if ros_message == self.lidar_2d_msg else False
                            )
                            self.label_data[label_idx].obstacles[obstacle_id].velocity = 0.0
                            self.label_data[label_idx].obstacles[obstacle_id].length = length
                            self.label_data[label_idx].obstacles[obstacle_id].width = width
                            self.label_data[label_idx].obstacles[obstacle_id].height = height

                remove_label = self.label_data[label_idx].set_label_position_added()
                if remove_label:
                    self.label_data[label_idx].generate_label_file(output_dir=output_file_path)
                    removed_label = self.label_data.pop(label_idx)
                    logging.debug(
                        f"Removed pointcloud index: {removed_label.scan_index}. New label_data length: {len(self.label_data)}"
                    )

        for label in self.label_data:
            label.generate_label_file(output_file_path)

    def find_label_with_timestamp(self, timestamp: str) -> int:
        for idx, label in enumerate(self.label_data):
            if label.timestamp == timestamp:
                return idx

        label_idx = len(self.label_data)
        label = Label(timestamp)
        self.label_data.append(label)
        return label_idx

    def get_message_timestamp(self, rawdata: bytes, connection: Connection) -> str:
        msg = deserialize_cdr(rawdata, connection.msgtype)
        return f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}"

    def pointcloud_to_pcd(self, reader: Reader, ros_message: str) -> None:
        i = 0
        for connection, timestamp, rawdata in reader.messages():
            lidar_points = []
            if connection.topic == ros_message:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                for pnt in msg.points:
                    data_pnt = np.array([pnt.x, pnt.y, pnt.z])
                    lidar_points.append(data_pnt)

                lidar_points_np = np.array(lidar_points)
                if lidar_points_np.shape[0] == 0:
                    logging.warning(f"PCD file {str(i).zfill(6)}.pcd created without any data")
                    lidar_points_np = np.zeros((1, 3))

                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(lidar_points_np)

                output_file = str(i).zfill(6)
                sensor_directory = "lidar_2d" if ros_message == self.lidar_2d_msg else "lidar_3d"
                output_file_path = (
                    f"{self.output_file}/{sensor_directory}/pointcloud/{output_file}.pcd"
                )
                o3d.io.write_point_cloud(output_file_path, pcd)
                i += 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Select input file and output directory (relative to the current location) to use in converter."
    )
    parser.add_argument("-i", "--input", type=str, help="select the input file to convert")
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="../out",
        help="select the output directory to store the converted data (default=../out)",
    )

    args = parser.parse_args()

    exporter = Exporter(args.input, args.output)
    exporter.export()
