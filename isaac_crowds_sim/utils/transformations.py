from typing import Tuple

import numpy as np


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> Tuple[float]:
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = np.zeros(4)
    q[0] = cy * cp * sr - sy * sp * cr
    q[1] = sy * cp * sr + cy * sp * cr
    q[2] = sy * cp * cr - cy * sp * sr
    q[3] = cy * cp * cr + sy * sp * sr

    return q


def isaac_quaternion_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Quaternions in isaac sim have the form quat = [w, x, y, z]
    The default convertions (quat = [x, y, z, w]) has therefore to be adjusted
    """
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    return np.roll(quaternion, 1)


def euler_angle_from_rotation_matrix(rot_mat: np.ndarray) -> np.ndarray:
    sy = np.sqrt(rot_mat[0][0] * rot_mat[0][0] + rot_mat[1][0] * rot_mat[1][0])

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(rot_mat[2][1], rot_mat[2][2])
        y = np.arctan2(-rot_mat[2][0], sy)
        z = np.arctan2(rot_mat[1][0], rot_mat[0][0])
    else:
        x = np.arctan2(-rot_mat[1][2], rot_mat[1][1])
        y = np.arctan2(-rot_mat[2][0], sy)
        z = 0

    return np.array([x, y, z])


def rotation_matrix_from_euler_angle(euler_angle: np.ndarray) -> np.ndarray:
    R = np.array(
        [
            [
                np.cos(euler_angle[1]) * np.cos(euler_angle[2]),
                np.sin(euler_angle[0]) * np.sin(euler_angle[1]) * np.cos(euler_angle[2])
                - np.sin(euler_angle[2]) * np.cos(euler_angle[0]),
                np.sin(euler_angle[1]) * np.cos(euler_angle[0]) * np.cos(euler_angle[2])
                + np.sin(euler_angle[0]) * np.sin(euler_angle[2]),
            ],
            [
                np.sin(euler_angle[2]) * np.cos(euler_angle[1]),
                np.sin(euler_angle[0]) * np.sin(euler_angle[1]) * np.sin(euler_angle[2])
                + np.cos(euler_angle[0]) * np.cos(euler_angle[2]),
                np.sin(euler_angle[1]) * np.sin(euler_angle[2]) * np.cos(euler_angle[0])
                - np.sin(euler_angle[0]) * np.cos(euler_angle[2]),
            ],
            [
                -np.sin(euler_angle[1]),
                np.sin(euler_angle[0]) * np.cos(euler_angle[1]),
                np.cos(euler_angle[0]) * np.cos(euler_angle[1]),
            ],
        ]
    )

    return R


def rotation_matrix_from_quaternion(x: float, y: float, z: float, w: float) -> np.ndarray:
    euler = euler_from_quaternion(x, y, z, w)
    euler_angle = np.array(list(euler))
    return rotation_matrix_from_euler_angle(euler_angle)
