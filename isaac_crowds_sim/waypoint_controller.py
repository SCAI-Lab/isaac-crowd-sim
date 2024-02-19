import csv
import os
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from isaac_crowds_sim.utils.coordinate import Coordinate
from isaac_crowds_sim.utils.transformations import euler_from_quaternion


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__("velocity_publisher")
        local_assets_root_path = os.path.dirname(os.path.abspath(__file__))
        self.velocity_pub = self.create_publisher(Twist, "move_wheelchair", 10)
        self.tf2_sub = self.create_subscription(TFMessage, "/tf", self._tf_cb, 10)
        self.read_csv(
            "/home/scai/dev/ros2_ws/src/isaac-crowds-sim/isaac_crowds_sim/config/coordinates.csv"
        )
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self._timer_cb)
        self.error_margin = 0.5
        self.max_vel = 0.5
        self.max_rot = 0.1

    def read_csv(self, path: str) -> None:
        self.coordinates: List[Coordinate] = []
        with open(path, mode="r") as file:
            # reading the CSV file
            # The file should have the following format coordinates x, coordinates y, rotation, time_to_reach_destiniation
            csvFile = csv.reader(file)
            # displaying the contents of the CSV file
            for lines in csvFile:
                pos_x, pos_y, rot, time_to_reach_goal = lines
                self.coordinates.append(
                    Coordinate(float(pos_x), float(pos_y), float(rot), float(time_to_reach_goal))
                )

            self.curr_pos = self.coordinates[0]

    def _timer_cb(self) -> None:
        msg = Twist()
        if len(self.coordinates) >= 1:
            x_ = -(self.coordinates[0].x - self.curr_pos.x) / self.coordinates[0].T
            y_ = -(self.coordinates[0].y - self.curr_pos.y) / self.coordinates[0].T
            theta = self.coordinates[0].rotation - self.curr_pos.rotation

            msg.linear.x = np.clip(
                x_ * np.cos(theta) + y_ * np.sin(theta), -self.max_vel, self.max_vel
            )
            msg.linear.y = np.clip(
                x_ * np.sin(theta) + y_ * np.cos(theta), -self.max_vel, self.max_vel
            )
            msg.angular.z = np.clip(
                theta / (100 * self.coordinates[0].T), -self.max_rot, self.max_rot
            )

            self.get_logger().info(
                f"vel x: {msg.linear.x}, vel y: {msg.linear.y}, rot: {msg.angular.z}"
            )

        else:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0

        self.velocity_pub.publish(msg)

    def _tf_cb(self, data: TFMessage) -> None:
        translation = data.transforms[0].transform.translation
        pos_x = translation.x
        pos_y = translation.y
        quaternion = data.transforms[0].transform.rotation

        roll, pitch, yaw = euler_from_quaternion(
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        )

        self.curr_pos = Coordinate(pos_x, pos_y, yaw, 0)

        goal_pos = self.coordinates[0]

        if (
            np.abs(goal_pos.x - self.curr_pos.x) <= self.error_margin
            and np.abs(goal_pos.y - self.curr_pos.y) <= self.error_margin
        ):
            self.get_logger().info(
                f"Waypoint reached: {len(self.coordinates)} waypoints left in list"
            )
            self.get_logger().info(
                f"Current position: {self.curr_pos.x}, {self.curr_pos.y}. Desired position: {goal_pos.x}, {goal_pos.y}"
            )
            self.coordinates.pop(0)
        else:
            self.get_logger().info(
                f"Current distance to goal. X = {self.curr_pos.x - goal_pos.x}, Y = {self.curr_pos.y - goal_pos.y}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    velocity_publisher = VelocityPublisher()

    rclpy.spin(velocity_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
