# Move this file to extscache/omni.anim.people-0.1.11/omni/anim/people/scripts/navigation_manager.py

from __future__ import annotations

import math

import carb
import omni.anim.graph.core as ag
import omni.anim.navigation.core as Navigation_core
from pxr import Gf

from .global_character_position_manager import GlobalCharacterPositionManager
from .utils import Utils


class NavigationManager:
    """
    Manages navigation for a character, such as generating static and dynamic obstacle free paths and publishing current and future positions of characters.
    """

    def __init__(self, character_name, navmesh_enabled, dynamic_avoidance_enabled=True):
        self.navigation_interface = Navigation_core.acquire_interface()
        self.character_manager = GlobalCharacterPositionManager.get_instance()
        self.character_name = character_name
        self.character = ag.get_character(self.character_name)
        self.navmesh_enabled = navmesh_enabled
        self.dynamic_avoidance_enabled = dynamic_avoidance_enabled
        self.collision_list = []
        self.positions_over_time = []
        self.delta_time_list = []
        self.path_points = []
        self.path_targets = []
        self.path_final_target_rot = None

    def destroy(self):
        self.navigation_interface = None
        self.character_manager = None
        self.character_name = None
        self.character = None
        self.navmesh_enabled = None
        self.collision_list = None
        self.positions_over_time = None
        self.delta_time_list = None
        self.path_points = None
        self.path_targets = None
        self.path_final_target_rot = None

    def calculate_rotation_diff(self):
        char_rot_angle = Utils.convert_to_angle(Utils.get_character_rot(self.character))
        target_rot_angle = Utils.convert_to_angle(self.get_path_target_rot())
        return Utils.cal_rotation_difference(char_rot_angle, target_rot_angle)

    def set_path_points(self, path_points):
        self.path_points = path_points

    def set_path_target_rot(self, rotation):
        self.path_final_target_rot = rotation

    def get_path_points(self):
        return self.path_points

    def get_path_target_pos(self):
        if not self.path_targets:
            return None
        return self.path_targets[-1]

    def get_path_target_rot(self):
        return self.path_final_target_rot

    def get_avoid_angle(self, radius_a, radius_b, collision_dist, movement_vector):
        movement_vector_length = Utils.length3(movement_vector)
        if movement_vector_length == 0.0:
            return 0.0
        dist_to_avoid = radius_a + radius_b - Utils.length3(collision_dist)
        return math.degrees(math.atan(dist_to_avoid / movement_vector_length))

    def is_still_moving(self, prim_path):
        return (
            Utils.length3(
                Utils.sub3(
                    self.character_manager.get_character_future_pos(prim_path),
                    self.character_manager.get_character_current_pos(prim_path),
                )
            )
            > 0.1
        )

    def check_proximity_to_point(self, point, proximity_dist):
        char_pos = Utils.get_character_pos(self.character)
        # get the point on the same xz plane as the obstacle
        point_on_character_plane = carb.Float3(point[0], point[1], char_pos[2])
        return Utils.dist3(char_pos, point_on_character_plane) < proximity_dist

    def clean_path_targets(self):
        self.path_targets = []
        self.path_final_target_rot = None

    def destination_reached(self):
        if not self.path_targets:
            return True

    def publish_character_positions(self, delta_time, radius):
        if delta_time == 0:
            return

        char_pos = Utils.get_character_pos(self.character)
        num_frames = 10

        # Store the positions and dts of the num_frames last frames
        if len(self.positions_over_time) < num_frames:
            self.positions_over_time.append(char_pos)
            self.delta_time_list.append(delta_time)
        else:
            self.positions_over_time.pop(0)
            self.positions_over_time.append(char_pos)
            self.delta_time_list.pop(0)
            self.delta_time_list.append(delta_time)

        # the estimated velocity (per sec) of the current obstacle
        self.velocity_vec = Utils.scale3(
            Utils.sub3(self.positions_over_time[-1], self.positions_over_time[0]),
            1 / sum(self.delta_time_list),
        )
        orientation = Utils.convert_to_angle(Utils.get_character_rot(self.character))
        self.character_manager.set_character_orientation(self.character_name, orientation)
        self.character_manager.set_character_current_pos(self.character_name, char_pos)
        self.character_manager.set_character_future_pos(
            self.character_name, Utils.add3(char_pos, Utils.scale3(self.velocity_vec, 1))
        )
        self.character_manager.set_character_radius(self.character_name, radius)

    def update_target_path_progress(self):
        if len(self.path_targets) == 1:
            if self.check_proximity_to_point(
                self.path_targets[0], Utils.CONFIG["MinDistanceToFinalTarget"]
            ):
                self.path_targets.pop(0)
        if len(self.path_targets) > 1:
            if self.check_proximity_to_point(
                self.path_targets[0], Utils.CONFIG["MinDistanceToIntermediateTarget"]
            ):
                self.path_targets.pop(0)

    def generate_path(self, coords, path_target_rot=None):
        self.path_targets = []
        prev_point = coords[0]
        path = []
        if not self.navmesh_enabled:
            path.append(prev_point)

        for point in coords[1:]:
            if self.navmesh_enabled:
                generated_path = self.navigation_interface.query_navmesh_path(prev_point, point)
                if generated_path is None:
                    carb.log_error(
                        "There is no valid path between point position : "
                        + str(prev_point)
                        + " and "
                        + "position : "
                        + str(point)
                    )
                    return
                points = generated_path.get_points()
                path.extend(points)
                prev_point = point
            else:
                path.append(point)
            self.path_targets.append(point)
        self.path_points = path
        if path_target_rot:
            self.path_final_target_rot = path_target_rot

    def generate_goto_path(self, coords):
        if len(coords) < 4 or len(coords) % 3 != 1:
            raise ValueError(
                "Invalid coordinate list for path generation. Coordinate list must be a sequence of x,y,z with the last cooridnate also specifying the ending rotation."
            )

        self.path_targets = []
        path = []
        for i in range(0, len(coords) // 3):
            curr_point = carb.Float3(
                float(coords[i * 3]), float(coords[i * 3 + 1]), float(coords[i * 3 + 2])
            )
            path.append(curr_point)
        path.insert(0, Utils.get_character_pos(self.character))

        target_rot_quatd = None
        if coords[len(coords) - 1] != "_":
            target_rot_quatd = Utils.convert_angle_to_quatd(float(coords[-1]))
        self.generate_path(path, target_rot_quatd)

    def detect_collision(self):
        # This will store all the characters that may collide with the current obstacle
        self.collision_list = []
        # A collision below this distance will be given first priority when avoiding.
        priortiy_collision_distance = 3.5
        for obstacle in self.character_manager.get_all_managed_characters():
            if obstacle != self.character_name:
                dist_between_future_pos = Utils.dist3(
                    self.character_manager.get_character_future_pos(self.character_name),
                    self.character_manager.get_character_future_pos(obstacle),
                )
                dist_between_curr_pos = Utils.dist3(
                    self.character_manager.get_character_current_pos(self.character_name),
                    self.character_manager.get_character_current_pos(obstacle),
                )
                radius_sum = self.character_manager.get_character_radius(
                    self.character_name
                ) + self.character_manager.get_character_radius(obstacle)

                if dist_between_future_pos < radius_sum:
                    self_movement_vector = Utils.sub3(
                        self.character_manager.get_character_future_pos(self.character_name),
                        self.character_manager.get_character_current_pos(self.character_name),
                    )
                    obstacle_movement_vector = Utils.sub3(
                        self.character_manager.get_character_future_pos(obstacle),
                        self.character_manager.get_character_current_pos(obstacle),
                    )
                    predicted_pos_difference = Utils.sub3(
                        self.character_manager.get_character_future_pos(obstacle),
                        self.character_manager.get_character_future_pos(self.character_name),
                    )

                    # Check whether self_movement_vector and predicted_pos_difference are on the same side of the axis, if it is, add the obstacle to the collision list
                    # Check whether self_movement_vector and obstacle_movement_vector are on different side of the axis, if so, add the obstacle to collision list
                    # Check whether if the other obstacle's speed is too slow, if so add it collision list regardless of its moving direction.
                    if (
                        Utils.dot3(self_movement_vector, predicted_pos_difference) > 0
                        or Utils.dot3(self_movement_vector, obstacle_movement_vector) < 0
                        or Utils.length3(obstacle_movement_vector) < radius_sum * 0.5
                    ):
                        # Avoid the closest obstacle first
                        if dist_between_curr_pos < priortiy_collision_distance:
                            priortiy_collision_distance = dist_between_curr_pos
                            self.collision_list.insert(0, obstacle)
                        else:
                            self.collision_list.append(obstacle)
        if len(self.collision_list) > 0:
            return True
        return False

    def update_path(self):
        self.update_target_path_progress()
        if self.destination_reached() or not self.dynamic_avoidance_enabled:
            return

        if self.detect_collision():
            current_pos = Utils.get_character_pos(self.character)

            # This part decides whether the obstacle should turn left or right to avoid
            self_movement_vector = Utils.normalize3(
                Utils.sub3(
                    self.character_manager.get_character_future_pos(self.character_name),
                    self.character_manager.get_character_current_pos(self.character_name),
                )
            )
            self_movement_vector_right = Utils.normalize3(
                Utils.cross3(self_movement_vector, carb.Float3(0, 0, 1))
            )

            if len(self.collision_list) > 0:
                # If your destination is closer than collision, ignore the collision.
                if Utils.length3(Utils.sub3(current_pos, self.path_targets[-1])) < Utils.length3(
                    Utils.sub3(
                        current_pos,
                        self.character_manager.get_character_future_pos(self.collision_list[0]),
                    )
                ):
                    return

                # Check if the other obstacle is on the current obstacle's left side or right side
                predicted_pos_difference = Utils.sub3(
                    self.character_manager.get_character_future_pos(self.collision_list[0]),
                    self.character_manager.get_character_future_pos(self.character_name),
                )
                radius_a = self.character_manager.get_character_radius(self.character_name)
                radius_b = self.character_manager.get_character_radius(self.collision_list[0])

                self_movement_vector = Utils.sub3(
                    self.character_manager.get_character_future_pos(self.character_name),
                    self.character_manager.get_character_current_pos(self.character_name),
                )
                angle = Gf.Clamp(
                    (radius_b / radius_a)
                    * self.get_avoid_angle(
                        radius_a, radius_b, predicted_pos_difference, self_movement_vector
                    ),
                    0,
                    60,
                )
                left_future_point = Utils.add3(
                    current_pos,
                    Utils.rotZ3(
                        Utils.sub3(
                            self.character_manager.get_character_future_pos(self.character_name),
                            current_pos,
                        ),
                        angle,
                    ),
                )
                right_future_point = Utils.add3(
                    current_pos,
                    Utils.rotZ3(
                        Utils.sub3(
                            self.character_manager.get_character_future_pos(self.character_name),
                            current_pos,
                        ),
                        -angle,
                    ),
                )
                direction_of_collision = Utils.dot3(
                    self_movement_vector_right, Utils.normalize3(predicted_pos_difference)
                )

                # If no navmesh, pick the best avoidance point
                if not self.navigation_interface or not self.navmesh_enabled:
                    # if the value is between 0.2 and -0.2 it is ambiguous whether to move right or left. Ignore till collision direction is more deterministic.
                    if direction_of_collision > 0.2:
                        new_position = left_future_point
                    elif direction_of_collision < -0.2:
                        new_position = right_future_point
                    else:
                        return
                else:
                    # If navmesh and the other object will avoid you, pick the best avoidance point if it is on the navmesh, if it is not then skip as the other object will avoid you anyway.
                    if Utils.is_character(self.collision_list[0]) and self.is_still_moving(
                        self.collision_list[0]
                    ):
                        if (
                            direction_of_collision > 0.2
                            and self.navigation_interface.validate_navmesh_point(
                                [left_future_point.x, left_future_point.y, 0]
                            )
                        ):
                            new_position = left_future_point
                        elif (
                            direction_of_collision <= -0.2
                            and self.navigation_interface.validate_navmesh_point(
                                [right_future_point.x, right_future_point.y, 0]
                            )
                        ):
                            new_position = right_future_point
                        else:
                            return
                    # If the other object is not going to avoid you, then pick the best avoidance point as long as it is on the navmesh
                    else:
                        if self.navigation_interface.validate_navmesh_point(
                            [left_future_point.x, left_future_point.y, 0]
                        ) and (
                            direction_of_collision > 0.2
                            or not self.navigation_interface.validate_navmesh_point(
                                [right_future_point.x, right_future_point.y, 0]
                            )
                        ):
                            new_position = left_future_point
                        elif self.navigation_interface.validate_navmesh_point(
                            [right_future_point.x, right_future_point.y, 0]
                        ) and (
                            direction_of_collision < -0.2
                            or not self.navigation_interface.validate_navmesh_point(
                                [left_future_point.x, left_future_point.y, 0]
                            )
                        ):
                            new_position = right_future_point
                        else:
                            return

                new_target_list = self.path_targets
                new_target_list.insert(0, new_position)
                self.generate_path(new_target_list)
                self.path_points.insert(0, current_pos)
