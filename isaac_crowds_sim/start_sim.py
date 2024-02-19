from omni.isaac.kit import SimulationApp

config = {
    "width": "800",
    "height": "600",
    "headless": False,
}
simulation_app = SimulationApp(config)


import csv
import json
import os
from random import random
from typing import Dict, Optional, Tuple

import carb
import numpy as np
import omni
import omni.graph.core as og
import omni.isaac.core.utils as utils
import omni.isaac.core.utils.prims as prim_utils
import omni.replicator.core as rep
from omni.isaac.core import SimulationContext, World
from omni.isaac.core.materials.physics_material import PhysicsMaterial
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.utils.semantics import add_update_semantics
from omni.isaac.sensor import Camera
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup
from omni.timeline import get_timeline_interface
from pxr import Gf, Sdf
from utils.transformations import (
    euler_from_quaternion,
    isaac_quaternion_from_euler,
    rotation_matrix_from_euler_angle,
)

utils.extensions.enable_extension("omni.isaac.ros2_bridge")
utils.extensions.enable_extension("omni.anim.people")

import omni.anim.navigation.core as nav
import rclpy
import utils.global_character_positions as gcp
from geometry_msgs.msg import Point32, Pose, PoseArray, Transform, TransformStamped, Twist
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import Image as SensorImage
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from utils.character_setup import CharacterSetup


class Simulator(Node):
    def __init__(
        self, character_config_file: str, position_static: bool, position_index: int
    ) -> None:
        super().__init__("airport_environment")

        self.local_root_path = os.path.dirname(os.path.abspath(__file__))
        assets_root_path = utils.nucleus.get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")

        self.character_config_file = character_config_file
        self.character_config_path = (
            f"{self.local_root_path}/config/scenes/{character_config_file}.txt"
        )

        empty_file_path = self.local_root_path + "/assets/empty_world.usd"
        omni.usd.get_context().open_stage(empty_file_path)

        self.my_world = World(stage_units_in_meters=1.0)
        self.timestep = 1.0 / 60.0
        self.sim_context = SimulationContext(
            physics_dt=self.timestep,
            rendering_dt=self.timestep,
            stage_units_in_meters=1.0,
        )
        self.timeline = get_timeline_interface()

        # wheelchair_asset_path = self.local_root_path + "/assets/daav/daav.usd"
        wheelchair_asset_path = self.local_root_path + "/assets/Kaya/kaya.usd"

        self.wheelchair_stage_path = "/World/Daav"

        initial_position, initial_orientation = self.get_initial_position(
            position_static, position_index
        )

        airport_file_path = self.local_root_path + "/assets/airport/airport.usd"
        airport_prim = "/World/Airport"
        usd_prim_airport = utils.stage.add_reference_to_stage(
            usd_path=airport_file_path, prim_path=airport_prim
        )
        add_update_semantics(usd_prim_airport, semantic_label="wall")

        usd_path = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
        utils.stage.add_reference_to_stage(usd_path=usd_path, prim_path="/World/GroundPlane")
        physics_material_path = utils.string.find_unique_string_name(
            initial_name="/World/Physics_Materials/physics_material",
            is_unique_fn=lambda x: not utils.prims.is_prim_path_valid(x),
        )
        physics_material = PhysicsMaterial(
            prim_path=physics_material_path,
            static_friction=0.5,
            dynamic_friction=0.5,
            restitution=0.8,
        )
        plane = GroundPlane(
            prim_path="/World/GroundPlane",
            name="airport_ground_plane",
            z_position=0,
            physics_material=physics_material,
            size=350,
        )
        self.my_world.scene.add(plane)
        utils.prims.set_prim_property("/World/GroundPlane", "xformOp:scale", (7.0, 7.0, 1.0))
        add_update_semantics(plane.prim, semantic_label="floor")

        self.wheelchair = self.my_world.scene.add(
            WheeledRobot(
                prim_path=self.wheelchair_stage_path,
                name="my_wheelchair",
                wheel_dof_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
                create_robot=True,
                usd_path=wheelchair_asset_path,
                position=initial_position,
                orientation=initial_orientation,
            )
        )

        self.wheelchair_setup = HolonomicRobotUsdSetup(
            robot_prim_path=self.wheelchair.prim_path,
            com_prim_path=self.wheelchair_stage_path + "/base_link/control_offset",
        )

        (
            wheel_radius,
            wheel_positions,
            wheel_orientations,
            mecanum_angles,
            wheel_axis,
            up_axis,
        ) = self.wheelchair_setup.get_holonomic_controller_params()

        self.controller = HolonomicController(
            name="holonomic_controller",
            wheel_radius=wheel_radius,
            wheel_positions=wheel_positions,
            wheel_orientations=wheel_orientations,
            mecanum_angles=mecanum_angles,
            wheel_axis=wheel_axis,
            up_axis=up_axis,
        )
        self.wheelchair_velocity = np.array([0, 0, 0])  # Start without any speed

        self.move_wheelchair_sub = self.create_subscription(
            Twist, "move_wheelchair", self._move_wheelchair_cb, 10
        )
        self.wheelchair_tf = self.create_subscription(TFMessage, "tf", self._wheelchair_tf_cb, 10)

        self.current_wheelchair_transform: Optional[Transform] = None

        self.obstacle_pub = self.create_publisher(PoseArray, "obstacle_pose", 10)
        # self.obstacle_vel_pub = self.create_publisher(
        #     TwistStamped, "obstacle_velocity", 10
        # )
        self.lidar_2d_pub = self.create_publisher(PointCloud, "pointcloud_2d_lidar", 10)
        self.lidar_3d_pub = self.create_publisher(PointCloud, "pointcloud_3d_lidar", 10)
        self.camera_sim_pub = self.create_publisher(SensorImage, "camera_sim", 10)
        self.camera_rgb_pub = self.create_publisher(SensorImage, "camera_rgb", 10)
        self.camera_label_pub = self.create_publisher(String, "camera_label", 10)
        self.camera_segmented_pub = self.create_publisher(SensorImage, "camera_segmented", 10)
        self.wheelchair_pos_pub = self.create_publisher(TransformStamped, "wheelchair_pos", 10)
        # timer_period = 0.1
        # self.timer = self.create_timer(timer_period, self._publish_msgs_cb)

        # Load additional sensors and publish their topics
        self.setup_lidar_sensors()
        camera_config_file = self.local_root_path + "/config/sensors/camera_config.json"
        self.setup_camera(camera_config_file, initial_position)

        self.create_omni_graph()

        # self.setup_assets(self.local_root_path + "/config/airport_models.csv")

        self.setup_character_animations(self.character_config_path)

        self.setup_navmeshes()

        self.my_world.reset()

    def setup_assets(self, asset_config_file: str, randomiser: bool = False) -> None:
        object_dict: Dict[str, int] = {}
        with open(asset_config_file, "r") as file:
            csv_reader = csv.reader(file)

            for line in csv_reader:
                object_name = line[0]
                random_movement = np.zeros(3)
                if randomiser:
                    random_movement[0] = (
                        2 * random() - 1
                    )  # Generate shift in x direction between -1 and 1
                    random_movement[1] = (
                        2 * random() - 1
                    )  # Generate shift in y direction between -1 and 1
                    random_movement[2] = (
                        np.pi() / 6 * random() - np.pi() / 12
                    )  # Generate shift in rotation between -15° and 15°
                translation = (
                    float(line[1]) - 300,
                    float(line[2]) + 300,
                    float(line[3]),
                )  # World is originally shifted by (-300,300)
                orientation = isaac_quaternion_from_euler(
                    float(line[4]), float(line[5]), float(line[6])
                )

                if object_name in object_dict.keys():
                    object_dict[object_name] += 1
                else:
                    object_dict[object_name] = 0

                asset_name = object_name.replace("_", " ")

                asset_path = f"{self.local_root_path}/assets/airport/output/models/{asset_name}/{asset_name}.usd"
                prim_path = f"/World/Obstacles/{object_name}_{object_dict[object_name]}"

                prim_utils.create_prim(
                    prim_path,
                    usd_path=asset_path,
                    translation=translation,
                    orientation=orientation,
                    semantic_label=f"{object_name}_{object_dict[object_name]}",
                )

    def get_initial_position(self, position_static: bool, index: int) -> Tuple[np.ndarray]:
        with open(
            f"{self.local_root_path}/config/scenes/{self.character_config_file}.json"
        ) as file:
            json_data = json.load(file)
            if position_static:
                position_data = json_data["wheelchair_pos"]["static"]
                position_index = index

            else:
                position_data = json_data["wheelchair_pos"]["dynamic"][f"path_{index}"]
                position_index = 0

            position = np.array(
                [
                    position_data["pos_x"][position_index],
                    position_data["pos_y"][position_index],
                    0,
                ]
            )
            orientation = utils.rotations.euler_angles_to_quat(
                np.array([0, 0, position_data["theta"][position_index]]), degrees=True
            )
            return position, orientation

    def setup_navmeshes(self) -> None:
        stage = omni.usd.get_context().get_stage()
        omni.kit.commands.execute(
            "CreateNavMeshVolumeCommand",
            parent_prim_path=Sdf.Path("/World/NavMesh"),
            layer=stage.GetRootLayer(),
        )
        # changes nav mesh "agent height" setting preference
        omni.kit.commands.execute(
            "ChangeSetting",
            path="/exts/omni.anim.navigation.core/navMesh/config/agentHeight",
            value=1.0,
        )

        # changes nav mesh "agent radius" setting preference
        omni.kit.commands.execute(
            "ChangeSetting",
            path="/exts/omni.anim.navigation.core/navMesh/config/agentRadius",
            value=0.2,
        )

        # changes nav mesh "voxel ceiling" setting preference
        omni.kit.commands.execute(
            "ChangeSetting",
            path="/exts/omni.anim.navigation.core/navMesh/config/voxelCeiling",
            value=4.6,
        )

        # Bake navmesh
        inav = nav.acquire_interface()
        inav.start_navmesh_baking()

    def setup_lidar_sensors(self) -> None:
        # Setup 2D LiDAR

        _, lidar_2d = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=f"{self.wheelchair_stage_path}/base_link/lidar_2d",
            parent=None,
            config="RPLIDAR_S2E",
            translation=(0, 0, 0.1),  # mount lidar 0.2 m above ground
            orientation=Gf.Quatd(1, 0, 0, 0),  # Gf.Quatd is w,i,j,k
        )

        _, lidar_3d = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=f"{self.wheelchair_stage_path}/base_link/lidar_3d",
            parent=None,
            config="OS1_32ch20hz1024res",
            translation=(0, 0, 0.2),  # mount lidar 0.2 m above ground
            orientation=Gf.Quatd(1, 0, 0, 0),  # Gf.Quatd is w,i,j,k
        )

        self.sensors = [lidar_2d, lidar_3d]

        (
            texture_2d_lidar,
            render_product_path_2d_lidar,
        ) = utils.render_product.create_hydra_texture([1, 1], lidar_2d.GetPath().pathString)
        (
            texture_3d_lidar,
            render_product_path_3d_lidar,
        ) = utils.render_product.create_hydra_texture([1, 1], lidar_3d.GetPath().pathString)

        self.hydra_textures = [texture_2d_lidar, texture_3d_lidar]

        self.annotator_2d = rep.AnnotatorRegistry.get_annotator(
            "RtxSensorCpuIsaacCreateRTXLidarScanBuffer"
        )
        self.annotator_2d.initialize(
            transformPoints=False, outputBeamId=True
        )  # Keep data in lidar coordinate system
        self.annotator_2d.attach([render_product_path_2d_lidar])

        self.annotator_3d = rep.AnnotatorRegistry.get_annotator(
            "RtxSensorCpuIsaacCreateRTXLidarScanBuffer"
        )
        self.annotator_3d.initialize(
            transformPoints=False, outputBeamId=True
        )  # Keep data in lidar coordinate system
        self.annotator_3d.attach([render_product_path_3d_lidar])

        self.render_product_path = [
            render_product_path_2d_lidar,
            render_product_path_3d_lidar,
        ]

        utils.viewports.create_viewport_for_camera(
            viewport_name="lidar_2d_viewport",
            camera_prim_path=f"{self.wheelchair_stage_path}/base_link/lidar_2d",
        )
        utils.viewports.create_viewport_for_camera(
            viewport_name="lidar_3d_viewport",
            camera_prim_path=f"{self.wheelchair_stage_path}/base_link/lidar_3d",
        )

    def setup_camera(self, camera_config_file_path: str, initial_position: np.ndarray) -> None:
        with open(camera_config_file_path) as file:
            config = json.load(file)["profile"]

        initial_position[2] = 0.25

        self.camera = Camera(
            prim_path=f"{self.wheelchair_stage_path}/base_link/camera",
            position=initial_position,
            frequency=config["frequency"],
            resolution=(config["resolution_x"], config["resolution_y"]),
            orientation=[1, 0, 0, 0],  # orientation
        )
        local_orientation = utils.rotations.euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)
        self.camera.set_local_pose(orientation=local_orientation)

        self.camera_rp = rep.create.render_product(self.camera.prim_path, (512, 512))

        self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb_annot.attach([self.camera_rp])
        self.sem_annot = rep.AnnotatorRegistry.get_annotator(
            "semantic_segmentation", init_params={"colorize": True}
        )
        self.sem_annot.attach([self.camera_rp])

        utils.viewports.create_viewport_for_camera(
            viewport_name="camera_viewport",
            camera_prim_path=f"{self.wheelchair_stage_path}/base_link/camera",
        )

        # Setup camera to see overview in simulator

        initial_position[2] = 10

        self.camera_sim = Camera(
            prim_path=f"{self.wheelchair_stage_path}/base_link/camera_sim",
            position=initial_position,
            frequency=config["frequency"],
            resolution=(config["resolution_x"], config["resolution_y"]),
            orientation=[1, 0, 0, 0],  # orientation
        )
        local_orientation = utils.rotations.euler_angles_to_quat(
            np.array([0, 90, 90]), degrees=True
        )
        self.camera_sim.set_local_pose(orientation=local_orientation)

        self.camera_sim_rp = rep.create.render_product(self.camera_sim.prim_path, (512, 512))
        self.rgb_annot_sim = rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb_annot_sim.attach([self.camera_sim_rp])

        utils.viewports.create_viewport_for_camera(
            viewport_name="camera_sim_viewport",
            camera_prim_path=f"{self.wheelchair_stage_path}/base_link/camera_sim",
        )

    def create_omni_graph(self) -> None:
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                        (
                            "ReadSimTime",
                            "omni.isaac.core_nodes.IsaacReadSimulationTime",
                        ),
                        ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                        (
                            "PublishTF",
                            "omni.isaac.ros2_bridge.ROS2PublishTransformTree",
                        ),
                        ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                        ("ComputeOdom", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                        ("PublishOdom", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnImpulseEvent.outputs:execOut", "PublishTF.inputs:execIn"),
                        (
                            "OnImpulseEvent.outputs:execOut",
                            "PublishClock.inputs:execIn",
                        ),
                        ("OnImpulseEvent.outputs:execOut", "ComputeOdom.inputs:execIn"),
                        ("OnImpulseEvent.outputs:execOut", "PublishOdom.inputs:execIn"),
                        ("Context.outputs:context", "PublishTF.inputs:context"),
                        ("Context.outputs:context", "PublishClock.inputs:context"),
                        ("Context.outputs:context", "PublishOdom.inputs:context"),
                        (
                            "ReadSimTime.outputs:simulationTime",
                            "PublishClock.inputs:timeStamp",
                        ),
                        (
                            "ReadSimTime.outputs:simulationTime",
                            "PublishTF.inputs:timeStamp",
                        ),
                        (
                            "ReadSimTime.outputs:simulationTime",
                            "PublishOdom.inputs:timeStamp",
                        ),
                        (
                            "ComputeOdom.outputs:angularVelocity",
                            "PublishOdom.inputs:angularVelocity",
                        ),
                        (
                            "ComputeOdom.outputs:linearVelocity",
                            "PublishOdom.inputs:linearVelocity",
                        ),
                        (
                            "ComputeOdom.outputs:orientation",
                            "PublishOdom.inputs:orientation",
                        ),
                        ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("ComputeOdom.inputs:chassisPrim", self.wheelchair_stage_path),
                    ],
                },
            )
        except Exception as e:
            carb.log_error(e)

        # Setting the /Daav target prim to Publish Transform Tree node
        utils.prims.set_targets(
            prim=utils.stage.get_current_stage().GetPrimAtPath("/ActionGraph/PublishTF"),
            attribute="inputs:targetPrims",
            target_prim_paths=[self.wheelchair_stage_path],
        )

    def setup_character_animations(self, config_file: str) -> None:
        self.char_setup = CharacterSetup()
        self.char_setup.load_characters(config_file)
        self.char_setup.setup_characters()

    def _move_wheelchair_cb(self, data: Twist) -> None:
        if self.my_world.is_playing():
            self.wheelchair_velocity = np.array([data.linear.x, data.linear.y, data.angular.z])

    def _wheelchair_tf_cb(self, data: TFMessage) -> None:
        tf_data = data.transforms
        for tf in tf_data:
            if tf.child_frame_id == "base_link":
                self.current_wheelchair_transform = tf.transform

    def _publish_msgs_cb(self) -> None:
        # Read all data first to have the timing as exact as possible
        timestamp = self.get_clock().now().to_msg()
        character_positions = gcp._character_positions
        character_future_positions = gcp._character_future_positions
        character_orientations = gcp._character_orientation
        lidar_data_2d = self.annotator_2d.get_data(device="cpu", do_array_copy=True)
        lidar_data_3d = self.annotator_3d.get_data(device="cpu", do_array_copy=True)
        if self.current_wheelchair_transform is None or not self.my_world.is_playing():
            # Don't publish data during setup or if simulation is stopped
            return
        self._publish_character_position(
            timestamp,
            character_positions,
            character_future_positions,
            character_orientations,
        )
        self._publish_lidar_data(timestamp, lidar_data_2d, is_2d=True)
        self._publish_lidar_data(timestamp, lidar_data_3d, is_2d=False)
        self._publish_camera_data(
            timestamp,
            self.rgb_annot.get_data(),
            self.sem_annot.get_data(),
            self.rgb_annot_sim.get_data(),
            f"{self.local_root_path}/out/camera",
        )
        self._publish_wheelchair_pos(timestamp)

    def _publish_character_position(
        self,
        timestamp,
        character_positions: Dict[str, np.ndarray],
        character_future_positions: Dict[str, np.ndarray],
        character_orientation: Dict[str, float],
    ) -> None:
        """
        Publishes a PoseArray with the current position and velocity of all characters
        Quaternion data struct is used to publish velocity to make struct more compact
        position.z = yaw rotation
        orientation.x = vx
        orientation.y = vy
        """
        transformation_matrix = np.zeros((4, 4))
        translation = np.array(
            [
                self.current_wheelchair_transform.translation.x,
                self.current_wheelchair_transform.translation.y,
                self.current_wheelchair_transform.translation.z,
                1.0,
            ]
        )
        transformation_matrix[:, 3] = translation
        roll, pitch, yaw = euler_from_quaternion(
            self.current_wheelchair_transform.rotation.x,
            self.current_wheelchair_transform.rotation.y,
            self.current_wheelchair_transform.rotation.z,
            self.current_wheelchair_transform.rotation.w,
        )
        rotation_matrix = rotation_matrix_from_euler_angle(np.array([roll, pitch, yaw]))
        transformation_matrix[0:3, 0:3] = rotation_matrix

        transformation_matrix_inv = np.linalg.inv(transformation_matrix)

        poses = []
        msg = PoseArray()
        msg.header.stamp = timestamp

        for character_name, position in character_positions.items():
            pose = Pose()

            pos = np.array([position[0], position[1], 0, 1])

            pos_tf = np.matmul(transformation_matrix_inv, pos)

            pose.position.x = pos_tf[0]
            pose.position.y = pos_tf[1]
            abs_character_orientation = (
                ((character_orientation[character_name] - 90) * np.pi / 180)
                if character_name in character_orientation.keys()
                else np.pi / 2
            )  # Move xaxis of character to point to the front of the character, set angle in radian
            rotation = abs_character_orientation + yaw
            pose.position.z = rotation

            future_pos = np.array(
                [
                    character_future_positions[character_name][0],
                    character_future_positions[character_name][1],
                    0,
                    1,
                ]
            )

            vel = future_pos - pos

            pose.orientation.x = vel[0]
            pose.orientation.y = vel[1]
            pose.orientation.z = 0.0

            poses.append(pose)

        msg.poses = poses
        self.obstacle_pub.publish(msg)

    def _publish_lidar_data(self, timestamp, data: Dict[str, np.ndarray], is_2d: bool) -> None:
        msg = PointCloud()
        msg.header.stamp = timestamp
        msg.header.frame_id = "pointcloud_2d_lidar" if is_2d else "pointcloud_3d_lidar"
        pos_data = data["data"]
        beamId = data["beamId"]
        pnt_list = []
        for i in range(len(beamId)):
            pos = pos_data[i]
            pnt = Point32()
            pnt.x = float(pos[0])
            pnt.y = float(pos[1])
            pnt.z = float(pos[2])

            pnt_list.append(pnt)

        msg.points = pnt_list

        if is_2d:
            self.lidar_2d_pub.publish(msg)
        else:
            self.lidar_3d_pub.publish(msg)

    def _publish_camera_data(
        self, timestamp, rgb_data, sem_data, rgb_sim_data, file_path: str
    ) -> None:
        rgb_image_data = np.frombuffer(rgb_data, dtype=np.uint8).reshape(*rgb_data.shape, -1)
        rgb_img = Image.fromarray(rgb_image_data, "RGBA")
        rgb_msg = self._create_image_msg(timestamp, rgb_img)
        self.camera_rgb_pub.publish(rgb_msg)

        rgb_sim_image_data = np.frombuffer(rgb_sim_data, dtype=np.uint8).reshape(
            *rgb_sim_data.shape, -1
        )
        rgb_sim_img = Image.fromarray(rgb_sim_image_data, "RGBA")
        rgb_sim_msg = self._create_image_msg(timestamp, rgb_sim_img)
        self.camera_sim_pub.publish(rgb_sim_msg)

        id_to_labels = sem_data["info"]["idToLabels"]
        labels = String()
        str_timestamp = f"{timestamp.sec}.{timestamp.nanosec}"

        labels.data = f"{str_timestamp}|{str(id_to_labels)}"
        self.camera_label_pub.publish(labels)

        sem_image_data = np.frombuffer(sem_data["data"], dtype=np.uint8).reshape(
            *sem_data["data"].shape, -1
        )
        sem_img = Image.fromarray(sem_image_data, "RGBA")
        sem_msg = self._create_image_msg(timestamp, sem_img)
        self.camera_segmented_pub.publish(sem_msg)

    def _publish_wheelchair_pos(self, timestamp) -> None:
        msg = TransformStamped()
        msg.header.stamp = timestamp
        msg.child_frame_id = "base_link"
        msg.transform = self.current_wheelchair_transform

        self.wheelchair_pos_pub.publish(msg)

    def _create_image_msg(self, timestamp, img: Image) -> SensorImage:
        img = img.convert("RGB")  # Convert image to rgb encoding

        msg = SensorImage()
        msg.header.stamp = timestamp
        msg.header.frame_id = "rgb_img"
        msg.height = img.height
        msg.width = img.width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * img.width
        msg.data = np.array(img).tobytes()

        return msg

    def run(self) -> None:
        # self.sim_context.play()
        self.my_world.reset()

        # Bake navmesh
        # inav = nav.acquire_interface()
        # inav.start_navmesh_baking()

        self.camera.initialize()
        while simulation_app.is_running():
            self.my_world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.my_world.is_playing():
                if self.my_world.current_time_step_index == 0:
                    self.my_world.reset()
                    self.controller.reset()
                og.Controller.set(
                    og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"),
                    True,
                )
                self.wheelchair.apply_wheel_actions(
                    self.controller.forward(command=self.wheelchair_velocity)
                )
                if self.my_world.current_time_step_index % int(0.1 / self.timestep) == 0:
                    self._publish_msgs_cb()

        simulation_app.close()


if __name__ == "__main__":
    rclpy.init()
    character_config_file = "random-movement"
    static = True
    position = 0
    sim = Simulator(character_config_file, static, position)
    sim.run()
