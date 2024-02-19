from omni.isaac.kit import SimulationApp

sim_app = SimulationApp({"headless": False})

import os
import pdb

import numpy as np
import omni
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup
from pxr import Sdf

enable_extension("omni.anim.people")

import omni.anim.navigation.core as nav
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup
from utils.character_setup import CharacterSetup


class Simulator:
    def __init__(self) -> None:
        self.local_root_path = os.path.dirname(os.path.abspath(__file__))

        file_path = self.local_root_path + "/assets/empty_world.usd"

        omni.usd.get_context().open_stage(file_path)

        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

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

        # omni.kit.commands.execute('ChangeSetting',
        #     path='/exts/omni.anim.navigation.core/navMesh/config/maxSlope',
        #     value=60
        # )

        # Bake navmesh
        inav = nav.acquire_interface()
        inav.start_navmesh_baking()

        assets_root_path = get_assets_root_path()

        # char_setup = CharacterSetup()
        # command_file = self.local_root_path + "/config/test.txt"
        # print(command_file)
        # char_setup.load_characters(command_file)

        wheelchair_asset_path = self.local_root_path + "/assets/Kaya/kaya.usd"

        self.wheelchair_stage_path = "/World/Kaya"

        # char_setup.setup_characters()
        self.wheelchair = self.world.scene.add(
            WheeledRobot(
                prim_path=self.wheelchair_stage_path,
                name="my_wheelchair",
                wheel_dof_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
                create_robot=True,
                usd_path=wheelchair_asset_path,
                position=[0, 0, 0],
                orientation=[1, 0, 0, 0],
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

    def run(self) -> None:
        self.world.reset()
        while sim_app.is_running():
            self.world.step(render=True)
            if self.world.is_playing():
                if self.world.current_time_step_index == 0:
                    self.world.reset()

            self.wheelchair.apply_wheel_actions(
                self.controller.forward(command=np.array([0.5, 0, 0]))
            )

        sim_app.close()


if __name__ == "__main__":
    sim = Simulator()
    sim.run()
