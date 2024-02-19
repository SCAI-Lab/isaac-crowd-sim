import omni
from omni.isaac.kit import SimulationApp

# The most basic usage for creating a simulation app
kit = SimulationApp()

import csv
import os
from typing import Dict

import numpy as np
import omni.isaac.core.utils as utils
import omni.usd

for i in range(100):
    kit.update()

omni.kit.app.get_app().print_and_log("Hello World!")

local_root_path = os.path.dirname(os.path.abspath(__file__)) + "/../.."
asset_config_file = local_root_path + "/config/airport_models.csv"
output_config_file = local_root_path + "/config/asset_size.csv"

object_dict: Dict[str, int] = {}
with open(asset_config_file, "r") as file:
    csv_reader = csv.reader(file)

    for line in csv_reader:
        object_name = line[0]

        if object_name in object_dict.keys():
            continue
        else:
            object_dict[object_name] = 0

        asset_name = object_name.replace("_", " ")

        asset_path = (
            f"{local_root_path}/assets/airport/output/models/{asset_name}/{asset_name}.usd"
        )
        prim_path = f"/{object_name}"

        utils.prims.create_prim(
            prim_path,
            usd_path=asset_path,
        )

        min_pnt, max_pnt = omni.usd.get_context().compute_path_world_bounding_box(prim_path)
        min_pnt = np.array(min_pnt)
        max_pnt = np.array(max_pnt)
        size = max_pnt - min_pnt
        size = np.round(size, decimals=2)
        with open(output_config_file, "a") as out:
            size_str = ", ".join(map(str, size.tolist()))
            row = f"{object_name}, {size_str}\n"
            out.write(row)

kit.close()  # Cleanup application
