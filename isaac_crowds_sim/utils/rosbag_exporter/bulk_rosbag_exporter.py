import os

from rosbag_exporter import Exporter
from tqdm import tqdm

input_dirs = [
    "/scai_data/data01/daav/isaac_sim_synthetic_data/synthetic_data/training_data/crossing",
    "/scai_data/data01/daav/isaac_sim_synthetic_data/synthetic_data/training_data/one-direction",
    "/scai_data/data01/daav/isaac_sim_synthetic_data/synthetic_data/training_data/random-movement",
    "/scai_data/data01/daav/isaac_sim_synthetic_data/synthetic_data/training_data/two-direction",
]

output_dirs = [
    "/scai_data/data01/daav/isaac_sim_synthetic_data/label_generation/training_data/crossing",
    "/scai_data/data01/daav/isaac_sim_synthetic_data/label_generation/training_data/one-direction",
    "/scai_data/data01/daav/isaac_sim_synthetic_data/label_generation/training_data/random-movement",
    "/scai_data/data01/daav/isaac_sim_synthetic_data/label_generation/training_data/two-direction",
]

if len(input_dirs) != len(output_dirs):
    raise Exception("Make sure length of input list is equal to output list")


for idx, dir in enumerate(input_dirs):
    inputs = os.listdir(dir)

    with tqdm(total=len(inputs)) as t:
        for input in inputs:
            input_path = f"{dir}/{input}"
            output_path = f"{output_dirs[idx]}/{input}"
            print(input_path)

            os.makedirs(output_path, exist_ok=True)

            exporter = Exporter(input=input_path, output=output_path)
            exporter.export()

            t.update()
