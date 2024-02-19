import os

import numpy as np
import open3d as o3d


def remove_pnts(arr: np.ndarray) -> np.ndarray:
    arr = arr[arr[:, 2] <= 2]
    return arr[np.sqrt(arr[:, 0] ** 2 + arr[:, 1] ** 2) <= 7]


def create_pcd_animation(pcd_path: str) -> None:
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    last_pcd_file = len(os.listdir(pcd_path)) - 2

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])

    geometry = o3d.io.read_point_cloud(f"{pcd_path}/{str(0).zfill(6)}.pcd")
    pnts = remove_pnts(np.asarray(geometry.points))
    geometry.points = o3d.utility.Vector3dVector(pnts)
    vis.add_geometry(geometry)
    # vis.add_geometry(mesh_frame)

    for i in range(last_pcd_file):
        # now modify the points of your geometry
        # you can use whatever method suits you best, this is just an example
        pcd = o3d.io.read_point_cloud(f"{pcd_path}/{str(i).zfill(6)}.pcd")

        # filter pcd points farther away than 7m
        pnts = remove_pnts(np.asarray(pcd.points))

        geometry.points = o3d.utility.Vector3dVector(pnts)
        # print(geometry)
        vis.update_geometry(geometry)
        vis.poll_events()
        vis.update_renderer()
        vis.capture_screen_image(f"{pcd_path}/images/{str(i).zfill(6)}.png")


if __name__ == "__main":
    pcd_path = "/home/ryan/dev/ros2_ws/src/isaac-crowds-sim/isaac_crowds_sim/out/label_test/lidar_3d/pointcloud"
    create_pcd_animation(pcd_path)
