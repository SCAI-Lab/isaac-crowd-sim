#!/bin/sh

source /opt/ros/humble/setup.bash
RNW_IMPLEMENTATION=rmw_fastrtps_cpp
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.local/share/ov/pkg/isaac_sim-2023.1.0-hotfix.1/exts/omni.isaac.ros2_bridge/humble/lib
bash ~/.local/share/ov/pkg/isaac_sim-2023.1.0-hotfix.1/python.sh isaac_crowds_sim/start_sim.py