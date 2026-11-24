#!/usr/bin/env bash
set -euo pipefail
ros2 launch nav2_bringup bringup_launch.py map:=/path/to/fab_map.yaml params_file:=config/nav2_fab_narrow_params.yaml use_sim_time:=True
