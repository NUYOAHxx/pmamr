#!/usr/bin/env bash
set -euo pipefail
# 把质心轨迹扩成车身扫掠带（RViz Add -> MarkerArray，topic: /swept_area）
# 先试 /trajectory（MPPI 实际要走的），再试 /plan（规划器意图）对照
python3 ros2_scripts/publish_swept_area.py --source /trajectory --mode all
