# 窄通道 AMR 可达性与避障验证

Isaac Sim 5.1 + ROS 2 Jazzy + Nav2 环境下的窄通道 AMR 可达性验证工具集：
在仿真中按不同车体尺寸与感知组合（2D LiDAR / +Nvblox 3D）跑批，自动输出碰撞与可达性报表。

完整方案说明见 [`AMR在Isaac Sim中的可达性仿真指南.md`](./AMR在Isaac%20Sim中的可达性仿真指南.md)。

## 目录结构

```
.
├── config/
│   ├── nav2_fab_narrow_params.yaml   # Nav2 参数 (SmacPlannerHybrid + MPPI + Nvblox + Collision Monitor)
│   └── fab_narrow_no_spin.xml        # 无 Spin/BackUp 的脱困行为树（禁用原地旋转）
├── isaac_scripts/
│   └── setup_fab_scene.py            # 场景碰撞体加载 + 参数化包络代理盒 + 碰撞监控
├── ros2_scripts/
│   ├── run_reachability_benchmark.py # 自动发点、监听碰撞、导出 CSV 报表
│   └── publish_swept_area.py         # 把质心轨迹扩成车身扫掠带（看车扫过的体积）
├── analysis/
│   └── analyze_fab_aisles.py         # 占据栅格图通道净宽 / 最窄瓶颈分析
├── scripts/
│   ├── 1_start_sim.sh                # 启动仿真场景
│   ├── 2_start_nav2.sh               # 启动 Nav2（含地图与参数）
│   ├── 3_run_benchmark.sh            # 运行压测
│   ├── 4_run_analysis.sh             # 运行净宽分析
│   └── 5_show_swept_area.sh          # 显示车身扫掠带 / 扫掠体
├── reports/                          # 自动生成的 CSV 报表
└── requirements.txt                  # 分析脚本依赖
```

## 快速开始

```bash
# 0. 依赖（仅分析脚本需要）
pip install -r requirements.txt

# 1. 起仿真（先改 setup_fab_scene.py 里的 USD_PATH 与 PROXY_BOX_SIZE）
bash scripts/1_start_sim.sh

# 2. 起 Nav2（先改脚本里的地图路径）
bash scripts/2_start_nav2.sh

# 3. 跑压测，结果写入 reports/
bash scripts/3_run_benchmark.sh

# 4. 分析最窄通道（先改 analyze_fab_aisles.py 顶部的 YAML_PATH）
bash scripts/4_run_analysis.sh

# 5. 看车身扫掠带（RViz Add -> MarkerArray，topic: /swept_area）
bash scripts/5_show_swept_area.sh
```

## 车身扫掠带

Nav2 发布的 `/plan`、`/trajectory` 都是 `base_link` 原点的单条线，不含宽度。
`publish_swept_area.py` 把每个位姿的矩形包络（0.95 × 0.65 × 1.4 m）投影出来：

- 白色质心线 + 青/橙左右边缘线 + 半透明黄色填充带（俯视面积）
- `--mode 3d` 时再加蓝色半透明立方体序列，形成立体隧道
- 终端同时打印：路径长度、扫掠带面积（位姿凸包并集）、直线距离、绕行比

拐角处扫掠带一旦压到机台轮廓，说明几何上真的过不去，与控制器调参无关。

## 关键前提

- 首轮跑通阶段定位使用**仿真真值 TF**（`map -> odom -> base_link`），不启动 AMCL：
  场景长直走廊多、机台外形相似，AMCL 易漂，混入定位误差会导致失败无法归因。
- 每条用例开始前通过 `/amr/reset_pose` 复位车体并清空代价地图，避免上一条用例卡死污染后续数据。
- 代理盒默认 `PROXY_RENDER_VISIBLE = False`（对 RTX 传感器不可见、物理碰撞仍生效），
  否则盒子会把车上的 LiDAR 与深度相机整体包住，导致感知全盲。
