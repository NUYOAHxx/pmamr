# AMR在Isaac Sim中的可达性仿真指南

> 导出时间: 2026-11-24 23:25:53

---

**Q：**

> 提问时间: 2026-11-24 22:07

关于使用amr底座，需要考虑转弯半径，在3d地图下测试能否到达，确认转弯半径、轮子数据，已知地图下移动amr 到目标位置区域，验证是否可行，分析轨迹无法到达的问题原因，是否需要计算最小半径和最大半径，还有其他需要考虑的问题吗。我在isaacsim内有一个3d场景地图的usd文件，需要让amr在这个场景下进行测试，需要考虑如何处理，目前阶段应该做哪些工作。amr没有模型，目前我只有一个场景usd，amr等参数都可以随意设置，目前主要是跑通这个场景并进行验证测试，为后续开发做提前准备，处理可达性边界问题，失败情况会如何。

**A：**

在没有实体 AMR 模型、仅有 3D 场景 USD 的阶段，核心目标不是“精确仿真某台车”，而是**以场景为基准，反推 AMR 的几何与运动学边界（最大允许尺寸、最小转弯半径、通道余量）**，并提前跑通“仿真环境 → 导航规划 → 可达性验证”的闭环。

## 一、 转弯半径：需要算最小半径还是最大半径？

- **最小转弯半径（$R_{\min}$）：必须明确。** 它直接决定车辆能否通过窄弯和死胡同。
  - **差速（Diff-Drive）/ 全向（Omni / Mecanum）底盘**：理论 $R_{\min} = 0$（可原地自转），此时决定通过性的不是转弯半径，而是**外接圆半径（旋转包络）**。
  - **阿克曼（Ackermann）/ 单舵轮底盘**：存在物理极限 $R_{\min} = L / \tan(\delta_{\max})$（$L$ 为轴距，$\delta_{\max}$ 为最大前轮转角），必须严格建模。
- **最大转弯半径（$R_{\max}$）：不需要计算。** 直线行驶时转弯半径趋于无穷大（$\infty$）。
- **真正容易漏算的核心指标——扫掠包络（Swept Volume）：** 矩形底盘转弯时，内侧存在**内轮差**，外侧车角存在**外摆（Overhang）**。测试时不能只看中心轨迹，必须校验整个车身 Footprint 沿轨迹扫过的区域是否碰撞。

## 二、 除转弯半径与轮子数据外，还需考虑的问题

在 3D 场景下验证可达性，以下因素直接影响仿真结果与真实落地的差距：

1. **3D 空间与地形约束**
  - **高度与悬空障碍物**：门框、横梁、货架底层、桌沿。2D 激光雷达可能扫不到高于雷达平面的障碍物，需提前评估 3D 碰撞包络。
  - **地面平整度与坡度**：斜坡角度（影响电机扭矩与防倾覆）、门槛高度、电梯缝隙（决定轮径下限与离地间隙）。
2. **轮子与物理材质**
  - 除了轮径、轮距（Track Width）、轴距（Wheelbase），在 Isaac Sim 中还需关注**地面与轮胎的摩擦系数（Physics Material）**。摩擦力过低会导致原地打滑、里程计漂移；过高会导致差速转向阻力过大。
3. **导航安全余量（Inflation）**
  - 实际运行不可能贴墙走，必须叠加**安全膨胀半径**（通常单侧预留 5~15 cm，视定位精度与车速而定）。
4. **目标点姿态（Goal Tolerance）**
  - 到达目标区域不仅看 $(x, y)$，还要看最终朝向（Yaw）。例如对接充电桩或货架时，受限空间内可能“开得进去，但转不过来”。

## 三、 目前阶段在 Isaac Sim 中的落地步骤

既然参数可随意设置，建议按以下 4 步快速跑通验证：

### 1. 场景 USD 预处理（最容易踩坑）

- **单位与坐标系检查**：确认场景单位为 **米（Meters）**，朝上轴为 **Z-up**，避免导入后车比房子大或比例失调。
- **碰撞体（Collider）配置**：确保地面、墙壁、障碍物都绑定了 `CollisionAPI`。复杂网格建议用 `Convex Decomposition` 或简化包围盒，避免物理引擎卡顿或穿模。
- **地面物理材质**：为地面赋予合理的静/动摩擦系数（例如 `0.8 ~ 1.0`）。

### 2. 用内置模型占位，快速跑通闭环

无需从零建模，直接使用 Isaac Sim 自带的成熟资产：

- **首选内置模型**：`Nova Carter`（差速底盘，带现成的 ROS 2 Action Graph、激光雷达和里程计配置，开箱即用）。
- **自定义包络测试**：如果只想测不同尺寸的通过性，可以在 Carter 的基础上，或用简单的 `Cube（车身） + 2个Cylinder（驱动轮） + 1个Sphere（万向轮）` 搭建极简底盘，通过脚本动态修改车身长宽和轮距。

### 3. 生成地图并接入规划器（ROS 2 + Nav2）

- **导出 2D 占据栅格图**：使用 Isaac Sim 内置的 **Occupancy Map Generator** 插件，设定采样高度区间（如离地 `0.1m ~ 1.8m`），直接将 3D USD 场景切片生成标准 `.pgm` + `.yaml` 地图。
- **接入 Nav2**：
  - 差速底盘：全局规划用 `NavFn` 或 `SmacPlanner2D`，局部规划用 `MPPI` 或 `DWB`。
  - 阿克曼底盘：全局规划必须用 `SmacPlannerHybrid`（支持最小转弯半径约束），局部规划用 `Regulated Pure Pursuit` 或 `TEB`。

### 4. 可达性边界测试（压力测试）

- **固定场景、扫描车体参数**：在地图中选定典型目标点（窄通道、90度急弯、尽头掉头区），逐步增大 AMR 的 Footprint（如从 0.5m×0.5m 递增到 1.2m×0.8m）或增大 $R_{\min}$，记录“首次失败”的临界值。
- **输出物**：直接得到该场景对未来 AMR 选型的硬性约束（例如：“车身宽度不得超过 0.75m，或必须采用可原地旋转的差速/全向底盘”）。

## 四、 轨迹无法到达（失败情况）的分类与排查

在测试中遇到规划失败或执行卡死，通常归为以下 4 类：

| 失败现象 | 常见原因 | 排查与验证手段 |
| --- | --- | --- |
| **全局规划直接报错（无路径）** | 通道宽度小于“车宽 + 2×膨胀半径”，导致自由空间在 Costmap 中断开；或目标点落在障碍物膨胀区内。 | 在 RViz 中打开 Global Costmap，检查目标点与通道处是否出现致命代价（Lethal / Inscribed）封闭。 |
| **有路径，但车卡在半路不动或原地震荡** | 局部规划器陷入局部极小值；或底盘运动学约束（如最小转弯半径过大）导致车头无法对齐路径；或物理引擎中轮子卡在地面网格缝隙。 | 观察局部规划轨迹是否频繁跳变；检查 Isaac Sim 中车轮碰撞体是否陷入地面或被台阶阻挡。 |
| **仿真中能到达，但明显擦碰障碍物** | 2D 占据栅格图切片高度设置不当，漏掉了悬空障碍物（如桌板）或低矮台阶；或 Footprint 配置小于实际车模。 | 开启 Isaac Sim 的物理碰撞可视化（Collision Mesh），对照 2D 地图检查盲区障碍物。 |
| **到达目标点附近但无法判定成功** | 目标容差（`xy_goal_tolerance` / `yaw_goal_tolerance`）设得过严，或受限空间内无法完成最终的姿态调整。 | 放宽容差测试，确认是“到不了”还是“调不好姿态”。 |

---

**Q：**

继续探讨其中的每一个细节，给我一个最好的方式的推荐。在厂内一个比较约束性、比较窄的场景下，还需要考虑避障。你给我一些建议

**A：**

在厂这种**高约束、极窄通道、高机台价值、强防震要求**的场景下，最稳妥的工业级推荐方案是：**“全向/差速紧凑底盘 + 虚拟路网（受限通道）全局约束 + 分级停障为主、小幅绕障为辅 + 2D/3D 融合体素避障”**。

以下将此前框架中的每一个环节拆解到可直接落地的工程细节：

## 一、 几何与运动学细节：用公式锁定“能不能过”

在没有实体车模时，不要靠肉眼估测，直接用三个硬指标在 3D 场景中划定底盘选型红线：

1. **窄道直行宽度公式**

$$
W_{\text{aisle}} \ge W_{\text{robot}} + 2 \times (d_{\text{loc}} + d_{\text{ctrl}} + d_{\text{safety}})
$$
  - **$d_{\text{loc}}$（定位误差）**：厂内特征重复度高（长直走廊、相似机台），纯激光 SLAM 易退化，通常按 $0.02 \sim 0.03\text{ m}$ 预留。
  - **$d_{\text{ctrl}}$（循迹误差）**：控制器跟踪偏移，通常按 $0.03 \sim 0.05\text{ m}$ 预留。
  - **$d_{\text{safety}}$（安全余量）**：洁净室严禁擦碰机台，单侧至少预留 $0.10 \sim 0.15\text{ m}$。
  - **结论**：通道净宽减去 **$0.30 \sim 0.46\text{ m}$**，才是你允许的最大车体宽度（含载具/机械臂外轮廓）。
2. **原地旋转包络（针对差速/全向底盘）**
  - 矩形车身原地自转的最小外接圆半径为 $R_{\text{rot}} = \frac{\sqrt{L^2 + W^2}}{2}$。
  - **工程优化**：窄道内，**将底盘设计为近似正方形或四角切角（八边形）**，能把旋转包络缩小 15%~20%。如果通道宽度小于 $2 R_{\text{rot}} + 0.2\text{ m}$，则该区域**严禁原地掉头**，必须支持双向行驶（前后对称传感器）或采用全向底盘横向平移对接。
3. **专属动力学约束**
  - 搬运晶圆盒（FOUP）对震动敏感，仿真中必须把最大线加速度限制在 **$0.3 \sim 0.5\text{ m/s}^2$**，急停减速度控制在 **$0.8 \sim 1.0\text{ m/s}^2$** 以内。这会直接拉长刹车距离，从而反过来决定你的避障触发距离。

## 二、 狭窄场景避障细节：为什么“停”比“绕”更重要

在开源导航的默认逻辑里，遇到障碍物就会尝试重规划绕行；但在窄通道里，**盲目绕障是撞机台和死锁的最大元凶**。推荐采用**三级避障 + 场景分区**策略：

### 1. 传感器布局（消除 3D 盲区）

- **2D 平面**：车身**对角（左前 + 右后）双 2D 激光雷达**，实现 360° 无死角扫描，且能满足双向行驶需求。
- **3D 空间**：机台 Load Port 伸出平台、推车扶手、悬空线槽都在 2D 雷达平面之上或之下。必须在行驶方向加装 **3D 深度相机（如 RealSense）或广角固态激光雷达**，向下微倾 15°~20°，同时覆盖低矮防撞条与高处突出物。

### 2. 分级安全区与避障动作矩阵

| 障碍物类型 | 通道余量情况 | 推荐策略 | 详细执行逻辑 |
| --- | --- | --- | --- |
| **动态行人 / 移动推车** | 任意宽度 | **三级停障等待** | 远距（如 2.5m）预警并降至 50% 速度；中距（1.2m）平滑刹停并语音/灯光提示；近距（0.5m）触发硬件级安全急停。障碍离开后延迟 2s 恢复。 |
| **静态临时障碍（如违停料车）** | 单侧绕行余量小于 15cm | **禁止绕行，上报调度** | 停障超过阈值时间（如 15s）后，直接向上层调度系统（FMS）上报“通道阻塞”，由调度重新分配全局替代路线或通知人工清理。 |
| **静态临时障碍** | 绕行后单侧余量大于等于 15cm | **受限微调绕障** | 仅允许局部规划器在预设通道边界（Corridor）内小幅偏移绕过，**严禁越过机台禁行红线**。 |

## 三、 规划与控制架构细节：Nav2 最佳组合

在 Isaac Sim 对接 ROS 2 Nav2 时，建议按以下模块组合，彻底避免窄道“画龙”和死锁：

1. **全局层：`Keepout Filter` + `Route Server`（虚拟路网）**
  - **不要用自由空间全局规划**（如默认 NavFn），它天生倾向于贴着 90° 拐角走内切线。
  - 在地图上用 `Keepout Filter` 给所有机台外扩一层硬性禁行区；同时用虚拟路网定义通道中线、单行道和会车区，强制 AMR 沿中线行驶。
2. **代价地图层：`STVL`（Spatio-Temporal Voxel Layer）**
  - 相比传统的 `VoxelLayer`，STVL 对 3D 深度相机点云的动态衰减处理更好，能有效防止行人走过后在窄通道里留下“幽灵障碍物”导致车卡死。
3. **局部控制器选择**
  - **首选 `Regulated Pure Pursuit (RPP)`**：适合极窄通道的“强循迹 + 停障”模式，路径贴合度极高，且支持进弯自动降速，极其可控。
  - **备选 `MPPI Controller`**：如果需要“窄道受限微调绕障”，MPPI 基于真实多边形 Footprint 采样轨迹，比 DWB 更平滑，不易在窄口震荡。
4. **必须修改的隐藏坑：关闭危险的 Recovery 行为**
  - Nav2 默认的脱困行为（Behavior Tree）包含 `Spin`（原地旋转）和 `BackUp`（盲目倒车）。在窄道中，**必须禁用无碰撞检测的 Spin**，否则车体卡住后一原地旋转就会直接扫到机台。

## 四、 Isaac Sim 现阶段实操细节：如何高效跑通与压测

目前你只有场景 USD，没有真实车模，按以下 4 个步骤做**参数化边界验证**最具性价比：

1. **USD 场景物理体检（最先做）**
  - 检查场景单位是否为 **米（Meters）**、Z 轴朝上。
  - 常见**防静电高架地板**，如果 USD 里的地板是由一块块独立网格拼成的，微小缝隙会导致物理引擎里的车轮卡顿或跳动。建议在仿真测试层用一个完整的隐藏平整平面（Invisible Ground Plane）作为物理碰撞地面，保留原地板仅作视觉渲染。
  - 开启 `Physics -> Colliders` 视图，重点核查机台 Load Port、门框、消防栓等突出物是否都有准确碰撞体。
2. **搭建“参数化假车”（无需等机械设计）**
  - 以 Isaac Sim 内置的 `Nova Carter` 为基础，或用极简刚体（1 个可调长宽高的 Box 车身 + 差速/全向轮组 + 对角 2D LiDAR + 前置深度相机）搭建测试车。
  - 将车身长、宽、高和传感器位置暴露为 Python 参数，方便批量改尺寸做对比测试。
3. **构建 4 类必测的极端用例**
  - **用例 A（极窄直道与 90° 拐角）**：验证最大允许车宽与内轮差/外摆是否擦碰。
  - **用例 B（尽头机台精准对接）**：验证车体到达 Load Port 时的最终姿态调整（Yaw 对齐）是否会扫碰对面机台。
  - **用例 C（动态行人横穿与迎面会车）**：在 Isaac Sim 中用移动刚体或 `People` 插件模拟操作员突然从机台后走出，测试刹车距离与停障恢复是否平滑。
  - **用例 D（3D 悬空/低矮障碍物）**：在通道中放置高于 2D 雷达的伸出板或低于雷达的矮箱子，验证 3D 点云能否准确触发停障。
4. **量化输出《场景可达性与选型边界报告》**
  - 每次测试失败，明确归因并记录在表中：是**物理空间不足**（需缩小车体或改全向底盘）、**传感器盲区**（需调整相机俯仰角），还是**规划器参数过严**（如膨胀半径过大导致自由空间断开）。这份数据就是后续机械选型和软件开发的直接输入。

---

**Q：**

如果要实际进行这一项验证，你还缺少哪些需要知道的信息？我没有给你提供这些信息你可以继续问我。另外，我现在的首要目的就是尽快把这个验证做完，以及测试完整有哪些方法，或者说已有的开源项目，可以给我一点建议参考？还有，目前参数化假车可以尽量在 isaacsim里面常有，目前就是在 isaacsim里面去做仿真和验证。如果没有问题的话，先帮我写一个完整的方案，包含每一个步骤。要求尽量清晰完整，注重细节和原理，并说明为什么选择这个方法

**A：**

为了让你以最快速度跑通并拿出有说服力的工程验证结论，下面先列出**还需你补充的关键信息**（用于后续微调参数）、**可直接复用的开源项目与内置资产**，随后直接给出一套**完整的端到端验证实施方案**。

## 一、 目前还缺少的关键信息（你可以后续补充）

这些信息不会阻碍现在启动框架搭建，但决定了最终测试结果的精度：

1. **场景物理与尺度**：USD 场景中最窄通道的净宽大概是多少？机台（Load Port）底部或上方是否有悬空突出物？地面是平整地面还是有接缝的高架地板？
2. **业务与负载约束**：未来 AMR 顶部是否需要搭载机械臂或 FOUP 载具（决定总高度和防震加减速上限）？到达目标机台后，是否要求**严格姿态对齐**（如侧向/背向对接精度需达到 $\pm 2\text{ cm}$、$\pm 2^\circ$）？
3. **交通规则**：窄通道内是否允许双向会车，还是严格单行道？是否需要测试“动态行人横穿”或“临时料车阻挡”？
4. **软硬件环境**：你当前使用的 **Isaac Sim 版本**（如 4.2 / 4.5 / 5.0）、**ROS 2 版本**（Humble / Jazzy）以及本机显卡型号（决定是否能流畅开启 3D 深度相机与 Nvblox）。

## 二、 推荐直接复用的开源项目与 Isaac Sim 内置资产

不要从零写仿真节点或规划算法，以下组合是工业界验证最快的“黄金搭档”：

- **Isaac Sim 内置现成车模（省去 1~2 周建模调参时间）**：
  - **`Nova Carter`**（差速底盘）：NVIDIA 官方标杆资产，内置完整的 ROS 2 Action Graph、激光雷达、深度相机和里程计发布，**开箱即用**。
  - **`O3dyn`**（全向麦轮底盘）：Isaac Sim 内置的工业级重载全向底盘，非常适合用来对比测试“差速 vs 全向”在窄弯和机台对接中的通过性差异。
- **核心开源导航栈**：
  - **`Navigation2 (Nav2)`**：ROS 2 标准导航栈，直接提供 `Keepout Filter`（机台禁行区）、`Regulated Pure Pursuit (RPP)`、`MPPI` 控制器以及 `Simple Commander API`（Python 自动化发点测试）。
  - **`Spatio-Temporal Voxel Layer (STVL)`**：Nav2 官方推荐的 3D 避障插件，比原生 VoxelLayer 更擅长处理动态障碍物衰减，防止行人在窄道留下“幽灵残影”。
  - **`Isaac ROS Nvblox`**（可选进阶）：NVIDIA 官方开源的 GPU 加速 3D 重建与避障包，直接无缝对接 Isaac Sim 深度相机，适合需要极高 3D 避障精度的场景。

## 三、窄通道 AMR 可达性与避障完整验证方案

### 阶段 1：USD 场景清洗与物理对齐（耗时：0.5 天）

**具体步骤：**

1. **单位与坐标系核查**：在 Isaac Sim 中打开 USD，确认 `Stage` 单位为 **Meters（米）**，朝上轴为 **Z-up**。使用测量工具量出 3 处典型窄通道宽度和 90° 拐角尺寸并记录。
2. **物理地面平滑化处理**：如果场景是拼接的高架地板，新建一个覆盖全场的隐藏平整平面（Invisible Ground Plane）作为物理碰撞地面，关闭原地板的 Collision（保留其视觉渲染），并赋予地面 `0.8 ~ 1.0` 的静/动摩擦系数。
3. **3D 碰撞体（Collider）体检**：开启视口 `Show By Type -> Physics -> Colliders`，逐一检查机台 Load Port 伸出平台、门框、消防栓、墙角防撞柱是否都绑定了准确的物理碰撞网格。

- **原理解析与选择理由**：物理引擎（PhysX）对网格接缝极其敏感，微小的地板拼接缝会导致车轮卡顿或里程计瞬间跳变，从而误判为“导航失败”。用隐藏平整平面替代拼接碰撞面，能 100% 排除地面假故障，确保暴露出来的全是真实的几何与规划问题。

### 阶段 2：基于内置车模构建“参数化测试车”（耗时：0.5 ~ 1 天）

**具体步骤：**

1. **加载基准车模**：在场景中导入 `Nova Carter`（测差速底盘）和 `O3dyn`（测全向底盘）。
2. **搭建“包络代理盒（Envelope Proxy）”**：不需要修改真实车模的机械结构。在车体 `base_link` 下挂接一个**半透明的 Box 刚体**，将其尺寸设为你要测试的目标车体包络（例如长 `0.9m` × 宽 `0.7m` × 高 `1.4m`），并为其添加 **Contact Sensor（接触传感器）**。
3. **补齐 360° 与 3D 避障传感器**：
  - 在车身对角（左前、右后）布置双 2D LiDAR，通过节点合并为完整的 360° `/scan`。
  - 在车头安装 1 个 3D 深度相机（向下俯倾 15°~20°），发布 `/depth/points`。

- **原理解析与选择理由**：在无实体车模阶段，验证的核心是**外轮廓扫掠包络是否碰撞**。通过挂载可随时在 Python 中修改长宽高的“带接触传感器的代理盒”，你可以在 1 分钟内把车宽从 `0.6m` 改成 `0.8m` 进行对比压测，且一旦发生哪怕 1 毫米的擦碰，Contact Sensor 就能在后台精准记录碰撞坐标与时间，远比肉眼观察可靠。

### 阶段 3：2D 地图切片与 Nav2 窄通道避障栈配置（耗时：1 天）

**具体步骤：**

1. **生成 2D 占据栅格图**：使用 Isaac Sim 内置的 **Occupancy Map Generator** 插件，将高度切片区间设为 **`0.1m`（高于地面）至 `车体总高 + 0.1m`**，分辨率设为 `0.02m ~ 0.05m`，导出 `.pgm` 和 `.yaml`。
2. **配置机台禁行区（Keepout Filter）**：在导出的地图上，给所有机台外扩 `0.10m ~ 0.15m` 的硬性禁行遮罩，加载到 Nav2 的 `KeepoutFilter` 中。
3. **配置 3D 避障层（STVL）**：在 Local Costmap 中接入深度相机点云，设置 `min_obstacle_height: 0.05`、`max_obstacle_height: 1.8`，让悬空突出物和低矮箱子都能实时投影到代价地图。
4. **选定控制器与安全策略**：
  - **极窄通道模式**：局部规划器选 **`Regulated Pure Pursuit (RPP)`**，开启进弯减速与前方碰撞检测（Collision Monitor），遇到障碍物执行“减速 → 停障等待”。
  - **微调绕障模式**：局部规划器选 **`MPPI Controller`**，配置真实的多边形 Footprint，允许在安全通道内小幅偏移绕过静态小障碍。
  - **关键安全修改**：在 Nav2 行为树（Behavior Tree）中**彻底删除 `Spin`（原地旋转）脱困节点**，防止窄道卡死时盲目自转扫撞机台。

- **原理解析与选择理由**：
  - 为什么切片要覆盖到车顶高度？因为单层 2D 雷达会漏掉悬空的机台平台，全高度切片 + STVL 3D 点云才能保证“2D 规划不撞墙，3D 避障不撞头”。
  - 为什么用 Keepout Filter？开源全局规划器默认走最短路径，一定会紧贴 90° 墙角内切，Keepout Filter 相当于给昂贵机台加上了“电子围栏”。

### 阶段 4：分级场景压力测试与自动化执行（耗时：1 ~ 2 天）

使用 ROS 2 的 **`nav2_simple_commander` (`BasicNavigator`)** 编写 Python 脚本，自动循环下发目标点，并同步监听 Isaac Sim 的 Contact Sensor 数据。按以下 4 级用例依次压测：

| 测试层级 | 测试内容与设置 | 核心验证指标（通过标准） |
| --- | --- | --- |
| **L1：静态几何可达性** | 无动态障碍，遍历所有机台点位、窄直道、90° 急弯。逐步增大代理盒尺寸（如每次加宽 `5cm`）。 | 全程 Contact Sensor 碰撞次数为 0；找出首次规划失败或擦碰的**最大允许车体尺寸**。 |
| **L2：受限对接与姿态** | 在尽头窄道要求 AMR 以精确 Yaw 角对接 Load Port，对比差速（Carter）与全向（O3dyn）。 | 最终位姿误差在容差内；验证原地旋转包络是否扫碰对面机台，明确是否必须选全向底盘。 |
| **L3：3D 盲区障碍物** | 在通道中放置高于雷达的悬空板（模拟机台伸出部）和低于雷达的矮箱子。 | 深度相机触发 STVL 并在安全距离（如大于 `0.5m`）前平滑刹停，无碰撞。 |
| **L4：动态避障** | 在 Isaac Sim 中用移动刚体或 `People` 插件模拟操作员迎面走来、突然横穿，以及违停料车。 | 满足加速度限制（防震）前提下平滑停障；行人离开后能自动恢复；死路阻塞能正确触发超时上报。 |

- **原理解析与选择理由**：手动在 RViz 里点目标点既耗时又无法复现。用 `BasicNavigator` + `Contact Sensor` 实现自动化跑批，可以在一晚上的时间里测完 5 种车体尺寸 × 2 种底盘 × 20 条路线，直接生成客观的统计报表。

### 阶段 5：失败归因与最终交付物输出

测试结束后，将所有失败案例按以下 4 类归因，汇总输出一份 **《场景 AMR 可达性与选型边界验证报告》**：

1. **物理几何受限（硬伤）**：通道净宽小于“车宽 + 2×安全余量”，或旋转包络大于通道宽 → **结论**：必须缩小车体尺寸，或改用全向底盘/双向行驶。
2. **感知盲区导致擦碰**：2D 地图能过但 3D 代理盒触发碰撞 → **结论**：明确后续硬件选型必须增加前置 3D 相机并固定安装倾角。
3. **规划/控制死锁**：物理空间够，但车在拐角震荡或停滞 → **结论**：需在局部区域改用虚拟路网固定路径，或放宽特定点位的姿态容差。
4. **动力学/刹车距离不足**：动态行人突然出现时刹不住 → **结论**：在盲区拐角处必须通过地图限速区（Speed Filter）强制降速至 `0.3 ~ 0.5 m/s`。

---

**Q：**

> 提问时间: 2026-11-24 23:14

我的 Isaac Sim 版本是 5.1，ROS 2 版本Jazzy，有Nvblox 和 gpu 没问题。USD 的这个文件目前来说应该是没有问题，但是还没有进行碰撞体的加载我不知道碰撞体要怎么加。其他的我现在还不太清晰甚至我连车体尺寸有多大都不知道，最窄的长度有多长我也不知道。是否有激光或者 3D 视觉，我也不知道这些都是需要去经过这次验证才知道的是否需要用激光 或者3D 视觉？有些区域需要比较窄，所以需要进行倒车，尝试一步一步挪，这种情况会发生吗？另外，这个测试的 AMR 小车的长度和宽度应该不一致吧？正常的 AMR 小车都不是正方体你刚才的这些阶段之中，还少了很多路径规划的问题吧？那我应该如何去规划它的路径呢？先把这些看清楚，然后再需要你帮我写一个完整的项目方案和项目落地的测试程序

**A：**

你的硬件与软件栈（**Isaac Sim 5.1 + ROS 2 Jazzy + GPU + Nvblox**）非常理想，完全具备跑通高保真 3D 避障与窄道规划的条件。在写完整方案和代码之前，先把你提出的 **4 个核心工程问题** 彻底理清。

## 一、 USD 场景的碰撞体（Collider）怎么加？

在 Isaac Sim 中，仅仅导入了 3D 模型（Mesh）是不带物理碰撞属性的，AMR 会直接“穿墙而过”。加载碰撞体不需要一个个手动点，有**批量操作**和**脚本化**两种方式：

1. **核心原则：静态场景 vs 动态车体**
  - **静态环境（墙壁、机台、货架）**：统一使用 **Triangle Mesh（三角网格）** 碰撞近似。它能 100% 贴合原始 3D 模型的凹凸形状（比如机台伸出的平台），且因为物体不动，PhysX 计算开销极低。
  - **动态物体（AMR 车身、轮子）**：使用 **Convex Hull（凸包）** 或基础几何体（Box/Cylinder），保证物理引擎解算稳定、不卡顿。
  - **地板特殊处理**：不要直接给拼接的机房地板加网格碰撞（容易卡轮子），而是新建一个覆盖全场的 **Ground Plane（平整碰撞面）**。
2. **UI 批量添加法（最快，1 分钟搞定）**
  - 在 Stage 面板中选中整个静态场景的父节点（例如 `/World/Scene`）。
  - 右键点击 → `Add` → `Physics` → **`Colliders Preset`**（会自动递归给底下所有 Mesh 加上静态碰撞体）。
  - **验证方法**：点击视口上方眼睛图标 → `Show By Type` → `Physics` → `Colliders` → `All`，看到所有机台和墙壁都被绿色线框包裹即成功。
3. **Python 脚本批量添加法**
  - 后续在测试程序中，我们可以写一段 10 行的 USD 遍历脚本，自动跳过地板和装饰灯，精准给所有静态障碍物绑定 `UsdPhysics.CollisionAPI`，保证每次启动仿真环境一致。

## 二、 矩形车身（长 ≠ 宽）与“窄道倒车、一步步挪”会发生吗？

**不仅会发生，而且是窄道验证中最核心的痛点。**

1. **为什么矩形车身不能按“正方形/圆形”算？**
  - 真实 AMR 通常是长方形（例如长 `0.9m` × 宽 `0.65m`）。
  - 如果导航算法把车近似成一个**外接圆**（直径 $\sqrt{0.9^2+0.65^2} \approx 1.11\text{ m}$），那么一条 `0.95m` 宽的走廊就会被误判为“绝对进不去”；但实际上车体正着开过去绰绰有余。
  - **结论**：在路径规划和避障中，**严禁使用 `robot_radius`（圆形半径）**，必须使用精确的 **多边形轮廓（Polygon Footprint）**。
2. **窄道倒车与“一步步挪（揉库）”的两种真实工况**
  - **工况 A：死胡同机台对接（直进直退）**。走廊宽度大于车宽，但小于车身对角线长度，车开进去后**无法原地掉头**。此时必须支持**双向行驶（倒车退出）**，这就要求车尾也必须有避障传感器（如对角双激光雷达）。
  - **工况 B：极窄 90° 拐角（K-turn / 揉库）**。车头转不过去时，需要“前进一点 → 打方向 → 倒车一点 → 再前进”。这要求全局规划器和局部控制器都必须**原生支持倒车轨迹生成**。

## 三、 路径规划到底该怎么做？（核心补全）

对于**矩形车身 + 窄道 + 可能需要倒车**的场景，传统的 2D 网格规划器（如 `NavFn`、`SmacPlanner2D`）**绝对不能用**，因为它们只考虑 $(x, y)$ 坐标，不考虑车身朝向 $\theta$，在窄道里要么规划失败，要么规划出车体转不过去的直角路径。

正确的 Nav2 规划架构分为三层：

| 规划层级 | 推荐算法 / 插件 | 为什么选它（核心原理） |
| --- | --- | --- |
| **全局规划器（Global Planner）** | **`SmacPlannerHybrid`**（Hybrid-A*）或 **`SmacPlannerLattice`** | 在 **$(x, y, \theta)$ 三维状态空间**里搜索路径。它在每一步都会用**真实的矩形 Footprint** 检测碰撞，并且原生支持设置 `allow_unknown` 和 **倒车惩罚（`reverse_penalty`）**——当正着转不过去时，它会自动规划出“倒车揉库”或“直进直退”的合法路径。 |
| **局部控制器（Local Controller）** | **`MPPI Controller`**（首选） | 基于采样的模型预测控制。它能在毫秒级内向前推演数千条轨迹，同时考虑矩形 Footprint、动态障碍物和倒车速度（`vx_min < 0`），在窄通道内避障和微调极其丝滑，远胜传统的 DWB。 |
| **安全兜底层（Safety Layer）** | **`Collision Monitor`** | 独立于规划器的底层硬安全防线。直接在车身四周划定“减速区”和“急停区”，哪怕规划器算错，传感器扫到障碍物也会在底层强制刹停，防止撞毁机台。 |

## 四、 怎么用这次仿真反推“车体尺寸”和“选激光还是 3D 视觉”？

既然目前尺寸、通道宽度、传感器选型全都是未知数，这次验证的正确思路是**控制变量法（A/B 对比压测）**：

1. **反推最大允许车体尺寸（几何边界测试）**
  - 先用 Python 脚本自动测量 USD 场景中关键通道的净宽。
  - 在 Isaac Sim 中，用参数化矩形代理盒设定 3 组典型尺寸（例如：紧凑型 `0.7m × 0.55m`、标准型 `0.9m × 0.65m`、重载型 `1.1m × 0.8m`），分别跑一遍所有目标点，直接得出“最大能塞进多大的车”。
2. **反推是否需要 3D 视觉（Nvblox A/B 对比测试）**
  - **实验组 A（纯 2D 激光雷达）**：只开 2D LiDAR 接入 Nav2，同时在 3D 矩形车身（含顶部载具高度）上挂载 **Contact Sensor（碰撞传感器）**。如果 2D 导航认为“畅通无阻”，但车身顶部或底部触发了碰撞报警 → **铁证：场景存在 2D 盲区（如机台伸出板），必须上 3D 视觉**。
  - **实验组 B（2D 激光 + 3D 深度相机 + Nvblox）**：利用你现有的 GPU 运行 **Isaac ROS Nvblox**，将 3D 深度点云实时重建为 3D ESDF 地图并投影为 2D 代价切片给 Nav2。如果同一条路线从“碰撞”变为“安全停障/绕行”，就完整闭环了 3D 视觉的选型依据。

以上 4 个核心问题（碰撞体加载、矩形车身与倒车、SE(2) 路径规划、传感器与尺寸反推逻辑）全部理清后，整个验证的架构就已经非常清晰了。

逻辑清楚了，现在直接为你输出完整的项目实施方案与可落地的测试程序？

---

**Q：**

请帮我写一个完整的项目实施方案，并给出可落地的测试程序框架（包含 Isaac Sim 5.1 批量加碰撞体与挂载参数化矩形车身的 Python 脚本、ROS 2 Jazzy Nav2 SmacPlannerHybrid + MPPI + Nvblox 核心配置，以及自动化发点与碰撞统计脚本）。

**A：**

以下是一套面向 **Isaac Sim 5.1 + ROS 2 Jazzy + Nvblox** 的完整项目实施方案与可落地测试程序框架。整套方案采用**模块化目录结构**，支持一键切换车体尺寸与传感器组合（2D 激光 vs 2D+3D Nvblox），并自动输出可达性与碰撞统计报告。

## 一、 项目整体架构与目录规划

建议在本地建立如下工作空间结构，将仿真脚本、导航配置与自动化测试解耦：

```
fab_amr_validation/
├── config/
│   ├── nav2_fab_narrow_params.yaml   # Nav2 核心配置 (Hybrid A* + MPPI + Nvblox + Collision Monitor)
│   └── test_waypoints.yaml           # 场景关键测试点位（含目标朝向 Yaw）
├── isaac_scripts/
│   └── setup_fab_scene.py            # Isaac Sim 5.1 场景碰撞体加载 + 参数化矩形车身 + 碰撞监控
├── ros2_scripts/
│   └── run_reachability_benchmark.py # 自动化发点、监听碰撞、导出 CSV 报告
└── reports/                          # 自动生成的测试结果报表
```

### 核心测试矩阵设计（控制变量法）

为了同时回答“车能做多大”**和**“要不要上 3D 视觉”两个问题，按以下矩阵跑批：

| 实验编号 | 矩形车身尺寸 ($L \times W \times H$) | 避障感知模式 | 核心验证目标 |
| --- | --- | --- | --- |
| **Exp_01** | 紧凑型 `0.75m × 0.55m × 1.2m` | 纯 2D LiDAR | 摸底场景最窄通道的基础通过性与倒车需求 |
| **Exp_02** | 标准型 `0.95m × 0.65m × 1.4m` | 纯 2D LiDAR | 暴露 2D 雷达盲区（机台悬空突出物/矮障碍）导致的碰撞 |
| **Exp_03** | 标准型 `0.95m × 0.65m × 1.4m` | 2D LiDAR + **Nvblox 3D** | 验证 3D 视觉能否消除 Exp_02 中的盲区碰撞 |
| **Exp_04** | 重载型 `1.15m × 0.80m × 1.6m` | 2D LiDAR + **Nvblox 3D** | 探测该场景的绝对几何尺寸上限 |

## 二、 核心程序 1：Isaac Sim 5.1 场景碰撞体与参数化车身脚本

**文件：`isaac_scripts/setup_fab_scene.py`**

**功能**：

1. 自动遍历你的 USD 场景，跳过地板，为所有机台/墙壁批量绑定精确的 `Triangle Mesh` 静态碰撞体，并铺设平整物理地面防止卡轮。
2. 加载内置 `Nova Carter` 作为驱动底座，在其上挂载一个**可动态修改长宽高的矩形包络代理盒**与 **Contact Sensor（接触传感器）**。
3. 实时监测物理碰撞，一旦代理盒擦碰机台，立即通过 ROS 2 话题 `/amr/collision_event` 发出报警（上升沿触发 + 0.5s 防抖，一次擦碰只报一次）。
4. 订阅 `/amr/reset_pose`，支持把车体瞬移回安全起点并清零速度，供自动化压测在用例之间复位。

```
import sys
import numpy as np
from isaacsim import SimulationApp

# 1. 启动 Isaac Sim 5.1 (调试时 headless=False，批量压测时可改为 True)
simulation_app = SimulationApp({"headless": False})

import carb
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.prims import Articulation
from isaacsim.sensors.physics import ContactSensor

# 启用 ROS 2 Bridge 扩展
enable_extension("isaacsim.ros2.bridge")
import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

# ================= 参数配置区（压测时只需改这里） =================
USD_PATH = "/path/to/your/fab_scene.usd"   # 替换为你的 USD 路径
SCENE_ROOT_PRIM = "/World/Scene"           # USD 挂载节点
FLOOR_KEYWORDS = ["floor", "ground", "tile"]   # 地板名称关键词（跳过网格碰撞）

# 参数化矩形车身包络尺寸（米）：[长 L, 宽 W, 高 H]
PROXY_BOX_SIZE = [0.95, 0.65, 1.40]
PROXY_Z_OFFSET = PROXY_BOX_SIZE[2] / 2.0 + 0.05  # 离地间隙 5cm
PROXY_RENDER_VISIBLE = False                   # False：代理盒不参与渲染，不遮挡 LiDAR/相机射线
RESET_POSE_Z = 0.05                            # 外部复位时车体的离地高度
# =================================================================

def batch_add_static_colliders(stage, root_path):
    """递归为静态机台/墙壁添加 Triangle Mesh 碰撞体，跳过拼接地板"""
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        carb.log_error(f"无效的场景路径: {root_path}")
        return

    count = 0
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdGeom.Mesh):
            name_lower = prim.GetName().lower()
            if any(kw in name_lower for kw in FLOOR_KEYWORDS):
                continue  # 跳过高架地板，避免接缝卡轮

            # 绑定物理碰撞 API 并设为精准三角网格近似
            UsdPhysics.CollisionAPI.Apply(prim)
            mesh_col_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh_col_api.CreateApproximationAttr().Set("none")  # "none" = Triangle Mesh
            count += 1
    print(f"[INFO] 已为 {count} 个静态机台/墙壁网格批量加载碰撞体。")

def attach_parameterized_proxy_box(stage, robot_base_path, size_lwh, z_offset, visible=False):
    """在底盘上挂载可调尺寸的矩形包络代理盒，用于检测 3D 扫掠碰撞

    注意：盒子放大到整车尺寸后会把车身上的 2D LiDAR / 深度相机整体包住，
    RTX 射线打在盒内壁上会导致雷达与相机全盲（有墙却直冲）。因此默认 MakeInvisible()：
    传感器不对不可见几何做射线求交，而物理碰撞仍然正常触发。
    需要肉眼确认包络形状时，把 PROXY_RENDER_VISIBLE 设为 True 即可。
    """
    proxy_path = f"{robot_base_path}/Envelope_Proxy_Box"
    cube = UsdGeom.Cube.Define(stage, proxy_path)
    cube.CreateSizeAttr(1.0)
    if not visible:
        UsdGeom.Imageable(cube).MakeInvisible()

    # 设置长宽高缩放与离地高度
    xform = UsdGeom.Xformable(cube)
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z_offset))
    xform.AddScaleOp().Set(Gf.Vec3d(size_lwh[0], size_lwh[1], size_lwh[2]))

    # 设为半透明红色便于可视化观察
    cube.CreateDisplayColorAttr([(1.0, 0.1, 0.1)])
    cube.CreateDisplayOpacityAttr([0.3])

    # 添加碰撞体（与底盘刚体绑定，触发接触报告）
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    PhysxSchema.PhysxContactReportAPI.Apply(cube.GetPrim())
    return proxy_path

def main():
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    # 1. 添加平整物理地面（作为唯一物理支撑面）
    world.scene.add_default_ground_plane(static_friction=0.9, dynamic_friction=0.9)

    # 2. 导入 USD 场景并批量加碰撞体
    add_reference_to_stage(usd_path=USD_PATH, prim_path=SCENE_ROOT_PRIM)
    batch_add_static_colliders(stage, SCENE_ROOT_PRIM)

    # 3. 导入内置 Nova Carter（自带 ROS 2 里程计、2D LiDAR、3D 深度相机）
    # 用 get_assets_root_path() 动态拼接路径，避免写死 localhost 后换机器/换版本找不到资产
    assets_root = get_assets_root_path()
    if assets_root is None:
        carb.log_error("未找到 Nucleus 资产根目录，请检查 Nucleus 连接或本地资产包")
        simulation_app.close()
        return
    carter_usd = f"{assets_root}/Isaac/Robots/NovaCarter/nova_carter.usd"
    robot_prim_path = "/World/Nova_Carter"
    add_reference_to_stage(usd_path=carter_usd, prim_path=robot_prim_path)
    # 包一层 Articulation，便于外部复位时直接 set_world_pose / 清零速度
    carter = world.scene.add(Articulation(prim_paths_expr=robot_prim_path, name="carter"))

    # 4. 挂载参数化矩形包络盒 + 接触传感器
    proxy_path = attach_parameterized_proxy_box(
        stage, f"{robot_prim_path}/chassis_link", PROXY_BOX_SIZE, PROXY_Z_OFFSET,
        visible=PROXY_RENDER_VISIBLE
    )
    contact_sensor = ContactSensor(
        prim_path=f"{proxy_path}/contact_sensor",
        name="amr_envelope_contact",
        min_threshold=0.1,   # 接触力 > 0.1N 即判定为擦碰
        max_threshold=10000.0,
        radius=-1.0
    )

    world.reset()

    # 初始化 ROS 2 节点用于广播碰撞报警
    rclpy.init()
    ros_node = rclpy.create_node("isaac_collision_monitor")
    col_pub = ros_node.create_publisher(String, "/amr/collision_event", 10)

    def on_reset_pose(msg):
        """外部复位：车体瞬移回安全起点并清零速度，避免上一条用例的卡死位姿污染下一条"""
        pos = np.array([msg.pose.position.x, msg.pose.position.y, RESET_POSE_Z], dtype=np.float64)
        quat = np.array([msg.pose.orientation.w, msg.pose.orientation.x,
                         msg.pose.orientation.y, msg.pose.orientation.z], dtype=np.float64)
        carter.set_world_pose(position=pos, orientation=quat)
        carter.set_linear_velocity(np.zeros(3))
        carter.set_angular_velocity(np.zeros(3))
        print(f"[INFO] 车体已复位至 ({pos[0]:.2f}, {pos[1]:.2f})")

    ros_node.create_subscription(PoseStamped, "/amr/reset_pose", on_reset_pose, 10)

    import time

    was_in_contact = False
    last_publish_time = 0.0
    COLLISION_COOLDOWN_SEC = 0.5  # 防抖冷却，避免接触面高频抖动重复计数

    print("[INFO] 仿真已启动，开始监听 3D 包络碰撞...")
    while simulation_app.is_running():
        world.step(render=True)
        rclpy.spin_once(ros_node, timeout_sec=0.0)   # 必须 spin，否则 /amr/reset_pose 收不到
        if not world.is_playing():
            was_in_contact = False
            continue

        reading = contact_sensor.get_current_frame()
        in_contact = bool(reading and reading.get("in_contact", False))

        # 仅在上升沿（未接触 -> 接触）且超过冷却时间时上报一次
        if in_contact and not was_in_contact:
            now = time.monotonic()
            if now - last_publish_time >= COLLISION_COOLDOWN_SEC:
                raw_force = reading.get("force", 0.0)
                # force 可能是标量 / 三维向量 / NumPy 数组，统一取模后转 float
                force_mag = float(np.linalg.norm(np.asarray(raw_force, dtype=float)))
                msg = String()
                msg.data = f"COLLISION_DETECTED|force={force_mag:.2f}"
                col_pub.publish(msg)
                last_publish_time = now

        was_in_contact = in_contact

    ros_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()

if __name__ == "__main__":
    main()
```

## 三、 核心程序 2：ROS 2 Jazzy Nav2 窄通道 + 倒车 + Nvblox 配置

**文件：`config/nav2_fab_narrow_params.yaml`**

**关键设计说明**：

- **矩形轮廓**：全篇使用 `footprint`（对应 `0.95m × 0.65m`），彻底弃用圆形 `robot_radius`。
- **窄道倒车与揉库**：`SmacPlannerHybrid` 开启 `motion_model_for_search: "REEDS_SHEPP"`，允许规划前进+倒车组合路径。
- **3D 避障融合**：Local Costmap 引入 `NvbloxCostmapLayer`，实时接收 GPU 重建的 3D 障碍物切片。
- **防撞机台底线**：启用 `CollisionMonitor` 划定减速/急停区，且在行为树中严禁无碰撞检测的原地旋转。

```
planner_server:
  ros__parameters:
    expected_planner_frequency: 5.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerHybrid"
      downsample_costmap: false
      allow_unknown: false
      max_iterations: 1000000
      max_planning_time: 5.0
      # 核心：REEDS_SHEPP 支持前进与倒车（实现窄道直进直退与 K-turn 揉库）
      motion_model_for_search: "REEDS_SHEPP"
      angle_quantization_bins: 72           # 每 5 度一个朝向采样，窄道更精准
      minimum_turning_radius: 0.25          # 差速底盘可设小值（如 0.2~0.3m）避免原地死转
      reverse_penalty: 1.5                  # 略微惩罚倒车：能正走优先正走，转不过去则果断倒车
      change_penalty: 0.2                   # 换向惩罚，防止频繁前后抽搐
      non_straight_penalty: 1.2             # 鼓励在窄道走直线
      cost_penalty: 2.5                     # 强推离机台边缘，尽量走通道中线
      footprint_padding: 0.02

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      wz_std: 0.4
      vx_max: 0.8                           # 窄道限速
      vx_min: -0.35                         # 允许倒车退出窄道
      wz_max: 0.8
      # 防震动力学约束（保护晶圆/载具）
      ax_max: 0.4
      ax_min: -0.6
      az_max: 0.8
      motion_model: "DiffDrive"             # 若换 O3dyn 全向底盘，改为 "Omni"
      critics: ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PreferForwardCritic"]
      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
        critical_cost: 300.0
        consider_footprint: true            # 核心：严格按矩形车身检测碰撞，而非圆心
        collision_cost: 1000000.0
      PreferForwardCritic:
        enabled: true
        cost_weight: 3.0                    # 权重不宜过大，否则窄道不愿倒车

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.03                      # 3cm 高分辨率，适配窄通道
      # 精确矩形 Footprint（长 0.95m x 宽 0.65m，中心为原点）
      footprint: "[[0.475, 0.325], [0.475, -0.325], [-0.475, -0.325], [-0.475, 0.325]]"
      # 实验组 A（纯 2D）：去掉 nvblox_layer；实验组 B（3D 视觉）：保留 nvblox_layer
      plugins: ["obstacle_layer", "nvblox_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
      nvblox_layer:
        plugin: "nvblox::nav2::NvbloxCostmapLayer"
        enabled: true
        nav2_costmap_global_frame: odom
        nvblox_map_slice_topic: "/nvblox_node/static_map_slice"
        convert_to_binary_costmap: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 0.25              # 窄道膨胀半径不宜过大，否则通道会被封死

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      resolution: 0.05
      footprint: "[[0.475, 0.325], [0.475, -0.325], [-0.475, -0.325], [-0.475, 0.325]]"
      plugins: ["static_layer", "inflation_layer"]
      # 首轮跑通先注释掉：没有 filter_mask_server + costmap_filter_info_server 时 Nav2 会一直等话题
      # filters: ["keepout_filter"]        # 修正：Costmap Filter 必须放在 filters，不能放 plugins
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: true
        filter_info_topic: "/costmap_filter_info"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 4.0
        inflation_radius: 0.25

collision_monitor:
  ros__parameters:
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    observation_sources: ["scan"]
    polygons: ["PolygonStop", "PolygonSlow"]
    PolygonStop:
      type: "polygon"
      # 比车身四周大 10cm 的硬件级急停区
      points: "[[0.575, 0.425], [0.575, -0.425], [-0.575, -0.425], [-0.575, 0.425]]"
      action_type: "stop"
      min_points: 3
      enabled: true
    PolygonSlow:
      type: "polygon"
      # 前后延伸 1.0m 的减速预警区
      points: "[[1.475, 0.50], [1.475, -0.50], [-1.475, -0.50], [-1.475, 0.50]]"
      action_type: "slowdown"
      slowdown_ratio: 0.4
      min_points: 3
      enabled: true
```

> 注：启用 `keepout_filter` 时，launch 中还需配套启动 `costmap_filter_info_server`，并用 `filter_mask_server` 加载禁行遮罩图，否则 `/costmap_filter_info` 无数据、过滤器不生效。

## 四、 核心程序 3：自动化可达性压测与报表脚本

**文件：`ros2_scripts/run_reachability_benchmark.py`**

**功能**：自动读取预设的机台目标点，依次控制 AMR 导航过去，后台实时监听 Isaac Sim 发来的 `/amr/collision_event`，自动判定每条路线的状态（`SUCCESS` / `COLLISION_FAIL` / `PLAN_OR_TIMEOUT_FAIL`），并生成 CSV 报告。每条用例开始前会先通过 `/amr/reset_pose` 把车体复位到安全起点并清空代价地图，保证用例之间互不污染。

```
import csv
import os
import time
import math
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# 每条用例开始前把车体复位到的安全起点（x, y, yaw_deg）
START_POSE = (0.0, 0.0, 180.0)
RESET_SETTLE_SEC = 2.0

# 测试点位定义：[名称, x, y, yaw_deg]（实际使用时可从 YAML 读取）
TEST_GOALS = [
    ("Bay1_Narrow_Aisle_Entry", 4.5, 0.0, 0.0),
    ("Bay1_LoadPort_01_Dock",   8.2, 1.5, 90.0),   # 窄道 90 度拐弯+对接
    ("Bay1_DeadEnd_Reverse",    8.2, -2.0, -90.0), # 死胡同测试（验证倒车退出）
    ("Return_To_Staging",       0.0, 0.0, 180.0),
]

class CollisionListener(Node):
    def __init__(self):
        super().__init__('benchmark_collision_listener')
        self.sub = self.create_subscription(String, '/amr/collision_event', self._cb, 10)
        self.reset_pub = self.create_publisher(PoseStamped, '/amr/reset_pose', 10)
        self.collision_count = 0

    def _cb(self, msg):
        self.collision_count += 1

    def reset(self):
        self.collision_count = 0

    def teleport_robot(self, x, y, yaw_deg, settle_sec=2.0):
        """把车体瞬移回安全起点（Isaac 侧订阅 /amr/reset_pose 执行），避免卡死位姿污染下一条用例"""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        yaw_rad = math.radians(yaw_deg)
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self.reset_pub.publish(msg)
        time.sleep(settle_sec)   # 等物理状态、TF 与代价地图稳定后再下发下一个目标

def create_pose(navigator, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    yaw_rad = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()
    col_listener = CollisionListener()

    print("[INFO] 等待 Nav2 导航栈激活...")
    navigator.waitUntilNav2Active()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs("reports", exist_ok=True)
    report_file = f"reports/fab_reachability_{timestamp}.csv"
    results = []

    for name, x, y, yaw in TEST_GOALS:
        print(f"\n>>> 开始测试目标点: {name} (x={x}, y={y}, yaw={yaw}°)")
        col_listener.reset()
        # 每条用例前后互相独立：先瞬移回安全起点，再清空残留障碍，避免上一条撞车卡死后污染后续数据
        col_listener.teleport_robot(*START_POSE, settle_sec=RESET_SETTLE_SEC)
        navigator.clearAllCostmaps()
        goal_pose = create_pose(navigator, x, y, yaw)

        start_time = time.time()
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            rclpy.spin_once(col_listener, timeout_sec=0.05)
            # 超时保护（如 90 秒未到达则判定为死锁）
            if time.time() - start_time > 90.0:
                navigator.cancelTask()
                break

        duration = round(time.time() - start_time, 2)
        nav_result = navigator.getResult()

        # 综合判定结果
        if col_listener.collision_count > 0:
            status = "COLLISION_FAIL (擦碰机台/3D盲区)"
        elif nav_result == TaskResult.SUCCEEDED:
            status = "PASS"
        elif nav_result == TaskResult.CANCELED:
            status = "TIMEOUT_DEADLOCK (卡死超时)"
        else:
            status = "PLAN_OR_EXEC_FAIL (无路径或执行失败)"

        print(f"<<< 结果: {status} | 耗时: {duration}s | 碰撞次数: {col_listener.collision_count}")
        results.append([name, x, y, yaw, status, duration, col_listener.collision_count])

    # 导出 CSV 报告
    with open(report_file, mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(["Goal_Name", "X", "Y", "Yaw_Deg", "Status", "Duration_Sec", "Collision_Count"])
        writer.writerows(results)

    print(f"\n[DONE] 压测完成，报告已保存至: {report_file}")
    col_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 五、 落地执行顺序（4 步跑通）

1. **生成静态 2D 地图**：在 Isaac Sim 5.1 中打开你的 USD，使用菜单 `Tools -> Robotics -> Occupancy Map` 插件，设置切片高度为 `0.1m ~ 1.8m`、分辨率 `0.05m`，导出 `fab_map.pgm` 与 `fab_map.yaml`。
2. **启动仿真环境**：修改 `setup_fab_scene.py` 中的 `USD_PATH` 和 `PROXY_BOX_SIZE`，使用 Isaac Sim 自带的 Python 环境运行：

```
~/.local/share/ov/pkg/isaac-sim-5.1.0/python.sh isaac_scripts/setup_fab_scene.py
```
3. **启动 Nav2（与 Nvblox）**：在 ROS 2 Jazzy 终端中加载导出的地图和上述 YAML 配置：

```
ros2 launch nav2_bringup bringup_launch.py map:=/path/to/fab_map.yaml params_file:=config/nav2_fab_narrow_params.yaml use_sim_time:=True
```

*(跑 Exp_03/Exp_04 时，另开终端启动 `isaac_ros_nvblox` 节点接入 Nova Carter 的深度相机话题)*。
4. **运行自动化压测**：

```
python3 ros2_scripts/run_reachability_benchmark.py
```

跑完后直接修改 `PROXY_BOX_SIZE` 和 YAML 中的 `footprint`，重复运行即可快速拿到不同尺寸与传感器方案的对比 CSV 报表。

---

**Q：**

> 提问时间: 2026-11-24 23:20

请帮我写一段 Python 脚本，直接读取 Isaac Sim 导出的 fab_map.pgm 占据栅格图，自动计算并标出场景中所有通道的净宽和最窄瓶颈位置。

**A：**

利用 **欧氏距离变换（Distance Transform）+ 骨架提取（Medial Axis / Skeletonization）** 是分析占据栅格图通道净宽最稳健的方法：

- **距离变换**：计算每个自由空间像素到最近障碍物的距离 $d$（即该点最大内切圆的半径）。
- **骨架提取**：提取所有通道的中心线（中轴线）。中心线上任意一点的通道净宽即为 $W = 2 \times d \times \text{resolution}$。
- **非极大值抑制（NMS）**：过滤同一瓶颈处相邻的像素点，精准挑出全图独立的 **Top-N 最窄瓶颈**，并直接换算为 ROS 世界坐标 $(x, y)$，方便你直接填进 Nav2 测试脚本。

### 1. 安装依赖

```
pip install opencv-python numpy pyyaml scikit-image
```

### 2. 完整分析脚本（`analyze_fab_aisles.py`）

```
import os
import yaml
import cv2
import numpy as np
from skimage.morphology import skeletonize

# ================= 配置参数区 =================
YAML_PATH = "fab_map.yaml"          # Isaac Sim 导出的 yaml 文件路径
OUTPUT_IMG = "fab_aisle_analysis.png"

MIN_VALID_WIDTH_M = 0.30            # 过滤小于 0.30m 的机台内部缝隙或噪点
MAX_AISLE_WIDTH_M = 3.00            # 超过 3.00m 视为大厅/开阔区，不计入窄道统计
TOP_N_BOTTLENECKS = 8               # 标出最窄的前 N 个独立瓶颈
MIN_BOTTLENECK_DIST_M = 1.50        # 两个瓶颈之间的最小间距（防止重复标注同一处）
# ==============================================

def load_map(yaml_path):
    with open(yaml_path, "r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)

    resolution = float(meta["resolution"])
    origin = meta["origin"]  # [origin_x, origin_y, origin_yaw]
    free_thresh = float(meta.get("free_thresh", 0.196))

    img_path = meta["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)

    gray = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise FileNotFoundError(f"无法读取地图图像: {img_path}")

    # ROS 栅格定义：像素值越接近 255 越空闲；自由空间阈值 = 255 * (1 - free_thresh)
    free_pixel_thresh = int(255 * (1.0 - free_thresh))
    free_mask = (gray >= free_pixel_thresh).astype(np.uint8)

    return gray, free_mask, resolution, origin

def keep_largest_connected_area(free_mask):
    """仅保留最大的连通自由空间，自动剔除机台内部封闭空腔"""
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(free_mask, connectivity=8)
    if num_labels <= 1:
        return free_mask
    # 跳过背景 label 0，找面积最大的连通域
    largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    return (labels == largest_label).astype(np.uint8)

def pixel_to_world(u, v, img_height, resolution, origin):
    """图像像素坐标 (u=列, v=行) 转 ROS 地图世界坐标 (x, y)"""
    world_x = origin[0] + u * resolution
    world_y = origin[1] + (img_height - 1 - v) * resolution
    return world_x, world_y

def analyze_fab_aisles():
    gray, free_mask, res, origin = load_map(YAML_PATH)
    h, w = gray.shape

    # 1. 清理机台内部空洞，只分析车能开进去的主连通区
    main_free = keep_largest_connected_area(free_mask)

    # 2. 欧氏距离变换：每个像素值为到最近障碍物的像素距离（半径）
    dist_px = cv2.distanceTransform(main_free, cv2.DIST_L2, 5)
    width_map_m = 2.0 * dist_px * res  # 换算为通道净宽（米）

    # 3. 提取通道中轴线（骨架）
    skeleton = skeletonize(main_free > 0)
    ys, xs = np.where(skeleton)

    # 过滤有效通道点
    valid_points = []
    for y, x in zip(ys, xs):
        width_m = width_map_m[y, x]
        if MIN_VALID_WIDTH_M <= width_m <= MAX_AISLE_WIDTH_M:
            valid_points.append((float(width_m), int(x), int(y)))

    if not valid_points:
        print("[WARN] 未检测到符合宽度范围的通道，请检查地图阈值或 MIN/MAX_WIDTH 参数。")
        return

    # 按宽度从小到大排序
    valid_points.sort(key=lambda item: item[0])

    # 4. 筛选独立的 Top-N 最窄瓶颈（空间距离去重）
    min_dist_px = MIN_BOTTLENECK_DIST_M / res
    bottlenecks = []
    for width_m, x, y in valid_points:
        too_close = False
        for _, bx, by in bottlenecks:
            if np.hypot(x - bx, y - by) < min_dist_px:
                too_close = True
                break
        if not too_close:
            bottlenecks.append((width_m, x, y))
        if len(bottlenecks) >= TOP_N_BOTTLENECKS:
            break

    # 5. 生成可视化热力图
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    # 给骨架中轴线上色：红(<0.9m) -> 黄(0.9~1.3m) -> 绿(>1.3m)
    for width_m, x, y in valid_points:
        if width_m < 0.90:
            color = (0, 0, 255)      # 红色：极窄高危区
        elif width_m < 1.30:
            color = (0, 200, 255)    # 黄色：中等受限区
        else:
            color = (0, 200, 0)      # 绿色：宽敞通行区
        cv2.circle(vis, (x, y), 1, color, -1)

    # 标出最窄瓶颈及其内切圆
    print("\n================ 场景通道净宽分析报告 ================")
    print(f"地图尺寸: {w}x{h} px | 分辨率: {res} m/px")
    print(f"全图全局最窄净宽: {bottlenecks[0][0]:.3f} m")
    print("-" * 58)
    print(f"{'编号':<6} {'通道净宽(m)':<12} {'建议最大车宽(m)':<16} {'ROS 世界坐标 (x, y)'}")
    print("-" * 58)

    for idx, (width_m, x, y) in enumerate(bottlenecks, start=1):
        wx, wy = pixel_to_world(x, y, h, res, origin)
        # 按单侧预留 0.15m（双侧共 0.30m）估算最大允许车宽
        max_robot_w = max(0.0, width_m - 0.30)
        print(f"#{idx:<5} {width_m:<12.3f} {max_robot_w:<16.3f} ({wx:.2f}, {wy:.2f})")

        # 在图上画出瓶颈处的最大内切圆（直观展示物理缝隙）
        radius_px = max(2, int((width_m / 2.0) / res))
        cv2.circle(vis, (x, y), radius_px, (255, 0, 255), 2)
        cv2.putText(
            vis, f"#{idx}:{width_m:.2f}m", (x + 6, y - 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 0, 200), 1, cv2.LINE_AA
        )

    cv2.imwrite(OUTPUT_IMG, vis)
    print("-" * 58)
    print(f"[DONE] 可视化标注图已保存至: {OUTPUT_IMG}\n")

if __name__ == "__main__":
    analyze_fab_aisles()
```

### 3. 运行与结果解读

1. **运行脚本**：确保 `fab_map.yaml` 和 `fab_map.pgm` 在同一目录下，修改脚本顶部的 `YAML_PATH` 后运行：

```
python3 analyze_fab_aisles.py
```
2. **如何看结果**：
  - **终端输出**：直接列出最窄的 8 处瓶颈的**精确净宽**、**反推的最大允许车宽**（按双侧共留 30cm 安全余量估算），以及对应的 **ROS 世界坐标 `(x, y)`**——你可以直接把这些坐标填进上一轮的 `run_reachability_benchmark.py` 中作为重点压测点位。
  - **可视化图片 (`fab_aisle_analysis.png`)**：
    - **红色中心线**：净宽小于 `0.9m` 的极窄通道（重点排查是否需要全向底盘或倒车退出）。
    - **黄色中心线**：`0.9m ~ 1.3m` 的受限通道。
    - **洋红色圆圈**：最窄瓶颈处的**最大内切圆**，圆圈的直径就是该处卡住车身的物理极限。

---

## 附：首轮跑通必改的 4 个隐坑与推进节奏

> 正文按「能长期跑」写；下面 4 条是「第一次就想不报错、不出现隐形失效」必须闭环的点，上文对应代码已按此修正。

### 1. 代理盒可能把雷达和深度相机「包」起来（最大隐藏坑）

- **现象**：代理盒挂在 `chassis_link` 上并带 `CollisionAPI`，尺寸放大到整车包络后，Nova Carter 的 2D LiDAR 与深度相机被整体包在盒内。RTX 射线打在盒内壁上 → 雷达与相机全盲，表现为「明明有墙却直冲」。
- **方案 A（默认采用，改动最小）**：代理盒只承担物理碰撞，不参与渲染。`PROXY_RENDER_VISIBLE = False` 时对盒子执行 `UsdGeom.Imageable.MakeInvisible()`；RTX 传感器不对不可见几何做射线求交，而 PhysX 接触仍然正常触发。调参阶段设回 `True` 即可肉眼确认包络形状。
- **方案 B（最真实）**：把 LiDAR / 相机 Prim 动态移到盒外表面（车头前脸、车顶前缘），代价是传感器外参要随车身尺寸一起改。
- **如需更细**：再叠加 `UsdPhysics.FilteredPairsAPI` 或 PhysX Collision Group，让代理盒「只与机台接触、不与地面/自车其他部件接触」。第一轮不必上。

### 2. Nav2 的两处「隐形依赖」

- **禁用 Spin 必须换行为树**：只改 YAML 不生效。`bt_navigator` 默认加载 `navigate_to_pose_w_replanning_and_recovery.xml`，`Spin` / `BackUp` 仍在里面，车卡住时照样原地转圈扫机台。需要自定义 BT 并指定 `default_nav_to_pose_bt_xml`：

```xml
<!-- config/fab_narrow_no_spin.xml：只保留「重规划 + 清理代价地图」，彻底去掉 Spin / BackUp / Wait -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <RecoveryNode number_of_retries="3" name="ComputePathToPose">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
          <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
        </RecoveryNode>
      </RateController>
      <RecoveryNode number_of_retries="1" name="FollowPath">
        <FollowPath path="{path}" controller_id="FollowPath"/>
        <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
      </RecoveryNode>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "<绝对路径>/config/fab_narrow_no_spin.xml"
```

- **KeepoutFilter 需要配套节点**：`filter_mask_server`（加载遮罩图）+ `costmap_filter_info_server`（发布 `/costmap_filter_info`）。所以首轮先把 `filters: ["keepout_filter"]` 注释掉（上文 YAML 已按此处理），等车能正常动了再开禁行区。

### 3. 定位先用「仿真真值」，别一上来就 AMCL

- 厂内长直走廊多、机台外形高度相似，AMCL 容易漂。第一轮把定位误差混进来，撞了也分不清是「定位飘了」还是「通道太窄」。
- 做法：本阶段直接在 Isaac 的 Action Graph 里发布真值 TF（`map -> odom -> base_link`，`map -> odom` 用静态单位变换），**不启动 AMCL**。等纯粹的几何与规划边界结论拿到后，再叠加 AMCL 做鲁棒性回归——这样每条失败都能唯一归因到几何或规划。

### 4. 资产路径 + 失败后的「状态重置」

- **资产路径**：不要写死 `omniverse://localhost/...`，改用 `get_assets_root_path()` 拼接（上文脚本已改），换机器/换版本都不会 `FileNotFoundError`。
- **卡死污染**：第 2 个点撞车卡死后，第 3 个点会从卡死位姿出发，后续结果全废。因此每条用例开始前通过 `/amr/reset_pose` 把车体瞬移回安全起点、清零速度，再 `clearAllCostmaps()` 清掉残留障碍，然后才下发新目标（上文两个脚本已打通：Isaac 侧订阅复位，压测侧发点）。

> 说明：Isaac 侧复位用的是 `Articulation.set_world_pose()`，属于纯仿真手段，不经过规划器；若希望更接近真实部署，可改为调用 Nav2 的 `initialpose` 重定位流程，但第一轮无需引入这层复杂度。

### 推荐推进节奏（每步都能单独验收）

| 阶段 | 做什么 | 验收标准 |
| --- | --- | --- |
| **第 0 步（今天就能跑）** | 用修好的 `analyze_fab_aisles.py` 跑一遍你的 `fab_map.pgm` | 10 分钟内拿到全图最窄净宽 + Top-N 瓶颈的 ROS 坐标，先把「车能做多大」摸清 |
| **第 1 步（最小闭环）** | 真值 TF + 纯 2D LiDAR + `SmacPlannerHybrid` + MPPI；先不开 `keepout_filter`、不开 Nvblox | Nova Carter 在场景里连续走完 3~5 个点，`Collision_Count = 0` |
| **第 2 步（叠加压测）** | 挂修正后的代理盒、开 Nvblox、换无 Spin 行为树，跑 Exp_01~04 | 拿到「尺寸 × 感知模式」对比 CSV，输出选型边界结论 |

---

