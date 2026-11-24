#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
