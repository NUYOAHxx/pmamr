#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 Nav2 的质心轨迹扩成「车身扫掠带 / 扫掠体」，让 RViz 直接看到车扫过的体积。

Nav2 发布的 /plan、/trajectory、/local_plan 都只是 base_link 原点的一条线，
宽度只在 footprint 参数里。本节点把每个位姿的矩形包络（默认 0.95 × 0.65 × 1.4）
投影出来，形成：

  1. 质心线（白）        —— 原本 Nav2 那条线，便于对照
  2. 左右两条边缘线（青/橙）—— 车身左右两侧扫过的边界
  3. 半透明填充带（黄）   —— 俯视看到的扫过面积，拐角会明显鼓出
  4. 可选 3D 隧道（蓝）   —— 每个位姿一个立方体，能看车顶会不会扫到悬空物

用法：
    ros2 run ...               # 或直接
    python3 ros2_scripts/publish_swept_area.py --source /trajectory
    python3 ros2_scripts/publish_swept_area.py --source /plan --mode all

RViz2 里 Add -> MarkerArray，topic 填 /swept_area 即可。
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

# 车身包络（与 setup_fab_scene.py 的 PROXY_BOX_SIZE 保持一致）
BODY_LENGTH = 0.95
BODY_WIDTH = 0.65
BODY_HEIGHT = 1.40

COLOR_CENTER = (1.0, 1.0, 1.0)
COLOR_LEFT = (0.0, 0.8, 1.0)
COLOR_RIGHT = (1.0, 0.55, 0.0)
COLOR_FILL = (1.0, 0.9, 0.2)
COLOR_BOX = (0.3, 0.5, 1.0)


def yaw_from_quat(q):
    """四元数取 yaw（只关心平面朝向）。"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def corners(pose, half_l, half_w):
    """返回该位姿下车身矩形的四角：前左、前右、后左、后右（世界坐标）。"""
    yaw = yaw_from_quat(pose.orientation)
    cx, cy = pose.position.x, pose.position.y
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    out = []
    for dx, dy in ((half_l, half_w), (half_l, -half_w), (-half_l, half_w), (-half_l, -half_w)):
        out.append((cx + dx * cos_y - dy * sin_y,
                    cy + dx * sin_y + dy * cos_y))
    return out  # FL, FR, RL, RR


def convex_hull(pts):
    """Andrew monotone chain，返回逆时针凸包（用于求两个位姿矩形的并集轮廓）。"""
    pts = sorted(set(pts))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def shoelace(pts):
    """多边形面积（世界坐标，返回绝对值）。"""
    s = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


class SweptAreaPublisher(Node):
    def __init__(self, source, mode, half_l, half_w, height):
        super().__init__("swept_area_publisher")
        self.mode = mode
        self.half_l = half_l
        self.half_w = half_w
        self.height = height

        # 结果要能被后打开的 RViz 看到，用 transient_local
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(MarkerArray, "/swept_area", qos)
        self.sub = self.create_subscription(Path, source, self.on_path, 10)
        self.get_logger().info(
            f"订阅 {source} | mode={mode} | 包络 {half_l * 2:.2f}×{half_w * 2:.2f}×{height:.2f} m"
        )

    # ---------- 主回调 ----------
    def on_path(self, msg: Path):
        poses = msg.poses
        if len(poses) < 2:
            self.get_logger().warn("路径点数不足 2，忽略")
            return

        frame = msg.header.frame_id or "map"
        stamp = msg.header.stamp
        arr = MarkerArray()

        centers = [(p.pose.position.x, p.pose.position.y) for p in poses]
        quads = [corners(p.pose, self.half_l, self.half_w) for p in poses]

        # 1. 质心线
        if self.mode in ("all", "band"):
            arr.markers.append(self._line(0, "center", centers, COLOR_CENTER, 0.03, frame, stamp))

            # 2. 左右边缘线：左 = FL->RL，右 = FR->RR
            left = [q[0] for q in quads]
            right = [q[1] for q in quads]
            arr.markers.append(self._line(1, "left_edge", left, COLOR_LEFT, 0.02, frame, stamp))
            arr.markers.append(self._line(2, "right_edge", right, COLOR_RIGHT, 0.02, frame, stamp))

        # 3. 半透明填充带：每段拆前后两个四边形（FL/FR/FR'/FL'、RL/RR/RR'/RL'）
        if self.mode in ("all", "fill"):
            tri = Marker()
            tri.header.frame_id, tri.header.stamp = frame, stamp
            tri.ns, tri.id, tri.type, tri.action = "swept_fill", 10, Marker.TRIANGLE_LIST, Marker.ADD
            tri.scale.x = tri.scale.y = tri.scale.z = 1.0
            tri.pose.orientation.w = 1.0
            tri.color.r, tri.color.g, tri.color.b = COLOR_FILL
            tri.color.a = 0.22
            for i in range(len(quads) - 1):
                # 用两位姿 8 个角点的凸包做并集轮廓，避免前后半矩形重复覆盖
                hull = convex_hull(quads[i] + quads[i + 1])
                for k in range(1, len(hull) - 1):
                    for x, y in (hull[0], hull[k], hull[k + 1]):
                        tri.points.append(Point(x=float(x), y=float(y), z=0.01))
            arr.markers.append(tri)

        # 4. 3D 隧道：按位姿放立方体（点太密时隔几个放一个）
        if self.mode in ("all", "3d"):
            step = max(1, len(poses) // 60)
            tunnel = Marker()
            tunnel.header.frame_id, tunnel.header.stamp = frame, stamp
            tunnel.ns, tunnel.id, tunnel.type, tunnel.action = "swept_tunnel", 100, Marker.CUBE_LIST, Marker.ADD
            tunnel.scale.x, tunnel.scale.y, tunnel.scale.z = self.half_l * 2, self.half_w * 2, self.height
            tunnel.pose.orientation.w = 1.0
            tunnel.color.r, tunnel.color.g, tunnel.color.b = COLOR_BOX
            tunnel.color.a = 0.12
            for i in range(0, len(poses), step):
                p = poses[i]
                tunnel.points.append(Point(x=float(p.pose.position.x),
                                           y=float(p.pose.position.y),
                                           z=self.height / 2.0))
            arr.markers.append(tunnel)

        self.pub.publish(arr)

        # 控制台给出量化结果（CSV 里看不到的东西）
        path_len = sum(math.dist(centers[i], centers[i + 1]) for i in range(len(centers) - 1))
        area = 0.0
        for i in range(len(quads) - 1):
            area += shoelace(convex_hull(quads[i] + quads[i + 1]))
        net = math.dist(centers[0], centers[-1])
        detour = (path_len / net) if net > 1e-3 else float("nan")
        self.get_logger().info(
            f"路径 {path_len:.2f} m | 扫掠带面积 {area:.2f} m² | 直线 {net:.2f} m | 绕行比 {detour:.2f}"
            f" | 点数 {len(poses)}"
        )

    # ---------- 工具 ----------
    def _line(self, mid, ns, pts, rgb, width, frame, stamp):
        m = Marker()
        m.header.frame_id, m.header.stamp = frame, stamp
        m.ns, m.id, m.type, m.action = ns, mid, Marker.LINE_STRIP, Marker.ADD
        m.scale.x = width
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = 1.0
        for x, y in pts:
            m.points.append(Point(x=float(x), y=float(y), z=0.02))
        return m


def main():
    ap = argparse.ArgumentParser(description="把质心轨迹扩成车身扫掠带 / 扫掠体")
    ap.add_argument("--source", default="/trajectory",
                    help="订阅的 Path 话题，如 /trajectory（MPPI 预测）、/plan（规划器）")
    ap.add_argument("--mode", default="all", choices=["all", "band", "fill", "3d"],
                    help="band=线框, fill=填充带, 3d=立体隧道, all=全部")
    ap.add_argument("--length", type=float, default=BODY_LENGTH, help="车长 m")
    ap.add_argument("--width", type=float, default=BODY_WIDTH, help="车宽 m")
    ap.add_argument("--height", type=float, default=BODY_HEIGHT, help="车高 m")
    args = ap.parse_args()

    rclpy.init()
    node = SweptAreaPublisher(args.source, args.mode,
                              args.length / 2.0, args.width / 2.0, args.height)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
