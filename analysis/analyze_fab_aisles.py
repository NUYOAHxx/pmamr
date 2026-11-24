#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
