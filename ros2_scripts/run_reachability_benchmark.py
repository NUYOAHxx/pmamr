#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
