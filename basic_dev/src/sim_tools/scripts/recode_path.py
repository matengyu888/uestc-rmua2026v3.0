#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import csv
import os
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class SmartPathRecorder:
    def __init__(self):
        rospy.init_node('smart_path_recorder', anonymous=True)

        # --- 配置参数 ---
        self.distance_threshold = 0.5  # 只有移动超过 0.1 米才记录
        self.csv_file_path = "../path/drone_path_log.csv"
        
        # --- 状态变量 ---
        self.last_pose = None
        self.path_data = [] # 用于最后统一保存到 CSV
        self.ros_path = Path()
        self.ros_path.header.frame_id = "world_enu" # 请根据实际情况修改或保持空

        # --- ROS 接口 ---
        self.pose_sub = rospy.Subscriber('/airsim_node/drone_1/debug/pose_gt', PoseStamped, self.pose_callback)
        self.path_pub = rospy.Publisher('/drone_1/smart_path', Path, queue_size=10)

        # 注册退出时的钩子函数
        rospy.on_shutdown(self.save_to_csv)
        
        rospy.loginfo("节点已启动。阈值: {}m, 目标文件: {}".format(self.distance_threshold, self.csv_file_path))

    def get_distance(self, p1, p2):
        """计算两个位姿之间的 3D 欧几里得距离"""
        return math.sqrt(
            (p1.x - p2.x)**2 + 
            (p1.y - p2.y)**2 + 
            (p1.z - p2.z)**2
        )

    def pose_callback(self, msg):
        current_pos = msg.pose.position
        
        # 1. 距离检查
        if self.last_pose is not None:
            dist = self.get_distance(current_pos, self.last_pose)
            if dist < self.distance_threshold:
                return # 移动距离太小，忽略此点
        
        # 2. 更新记录
        self.last_pose = current_pos
        self.ros_path.header.stamp = msg.header.stamp
        self.ros_path.header.frame_id = msg.header.frame_id
        self.ros_path.poses.append(msg)
        
        # 3. 缓存数据用于 CSV (保存: 时间戳, x, y, z, qx, qy, qz, qw)
        self.path_data.append([
            msg.header.stamp.to_sec(),
            current_pos.x, current_pos.y, current_pos.z,
            msg.pose.orientation.x, msg.pose.orientation.y, 
            msg.pose.orientation.z, msg.pose.orientation.w
        ])

        # 4. 发布路径用于 Rviz 可视化
        self.path_pub.publish(self.ros_path)

    def save_to_csv(self):
        """关闭节点时将数据写入文件"""
        rospy.loginfo("正在保存路径到 CSV...")
        try:
            with open(self.csv_file_path, 'w') as f:
                writer = csv.writer(f)
                # 写入表头
                writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
                writer.writerows(self.path_data)
            rospy.loginfo("保存成功！文件路径: {}".format(os.path.abspath(self.csv_file_path)))
        except Exception as e:
            rospy.logerr("保存失败: {}".format(e))

if __name__ == '__main__':
    try:
        recorder = SmartPathRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass