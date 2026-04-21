#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import Image

try:
    import torch
    import torch.nn.functional as F
except Exception as exc:
    rospy.logerr("导入 torch 失败: %s", exc)
    rospy.logerr("请使用包含 PyTorch 的 conda 环境启动该节点。")
    raise

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from model import Model


class DPPlanner:
    def __init__(self):
        rospy.init_node("dp_planner_node", anonymous=True)

        self.max_speed = rospy.get_param("~max_speed", 9.0)
        self.margin = rospy.get_param("~margin", 0.5)
        self.thr_est_error = rospy.get_param("~thr_est_error", 1.0)
        self.goal_reached_dist = rospy.get_param("~goal_reached_dist", 0.4)

        self.odom_topic = rospy.get_param("~odom_topic", "/airsim_node/drone_1/debug/pose_gt")
        self.odom_type = rospy.get_param("~odom_type", "pose")
        self.depth_topic = rospy.get_param("~depth_topic", "/airsim_node/drone_1/front_left/DepthPerspective")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal_pose")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/position_cmd")

        default_model_path = os.path.join(SCRIPT_DIR, "..", "models", "dp_planner", "3-19.pth")
        self.model_path = rospy.get_param("~model_path", default_model_path)

        self.g_std = torch.tensor([0.0, 0.0, -9.80665], dtype=torch.float32)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = Model(dim_obs=10, dim_action=6).to(self.device)

        if not os.path.isfile(self.model_path):
            raise FileNotFoundError("未找到模型文件: {}".format(self.model_path))

        self.model.load_state_dict(torch.load(self.model_path, map_location=self.device))
        self.model.eval()
        self.h = None

        self.curr_pos = None
        self.curr_vel = None
        self.curr_q = None
        self.curr_yaw = 0.0
        self.depth_tensor = None
        self.target_pos = None
        self.is_active = False
        self.last_depth_stamp = None

        if self.odom_type == "odometry":
            rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        else:
            rospy.Subscriber(self.odom_topic, PoseStamped, self.pose_cb)
        rospy.Subscriber(self.depth_topic, Image, self.depth_cb)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_cb)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, PositionCommand, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / 15.0), self.control_loop)

        rospy.loginfo("DP-Planner 已加载。model=%s", self.model_path)
        rospy.loginfo("odom_topic=%s (%s), depth_topic=%s, goal_topic=%s, cmd_topic=%s",
                      self.odom_topic, self.odom_type, self.depth_topic, self.goal_topic, self.cmd_topic)
        rospy.logwarn("当前控制器只能直接利用 PositionCommand 的 position/velocity/yaw，acceleration 暂未直接用于速度内环。")

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_reference_rotation(self):
        q = self.curr_q
        qw, qx, qy, qz = q.w, q.x, q.y, q.z

        rot = np.array([
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ], dtype=np.float32)

        fwd = rot[:, 0].copy()
        fwd[2] = 0.0
        fwd_norm = np.linalg.norm(fwd[:2])
        if fwd_norm < 1e-6:
            yaw = self.curr_yaw
            fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float32)
        else:
            fwd /= fwd_norm

        up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        left = np.cross(up, fwd)
        left_norm = np.linalg.norm(left)
        if left_norm < 1e-6:
            left = np.array([-math.sin(self.curr_yaw), math.cos(self.curr_yaw), 0.0], dtype=np.float32)
        else:
            left /= left_norm

        return torch.from_numpy(np.stack([fwd, left, up], axis=-1)).to(self.device)

    def goal_cb(self, msg):
        new_target = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=np.float32)
        if self.target_pos is not None and np.linalg.norm(new_target - self.target_pos) < 1e-3:
            return
        self.target_pos = new_target
        self.is_active = True
        self.h = None
        rospy.loginfo("收到新目标点: [%.3f, %.3f, %.3f]", *self.target_pos)

    def odom_cb(self, msg):
        self.curr_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ], dtype=np.float32)
        self.curr_vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ], dtype=np.float32)
        q = msg.pose.pose.orientation
        self.curr_q = q
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.curr_yaw = self.normalize_angle(math.atan2(siny_cosp, cosy_cosp))

        if self.target_pos is None:
            self.target_pos = self.curr_pos.copy()

    def pose_cb(self, msg):
        new_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=np.float32)
        if self.curr_pos is None:
            self.curr_vel = np.zeros(3, dtype=np.float32)
        else:
            dt = max((msg.header.stamp.to_sec() - getattr(self, "_last_pose_stamp", 0.0)), 1e-3)
            self.curr_vel = (new_pos - self.curr_pos) / dt
        self.curr_pos = new_pos
        self._last_pose_stamp = msg.header.stamp.to_sec() if not msg.header.stamp.is_zero() else rospy.Time.now().to_sec()
        q = msg.pose.orientation
        self.curr_q = q
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.curr_yaw = self.normalize_angle(math.atan2(siny_cosp, cosy_cosp))

        if self.target_pos is None:
            self.target_pos = self.curr_pos.copy()

    def depth_cb(self, msg):
        try:
            if msg.encoding == "32FC1":
                depth_array = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            elif msg.encoding == "16UC1":
                depth_array = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32)
                depth_array *= 0.001
            else:
                rospy.logwarn_throttle(1.0, "不支持的深度图编码: %s", msg.encoding)
                return

            depth_array = np.nan_to_num(depth_array, nan=24.0, posinf=24.0, neginf=0.3)
            depth_array = np.clip(depth_array, 0.3, 24.0)
            depth_t = torch.from_numpy(depth_array.copy()).to(self.device).unsqueeze(0).unsqueeze(0)
            self.depth_tensor = F.interpolate(depth_t, size=(48, 64), mode="bilinear", align_corners=False)
            self.last_depth_stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "深度图处理失败: %s", exc)

    def control_loop(self, _event):
        if self.curr_pos is None or self.curr_q is None or self.curr_vel is None:
            return
        if self.target_pos is None or not self.is_active:
            return
        if self.depth_tensor is None:
            rospy.logwarn_throttle(2.0, "尚未收到深度图，DP-Planner 不发布指令。depth_topic=%s", self.depth_topic)
            return

        goal_dist = np.linalg.norm(self.target_pos - self.curr_pos)
        if goal_dist < self.goal_reached_dist:
            self.is_active = False
            self.h = None
            rospy.loginfo_throttle(1.0, "目标已到达，停止发布 DP 规划指令。")
            return

        with torch.no_grad():
            x = 3.0 / self.depth_tensor.clamp(0.3, 24.0) - 0.6
            x = F.max_pool2d(x, 4, 4)

            R_ref = self.get_reference_rotation()

            diff = self.target_pos - self.curr_pos
            dist = np.linalg.norm(diff)
            target_v_world = (diff / dist) * min(dist, self.max_speed) if dist > 0.1 else np.zeros(3, dtype=np.float32)
            target_v_xy_norm = np.linalg.norm(target_v_world[:2])
            desired_yaw = self.curr_yaw
            if target_v_xy_norm > 1e-3:
                desired_yaw = self.normalize_angle(math.atan2(target_v_world[1], target_v_world[0]))

            target_v_local = torch.from_numpy(target_v_world).float().to(self.device) @ R_ref
            local_v = torch.from_numpy(self.curr_vel).float().to(self.device) @ R_ref
            r_z = torch.tensor([0.0, 0.0, 1.0], device=self.device)
            margin_t = torch.tensor([self.margin], device=self.device)
            state = torch.cat([local_v, target_v_local, r_z, margin_t]).view(1, -1)

            act, _, self.h = self.model(x, state, self.h)
            act_local = act.view(1, 3, 2)
            act_world = torch.matmul(R_ref.unsqueeze(0), act_local)
            a_pred_world = act_world[0, :, 0]
            v_pred_world = act_world[0, :, 1]
            acc_cmd_world = (a_pred_world - v_pred_world - self.g_std.to(self.device)) * self.thr_est_error \
                            + self.g_std.to(self.device)

            cmd = PositionCommand()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "world"
            cmd.position.x = self.curr_pos[0]
            cmd.position.y = self.curr_pos[1]
            cmd.position.z = self.curr_pos[2]
            cmd.velocity.x = target_v_world[0]
            cmd.velocity.y = target_v_world[1]
            cmd.velocity.z = target_v_world[2]
            cmd.acceleration.x = acc_cmd_world[0].item()
            cmd.acceleration.y = acc_cmd_world[1].item()
            cmd.acceleration.z = acc_cmd_world[2].item()
            cmd.jerk.x = 0.0
            cmd.jerk.y = 0.0
            cmd.jerk.z = 0.0
            cmd.yaw = desired_yaw
            cmd.yaw_dot = 0.0
            cmd.kx = [2.0, 2.0, 2.5]
            cmd.kv = [1.8, 1.8, 2.5]
            cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            self.cmd_pub.publish(cmd)

            rospy.loginfo_throttle(
                0.5,
                "DP cmd | goal_dist=%.2f vel=[%.2f %.2f %.2f] acc=[%.2f %.2f %.2f] yaw=%.2f",
                goal_dist,
                cmd.velocity.x, cmd.velocity.y, cmd.velocity.z,
                cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z,
                cmd.yaw,
            )


if __name__ == "__main__":
    try:
        planner = DPPlanner()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        sys.exit(0)
