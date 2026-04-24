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
    rospy.logerr("请确认镜像内已安装 PyTorch 运行依赖。")
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
        self.cmd_pos_horizon_sec = rospy.get_param("~cmd_pos_horizon_sec", 1.0)
        self.cmd_pos_horizon_z_sec = rospy.get_param("~cmd_pos_horizon_z_sec", 0.45)
        self.stop_at_goal = rospy.get_param("~stop_at_goal", False)
        self.max_speed_z = rospy.get_param("~max_speed_z", 0.9)
        self.turn_slowdown_start_deg = rospy.get_param("~turn_slowdown_start_deg", 70.0)
        self.turn_reverse_trigger_deg = rospy.get_param("~turn_reverse_trigger_deg", 135.0)
        self.turn_min_speed_scale = rospy.get_param("~turn_min_speed_scale", 0.45)
        self.reverse_turn_speed_scale = rospy.get_param("~reverse_turn_speed_scale", 0.60)
        self.yaw_slew_rate = rospy.get_param("~yaw_slew_rate", 0.55)
        self.use_network_velocity = rospy.get_param("~use_network_velocity", True)
        self.network_velocity_blend = rospy.get_param("~network_velocity_blend", 0.85)
        self.network_velocity_min_goal_scale = rospy.get_param("~network_velocity_min_goal_scale", 0.25)
        self.network_velocity_max_speed = rospy.get_param("~network_velocity_max_speed", self.max_speed)
        self.network_velocity_max_speed_z = rospy.get_param("~network_velocity_max_speed_z", self.max_speed_z)

        self.odom_topic = rospy.get_param("~odom_topic", "/airsim_node/drone_1/debug/pose_gt")
        self.odom_type = rospy.get_param("~odom_type", "pose")
        self.depth_topic = rospy.get_param("~depth_topic", "/airsim_node/drone_1/front_left/DepthPerspective")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal_pose")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/position_cmd")
        self.publish_debug_depth = rospy.get_param("~publish_debug_depth", True)

        default_model_path = os.path.join(SCRIPT_DIR, "..", "models", "dp_planner", "3-19.pth")
        self.model_path = rospy.get_param("~model_path", default_model_path)

        self.g_std = torch.tensor([0.0, 0.0, -9.80665], dtype=torch.float32)
        self.device = self.select_device()
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
        self.last_cmd_yaw = None
        self.last_cmd_time = None

        if self.odom_type == "odometry":
            rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        else:
            rospy.Subscriber(self.odom_topic, PoseStamped, self.pose_cb)
        rospy.Subscriber(self.depth_topic, Image, self.depth_cb)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_cb)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, PositionCommand, queue_size=1)
        self.debug_depth_resized_pub = None
        self.debug_depth_feature_pub = None
        self.debug_depth_feature_viz_pub = None
        if self.publish_debug_depth:
            self.debug_depth_resized_pub = rospy.Publisher(
                "/dp_planner/debug/depth_resized", Image, queue_size=1
            )
            self.debug_depth_feature_pub = rospy.Publisher(
                "/dp_planner/debug/depth_feature", Image, queue_size=1
            )
            self.debug_depth_feature_viz_pub = rospy.Publisher(
                "/dp_planner/debug/depth_feature_viz", Image, queue_size=1
            )
        self.timer = rospy.Timer(rospy.Duration(1.0 / 15.0), self.control_loop)

        rospy.loginfo("DP-Planner 已加载。model=%s", self.model_path)
        rospy.loginfo("odom_topic=%s (%s), depth_topic=%s, goal_topic=%s, cmd_topic=%s",
                      self.odom_topic, self.odom_type, self.depth_topic, self.goal_topic, self.cmd_topic)
        rospy.loginfo(
            "当前控制链路将网络速度写入 PositionCommand.velocity，并用该速度生成 position 前视点；"
            "acceleration 仍仅作为附带输出。"
        )

    def publish_debug_depth_images(self, depth_resized, feature_tensor):
        if not self.publish_debug_depth or self.debug_depth_resized_pub is None:
            return

        stamp = self.last_depth_stamp if self.last_depth_stamp is not None else rospy.Time.now()

        depth_np = depth_resized.detach().cpu().numpy().astype(np.float32)
        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "uav_base_link"
        depth_msg.height = depth_np.shape[0]
        depth_msg.width = depth_np.shape[1]
        depth_msg.encoding = "32FC1"
        depth_msg.is_bigendian = 0
        depth_msg.step = depth_np.shape[1] * 4
        depth_msg.data = depth_np.tobytes()
        self.debug_depth_resized_pub.publish(depth_msg)

        feature_np = feature_tensor.detach().cpu().numpy().astype(np.float32)
        feature_msg = Image()
        feature_msg.header.stamp = stamp
        feature_msg.header.frame_id = "uav_base_link"
        feature_msg.height = feature_np.shape[0]
        feature_msg.width = feature_np.shape[1]
        feature_msg.encoding = "32FC1"
        feature_msg.is_bigendian = 0
        feature_msg.step = feature_np.shape[1] * 4
        feature_msg.data = feature_np.tobytes()
        self.debug_depth_feature_pub.publish(feature_msg)

        feature_min = float(np.min(feature_np))
        feature_max = float(np.max(feature_np))
        if feature_max - feature_min < 1e-6:
            feature_viz = np.zeros_like(feature_np, dtype=np.uint8)
        else:
            normalized = (feature_np - feature_min) / (feature_max - feature_min)
            feature_viz = np.clip(normalized * 255.0, 0.0, 255.0).astype(np.uint8)

        viz_msg = Image()
        viz_msg.header.stamp = stamp
        viz_msg.header.frame_id = "uav_base_link"
        viz_msg.height = feature_viz.shape[0]
        viz_msg.width = feature_viz.shape[1]
        viz_msg.encoding = "mono8"
        viz_msg.is_bigendian = 0
        viz_msg.step = feature_viz.shape[1]
        viz_msg.data = feature_viz.tobytes()
        self.debug_depth_feature_viz_pub.publish(viz_msg)

    @staticmethod
    def select_device():
        preferred = os.environ.get("DP_PLANNER_DEVICE", "").strip().lower()
        if preferred == "cpu":
            rospy.logwarn("DP-Planner 按环境变量要求强制使用 CPU。")
            return torch.device("cpu")
        if preferred == "cuda":
            if torch.cuda.is_available():
                return torch.device("cuda")
            rospy.logwarn("DP_PLANNER_DEVICE=cuda 但当前 CUDA 不可用，回退到 CPU。")
            return torch.device("cpu")
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")

    def move_runtime_to_cpu(self, reason):
        if self.device.type != "cuda":
            return
        rospy.logwarn("DP-Planner CUDA 运行失败，回退到 CPU。reason=%s", reason)
        self.device = torch.device("cpu")
        self.model = self.model.to(self.device)
        self.g_std = self.g_std.to(self.device)
        self.h = None
        self.depth_tensor = None

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def limit_velocity_components(vel, max_speed_xy, max_speed_z):
        out = np.array(vel, dtype=np.float32, copy=True)
        max_speed_xy = max(0.0, float(max_speed_xy))
        max_speed_z = max(0.0, float(max_speed_z))
        xy_norm = float(np.linalg.norm(out[:2]))
        if xy_norm > max_speed_xy and xy_norm > 1e-6:
            out[:2] *= max_speed_xy / xy_norm
        out[2] = float(np.clip(out[2], -max_speed_z, max_speed_z))
        return out

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
            if self.device.type == "cuda" and "CUDA" in str(exc):
                self.move_runtime_to_cpu(exc)
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
        if self.stop_at_goal and goal_dist < self.goal_reached_dist:
            self.is_active = False
            self.h = None
            rospy.loginfo_throttle(1.0, "目标已到达，停止发布 DP 规划指令。")
            return

        try:
            with torch.no_grad():
                x = 3.0 / self.depth_tensor.clamp(0.3, 24.0) - 0.6
                x = F.max_pool2d(x, 4, 4)
                self.publish_debug_depth_images(self.depth_tensor[0, 0], x[0, 0])

                R_ref = self.get_reference_rotation()

                diff = self.target_pos - self.curr_pos
                dist = np.linalg.norm(diff)
                target_v_world = (diff / dist) * min(dist, self.max_speed) if dist > 0.1 else np.zeros(3, dtype=np.float32)
                target_v_world = self.limit_velocity_components(target_v_world, self.max_speed, self.max_speed_z)
                path_yaw = self.curr_yaw
                target_v_xy_norm = np.linalg.norm(target_v_world[:2])
                if target_v_xy_norm > 1e-3:
                    path_yaw = self.normalize_angle(math.atan2(target_v_world[1], target_v_world[0]))

                forward_err = abs(self.normalize_angle(path_yaw - self.curr_yaw))
                reverse_yaw = self.normalize_angle(path_yaw + math.pi)
                reverse_err = abs(self.normalize_angle(reverse_yaw - self.curr_yaw))
                use_reverse_heading = forward_err >= math.radians(self.turn_reverse_trigger_deg) and reverse_err < forward_err

                # Keep yaw rotating toward the actual path direction even when we
                # temporarily allow backing up through a sharp turnaround.
                desired_yaw_target = path_yaw
                speed_scale = 1.0
                if forward_err > math.radians(self.turn_slowdown_start_deg):
                    blend = (forward_err - math.radians(self.turn_slowdown_start_deg)) / max(
                        math.pi - math.radians(self.turn_slowdown_start_deg), 1e-3)
                    blend = float(np.clip(blend, 0.0, 1.0))
                    min_scale = self.reverse_turn_speed_scale if use_reverse_heading else self.turn_min_speed_scale
                    speed_scale = 1.0 - (1.0 - min_scale) * blend
                    target_v_world *= speed_scale
                    target_v_world = self.limit_velocity_components(target_v_world, self.max_speed, self.max_speed_z)

                cmd_now = rospy.Time.now()
                if self.last_cmd_time is None:
                    dt_cmd = 1.0 / 15.0
                else:
                    dt_cmd = max((cmd_now - self.last_cmd_time).to_sec(), 1e-3)

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
                net_v_world = v_pred_world.detach().cpu().numpy().astype(np.float32)
                net_v_world = self.limit_velocity_components(
                    net_v_world,
                    self.network_velocity_max_speed,
                    self.network_velocity_max_speed_z,
                )

                blend = float(np.clip(self.network_velocity_blend, 0.0, 1.0))
                cmd_v_world = target_v_world.copy()
                if self.use_network_velocity:
                    cmd_v_world = (1.0 - blend) * target_v_world + blend * net_v_world
                    if dist > 0.1:
                        goal_dir = diff / dist
                        min_goal_speed = min(dist, self.max_speed) * speed_scale * max(
                            0.0, float(self.network_velocity_min_goal_scale)
                        )
                        goal_progress = float(np.dot(cmd_v_world, goal_dir))
                        if goal_progress < min_goal_speed:
                            cmd_v_world += (min_goal_speed - goal_progress) * goal_dir
                cmd_v_world = self.limit_velocity_components(
                    cmd_v_world,
                    self.max_speed,
                    self.max_speed_z,
                )

                cmd_path_yaw = path_yaw
                cmd_v_xy_norm = np.linalg.norm(cmd_v_world[:2])
                if cmd_v_xy_norm > 1e-3:
                    cmd_path_yaw = self.normalize_angle(math.atan2(cmd_v_world[1], cmd_v_world[0]))

                cmd = PositionCommand()
                cmd.header.stamp = cmd_now
                cmd.header.frame_id = "world"
                cmd.velocity.x = cmd_v_world[0]
                cmd.velocity.y = cmd_v_world[1]
                cmd.velocity.z = cmd_v_world[2]

                horizon_xy = max(0.1, float(self.cmd_pos_horizon_sec))
                horizon_z = max(0.1, float(self.cmd_pos_horizon_z_sec))
                pos_ref = self.curr_pos.copy()
                pos_ref[0] += cmd_v_world[0] * horizon_xy
                pos_ref[1] += cmd_v_world[1] * horizon_xy
                pos_ref[2] += cmd_v_world[2] * horizon_z
                if np.linalg.norm(self.target_pos - self.curr_pos) <= np.linalg.norm(pos_ref - self.curr_pos):
                    pos_ref = self.target_pos.copy()

                cmd.position.x = pos_ref[0]
                cmd.position.y = pos_ref[1]
                cmd.position.z = pos_ref[2]
                cmd.acceleration.x = acc_cmd_world[0].item()
                cmd.acceleration.y = acc_cmd_world[1].item()
                cmd.acceleration.z = acc_cmd_world[2].item()
                cmd.jerk.x = 0.0
                cmd.jerk.y = 0.0
                cmd.jerk.z = 0.0
                yaw_target = cmd_path_yaw if self.use_network_velocity else desired_yaw
                desired_yaw_target = yaw_target if cmd_v_xy_norm > 1e-3 else desired_yaw_target
                if self.last_cmd_yaw is None:
                    desired_yaw = desired_yaw_target
                else:
                    yaw_step = self.normalize_angle(desired_yaw_target - self.last_cmd_yaw)
                    max_step = max(self.yaw_slew_rate * dt_cmd, 1e-3)
                    yaw_step = float(np.clip(yaw_step, -max_step, max_step))
                    desired_yaw = self.normalize_angle(self.last_cmd_yaw + yaw_step)

                self.last_cmd_yaw = desired_yaw
                self.last_cmd_time = cmd_now

                cmd.yaw = desired_yaw
                cmd.yaw_dot = 0.0
                cmd.kx = [2.0, 2.0, 2.5]
                cmd.kv = [1.8, 1.8, 2.5]
                cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
                self.cmd_pub.publish(cmd)

                rospy.loginfo_throttle(
                    0.5,
                    "DP cmd | goal_dist=%.2f path_vel=[%.2f %.2f %.2f] net_vel=[%.2f %.2f %.2f] cmd_vel=[%.2f %.2f %.2f] acc=[%.2f %.2f %.2f] yaw=%.2f reverse=%s blend=%.2f speed_scale=%.2f",
                    goal_dist,
                    target_v_world[0], target_v_world[1], target_v_world[2],
                    net_v_world[0], net_v_world[1], net_v_world[2],
                    cmd.velocity.x, cmd.velocity.y, cmd.velocity.z,
                    cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z,
                    cmd.yaw,
                    "true" if use_reverse_heading else "false",
                    blend if self.use_network_velocity else 0.0,
                    speed_scale,
                )
        except Exception as exc:
            if self.device.type == "cuda" and "CUDA" in str(exc):
                self.move_runtime_to_cpu(exc)
                rospy.logwarn_throttle(1.0, "DP-Planner CUDA 推理失败，下一周期将改用 CPU。")
                return
            raise


if __name__ == "__main__":
    try:
        planner = DPPlanner()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        sys.exit(0)
