#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


class SuperPlannerCompat:
    """A light-weight SUPER-style local planner for the existing RMUA stack.

    The original project already has:
    global route -> local goal publisher -> local planner -> PositionCommand controller

    SUPER also expects a local goal plus a local obstacle representation. To keep
    the current project structure stable, this node consumes the existing local
    goal and LiDAR cloud, then outputs a PositionCommand compatible with the
    controller and planner switcher.
    """

    def __init__(self):
        rospy.init_node("super_planner_compat", anonymous=True)

        self.pose_topic = rospy.get_param("~pose_topic", "/airsim_node/drone_1/debug/pose_gt")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/airsim_node/drone_1/lidar")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal_pose")
        self.fallback_goal_topic = rospy.get_param("~fallback_goal_topic", "/airsim_node/end_goal")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/dp_planner/position_cmd")

        self.replan_rate = rospy.get_param("~replan_rate", 15.0)
        self.goal_reached_dist = rospy.get_param("~goal_reached_dist", 1.2)
        self.planning_horizon = rospy.get_param("~planning_horizon", 8.0)
        self.position_lookahead = rospy.get_param("~position_lookahead", 3.0)
        self.safe_radius = rospy.get_param("~safe_radius", 1.4)
        self.clearance_margin = rospy.get_param("~clearance_margin", 0.8)
        self.max_speed = rospy.get_param("~max_speed", 10.0)
        self.min_speed = rospy.get_param("~min_speed", 2.5)
        self.max_climb_speed = rospy.get_param("~max_climb_speed", 2.5)
        self.z_p_gain = rospy.get_param("~z_p_gain", 1.2)
        self.goal_align_weight = rospy.get_param("~goal_align_weight", 2.5)
        self.clearance_weight = rospy.get_param("~clearance_weight", 4.0)
        self.cloud_max_age = rospy.get_param("~cloud_max_age", 0.35)
        self.heading_samples = max(5, int(rospy.get_param("~heading_samples", 31)))
        self.yaw_spread = math.radians(rospy.get_param("~yaw_spread_deg", 95.0))
        self.sample_stride = max(1, int(rospy.get_param("~sample_stride", 3)))
        self.vertical_gate = rospy.get_param("~vertical_gate", 1.8)
        self.emergency_stop_dist = rospy.get_param("~emergency_stop_dist", 1.2)

        self.curr_pose: Optional[PoseStamped] = None
        self.goal_pose: Optional[PoseStamped] = None
        self.fallback_goal: Optional[PoseStamped] = None
        self.cloud_xy = np.empty((0, 2), dtype=np.float32)
        self.cloud_z = np.empty((0,), dtype=np.float32)
        self.cloud_stamp = rospy.Time(0)
        self.last_cmd = None
        self.trajectory_id = 0

        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_cb, queue_size=1)
        rospy.Subscriber(self.fallback_goal_topic, PoseStamped, self.fallback_goal_cb, queue_size=1)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, PositionCommand, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.replan_rate, 1.0)), self.control_loop)

        rospy.loginfo(
            "SUPER-compatible planner ready. pose=%s cloud=%s goal=%s fallback_goal=%s cmd=%s",
            self.pose_topic,
            self.cloud_topic,
            self.goal_topic,
            self.fallback_goal_topic,
            self.cmd_topic,
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def pose_cb(self, msg: PoseStamped):
        self.curr_pose = msg

    def goal_cb(self, msg: PoseStamped):
        self.goal_pose = msg

    def fallback_goal_cb(self, msg: PoseStamped):
        self.fallback_goal = msg

    def cloud_cb(self, msg: PointCloud2):
        points = []
        for idx, point in enumerate(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if idx % self.sample_stride != 0:
                continue
            points.append((point[0], point[1], point[2]))

        if points:
            cloud = np.asarray(points, dtype=np.float32)
            self.cloud_xy = cloud[:, :2]
            self.cloud_z = cloud[:, 2]
        else:
            self.cloud_xy = np.empty((0, 2), dtype=np.float32)
            self.cloud_z = np.empty((0,), dtype=np.float32)
        self.cloud_stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()

    def active_goal(self) -> Optional[PoseStamped]:
        if self.goal_pose is not None:
            return self.goal_pose
        return self.fallback_goal

    def build_hold_cmd(self, yaw: float) -> PositionCommand:
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        cmd.position = self.curr_pose.pose.position
        cmd.velocity.x = 0.0
        cmd.velocity.y = 0.0
        cmd.velocity.z = 0.0
        cmd.acceleration.x = 0.0
        cmd.acceleration.y = 0.0
        cmd.acceleration.z = 0.0
        cmd.jerk.x = 0.0
        cmd.jerk.y = 0.0
        cmd.jerk.z = 0.0
        cmd.yaw = yaw
        cmd.yaw_dot = 0.0
        cmd.kx = [2.0, 2.0, 2.5]
        cmd.kv = [1.8, 1.8, 2.3]
        cmd.trajectory_id = self.trajectory_id
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        return cmd

    def cloud_is_fresh(self) -> bool:
        if self.cloud_stamp.is_zero():
            return False
        return (rospy.Time.now() - self.cloud_stamp).to_sec() <= self.cloud_max_age

    def score_heading(self, pos_xy: np.ndarray, heading: float, goal_heading: float, target_z: float):
        if self.cloud_xy.shape[0] == 0:
            return self.planning_horizon, self.goal_align_weight

        dir_xy = np.array([math.cos(heading), math.sin(heading)], dtype=np.float32)
        rel_xy = self.cloud_xy - pos_xy
        rel_z = np.abs(self.cloud_z - target_z)

        along = rel_xy @ dir_xy
        lateral = np.abs(rel_xy[:, 0] * dir_xy[1] - rel_xy[:, 1] * dir_xy[0])
        valid = (
            (along > 0.0)
            & (along < self.planning_horizon)
            & (lateral < self.safe_radius)
            & (rel_z < self.vertical_gate)
        )

        if not np.any(valid):
            clearance = self.planning_horizon
        else:
            clearance = float(np.min(along[valid]))

        align = math.cos(self.normalize_angle(heading - goal_heading))
        score = self.goal_align_weight * align + self.clearance_weight * (clearance / self.planning_horizon)
        if clearance < self.emergency_stop_dist:
            score -= 100.0
        return clearance, score

    def choose_heading(self, pos_xy: np.ndarray, goal_xy: np.ndarray, target_z: float):
        diff = goal_xy - pos_xy
        goal_heading = math.atan2(diff[1], diff[0])
        best_heading = goal_heading
        best_clearance = 0.0
        best_score = -1e9

        for offset in np.linspace(-self.yaw_spread, self.yaw_spread, self.heading_samples):
            heading = self.normalize_angle(goal_heading + float(offset))
            clearance, score = self.score_heading(pos_xy, heading, goal_heading, target_z)
            if score > best_score:
                best_score = score
                best_heading = heading
                best_clearance = clearance

        return goal_heading, best_heading, best_clearance

    def control_loop(self, _event):
        if self.curr_pose is None:
            return

        goal = self.active_goal()
        if goal is None:
            return

        curr = self.curr_pose.pose.position
        tgt = goal.pose.position
        pos_xy = np.array([curr.x, curr.y], dtype=np.float32)
        goal_xy = np.array([tgt.x, tgt.y], dtype=np.float32)

        goal_vec = goal_xy - pos_xy
        goal_dist_xy = float(np.linalg.norm(goal_vec))
        goal_dist_3d = math.sqrt(goal_dist_xy * goal_dist_xy + (tgt.z - curr.z) ** 2)
        if goal_dist_3d < self.goal_reached_dist:
            hold = self.build_hold_cmd(0.0 if self.last_cmd is None else self.last_cmd.yaw)
            self.cmd_pub.publish(hold)
            rospy.loginfo_throttle(1.0, "SUPER-compat goal reached, holding position.")
            return

        target_z = tgt.z
        if self.cloud_is_fresh():
            goal_heading, best_heading, best_clearance = self.choose_heading(pos_xy, goal_xy, target_z)
        else:
            goal_heading = math.atan2(goal_vec[1], goal_vec[0])
            best_heading = goal_heading
            best_clearance = self.planning_horizon

        effective_clearance = max(0.0, best_clearance - self.clearance_margin)
        speed_scale = min(1.0, effective_clearance / max(self.planning_horizon * 0.6, 1e-3))
        speed = self.min_speed + (self.max_speed - self.min_speed) * speed_scale
        speed = min(speed, max(self.min_speed, goal_dist_xy))

        if best_clearance < self.emergency_stop_dist:
            speed = 0.0

        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"

        lookahead = min(goal_dist_xy, max(self.position_lookahead, speed * 0.45))
        dir_x = math.cos(best_heading)
        dir_y = math.sin(best_heading)

        cmd.position.x = curr.x + dir_x * lookahead
        cmd.position.y = curr.y + dir_y * lookahead
        cmd.position.z = target_z
        cmd.velocity.x = dir_x * speed
        cmd.velocity.y = dir_y * speed
        cmd.velocity.z = float(np.clip((target_z - curr.z) * self.z_p_gain, -self.max_climb_speed, self.max_climb_speed))
        cmd.acceleration.x = 0.0
        cmd.acceleration.y = 0.0
        cmd.acceleration.z = 0.0
        cmd.jerk.x = 0.0
        cmd.jerk.y = 0.0
        cmd.jerk.z = 0.0
        cmd.yaw = best_heading if speed > 0.2 else goal_heading
        cmd.yaw_dot = 0.0
        cmd.vel_norm = math.sqrt(cmd.velocity.x ** 2 + cmd.velocity.y ** 2 + cmd.velocity.z ** 2)
        cmd.acc_norm = 0.0
        cmd.kx = [2.0, 2.0, 2.5]
        cmd.kv = [1.8, 1.8, 2.3]
        self.trajectory_id += 1
        cmd.trajectory_id = self.trajectory_id
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY

        if speed <= 0.01:
            cmd.position = self.curr_pose.pose.position

        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)

        rospy.loginfo_throttle(
            0.5,
            "SUPER-compat cmd | goal_xy=%.2f heading(goal=%.2f best=%.2f) clearance=%.2f speed=%.2f pos=(%.2f, %.2f, %.2f)",
            goal_dist_xy,
            goal_heading,
            best_heading,
            best_clearance,
            speed,
            cmd.position.x,
            cmd.position.y,
            cmd.position.z,
        )


if __name__ == "__main__":
    try:
        SuperPlannerCompat()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
