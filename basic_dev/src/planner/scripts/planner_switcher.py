#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String


class PlannerSwitcher:
    STATE_FOLLOW_PATH = "FOLLOW_PATH"
    STATE_USE_DP = "USE_DP"

    def __init__(self):
        rospy.init_node("planner_switcher", anonymous=True)

        self.path_topic = rospy.get_param("~path_topic", "/drone_1/saved_path")
        self.pose_topic = rospy.get_param("~pose_topic", "/airsim_node/drone_1/debug/pose_gt")
        self.path_cmd_topic = rospy.get_param("~path_cmd_topic", "/path_follower/position_cmd")
        self.dp_cmd_topic = rospy.get_param("~dp_cmd_topic", "/dp_planner/position_cmd")
        self.output_cmd_topic = rospy.get_param("~output_cmd_topic", "/position_cmd")
        self.state_topic = rospy.get_param("~state_topic", "/planner_switch/state")

        self.forward_search_window = rospy.get_param("~forward_search_window", 80)
        self.fixed_turnaround_activate_dist = rospy.get_param("~fixed_turnaround_activate_dist", 5.0)
        self.fixed_turnaround_trigger_ahead_points = rospy.get_param("~fixed_turnaround_trigger_ahead_points", 80)
        self.switch_confirm_margin_points = rospy.get_param("~switch_confirm_margin_points", 1)

        default_points = [
            {"x": 546.482, "y": 521.007},
            {"x": 1170.496, "y": -423.165},
            {"x": 714.613, "y": -722.970},
            {"x": 652.462, "y": -172.698},
            {"x": 267.311, "y": 393.279},
            {"x": 1321.899, "y": 151.379},
        ]
        self.fixed_turnaround_points = rospy.get_param("~fixed_turnaround_points", default_points)

        self.current_path = None
        self.curr_pose = None
        self.progress_idx = -1

        self.turnaround_indices = []
        self.first_turnaround_idx = -1
        self.first_trigger_idx = -1
        self.first_turn_detected = False
        self.state = self.STATE_FOLLOW_PATH

        self.latest_path_cmd = None
        self.latest_dp_cmd = None

        rospy.Subscriber(self.path_topic, Path, self.path_cb)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_cb)
        rospy.Subscriber(self.path_cmd_topic, PositionCommand, self.path_cmd_cb)
        rospy.Subscriber(self.dp_cmd_topic, PositionCommand, self.dp_cmd_cb)

        self.cmd_pub = rospy.Publisher(self.output_cmd_topic, PositionCommand, queue_size=1)
        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=1, latch=True)

        self.publish_state(force=True)
        rospy.loginfo("PlannerSwitcher initialized. path_cmd=%s dp_cmd=%s output=%s",
                      self.path_cmd_topic, self.dp_cmd_topic, self.output_cmd_topic)

    @staticmethod
    def horizontal_dist(p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        return math.sqrt(dx * dx + dy * dy)

    def path_cb(self, msg):
        self.current_path = msg
        self.refresh_turnaround_indices()
        self.progress_idx = -1
        self.first_turn_detected = False
        if self.state != self.STATE_USE_DP:
            self.state = self.STATE_FOLLOW_PATH
            self.publish_state(force=True)
        rospy.loginfo("PlannerSwitcher received path with %d poses. first_turnaround_idx=%d trigger_idx=%d",
                      len(msg.poses), self.first_turnaround_idx, self.first_trigger_idx)

    def refresh_turnaround_indices(self):
        self.turnaround_indices = []
        self.first_turnaround_idx = -1
        self.first_trigger_idx = -1

        if self.current_path is None or not self.current_path.poses:
            return

        for point in self.fixed_turnaround_points:
            best_idx = -1
            best_dist = 1e9
            for i, pose_stamped in enumerate(self.current_path.poses):
                pos = pose_stamped.pose.position
                dx = pos.x - float(point["x"])
                dy = pos.y - float(point["y"])
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i

            if best_idx >= 0:
                trigger_idx = max(0, best_idx - self.fixed_turnaround_trigger_ahead_points)
                self.turnaround_indices.append({
                    "path_idx": best_idx,
                    "trigger_idx": trigger_idx,
                })

        self.turnaround_indices.sort(key=lambda item: item["path_idx"])
        if self.turnaround_indices:
            self.first_turnaround_idx = self.turnaround_indices[0]["path_idx"]
            self.first_trigger_idx = self.turnaround_indices[0]["trigger_idx"]

    def pose_cb(self, msg):
        self.curr_pose = msg
        if self.current_path is None or not self.current_path.poses:
            return

        nearest_idx = self.find_nearest_progress_idx(msg)
        self.progress_idx = nearest_idx
        self.update_state_machine()

    def find_nearest_progress_idx(self, pose_msg):
        path_size = len(self.current_path.poses)
        if path_size == 0:
            return -1

        if self.progress_idx < 0:
            best_idx = 0
            best_dist = 1e9
            for i, pose_stamped in enumerate(self.current_path.poses):
                dist = self.horizontal_dist(pose_msg.pose.position, pose_stamped.pose.position)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
            return best_idx

        nearest_idx = self.progress_idx
        min_dist = self.horizontal_dist(
            pose_msg.pose.position, self.current_path.poses[self.progress_idx].pose.position
        )
        search_end = min(self.progress_idx + self.forward_search_window, path_size - 1)
        for i in range(self.progress_idx, search_end + 1):
            dist = self.horizontal_dist(pose_msg.pose.position, self.current_path.poses[i].pose.position)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        return nearest_idx

    def update_state_machine(self):
        if self.state == self.STATE_USE_DP:
            return
        if self.first_turnaround_idx < 0 or self.first_trigger_idx < 0 or self.curr_pose is None:
            return

        trigger_pose = self.current_path.poses[self.first_trigger_idx].pose.position
        trigger_dist = self.horizontal_dist(self.curr_pose.pose.position, trigger_pose)

        if not self.first_turn_detected:
            if trigger_dist <= self.fixed_turnaround_activate_dist or self.progress_idx >= self.first_trigger_idx:
                self.first_turn_detected = True
                rospy.loginfo("PlannerSwitcher detected first turnaround. progress_idx=%d turnaround_idx=%d trigger_dist=%.2f",
                              self.progress_idx, self.first_turnaround_idx, trigger_dist)

        if self.first_turn_detected and self.progress_idx > self.first_turnaround_idx + self.switch_confirm_margin_points:
            self.state = self.STATE_USE_DP
            self.publish_state(force=True)
            rospy.loginfo("PlannerSwitcher switched to DP planner. progress_idx=%d first_turnaround_idx=%d",
                          self.progress_idx, self.first_turnaround_idx)
            if self.latest_dp_cmd is not None:
                self.cmd_pub.publish(self.latest_dp_cmd)

    def publish_state(self, force=False):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)
        if force:
            rospy.loginfo("PlannerSwitcher state=%s", self.state)

    def path_cmd_cb(self, msg):
        self.latest_path_cmd = msg
        if self.state == self.STATE_FOLLOW_PATH:
            self.cmd_pub.publish(msg)

    def dp_cmd_cb(self, msg):
        self.latest_dp_cmd = msg
        if self.state == self.STATE_USE_DP:
            self.cmd_pub.publish(msg)


if __name__ == "__main__":
    PlannerSwitcher()
    rospy.spin()
