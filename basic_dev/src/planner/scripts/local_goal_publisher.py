#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path


class LocalGoalPublisher:
    @staticmethod
    def get_float_param(name, default):
        value = rospy.get_param(name, default)
        try:
            return float(value)
        except (TypeError, ValueError):
            rospy.logwarn("Invalid float param %s=%r, fallback to %r", name, value, default)
            return float(default)

    @staticmethod
    def get_int_param(name, default):
        value = rospy.get_param(name, default)
        try:
            return int(value)
        except (TypeError, ValueError):
            rospy.logwarn("Invalid int param %s=%r, fallback to %r", name, value, default)
            return int(default)

    def __init__(self):
        rospy.init_node("local_goal_publisher", anonymous=True)

        self.path_topic = rospy.get_param("~path_topic", "/drone_1/saved_path")
        self.pose_topic = rospy.get_param("~pose_topic", "/airsim_node/drone_1/debug/pose_gt")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal_pose")
        self.look_ahead_distance = self.get_float_param("~look_ahead_distance", 8.0)
        self.min_look_ahead_distance = self.get_float_param("~min_look_ahead_distance", self.look_ahead_distance)
        self.max_look_ahead_distance = self.get_float_param("~max_look_ahead_distance", self.look_ahead_distance)
        self.look_ahead_expand_start_idx = self.get_int_param("~look_ahead_expand_start_idx", 0)
        self.look_ahead_expand_end_idx = self.get_int_param("~look_ahead_expand_end_idx", 0)
        self.forward_search_window = self.get_int_param("~forward_search_window", 120)
        self.backward_search_window = self.get_int_param("~backward_search_window", 40)
        self.relocalize_forward_window = self.get_int_param("~relocalize_forward_window", 200)
        self.max_forward_jump = self.get_int_param("~max_forward_jump", 12)
        self.publish_rate = self.get_float_param("~publish_rate", 10.0)
        self.goal_republish_dist = self.get_float_param("~goal_republish_dist", 0.5)
        self.max_republish_interval = self.get_float_param("~max_republish_interval", 1.0)
        self.goal_z_sign = self.get_float_param("~goal_z_sign", 1.0)
        self.pose_z_sign = self.get_float_param("~pose_z_sign", 1.0)
        self.relocalize_dist = self.get_float_param("~relocalize_dist", 1.5)
        self.global_relocalize = rospy.get_param("~global_relocalize", False)
        self.max_backward_jump = self.get_int_param("~max_backward_jump", 0)
        self.look_ahead_shrink_dist = self.get_float_param("~look_ahead_shrink_dist", 1.0)
        self.look_ahead_shrink_z_dist = self.get_float_param("~look_ahead_shrink_z_dist", float("inf"))
        self.max_goal_z = self.get_float_param("~max_goal_z", float("inf"))
        self.max_goal_z_step_up = self.get_float_param("~max_goal_z_step_up", float("inf"))
        self.max_goal_z_step_down = self.get_float_param("~max_goal_z_step_down", float("inf"))

        self.current_path = None
        self.current_pose = None
        self.progress_idx = -1
        self.last_goal = None
        self.last_publish_time = rospy.Time(0)

        rospy.Subscriber(self.path_topic, Path, self.path_cb, queue_size=1)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_cb, queue_size=1)
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = rospy.Timer(rospy.Duration(period), self.publish_goal)

        rospy.loginfo(
            "LocalGoalPublisher started. path=%s pose=%s goal=%s look_ahead=%.2f z_sign=%.2f",
            self.path_topic,
            self.pose_topic,
            self.goal_topic,
            self.look_ahead_distance,
            self.goal_z_sign,
        )

    @staticmethod
    def horizontal_dist(p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        return math.sqrt(dx * dx + dy * dy)

    def pose_z_up(self, pos):
        return pos.z * self.pose_z_sign

    @staticmethod
    def yaw_to_quaternion(yaw):
        half = 0.5 * yaw
        q = Quaternion()
        q.w = math.cos(half)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half)
        return q

    def path_cb(self, msg):
        old_size = len(self.current_path.poses) if self.current_path is not None else 0
        self.current_path = msg
        if len(msg.poses) != old_size:
            self.progress_idx = -1
        rospy.loginfo_throttle(1.0, "Local goal path updated: %d poses", len(msg.poses))

    def pose_cb(self, msg):
        self.current_pose = msg

    def find_nearest_idx(self):
        if self.current_path is None or not self.current_path.poses or self.current_pose is None:
            return None

        curr_pt = self.current_pose.pose.position
        path_size = len(self.current_path.poses)

        if self.progress_idx < 0:
            start_idx = 0
            end_idx = path_size - 1
        else:
            start_idx = max(0, self.progress_idx - self.backward_search_window)
            end_idx = min(path_size - 1, self.progress_idx + self.forward_search_window)

        best_idx = start_idx
        best_dist = float("inf")
        for i in range(start_idx, end_idx + 1):
            d = self.horizontal_dist(curr_pt, self.current_path.poses[i].pose.position)
            if d < best_dist:
                best_dist = d
                best_idx = i

        if best_dist > self.relocalize_dist and (start_idx > 0 or end_idx < path_size - 1):
            if self.global_relocalize:
                relocalize_start = 0
                relocalize_end = path_size - 1
            else:
                relocalize_start = max(0, self.progress_idx - self.max_backward_jump)
                relocalize_end = min(path_size - 1, self.progress_idx + self.relocalize_forward_window)

            relocalize_idx = relocalize_start
            relocalize_dist = float("inf")
            for i in range(relocalize_start, relocalize_end + 1):
                d = self.horizontal_dist(curr_pt, self.current_path.poses[i].pose.position)
                if d < relocalize_dist:
                    relocalize_dist = d
                    relocalize_idx = i

            best_idx = relocalize_idx
            best_dist = relocalize_dist
            rospy.logwarn_throttle(
                1.0,
                "Local goal relocalized: progress=%d -> nearest=%d dist=%.2f scope=%s[%d,%d]",
                self.progress_idx,
                best_idx,
                best_dist,
                "global" if self.global_relocalize else "forward",
                relocalize_start,
                relocalize_end,
            )

        if self.progress_idx >= 0 and best_idx < self.progress_idx - self.max_backward_jump:
            rospy.logwarn_throttle(
                1.0,
                "Local goal clamped backward jump: progress=%d nearest=%d max_back=%d",
                self.progress_idx,
                best_idx,
                self.max_backward_jump,
            )
            best_idx = max(0, self.progress_idx - self.max_backward_jump)

        if self.progress_idx >= 0 and best_idx > self.progress_idx + self.max_forward_jump:
            rospy.logwarn_throttle(
                1.0,
                "Local goal clamped forward jump: progress=%d nearest=%d max_fwd=%d",
                self.progress_idx,
                best_idx,
                self.max_forward_jump,
            )
            best_idx = self.progress_idx + self.max_forward_jump

        self.progress_idx = best_idx
        return best_idx

    def compute_look_ahead_distance(self, nearest_idx):
        base = self.look_ahead_distance
        min_lh = min(self.min_look_ahead_distance, self.max_look_ahead_distance)
        max_lh = max(self.min_look_ahead_distance, self.max_look_ahead_distance)

        if self.look_ahead_expand_end_idx > self.look_ahead_expand_start_idx:
            ratio = (nearest_idx - self.look_ahead_expand_start_idx) / max(
                self.look_ahead_expand_end_idx - self.look_ahead_expand_start_idx,
                1,
            )
            ratio = min(max(ratio, 0.0), 1.0)
            base = min_lh + (max_lh - min_lh) * ratio

        if self.progress_idx >= 0:
            curr_pt = self.current_pose.pose.position
            nearest_pt = self.current_path.poses[nearest_idx].pose.position
            lateral_dist = self.horizontal_dist(curr_pt, nearest_pt)
            if lateral_dist > self.look_ahead_shrink_dist:
                shrink_ratio = min(
                    lateral_dist / max(self.look_ahead_shrink_dist, 1e-3),
                    2.0,
                )
                base = max(min_lh, base / shrink_ratio)

            if math.isfinite(self.look_ahead_shrink_z_dist):
                z_err = abs(self.pose_z_up(curr_pt) - nearest_pt.z * self.goal_z_sign)
                if z_err > self.look_ahead_shrink_z_dist:
                    shrink_ratio = min(
                        z_err / max(self.look_ahead_shrink_z_dist, 1e-3),
                        2.0,
                    )
                    base = max(min_lh, base / shrink_ratio)

        return min(max(base, min_lh), max_lh)

    def find_target_idx(self, nearest_idx, look_ahead_distance):
        path_size = len(self.current_path.poses)
        if nearest_idx >= path_size - 1:
            return path_size - 1

        accum = 0.0
        target_idx = nearest_idx
        for i in range(nearest_idx, path_size - 1):
            p0 = self.current_path.poses[i].pose.position
            p1 = self.current_path.poses[i + 1].pose.position
            accum += self.horizontal_dist(p0, p1)
            target_idx = i + 1
            if accum >= look_ahead_distance:
                break
        return target_idx

    def build_goal(self, target_idx):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.current_path.header.frame_id or "world"
        src_pos = self.current_path.poses[target_idx].pose.position
        goal.pose.position.x = src_pos.x
        goal.pose.position.y = src_pos.y
        goal.pose.position.z = src_pos.z * self.goal_z_sign
        curr_z = self.pose_z_up(self.current_pose.pose.position)
        if math.isfinite(self.max_goal_z_step_up):
            goal.pose.position.z = min(goal.pose.position.z, curr_z + self.max_goal_z_step_up)
        if math.isfinite(self.max_goal_z_step_down):
            goal.pose.position.z = max(goal.pose.position.z, curr_z - self.max_goal_z_step_down)
        if math.isfinite(self.max_goal_z):
            goal.pose.position.z = min(goal.pose.position.z, self.max_goal_z)

        if target_idx < len(self.current_path.poses) - 1:
            p0 = self.current_path.poses[target_idx].pose.position
            p1 = self.current_path.poses[target_idx + 1].pose.position
        elif target_idx > 0:
            p0 = self.current_path.poses[target_idx - 1].pose.position
            p1 = self.current_path.poses[target_idx].pose.position
        else:
            p0 = self.current_path.poses[target_idx].pose.position
            p1 = self.current_path.poses[target_idx].pose.position

        dx = p1.x - p0.x
        dy = p1.y - p0.y
        yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-6 else 0.0
        goal.pose.orientation = self.yaw_to_quaternion(yaw)
        return goal

    def should_publish(self, goal):
        if self.last_goal is None:
            return True
        moved = self.horizontal_dist(goal.pose.position, self.last_goal.pose.position) >= self.goal_republish_dist
        stale = (rospy.Time.now() - self.last_publish_time).to_sec() >= self.max_republish_interval
        return moved or stale

    def publish_goal(self, _event):
        if self.current_path is None or not self.current_path.poses or self.current_pose is None:
            return

        nearest_idx = self.find_nearest_idx()
        if nearest_idx is None:
            return

        look_ahead_distance = self.compute_look_ahead_distance(nearest_idx)
        target_idx = self.find_target_idx(nearest_idx, look_ahead_distance)
        goal = self.build_goal(target_idx)

        if self.should_publish(goal):
            self.goal_pub.publish(goal)
            self.last_goal = goal
            self.last_publish_time = rospy.Time.now()
            rospy.loginfo_throttle(
                0.5,
                "Local goal published: nearest=%d target=%d look_ahead=%.2f pos=(%.2f, %.2f, %.2f)",
                nearest_idx,
                target_idx,
                look_ahead_distance,
                goal.pose.position.x,
                goal.pose.position.y,
                goal.pose.position.z,
            )


if __name__ == "__main__":
    try:
        LocalGoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
