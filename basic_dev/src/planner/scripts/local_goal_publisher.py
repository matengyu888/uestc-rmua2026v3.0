#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path


class LocalGoalPublisher:
    def __init__(self):
        rospy.init_node("local_goal_publisher", anonymous=True)

        self.path_topic = rospy.get_param("~path_topic", "/drone_1/saved_path")
        self.pose_topic = rospy.get_param("~pose_topic", "/airsim_node/drone_1/debug/pose_gt")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal_pose")
        self.look_ahead_distance = rospy.get_param("~look_ahead_distance", 8.0)
        self.forward_search_window = rospy.get_param("~forward_search_window", 120)
        self.publish_rate = rospy.get_param("~publish_rate", 10.0)
        self.goal_republish_dist = rospy.get_param("~goal_republish_dist", 0.5)

        self.current_path = None
        self.current_pose = None
        self.progress_idx = -1
        self.last_goal = None

        rospy.Subscriber(self.path_topic, Path, self.path_cb, queue_size=1)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_cb, queue_size=1)
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = rospy.Timer(rospy.Duration(period), self.publish_goal)

        rospy.loginfo(
            "LocalGoalPublisher started. path=%s pose=%s goal=%s look_ahead=%.2f",
            self.path_topic,
            self.pose_topic,
            self.goal_topic,
            self.look_ahead_distance,
        )

    @staticmethod
    def horizontal_dist(p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        return math.sqrt(dx * dx + dy * dy)

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

        path_size = len(self.current_path.poses)
        curr_pt = self.current_pose.pose.position

        if self.progress_idx < 0:
            best_idx = 0
            best_dist = float("inf")
            for i, pose in enumerate(self.current_path.poses):
                d = self.horizontal_dist(curr_pt, pose.pose.position)
                if d < best_dist:
                    best_dist = d
                    best_idx = i
            self.progress_idx = best_idx
            return best_idx

        best_idx = self.progress_idx
        best_dist = self.horizontal_dist(curr_pt, self.current_path.poses[self.progress_idx].pose.position)
        search_end = min(self.progress_idx + self.forward_search_window, path_size - 1)
        for i in range(self.progress_idx, search_end + 1):
            d = self.horizontal_dist(curr_pt, self.current_path.poses[i].pose.position)
            if d < best_dist:
                best_dist = d
                best_idx = i

        self.progress_idx = best_idx
        return best_idx

    def find_target_idx(self, nearest_idx):
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
            if accum >= self.look_ahead_distance:
                break
        return target_idx

    def build_goal(self, target_idx):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.current_path.header.frame_id or "world"
        goal.pose.position = self.current_path.poses[target_idx].pose.position

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
        return self.horizontal_dist(goal.pose.position, self.last_goal.pose.position) >= self.goal_republish_dist

    def publish_goal(self, _event):
        if self.current_path is None or not self.current_path.poses or self.current_pose is None:
            return

        nearest_idx = self.find_nearest_idx()
        if nearest_idx is None:
            return

        target_idx = self.find_target_idx(nearest_idx)
        goal = self.build_goal(target_idx)

        if self.should_publish(goal):
            self.goal_pub.publish(goal)
            self.last_goal = goal
            rospy.loginfo_throttle(
                0.5,
                "Local goal published: nearest=%d target=%d pos=(%.2f, %.2f, %.2f)",
                nearest_idx,
                target_idx,
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
