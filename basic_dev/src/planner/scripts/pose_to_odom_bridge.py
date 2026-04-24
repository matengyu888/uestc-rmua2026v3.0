#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class PoseToOdomBridge:
    def __init__(self):
        rospy.init_node("pose_to_odom_bridge", anonymous=True)

        self.pose_topic = rospy.get_param("~pose_topic", "/airsim_node/drone_1/debug/pose_gt")
        self.odom_topic = rospy.get_param("~odom_topic", "/lidar_slam/odom")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "drone")
        self.z_sign = rospy.get_param("~z_sign", -1.0)

        self.last_pose = None
        self.last_stamp = None

        self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_cb, queue_size=10)

        rospy.loginfo("PoseToOdomBridge started. pose=%s odom=%s", self.pose_topic, self.odom_topic)

    def pose_cb(self, msg: PoseStamped):
        stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose = msg.pose
        odom.pose.pose.position.z *= self.z_sign

        q = odom.pose.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm > 1e-6:
            q.x /= norm
            q.y /= norm
            q.z /= norm
            q.w /= norm

        if self.last_pose is not None and self.last_stamp is not None:
            dt = max((stamp - self.last_stamp).to_sec(), 1e-3)
            odom.twist.twist.linear.x = (msg.pose.position.x - self.last_pose.position.x) / dt
            odom.twist.twist.linear.y = (msg.pose.position.y - self.last_pose.position.y) / dt
            odom.twist.twist.linear.z = self.z_sign * (msg.pose.position.z - self.last_pose.position.z) / dt
        else:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0

        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        if self.last_pose is not None and self.last_stamp is not None:
            last_q = self.last_pose.orientation
            last_siny_cosp = 2.0 * (last_q.w * last_q.z + last_q.x * last_q.y)
            last_cosy_cosp = 1.0 - 2.0 * (last_q.y * last_q.y + last_q.z * last_q.z)
            last_yaw = math.atan2(last_siny_cosp, last_cosy_cosp)
            dt = max((stamp - self.last_stamp).to_sec(), 1e-3)
            yaw_err = math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
            odom.twist.twist.angular.z = yaw_err / dt

        self.pub.publish(odom)
        self.last_pose = msg.pose
        self.last_stamp = stamp


if __name__ == "__main__":
    try:
        PoseToOdomBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
