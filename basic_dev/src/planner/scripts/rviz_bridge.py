#!/usr/bin/env python3

import math
from collections import deque

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class RvizBridge:
    def __init__(self):
        rospy.init_node("dp_rviz_bridge", anonymous=True)

        self.world_frame = rospy.get_param("~world_frame", "world")
        self.base_frame = rospy.get_param("~base_frame", "uav_base_link")
        self.depth_frame = rospy.get_param("~depth_frame", self.base_frame)

        self.pose_topic = rospy.get_param("~pose_topic", "/airsim_node/drone_1/debug/pose_gt")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/position_cmd")
        self.depth_topic = rospy.get_param("~depth_topic", "/airsim_node/drone_1/front_stereo/DepthPerspective")
        self.left_info_topic = rospy.get_param(
            "~left_info_topic", "/airsim_node/drone_1/front_left/Scene/camera_info"
        )

        self.command_path_topic = rospy.get_param("~command_path_topic", "/dp_rviz/command_path")
        self.pose_path_topic = rospy.get_param("~pose_path_topic", "/dp_rviz/pose_path")
        self.obstacle_cloud_topic = rospy.get_param("~obstacle_cloud_topic", "/dp_rviz/obstacle_cloud")
        self.depth_viz_topic = rospy.get_param("~depth_viz_topic", "/dp_rviz/depth_viz")

        self.path_history_size = rospy.get_param("~path_history_size", 400)
        self.depth_sample_step = rospy.get_param("~depth_sample_step", 6)
        self.min_depth_m = rospy.get_param("~min_depth_m", 0.4)
        self.max_depth_m = rospy.get_param("~max_depth_m", 18.0)
        self.depth_viz_max_m = rospy.get_param("~depth_viz_max_m", 8.0)
        self.depth_viz_gamma = rospy.get_param("~depth_viz_gamma", 0.7)
        self.fallback_width = rospy.get_param("~fallback_width", 960)
        self.fallback_height = rospy.get_param("~fallback_height", 720)
        self.fallback_fov_deg = rospy.get_param("~fallback_fov_deg", 60.0)

        self.pose_path = deque(maxlen=max(10, int(self.path_history_size)))
        self.last_pose = None
        self.camera_info = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.pose_path_pub = rospy.Publisher(self.pose_path_topic, Path, queue_size=1)
        self.command_path_pub = rospy.Publisher(self.command_path_topic, Path, queue_size=1)
        self.obstacle_cloud_pub = rospy.Publisher(self.obstacle_cloud_topic, PointCloud2, queue_size=1)
        self.depth_viz_pub = rospy.Publisher(self.depth_viz_topic, Image, queue_size=1)

        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber(self.cmd_topic, PositionCommand, self.cmd_cb, queue_size=10)
        rospy.Subscriber(self.left_info_topic, CameraInfo, self.camera_info_cb, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=1)

        rospy.loginfo(
            "dp_rviz_bridge ready pose=%s cmd=%s depth=%s cloud=%s",
            self.pose_topic,
            self.cmd_topic,
            self.depth_topic,
            self.obstacle_cloud_topic,
        )

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def pose_cb(self, msg):
        self.last_pose = msg

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = msg.pose.position.x
        tf_msg.transform.translation.y = msg.pose.position.y
        tf_msg.transform.translation.z = msg.pose.position.z
        tf_msg.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        pose = PoseStamped()
        pose.header = tf_msg.header
        pose.pose = msg.pose
        self.pose_path.append(pose)
        self.publish_path(self.pose_path_pub, self.pose_path)

    def cmd_cb(self, msg):
        path = Path()
        path.header.stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
        path.header.frame_id = self.world_frame

        if self.last_pose is not None:
            start_pose = PoseStamped()
            start_pose.header = path.header
            start_pose.pose = self.last_pose.pose
            path.poses.append(start_pose)

        target_pose = PoseStamped()
        target_pose.header = path.header
        target_pose.pose.position.x = msg.position.x
        target_pose.pose.position.y = msg.position.y
        target_pose.pose.position.z = msg.position.z
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, msg.yaw)
        target_pose.pose.orientation.x = quat[0]
        target_pose.pose.orientation.y = quat[1]
        target_pose.pose.orientation.z = quat[2]
        target_pose.pose.orientation.w = quat[3]
        path.poses.append(target_pose)

        self.command_path_pub.publish(path)

    def publish_path(self, publisher, poses):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.world_frame
        path.poses = list(poses)
        publisher.publish(path)

    def resolve_intrinsics(self, width, height):
        if self.camera_info and self.camera_info.K[0] > 1e-6 and self.camera_info.K[4] > 1e-6:
            scale_x = float(width) / max(float(self.camera_info.width), 1.0)
            scale_y = float(height) / max(float(self.camera_info.height), 1.0)
            fx = self.camera_info.K[0] * scale_x
            fy = self.camera_info.K[4] * scale_y
            cx = self.camera_info.K[2] * scale_x
            cy = self.camera_info.K[5] * scale_y
            return fx, fy, cx, cy

        fov_rad = math.radians(self.fallback_fov_deg)
        fx = float(width) / (2.0 * math.tan(max(fov_rad * 0.5, 1e-3)))
        fy = fx
        cx = float(width) * 0.5
        cy = float(height) * 0.5
        return fx, fy, cx, cy

    def depth_cb(self, msg):
        if msg.encoding not in ("32FC1", "16UC1"):
            rospy.logwarn_throttle(2.0, "dp_rviz_bridge unsupported depth encoding: %s", msg.encoding)
            return

        width = msg.width
        height = msg.height
        fx, fy, cx, cy = self.resolve_intrinsics(width, height)

        if msg.encoding == "32FC1":
            import struct

            unpacked = struct.iter_unpack("<f", msg.data)
            depths = [v[0] for v in unpacked]
        else:
            import struct

            unpacked = struct.iter_unpack("<H", msg.data)
            depths = [v[0] * 0.001 for v in unpacked]

        viz_max = max(self.min_depth_m + 1e-3, min(self.depth_viz_max_m, self.max_depth_m))
        depth_range = max(viz_max - self.min_depth_m, 1e-3)
        viz_data = bytearray(width * height)
        points = []
        step = max(1, int(self.depth_sample_step))
        for v in range(height):
            row_offset = v * width
            for u in range(width):
                depth = depths[row_offset + u]
                if math.isfinite(depth) and self.min_depth_m <= depth <= self.max_depth_m:
                    clipped_depth = min(depth, viz_max)
                    norm = (clipped_depth - self.min_depth_m) / depth_range
                    norm = math.pow(max(0.0, min(1.0, norm)), self.depth_viz_gamma)
                    viz_data[row_offset + u] = int(max(0.0, min(1.0, norm)) * 255.0)
                else:
                    viz_data[row_offset + u] = 255

                if v % step != 0 or u % step != 0:
                    continue
                if not math.isfinite(depth) or depth < self.min_depth_m or depth > self.max_depth_m:
                    continue
                x = depth
                y = (u - cx) * depth / max(fx, 1e-6)
                z = (v - cy) * depth / max(fy, 1e-6)
                points.append((x, y, z))

        depth_viz = Image()
        depth_viz.header = msg.header
        if not depth_viz.header.frame_id:
            depth_viz.header.frame_id = self.depth_frame
        depth_viz.height = height
        depth_viz.width = width
        depth_viz.encoding = "mono8"
        depth_viz.is_bigendian = 0
        depth_viz.step = width
        depth_viz.data = bytes(viz_data)
        self.depth_viz_pub.publish(depth_viz)

        header = msg.header
        if not header.frame_id:
            header.frame_id = self.depth_frame
        cloud = pc2.create_cloud_xyz32(header, points)
        self.obstacle_cloud_pub.publish(cloud)
        rospy.loginfo_throttle(1.0, "dp_rviz_bridge obstacle points=%d", len(points))


if __name__ == "__main__":
    try:
        RvizBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
