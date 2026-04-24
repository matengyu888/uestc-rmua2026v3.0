#!/usr/bin/env python3

import math

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


class LidarFilterBridge:
    def __init__(self):
        rospy.init_node("lidar_filter_bridge", anonymous=True)

        self.input_topic = rospy.get_param("~input_topic", "/airsim_node/drone_1/lidar")
        self.output_topic = rospy.get_param("~output_topic", "/super/lidar_filtered")
        self.min_radius = rospy.get_param("~min_radius", 1.0)
        self.min_z = rospy.get_param("~min_z", -100.0)
        self.max_z = rospy.get_param("~max_z", 100.0)

        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        rospy.Subscriber(self.input_topic, PointCloud2, self.cloud_cb, queue_size=1)

        rospy.loginfo(
            "LidarFilterBridge started. input=%s output=%s min_radius=%.2f",
            self.input_topic,
            self.output_topic,
            self.min_radius,
        )

    def cloud_cb(self, msg: PointCloud2):
        kept_points = []
        radius_sq = self.min_radius * self.min_radius
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            if z < self.min_z or z > self.max_z:
                continue
            if x * x + y * y + z * z < radius_sq:
                continue
            kept_points.append((x, y, z))

        cloud = pc2.create_cloud_xyz32(msg.header, kept_points)
        self.pub.publish(cloud)


if __name__ == "__main__":
    try:
        LidarFilterBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
