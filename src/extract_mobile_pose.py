#!/usr/bin/env python

import rosbag
import rospy
import tf2_ros
import tf2_py
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage


def extract_map_base_pose(bagfile):

    tf_buffer = tf2_ros.Buffer()
    poses = []

    with rosbag.Bag(bagfile) as bag:

        # --- load static transforms first ---
        for topic, msg, t in bag.read_messages(topics=["/tf_static"]):
            for transform in msg.transforms:
                tf_buffer.set_transform_static(transform, "bag")

        # --- now stream dynamic tf ---
        for topic, msg, t in bag.read_messages(topics=["/tf"]):

            for transform in msg.transforms:
                tf_buffer.set_transform(transform, "bag")

            # try to compute map -> base_link
            try:
                transformStamped = tf_buffer.lookup_transform(
                    "map",
                    "base_link",
                    msg.transforms[0].header.stamp
                )

                x = transformStamped.transform.translation.x
                y = transformStamped.transform.translation.y

                q = transformStamped.transform.rotation
                quat = [q.x, q.y, q.z, q.w]
                (_, _, yaw) = euler_from_quaternion(quat)

                poses.append({
                    "t": transformStamped.header.stamp.to_sec(),
                    "x": x,
                    "y": y,
                    "theta": yaw
                })

            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                pass

    return poses


if __name__ == "__main__":

    bag_file = "/home/docker/ros_ws/data/KATE_AA_Video/KATE_AA_U099_10_green_line_force_both_80-1_2025-08-22-12-09-03.bag"

    rospy.init_node("bag_tf_extractor")

    poses = extract_map_base_pose(bag_file)

    for p in poses:
        print(p)
