#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped
from ipr_helpers.msg import Pose2DStamped

class PublishTransformFromTopic:
    def __init__(self):

        # topic_name = rospy.get_param('/gait_estimation/topic_name', True)
        self.sub_pose = rospy.Subscriber("/mobile_robot_pose", Pose2DStamped, self.listen_pose)
        self.br = tf.TransformBroadcaster()


    # publish transform from topic
    # If we are reading from a bag, we should publish the map to base_link transform
    def listen_pose(self, data):

        # Pose2DStamped give the angle theta around the z-axis
        q = tf.transformations.quaternion_about_axis(data.pose.theta, (0, 0, 1))

        t = TransformStamped()
        t.header.stamp = data.header.stamp
        t.header.frame_id = data.header.frame_id # Should be 'map' frame
        # TODO:(Andreas) Eigentlich ist der tf tree map -> base_footprint -> base_link
        # Hier wird mit absicht der tf tree unterbrochen damit sowohl die Raeder positionen (werden mit aktueller Zeit neu gepublished) sowie auch die aufgenommenen Daten (laserscan, depthcamera) in rviz richtig dargestellt werden.
        # Das sollte langfristig behoben werden indem der tf tree beim bag file aufgenommen wird.
        t.child_frame_id = 'base_link' # base_footprint
        t.transform.translation.x = data.pose.x
        t.transform.translation.y = data.pose.y
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransformMessage(t)

if __name__ == '__main__':
    rospy.init_node('publish_transform_from_topic', anonymous=True)
    rospy.get_rostime()
    rospy.get_time()

    node = PublishTransformFromTopic()

    rospy.loginfo("Start tf publisher from topic '/mobile_robot_pose' to frame 'map' -> 'base_link'")

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass