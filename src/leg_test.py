#!/usr/bin/env python

from __future__ import division
import rospy
import numpy as np
from leg_tracker.msg import PersonMsg, LegMsg
from geometry_msgs.msg import WrenchStamped, Twist,PointStamped, PolygonStamped, Point32
import tf2_ros
import tf2_geometry_msgs

pub = rospy.Publisher('/leg_detection/people_msg_stamped', PersonMsg,tcp_nodelay=True, queue_size=1024)
pub_f = rospy.Publisher('/base/sensor_data', WrenchStamped,tcp_nodelay=True, queue_size=1024)
pub_tf = rospy.Publisher('/base/transformed_data', WrenchStamped,tcp_nodelay=True, queue_size=1024)
pub_v = rospy.Publisher("/base/fts_controller/fts_command",Twist, tcp_nodelay=True, queue_size=1024)
pub_tr = rospy.Publisher("/right_toe",PointStamped, tcp_nodelay=True, queue_size=1024)
pub_tl = rospy.Publisher("/left_toe",PointStamped, tcp_nodelay=True, queue_size=1024)
pub_sh = rospy.Publisher("/human_body_detection/points",PolygonStamped, tcp_nodelay=True, queue_size=1024)
global last_shift
mode = 0
global freq, freq_y
freq = 1.6 / 2.0
freq_y = 1.1 / 2.0

tfBuffer = 0
tflistener = 0

def pub_vel(event):
    global mode
    global freq_y
    global last_shift
    t_now = rospy.Time.now()
    if (t_now - last_shift).to_sec() >= 6:
        mode += 1
        mode = mode % 3
        if mode == 1:
            freq_y = 1.1 / 2.0
        if mode == 2:
            freq_y = 0.6 / 2.0
        print "switch mode to " + str(mode)
        last_shift = t_now
    msg = Twist()
    if mode == 0:
        msg.linear.x = 1.0
    elif mode == 1:
        msg.linear.y = 1.0
    elif mode == 2:
        msg.linear.x = 1.0
        msg.angular.z = 2.0
    pub_v.publish(msg)

def pub_msg(event):
    t = rospy.Time.now().to_sec()

    msg = PersonMsg()
    leg1 = LegMsg()
    leg2 = LegMsg()
    msg.header.stamp = rospy.Time.now()

    c_freq_x = freq
    c_freq_y = freq_y
    if mode == 0:
        c_freq_y = 0.0
    if mode in [1,2]:
        c_freq_x = 0.0
    leg1.position.x = 0.5 * np.sin(2 * np.pi * c_freq_x * t) - 0.6
    leg2.position.x = -0.5 * np.sin(2 * np.pi * c_freq_x * (t+0.10)) - 0.6
    leg1.position.y = 0.5 * (np.sin(2 * np.pi * c_freq_y * t) + 1.2)
    leg2.position.y = -0.5 * (np.sin(2 * np.pi * c_freq_y * (t+0.10)) - 1.2)
    msg.leg1 = leg1
    msg.leg2 = leg2

    pub.publish(msg)

def pub_toe(event):
    t = rospy.Time.now()

    msg_l = PointStamped()
    msg_r = PointStamped()
    msg_l.header.stamp = t 
    msg_r.header.stamp = t

    t = t.to_sec()

    msg_l.point.x = 0.5 * np.sin(2 * np.pi * freq * t) - 0.6
    msg_r.point.x = -0.5 * np.sin(2 * np.pi * freq * (t+0.10)) - 0.6
    msg_l.point.y = 0.5 * np.sin(2 * np.pi * freq_y * t) - 0.6
    msg_r.point.y = -0.5 * np.sin(2 * np.pi * freq_y * (t+0.10)) - 0.6

    pub_tl.publish(msg_l)
    pub_tr.publish(msg_r)

def pub_force(event):
    t = rospy.Time.now()

    f = WrenchStamped()
    f.header.stamp = t
    f.header.frame_id = 'fts_base_link'

    t = t.to_sec()

    f.wrench.force.x = 10.0 + np.sin(4 * np.pi * freq * t) * 2
    f.wrench.force.y = -10.0 + np.sin(4 * np.pi * freq_y * t) * 2

    try:
       tff = transform_sensor(f)
       pub_f.publish(tff)
       pub_tf.publish(f)
    except Exception:
        rospy.loginfo("Transform failed")
    

def transform_sensor(data):
        try:
            tf = tfBuffer.lookup_transform('fts_reference_link','fts_base_link', rospy.Time().now(), timeout = rospy.Duration(0.1))
            data_tf = tf2_geometry_msgs.do_transform_wrench(data, tf)
            data_tf.header.stamp = data.header.stamp
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Transformation Data from 'fts_reference_link' to 'fts_base_link' not found")
            return None
    
        
        return data_tf

def pub_should(event):
        #Index Left shoulder is 5, Right is 2
        # frame: 'camera_body_rgb_optical_frame'
        global mode
        t = rospy.Time.now()
        poly = PolygonStamped()
        poly.header.stamp = t
        poly.header.frame_id = 'camera_body_rgb_optical_frame'
        poly.polygon.points = [Point32(float('nan'),0.0,0.0) for _ in range(20)] 


        lshoulder = PointStamped()
        rshoulder = PointStamped()

        if mode == 0:
            lshoulder.header.frame_id = 'base_link'
            lshoulder.point.x = 1.0    
            lshoulder.point.y = 0.5 + np.sin(2 * np.pi * freq * t.to_sec())
            lshoulder.point.z = 1.8

            rshoulder.header.frame_id = 'base_link'    
            rshoulder.point.x = 1.0 
            rshoulder.point.y = -0.5 + np.sin(2 * np.pi * freq * t.to_sec()+0.05)
            rshoulder.point.z = 1.8
        elif mode in [1,2]:
            lshoulder.header.frame_id = 'base_link'
            lshoulder.point.x = 1.0    
            lshoulder.point.y = 0.5 + np.sin(2 * np.pi * freq_y * t.to_sec())
            lshoulder.point.z = 1.8

            rshoulder.header.frame_id = 'base_link'    
            rshoulder.point.x = 1.0 
            rshoulder.point.y = -0.5 + np.sin(2 * np.pi * freq_y * t.to_sec()+0.05)
            rshoulder.point.z = 1.8

        tf = None
        try:
            tf = tfBuffer.lookup_transform('camera_body_rgb_optical_frame','base_link', rospy.Time().now(), timeout = rospy.Duration(1.0))
        except Exception:
            rospy.loginfo("No transform for shoulder")
        if tf:
            l_shoulder_tf = tf2_geometry_msgs.do_transform_point(lshoulder, tf)
            r_shoulder_tf = tf2_geometry_msgs.do_transform_point(rshoulder, tf)

            poly.polygon.points[5] = Point32(l_shoulder_tf.point.x, l_shoulder_tf.point.y, l_shoulder_tf.point.z)
            poly.polygon.points[2] = Point32(r_shoulder_tf.point.x, r_shoulder_tf.point.y, r_shoulder_tf.point.z)


            pub_sh.publish(poly)
        



if __name__ == '__main__':
    import sys
    global last_shift
    rospy.init_node('leg_pub_test', log_level=rospy.INFO)
    last_shift = rospy.Time.now()

    tfBuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfBuffer)

    rospy.timer.Timer(rospy.Duration(1.0 / 20.0), pub_msg)
    rospy.timer.Timer(rospy.Duration(1.0 / 10.0), pub_toe)
    rospy.timer.Timer(rospy.Duration(1.0 / 200.0), pub_vel)
    rospy.timer.Timer(rospy.Duration(1.0 / 200.0), pub_force)
    rospy.timer.Timer(rospy.Duration(1.0 / 8.0), pub_should)
    while not rospy.is_shutdown():
        rospy.spin()