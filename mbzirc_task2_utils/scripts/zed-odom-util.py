#!/usr/bin/env python

import std_srvs.srv
import tf2_ros
import rospy
import geometry_msgs.msg
from multiprocessing import Pool


def init_odom(req):
    try:
        zed_init = rospy.ServiceProxy('/zed/initialize_odom', std_srvs.srv.Trigger)
        zed_res = zed_init()
    except:
        print("initializing zed odometry failed!")
        return std_srvs.srv.TriggerResponse(False, "")

    try:
        current_transform = tfBuffer.lookup_transform('zed_current_frame', 'ground', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print("getting transform from zed to ground failed!")
        return std_srvs.srv.TriggerResponse(False, "")

    initial_ground_tf.header.stamp = rospy.Time.now()
    initial_ground_tf.header.frame_id = "zed_initial_frame"
    initial_ground_tf.child_frame_id = 'initial_ground'
    initial_ground_tf.transform = current_transform.transform
    set_tf = True;
    br.sendTransform(initial_ground_tf)
    print("Odometry Initialized.")
    return std_srvs.srv.TriggerResponse(True, "")


initial_ground_tf = geometry_msgs.msg.TransformStamped()
rospy.init_node('initialize_odom_server')
rospy.wait_for_service('/zed/initialize_odom')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
br = tf2_ros.TransformBroadcaster()
s = rospy.Service('initialize_odom', std_srvs.srv.Trigger, init_odom)
set_tf = False

def odom_init_server():
    print("start service.")
    rospy.spin()

if __name__ == "__main__":
    odom_init_server()
    broadcast_tf()
