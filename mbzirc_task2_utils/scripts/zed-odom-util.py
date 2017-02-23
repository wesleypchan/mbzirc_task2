#!/usr/bin/env python

import std_srvs.srv
import tf2_ros
import rospy

def handle_add_two_ints(req):
    try:
        zed_init = rospy.ServiceProxy('initialize_zed_odom', std_srvs.srv.Trigger)
        zed_res = zed_init()
    except:
        return std_srvs.srv.TriggerResponse(False, "")

    try:
        current_transform = tfBuffer.lookup_transform('zed_current_frame', 'ground', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return std_srvs.srv.TriggerResponse(False, "")

    initial_ground_tf = geometry_msgs.msg.TransformStamped()
    initial_ground_tf.header.stamp = rospy.Time.now()
    initial_ground_tf.header.frame_id = "zed_initial_frame"
    initial_ground_tf.child_frame_id = 'initial_ground'
    initial_ground_tf.transform = current_transform.transform
    br.sendTransform(initial_ground_tf)
    print("Odometry Initialized.")
    return std_srvs.srv.TriggerResponse(True, "")

def odom_init_server():
    rospy.init_node('initialize_odom_server')
    rospy.wait_for_service('initialize_zed_odom')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.StaticTransformBroadcaster()
    s = rospy.Service('initialize_odom', std_srvs.srvTrigger, init_odom)
    rospy.spin()

if __name__ == "__main__":
    odom_init_server()
