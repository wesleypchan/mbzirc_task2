#!/usr/bin/env python  
import roslib
roslib.load_manifest('aero_utils')
import rospy
import tf
import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('aero_ground_publisher')

    tfl = tf.TransformListener()
    tfb = tf.TransformBroadcaster()

    calf_vec = [-0.024, 0.0, -0.384]
    ankle_vec = [-0.029, 0.0, -0.032]
    ground_vec = [-0.0, 0.0, -0.230]

    print "Start Publishing ground TF."

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        try:
            (foreleg_to_base_trans,foreleg_to_base_rot) = tfl.lookupTransform('/leg_foreleg_link', '/leg_base_link', rospy.Time(0))
            
            tfb.sendTransform(calf_vec, 
                              foreleg_to_base_rot,
                              rospy.Time.now(),
                              "leg_foreleg_ankle",
                              "leg_foreleg_link")            
            tfb.sendTransform(ankle_vec, 
                              tf.transformations.quaternion_from_euler(0,0,0),
                              rospy.Time.now(),
                              "leg_hindleg_ankle",
                              "leg_foreleg_ankle")
            tfb.sendTransform(ground_vec, 
                              tf.transformations.quaternion_from_euler(0,0,0),
                              rospy.Time.now(),
                              "ground",
                              "leg_hindleg_ankle")                        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "groundPublihser tf exception caught"
            continue

        rate.sleep()

    print "groundPublisher exited."
