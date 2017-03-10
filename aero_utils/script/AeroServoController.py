#!/usr/bin/env python

import os.path
from seed_command import SeedCommand
from aero_utils.srv import *
from aero_dgripper.srv import *
import rospy

def callback(req):
    # ID -
    #   -1 : all joints 
    #   0 - 7: aero upper
    #   8: wheels

    
    # print "AeroServoController callback AERO_SndServo(%d, %d)" % (req.id, req.data)
    if req.id == -1: # aero uppoer + lower
        if seedUpper != -1:
            seedUpper.AERO_Snd_Servo_all(req.data)
        if seedLower != -1:
            seedLower.AERO_Snd_Servo_all(req.data)
    elif req.id <= 7: # joints of aero lower
        if seedUpper != -1:
            seedUpper.AERO_Snd_Servo(req.id,req.data)
    elif req.id == 8: # all 4 wheels
        if seedLower != -1:
            seedLower.AERO_Snd_Servo(3, req.data)
            seedLower.AERO_Snd_Servo(4, req.data)
            seedLower.AERO_Snd_Servo(5, req.data)
            seedLower.AERO_Snd_Servo(6, req.data)
    elif req.id == 9: # lifter
        if seedLower != -1:
            seedLower.AERO_Snd_Servo(1, req.data)
            seedLower.AERO_Snd_Servo(2, req.data)
    elif req.id >= 10: #aero lower
        if seedLower != -1:
            seedLower.AERO_Snd_Servo(req.id - 10,req.data)
            
    return AeroServoControllerResponse()

if __name__ == "__main__":
    rospy.init_node('aero_servo_controller_service')
    s = rospy.Service('aero_servo_controller', AeroServoController, callback)
    if os.path.islink('/dev/aero_upper'):
        seedUpper = SeedCommand("aero_upper",1000000)
        print "AeroServoControllerService ready."
    else:
        seedUpper = -1
        print('Link /dev/aero_upper does not exist. Running dummy servo controller instead.')

    if os.path.islink('/dev/aero_lower'):
        seedLower = SeedCommand("aero_lower",1000000)
        print "AeroServoControllerService ready."

    else:
        seedLower = -1
        print('Link /dev/aero_lower does not exist. Running dummy servo controller instead.')

    rospy.spin()

