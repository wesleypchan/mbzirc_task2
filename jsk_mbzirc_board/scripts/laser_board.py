#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int16

class Serial_board:
    def __init__(self):
        self.laserdata = 'laser:#'
        self.port = '/dev/ttyUSB1'
        self.baud = 115200
        self.serialtimeout = 0.01
        self.updaterate = 20    #20hz
        #initialize the parameters
        self.port = rospy.get_param("~port", self.port)
        self.baud = rospy.get_param('~baud', self.baud)
        self.serialtimeout = rospy.get_param('~serialtimeout', self.serialtimeout)
        self.updaterate = rospy.get_param('~updaterate', self.updaterate)
        rospy.loginfo("port is : %s" % self.port)
        rospy.loginfo("baudrate is: %s " % self.baud )
        self.ser = serial.Serial(self.port, self.baud, timeout=self.serialtimeout)
        self._pub_pointlaser = rospy.Publisher("/serial_board/point_laser", Int16, queue_size = 1)

    #delete function
    def __del__(self):
        self.ser.close()

    def Serial_Update(self):
        #always read the data
        r = rospy.Rate(self.updaterate)
        while not rospy.is_shutdown():

            data_in = self.ser.readall()
            if data_in:
                rospy.loginfo(data_in)
                #data will be like mag:!5!
                index = data_in.find(self.laserdata)
                if not (index < 0):
                    index += 7
                    databytes = data_in[index:data_in.find('#',index)]
                    rospy.loginfo(databytes)
                    self._pub_pointlaser.publish(int(databytes.strip('\0')))
            r.sleep()


if __name__ == "__main__":
    rospy.init_node('serial_board_laser')
    ex = Serial_board()
    try:
        ex.Serial_Update()
    except rospy.ROSInterruptException: pass
