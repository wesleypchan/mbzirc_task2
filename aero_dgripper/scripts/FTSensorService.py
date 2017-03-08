#!/usr/bin/env python

# from struct import *

import time
import array
import math
import curses
import subprocess
from seed_command import SeedCommand
import rospy
from geometry_msgs.msg import Wrench
import os.path
from aero_dgripper.srv import *

global wrench
wrench = Wrench()
global prev_wrench
prev_wrench = Wrench()


class Aero:
	def __init__(self):
		self.POS = 32 * [0]

		self.waist_z = 0

	#Print ServoData
	def Print_POS(self):
		stdscr.addstr(0, 0, "/////////  Position  Data  //////////")
		stdscr.addstr(1, 0, "1:{0}   ".format(self.POS[0]))
		stdscr.addstr(1, 20, "2:{0}   ".format(self.POS[1]))
		stdscr.addstr(1, 40, "3:{0}   ".format(self.POS[2]))
		stdscr.addstr(2, 0, "4:{0}   ".format(self.POS[3]))
		stdscr.addstr(2, 20, "5:{0}   ".format(self.POS[4]))
		stdscr.addstr(2, 40, "6:{0}   ".format(self.POS[5]))
		stdscr.addstr(3, 0, "7:{0}   ".format(self.POS[6]))
		stdscr.addstr(3, 20, "8:{0}   ".format(self.POS[7]))
		stdscr.addstr(3, 40, "9:{0}   ".format(self.POS[8]))
		stdscr.addstr(4, 0, "10:{0}   ".format(self.POS[9]))
		stdscr.addstr(4, 20, "11:{0}   ".format(self.POS[10]))
		stdscr.addstr(4, 40, "12:{0}   ".format(self.POS[11]))
		stdscr.addstr(5, 0, "13:{0}   ".format(self.POS[12]))
		stdscr.addstr(5, 20, "14:{0}   ".format(self.POS[13]))
		stdscr.addstr(5, 40, "15:{0}   ".format(self.POS[14]))

		stdscr.refresh()

	#Get Position Data
	def Read_POS(self,disp=None):
		POS = 31 * [0]

		Rec = seed.AERO_Data_Read(disp)

		if (Rec is None):
			#seed.ser.flushInput()
			return
		else :
			headder = int(Rec[0] + Rec[1] + Rec[2] + Rec[3],16)
			data_cmd = int(Rec[6] + Rec[7],16)

			if (headder == 0xDFFD):
				if (data_cmd == 0x14 or data_cmd == 0x41 or data_cmd == 0x42 or data_cmd == 0x43 or data_cmd == 0x44 or data_cmd == 0x45 or data_cmd == 0x52):
					for i in range(0,31):
						POS[i] = int(Rec[10+i*4] + Rec[10+i*4+1] + Rec[10+i*4+2] + Rec[10+i*4+3],16)
						if (i<10):
							if (POS[i] > 0x7FFF):
								self.POS[i] = POS[i] - pow(2,16)
							elif (POS[i] == 0x7FFF):
								self.POS[i] = 0
							else:
								self.POS[i] = POS[i]
						else:
							self.POS[i] = POS[i]
			else :
				seed.ser.flushInput()

def callback(req):
 
        global prev_wrench
        if seed == -1:
                print " seed == -1"
                # return zero ft
                wrench.force.x = 0.0
                wrench.force.y = 0.0
                wrench.force.z = 0.0
                wrench.torque.x = 0.0
                wrench.torque.y = 0.0
                wrench.torque.z = 0.0

        else:
                gotData = False 
                while not gotData:
                        try:
                                seed.AERO_Get_Pos(0)
                                
                                aero.Read_POS(0)
                                # aero.Print_POS()
                                time.sleep(0.05)
                                wrench.force.x = aero.POS[9]
                                wrench.force.y = aero.POS[10]
                                wrench.force.z = aero.POS[11]
                                wrench.torque.x = aero.POS[12]
                                wrench.torque.y = aero.POS[13]
                                wrench.torque.z = aero.POS[14]
                                gotData = True
                        except:
                                print "exeption"
                                wrench = prev_wrench
            
        res = AeroFTSensorResponse()
        res.wrench = wrench

        prev_wrench = wrench

        return res

                                

#----------- main -----------------#
if __name__ == "__main__":

    rospy.init_node('aero_ft_sensor_service_server', anonymous=True)
    s = rospy.Service('aero_ft_sensor_service', AeroFTSensor, callback)
    
    if os.path.islink('/dev/aero_upper'):
        seed = SeedCommand("aero_upper",1000000)
        aero=Aero()
                
        # Buffer Clear
        seed.ser.flushInput()		#flush input buffer
        seed.ser.flushOutput()		#flush output buffer
        
        # seed.AERO_Snd_Servo_all(0);
                
        time.sleep(1)

        print "AeroFTServiceServer ready."

    else:
        seed = -1
        print('Link /dev/aero_upper does not exist. Running dummy Aero ft server instead.')




    rospy.spin()
