#!/usr/bin/env python

from struct import*

import serial
import time
import array
import sys
import random
import os.path

class SeedCommand:

	def __init__(self,port=0,baud=460800):
                if port == "aero_upper":
                        if os.path.islink('/dev/aero_upper'):
                                self.ser=serial.Serial('/dev/aero_upper', baud, timeout=0.1)
                elif port == "aero_lower":
                        if os.path.islink('/dev/aero_lower'):
                                self.ser=serial.Serial('/dev/aero_lower', baud, timeout=0.1)
                else:
                        self.ser=serial.Serial('/dev/ttyUSB'+str(port), baud, timeout=0.1)

	#COM Port Open
	def COM_Open(self):

		self.ser.write("S8\r")
		self.ser.write("O\r")

	#Com Port Close
	def COM_Close(self):
		#self.ser.write("C\r")
		self.ser.close()

	def BT_Open(self):
		bLength = 18		#number of data
		data= bLength*[0]

		data[0] = 0x42	#"B"
		data[1] = 0x54	#"T"
		data[2] = 0x54	#"T"
		data[3] = 0x54	#"T"
		data[4] = 0x31	#"1"
		data[5] = 0x30	#"0"
		data[6] = 0x30	#"0"
		data[7] = 0x30	#"0"
		data[8] = 0x36	#"6"
		data[9] = 0x46	#"F"
		data[10] = 0x37	#"7"
		data[11] = 0x41	#"A"
		data[12] = 0x46	#"F"
		data[13] = 0x46	#"F"
		data[14] = 0x34	#"4"
		data[15] = 0x45	#"E"
		data[16] = 0x34	#"4"
		data[17] = 0x0D	#CR

		self.SEED_UART_Snd(data)

		bLength = 5		#number of data
		data= bLength*[0]

		data[0] = 0x42	#"B"
		data[1] = 0x54	#"T"
		data[2] = 0x43	#"C"
		data[3] = 0x0D	#CR

		self.SEED_UART_Snd(data)

	#Binary data to Hex String data and decide number of byte
	def num2str(self,value,byte=1):
		if value < 0:
			value = 0xFFFFFF + 1 + value
			#return str(hex(value).upper()[3:][-2:].rjust(2**byte,"F"))
		return str(hex(value).upper()[2:][-2:].rjust(2**byte,"0"))

	#Binary data to Hex data and decide number of byte
	def num2hex(self,value,byte=1):
		if value < 0:
			value = 0xFFFFFF + 1 + value
		return hex(value).upper()[2:][-2:].rjust(2**byte,"0")

	#SEED CAN data send
	def SEED_CAN_Snd(self,SndData):

		CAN_Transmit = ''.join(SndData) + '\r'
		self.ser.write(CAN_Transmit)

		print "Send Data \t:%s" % CAN_Transmit

	#SEED CAN data read
	def SEED_CAN_Read(self):

		ret_str = ''

		while(self.ser.inWaiting() < 1):
			pass

		while(self.ser.read(1) != 't'):
			pass

		ret_str = 't'

		# Get Data Length 
		ret = self.ser.read(4)
		data_len = int(ret[-1],16)
		ret_str += ret
	
		# Get All data
		ret = self.ser.read(data_len*2)
		ret_str += ret

		#self.ser.flushInput()

		#print "Receive Data \t:%s" % ret_str

		return ret_str

	#SEED UART data send
	def SEED_UART_Snd(self,SndData):

		self.ser.write(''.join(map(chr,SndData)))

		#print "Send Data \t:%s" % ''.join("{:02X}".format(ord(c)) for c in ''.join(map(chr,SndData)))
		#print ''.join("{:02X}".format(ord(c)) for c in ''.join(map(chr,SndData)))

	#SEED UART data read
	def SEED_UART_Read(self,number):

		while(self.ser.inWaiting() < number):
			pass

		RcvData = ''.join("{:02X}".format(ord(c)) for c in self.ser.read(number))

		if (len(RcvData) == number*2):
			self.pre_RcvData = RcvData

			print "Receive Data \t:%s" % RcvData
	
			return RcvData
		else :
			print "Read Error"
			return self.pre_RcvData

		#print "Busy: %s, IO:%s" % (RcvData[9],RcvData[11])

		#print "Receive Data \t:%s" % RcvData
	
	#####################################################################################################
	########################## SEED Command Function  ##################################
	#####################################################################################################
	'''
	id_num	:ID data
	d3		:SEED Command
	d4~d5	:Parameter
	'''
	#####################################################################################################
	def SEED_SCM(self,id_num,d3,d4,d5,d6,d7,d8):

		SndData = 12*[0]

		SndData[0] ='t'

		if d3 == 0x53:
			SndData[1] = self.num2str(0x00,0)
		else:
			SndData[1] = self.num2str(0x03,0)

		SndData[2] = self.num2str(id_num)
		SndData[3] = self.num2str(8,0)
		SndData[4] = self.num2str(0xF0 + id_num)
		SndData[5] = self.num2str(0x00)
		SndData[6] = self.num2str(d3)
		SndData[7] = self.num2str(d4)
		SndData[8] = self.num2str(d5)
		SndData[9] = self.num2str(d6)
		SndData[10] = self.num2str(d7)
		SndData[11] = self.num2str(d8)

		self.SEED_CAN_Snd(SndData)

	#Set Motor Current
	def SEED_Motor_Current(self,id_num, data1, data2):
		data = 3*[0]

		data[0] = data1 >> 8
		data[1] = data1  
		data[2] = data2

		self.SEED_SCM(id_num, 0x20, data[0], data[1], data[2],0x00,0x00)

	#Get Motor Position
	def SEED_Get_Pos(self,id_num):
		self.SEED_SCM(id_num,0x42,id_num,0x00,0x00,0x00,0x00)
		data = self.SEED_CAN_Read()
		speed = int(data[11]+data[12]+data[13]+data[14],16)
		position = int(data[15]+data[16]+data[17]+data[18]+data[19]+data[20],16)
		return [speed,position]

	#Get Current Data
	def SEED_Get_Current(self,id_num):
		self.SEED_SCM(id_num,0x43,id_num,0x00,0x00,0x00,0x00)
		data = self.SEED_CAN_Read()
		current = int(data[11]+data[12]+data[13]+data[14],16)
		error = int(data[15]+data[16]+data[17]+data[18]+data[19]+data[20],16)
		return [current,error]

	#Get Operation Information Data
	def SEED_Get_OpeInfo(self,id_num):
		self.SEED_SCM(id_num,0x44,id_num,0x00,0x00,0x00,0x00)
		data = self.SEED_CAN_Read()
		status = int(data[11]+data[12],16)
		motor_state = int(data[13]+data[14],16)
		script_no = int(data[15]+data[16],16)
		script_row = int(data[17]+data[18],16)
		point_no = int(data[19]+data[20],16)
		return [status,motor_state,script_no,script_row,point_no]

	#servo ON/OFF 
	def SEED_Servo(self,id_num,data):
		self.SEED_SCM(id_num,0x50,id_num,data,0x00,0x00,0x00)

	#Motor Stop
	def SEED_Motor_Stop(self,id_num):
		self.SEED_SCM(id_num,0x51,0x00,0x00,0x00,0x00,0x00)

	#Run SEED Script
	def SEED_SGo(self,id_num,s_num):
		if s_num > 0x00 and s_num < 0x0F :
			self.SEED_SCM(id_num,0x5F,id_num,s_num,0x00,0x00,0x00)

	#Speed and Icrement Go (servo mode)
	def SEED_SMove_Inc(self,id_num,speed,pos):
		data = 5*[0]

		if pos >= 0xFFFFFF/2:
			pos = 0xFFFFFF/2
		elif pos <= -0xFFFFFF/2:
			pos = -0xFFFFFF/2

		data[0] = speed >> 8
		data[1] = speed  
		data[2] = pos >> 16
		data[3] = pos >> 8
		data[4] = pos 

		#print data
		self.SEED_SCM(id_num,0x63,data[0],data[1],data[2],data[3],data[4])

	#Time and Abusolute Go (servo mode)
	def SEED_TMove_Abs(self,id_num,time,pos):
		data = 5*[0]

		if pos >= 0xFFFFFF/2:
			pos = 0xFFFFFF/2
		elif pos <= -0xFFFFFF/2:
			pos = -0xFFFFFF/2

		data[0] = time >> 8
		data[1] = time  
		data[2] = pos >> 16
		data[3] = pos >> 8
		data[4] = pos 

		#print data
		self.SEED_SCM(id_num,0x64,data[0],data[1],data[2],data[3],data[4])

	#Current and Abusolute Go (servo mode)
	def SEED_CMove_Abs(self,id_num,current,pos):
		data = 5*[0]

		if pos >= 0xFFFFFF/2:
			pos = 0xFFFFFF/2
		elif pos <= -0xFFFFFF/2:
			pos = -0xFFFFFF/2

		data[0] = current >> 8
		data[1] = current  
		data[2] = pos >> 16
		data[3] = pos >> 8
		data[4] = pos 

		#print data
		self.SEED_SCM(id_num,0x65,data[0],data[1],data[2],data[3],data[4])

	#Speed and Absolute Go (servo mode)
	def SEED_SMove_Abs(self,id_num,speed,pos):
		data = 5*[0]

		if pos >= 0xFFFFFF/2:
			pos = 0xFFFFFF/2
		elif pos <= -0xFFFFFF/2:
			pos = -0xFFFFFF/2

		data[0] = speed >> 8
		data[1] = speed  
		data[2] = pos >> 16
		data[3] = pos >> 8
		data[4] = pos 

		#print data
		self.SEED_SCM(id_num,0x66,data[0],data[1],data[2],data[3],data[4])

	#Icrement Go (self.servo mode)
	def SEED_Move_Inc(self,id_num,time,pos):
		data = 5*[0]

		if pos >= 0xFFFFFF/2:
			pos = 0xFFFFFF/2
		elif pos <= -0xFFFFFF/2:
			pos = -0xFFFFFF/2

		data[0] = time >> 8
		data[1] = time  
		data[2] = pos >> 16
		data[3] = pos >> 8
		data[4] = pos 

		#print data
		self.SEED_SCM(id_num,0x67,data[0],data[1],data[2],data[3],data[4])

	#Absolute Go (self.servo mode)
	def SEED_Move_Abs(self,id_num,time,pos):
		data = 5*[0]

		if pos >= 0xFFFFFF/2:
			pos = 0xFFFFFF/2
		elif pos <= -0xFFFFFF/2:
			pos = -0xFFFFFF/2

		data[0] = time >> 8
		data[1] = time  
		data[2] = pos >> 16
		data[3] = pos >> 8
		data[4] = pos 

		self.SEED_SCM(id_num,0x68,data[0],data[1],data[2],data[3],data[4])

	#Speed Go
	def SEED_Move_Spd(self,id_num,spd):
		data = 3*[0]

		if spd >= 0xFFFF/2:
			spd = 0xFFFF/2
		elif spd <= -0xFFFF/2:
			spd = -0xFFFF/2

		data[0] = spd >> 8
		data[1] = spd

		if spd < 0:
			data[2] = 1
		else :
			data[2] = 0

		print data

		self.SEED_SCM(id_num,0x6C,data[0],data[1],data[2],0,0)


	#Set Position
	def SEED_Set_Pos(self,id_num):
		SEED_SCM(id_num,0x6F,0x00,0x00,0x00,0x00,0x00)

	# Return to Origin
	def ORIGIN(self,id_num):
		self.SEED_Servo(id_num,0)
		self.SEED_Motor_Current(id_num,100,0)
		self.SEED_Servo(id_num,1)
		self.SEED_SMove_Inc(id_num,5000,-200000)
		while (self.SEED_Get_Current(id_num)[0] < 80):
			pass
		self.SEED_Motor_Stop(id_num)
		self.SEED_SMove_Inc(id_num,1000,10000)
		time.sleep(1.5)					#wait for moving position complete
		self.SEED_Motor_Stop(id_num)
		self.SEED_Set_Pos(id_num)



	#####################################################################################################
	########################## THKR-7 Command  ##################################
	#####################################################################################################

	#Get Information Command(0x42, 0x43, 0x45)
	def Get_Command_old(self,cmd):
		bCheckSum = 0
		bCount = 0
		bLength = 68		#number of data +1

		data= bLength*[0]

		data[0] = 0xFD
		data[1] = 0xDF
		data[2] = 64

		data[3] = cmd

		data[4] = 0x00

		for i in range(4,bLength-1):
			data[i] = 0

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(data[bCount]),16)

		data[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		self.SEED_UART_Snd(data)

	#Get Information Command(0x42, 0x43, 0x45)
	def Get_Command(self,cmd):
		bCheckSum = 0
		bCount = 0
		bLength = 6		#number of data +1

		data= bLength*[0]

		data[0] = 0xFD
		data[1] = 0xDF
		data[2] = 2

		data[3] = cmd

		data[4] = 0x00

		for i in range(4,bLength-1):
			data[i] = 0

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(data[bCount]),16)

		data[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		self.SEED_UART_Snd(data)


	#Set Parameter Command(0x20, 0x25, 0x24, 0x67, 0x68)
	def Set_Command(self,id_num,cmd,d0,d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11,d12,d13,d14,d15,d16,d17,d18,d19,d20,d21,d22,d23,d24,d25,d26,d27,d28,d29,d30,d31,d32,d33,d34,d35):

		bCheckSum = 0
		bCount = 0
		bLength = 77		#number of data +1

		data= bLength*[0]

		data[0] = 0xFA
		data[1] = 0xAF
		data[2] = 0xF0 + id_num

		data[3] = cmd

		#time 
		data[4] = d0 >> 8
		data[5] = d0
		#CAN1 Actuator
		data[6] = d1 >> 8
		data[7] = d1
		data[8] = d2 >> 8
		data[9] = d2
		data[10] = d3 >> 8
		data[11] = d3
		data[12] = d4 >> 8
		data[13] = d4
		data[14] = d5 >> 8
		data[15] = d5
		data[16] = d6 >> 8
		data[17] = d6
		data[18] = d7 >> 8
		data[19] = d7
		data[20] = d8 >> 8
		data[21] = d8
		data[22] = d9 >> 8
		data[23] = d9
		data[24] = d10 >> 8
		data[25] = d10
		data[26] = d11 >> 8
		data[27] = d11
		data[28] = d12 >> 8
		data[29] = d12
		#CAN1 Sensor
		data[30] = d13 >> 8
		data[31] = d13
		data[32] = d14 >> 8
		data[33] = d14
		data[34] = d15 >> 8
		data[35] = d15
		data[36] = d16 >> 8
		data[37] = d16
		#CAN2
		data[38] = d17 >> 8
		data[39] = d17
		data[40] = d18 >> 8
		data[41] = d18
		data[42] = d19 >> 8
		data[43] = d19
		data[44] = d20 >> 8
		data[45] = d20
		data[46] = d21 >> 8
		data[47] = d21
		data[48] = d22 >> 8
		data[49] = d22
		data[50] = d23 >> 8
		data[51] = d23
		data[52] = d24 >> 8
		data[53] = d24
		data[54] = d25 >> 8
		data[55] = d25
		data[56] = d26 >> 8
		data[57] = d26
		data[58] = d27 >> 8
		data[59] = d27
		data[60] = d28 >> 8
		data[61] = d28
		#CAN2 Sensor
		data[62] = d29 >> 8
		data[63] = d29
		data[64] = d30 >> 8
		data[65] = d30
		data[66] = d31 >> 8
		data[67] = d31
		data[68] = d32 >> 8
		data[69] = d32
		#IMU
		data[70] = d33 >> 8
		data[71] = d33
		data[72] = d34 >> 8
		data[73] = d34
		data[74] = d35 >> 8
		data[75] = d35

		#num to hex
		for i in range(0,bLength-1):
			data[i] = int(self.num2hex(data[i]),16)
	
		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(data[bCount]),16)

		data[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		self.SEED_UART_Snd(data)

	#Motor ON / OFF Command(0x50)
	def Servo_Command(self,id_num,d0):
		bCheckSum = 0
		bCount = 0
		bLength = 77		#number of data +1

		data= bLength*[0]

		data[0] = 0xFA
		data[1] = 0xAF
		data[2] = 0xF0 + id_num

		data[3] = 0x50

		for i in range(4,bLength-1):
			data[i] = d0

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(data[bCount]),16)

		data[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		self.SEED_UART_Snd(data)


	# Move Command
	def Right_turn(self,time_data):
		self.Set_Command(2,0x68,time_data,1500,0,0,0,650,-1000,  -1500,0,0,0,650,1000,  0,0,0,0,  -1500,0,0,0,650,1000,  1500,0,0,0,650,-1000,  0,0,0,0,0,0,0)

	def Left_turn(self,time_data):
		self.Set_Command(2,0x68,time_data,1500,0,0,0,650,1000,  -1500,0,0,0,650,-1000,  0,0,0,0,  -1500,0,0,0,650,-1000,  1500,0,0,0,650,1000,  0,0,0,0,0,0,0)

	def Go_front(self,time_data):
		self.Set_Command(2,0x68,time_data,1500,0,0,0,650,3000,  -1500,0,0,0,650,-3000,  0,0,0,0,  -1500,0,0,0,650,3000,  1500,0,0,0,650,-3000,  0,0,0,0,0,0,0)

	def Go_back(self,time_data):
		self.Set_Command(2,0x68,time_data,1500,0,0,0,650,-3000,  -1500,0,0,0,650,3000,  0,0,0,0,  -1500,0,0,0,650,-3000,  1500,0,0,0,650,3000,  0,0,0,0,0,0,0)

	def Stop(self):
		self.Set_Command(2,0x68,0,1500,0,0,0,650,0,  -1500,0,0,0,650,0,  0,0,0,0,  -1500,0,0,0,650,0,  1500,0,0,0,650,0,  0,0,0,0,0,0,0)


	#####################################################################################################
	########################## AERO Command Function  ##################################
	#####################################################################################################

	#Aero Data Read
	def AERO_Data_Read(self,disp=None):

		timeout = time.time() + 1

		while(self.ser.inWaiting() < 3):
			if(time.time() > timeout):
				print "Time Out"
				self.ser.flushInput()
				self.ser.flushOutput()
				return None
			else :
				pass

		headder1 = ord(self.ser.read(1))

		if (headder1 == 0xBF or headder1 == 0xDF or headder1 == 0xFD):
			pass
		else:
			print "Read Error. Headder is %d" % headder1
			self.ser.flushInput()
			self.ser.flushOutput()
			return None

		headder2 = ord(self.ser.read(1))
		data_length = ord(self.ser.read(1))

		timeout = time.time() + 1

		while(self.ser.inWaiting() < data_length):
			if(time.time() > timeout):
				print "Read Error. data_length is %d" % data_length
				self.ser.flushInput()
				self.ser.flushOutput()
				return None
			else:
				pass

		data_cmd = ord(self.ser.read(1))

		RcvData = self.num2str(headder1) + self.num2str(headder2) + self.num2hex(data_length) +  self.num2hex(data_cmd) \
				+ ''.join("{:02X}".format(ord(c)) for c in self.ser.read(data_length))

		if disp== 1:
			print "Receive Data \t:%s" % RcvData
		else :
			pass

		return RcvData

	#Current Set
	def AERO_Set_Current(self,max_c,down_c):
		bCheckSum = 0
		bCount = 0
		bLength = 68
		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 64
		SndData[3] = 0x01
		SndData[4] = 0x00

		for i in range(1,61):
			if ( i % 2 != 0):
				SndData[i + 4] = max_c
			else:
				SndData[i + 4] = down_c

		SndData[65] = 0
		SndData[66] = 0

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)


		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	#Servo ON/OFF ->ON:1 , OFF:0
	def AERO_Snd_Servo(self,ID,data):
		bCheckSum = 0
		bCount = 0
		bLength = 8
		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 0x04
		SndData[3] = 0x21
		SndData[4] = ID
		SndData[5] = data >> 8
		SndData[6] = data

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	#Servo ON/OFF ->ON:1 , OFF:0
	def AERO_Snd_Servo_all(self,data):
		bLength = 68
		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 64
		SndData[3] = 0x21

		for i in range(1,30):
			SndData[i*2 + 3 + 1] = data

		SndData[65] = 0
		SndData[66] = 0

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)


	#Servo ON/OFF ->ON:1 , OFF:0
	def AERO_Snd_Speed(self,ID,data):
		bCheckSum = 0
		bCount = 0
		bLength = 8
		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 0x04
		SndData[3] = 0x02
		SndData[4] = ID
		SndData[5] = data >> 8
		SndData[6] = data

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)


	#Motion Play
	def AERO_Snd_Motion(self,data):
		bCheckSum = 0
		bCount = 0
		bLength = 6
		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 0x02
		SndData[3] = 0x32
		SndData[4] = data

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	def AERO_Get_State(self):
		bCheckSum = 0
		bCount = 0
		bLength = 6
		SndData = bLength*[0]

		SndData[0] = 0xFB
		SndData[1] = 0xBF
		SndData[2] = 0x02
		SndData[3] = 0x06
		SndData[4] = 0x0D

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	#Servo ON/OFF ->ON:1 , OFF:0
	def AERO_Get_Pos(self,ID):
		bCheckSum = 0
		bCount = 0
		bLength = 6
		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 0x02
		SndData[3] = 0x41
		SndData[4] = ID

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)


	#Script Run
	def AERO_Snd_Script(self,ID,data):
		bCheckSum = 0
		bCount = 0
		bLength = 8
		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 0x04
		SndData[3] = 0x22
		SndData[4] = ID
		SndData[5] = data >> 8
		SndData[6] = data

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	#Position Get
	def AERO_Get_Pos(self,ID):
		bCheckSum = 0
		bCount = 0
		bLength = 6
		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 0x02
		SndData[3] = 0x41
		SndData[4] = ID

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	#Current Get
	def AERO_Get_Cur(self,ID):
		bCheckSum = 0
		bCount = 0
		bLength = 6
		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 0x02
		SndData[3] = 0x42
		SndData[4] = ID

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	#Volt Temp Get
	def AERO_Get_Temp(self,ID):
		bCheckSum = 0
		bCount = 0
		bLength = 6
		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 0x02
		SndData[3] = 0x43
		SndData[4] = ID

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	#AD Get
	def AERO_Get_AD(self,ID,data):
		bCheckSum = 0
		bCount = 0
		bLength = 7
		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 0x03
		SndData[3] = 0x44
		SndData[4] = ID
		SndData[5] = data

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	#Dio Get
	def AERO_Get_DIO(self,ID):
		bCheckSum = 0
		bCount = 0
		bLength = 6
		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 0x02
		SndData[3] = 0x45
		SndData[4] = ID

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	#CAN Send
	def AERO_Snd_CAN(self,ID,CMD,data1,data2,data3,data4,data5):
		bCheckSum = 0
		bCount = 0
		bLength = 12
		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 0x08
		SndData[3] = 0x51
		SndData[4] = ID
		SndData[5] = CMD
		SndData[6] = data1
		SndData[7] = data2
		SndData[8] = data3
		SndData[9] = data4
		SndData[10] = data5

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	#Status Get
	def AERO_Get_Status(self,ID):
		bCheckSum = 0
		bCount = 0
		bLength = 6
		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 0x02
		SndData[3] = 0x52
		SndData[4] = ID

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	#PS3 Controller
	def AERO_Snd_PS3(self,BTL,BTR,LLR,LUD,RLR,RUD):
		bCheckSum = 0
		bCount = 0
		bLength = 11

		SndData = bLength*[0]

		SndData[0] = 0xFB
		SndData[1] = 0xBF
		SndData[2] = 0x06
		SndData[3] = 0x00
		SndData[4] = BTL
		SndData[5] = BTR
		SndData[6] = LLR
		SndData[7] = LUD
		SndData[8] = RLR
		SndData[9] = RUD

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):

			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	def AERO_Snd_PosSet(self,m_time,POS1,POS2,POS3,POS4,POS5,POS6,POS7,POS8,POS9,POS10,
POS11,POS12,POS13,POS14,POS15,POS16,POS17,POS18,POS19,POS20,
POS21,POS22,POS23,POS24,POS25,POS26,POS27,POS28,POS29,POS30):

		bCheckSum = 0
		bCount = 0
		bLength = 68

		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 64				#Data Length
		SndData[3] = 0x14			#Command
		SndData[4] = 0x00			#ID
		SndData[5] = POS1 >> 8
		SndData[6] = POS1
		SndData[7] = POS2 >> 8
		SndData[8] = POS2
		SndData[9] = POS3 >> 8
		SndData[10] = POS3
		SndData[11] = POS4 >> 8
		SndData[12] = POS4
		SndData[13] = POS5 >> 8
		SndData[14] = POS5
		SndData[15] = POS6 >> 8
		SndData[16] = POS6
		SndData[17] = POS7 >> 8
		SndData[18] = POS7
		SndData[19] = POS8 >> 8
		SndData[20] = POS8
		SndData[21] = POS9 >> 8
		SndData[22] = POS9
		SndData[23] = POS10 >> 8
		SndData[24] = POS10
		SndData[25] = POS11 >> 8
		SndData[26] = POS11
		SndData[27] = POS12 >> 8
		SndData[28] = POS12
		SndData[29] = POS13 >> 8
		SndData[30] = POS13
		SndData[31] = POS14 >> 8
		SndData[32] = POS14
		SndData[33] = POS15 >> 8
		SndData[34] = POS15
		SndData[35] = POS16 >> 8
		SndData[36] = POS16
		SndData[37] = POS17 >> 8
		SndData[38] = POS17
		SndData[39] = POS18 >> 8
		SndData[40] = POS18
		SndData[41] = POS19 >> 8
		SndData[42] = POS19
		SndData[43] = POS20 >> 8
		SndData[44] = POS20
		SndData[45] = POS21 >> 8
		SndData[46] = POS21
		SndData[47] = POS22 >> 8
		SndData[48] = POS22
		SndData[49] = POS23 >> 8
		SndData[50] = POS23
		SndData[51] = POS24 >> 8
		SndData[52] = POS24
		SndData[53] = POS25 >> 8
		SndData[54] = POS25
		SndData[55] = POS26 >> 8
		SndData[56] = POS26
		SndData[57] = POS27 >> 8
		SndData[58] = POS27
		SndData[59] = POS28 >> 8
		SndData[60] = POS28
		SndData[61] = POS29 >> 8
		SndData[62] = POS29
		SndData[63] = POS30 >> 8
		SndData[64] = POS30
		SndData[65] = m_time >> 8	#time[msec] high
		SndData[66] = m_time		#time[msec] low

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)
		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	def AERO_Snd_VelSet(self,m_time,POS1,POS2,POS3,POS4):

		bCheckSum = 0
		bCount = 0
		bLength = 68

		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 64				#Data Length
		SndData[3] = 0x15			#Command
		SndData[4] = 0x00			#ID
		SndData[5] = 0
		SndData[6] = 0
		SndData[7] = 0
		SndData[8] = 0
		SndData[9] = POS1 >> 8
		SndData[10] = POS1
		SndData[11] = POS2 >> 8
		SndData[12] = POS2
		SndData[13] = POS3 >> 8
		SndData[14] = POS3
		SndData[15] = POS4 >> 8
		SndData[16] = POS4
		SndData[17] = 0
		SndData[18] = 0
		SndData[19] = 0
		SndData[20] = 0
		SndData[21] = 0
		SndData[22] = 0
		SndData[23] = 0
		SndData[24] = 0
		SndData[25] = 0
		SndData[26] = 0
		SndData[27] = 0
		SndData[28] = 0
		SndData[29] = 0
		SndData[30] = 0
		SndData[31] = 0
		SndData[32] = 0
		SndData[33] = 0
		SndData[34] = 0
		SndData[35] = 0
		SndData[36] = 0
		SndData[37] = 0
		SndData[38] = 0
		SndData[39] = 0
		SndData[40] = 0
		SndData[41] = 0
		SndData[42] = 0
		SndData[43] = 0
		SndData[44] = 0
		SndData[45] = 0
		SndData[46] = 0
		SndData[47] = 0
		SndData[48] = 0
		SndData[49] = 0
		SndData[50] = 0
		SndData[51] = 0
		SndData[52] = 0
		SndData[53] = 0
		SndData[54] = 0
		SndData[55] = 0
		SndData[56] = 0
		SndData[57] = 0
		SndData[58] = 0
		SndData[59] = 0
		SndData[60] = 0
		SndData[61] = 0
		SndData[62] = 0
		SndData[63] = 0
		SndData[64] = 0
		SndData[65] = m_time >> 8	#time[msec] high
		SndData[66] = m_time		#time[msec] low

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)
		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)


	def AERO_Snd_IncSet(self,m_time,POS1,POS2,POS3,POS4,POS5,POS6,POS7,POS8,POS9,POS10,
POS11,POS12,POS13,POS14,POS15,POS16,POS17,POS18,POS19,POS20,
POS21,POS22,POS23,POS24,POS25,POS26,POS27,POS28,POS29,POS30):

		bCheckSum = 0
		bCount = 0
		bLength = 68

		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 64				#Data Length
		SndData[3] = 0x16			#Command
		SndData[4] = 0x00			#ID
		SndData[5] = POS1 >> 8
		SndData[6] = POS1
		SndData[7] = POS2 >> 8
		SndData[8] = POS2
		SndData[9] = POS3 >> 8
		SndData[10] = POS3
		SndData[11] = POS4 >> 8
		SndData[12] = POS4
		SndData[13] = POS5 >> 8
		SndData[14] = POS5
		SndData[15] = POS6 >> 8
		SndData[16] = POS6
		SndData[17] = POS7 >> 8
		SndData[18] = POS7
		SndData[19] = POS8 >> 8
		SndData[20] = POS8
		SndData[21] = POS9 >> 8
		SndData[22] = POS9
		SndData[23] = POS10 >> 8
		SndData[24] = POS10
		SndData[25] = POS11 >> 8
		SndData[26] = POS11
		SndData[27] = POS12 >> 8
		SndData[28] = POS12
		SndData[29] = POS13 >> 8
		SndData[30] = POS13
		SndData[31] = POS14 >> 8
		SndData[32] = POS14
		SndData[33] = POS15 >> 8
		SndData[34] = POS15
		SndData[35] = POS16 >> 8
		SndData[36] = POS16
		SndData[37] = POS17 >> 8
		SndData[38] = POS17
		SndData[39] = POS18 >> 8
		SndData[40] = POS18
		SndData[41] = POS19 >> 8
		SndData[42] = POS19
		SndData[43] = POS20 >> 8
		SndData[44] = POS20
		SndData[45] = POS21 >> 8
		SndData[46] = POS21
		SndData[47] = POS22 >> 8
		SndData[48] = POS22
		SndData[49] = POS23 >> 8
		SndData[50] = POS23
		SndData[51] = POS24 >> 8
		SndData[52] = POS24
		SndData[53] = POS25 >> 8
		SndData[54] = POS25
		SndData[55] = POS26 >> 8
		SndData[56] = POS26
		SndData[57] = POS27 >> 8
		SndData[58] = POS27
		SndData[59] = POS28 >> 8
		SndData[60] = POS28
		SndData[61] = POS29 >> 8
		SndData[62] = POS29
		SndData[63] = POS30 >> 8
		SndData[64] = POS30
		SndData[65] = m_time >> 8	#time[msec] high
		SndData[66] = m_time		#time[msec] low

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)
		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	def AERO_Snd_ServoSet(self,POS1,POS2,POS3,POS4,POS5,POS6,POS7,POS8,POS9,POS10,
POS11,POS12,POS13,POS14,POS15,POS16,POS17,POS18,POS19,POS20,
POS21,POS22,POS23,POS24,POS25,POS26,POS27,POS28,POS29,POS30):

		bCheckSum = 0
		bCount = 0
		bLength = 68

		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 64				#Data Length
		SndData[3] = 0x21			#Command
		SndData[4] = 0x00			#ID
		SndData[5] = POS1 >> 8
		SndData[6] = POS1
		SndData[7] = POS2 >> 8
		SndData[8] = POS2
		SndData[9] = POS3 >> 8
		SndData[10] = POS3
		SndData[11] = POS4 >> 8
		SndData[12] = POS4
		SndData[13] = POS5 >> 8
		SndData[14] = POS5
		SndData[15] = POS6 >> 8
		SndData[16] = POS6
		SndData[17] = POS7 >> 8
		SndData[18] = POS7
		SndData[19] = POS8 >> 8
		SndData[20] = POS8
		SndData[21] = POS9 >> 8
		SndData[22] = POS9
		SndData[23] = POS10 >> 8
		SndData[24] = POS10
		SndData[25] = POS11 >> 8
		SndData[26] = POS11
		SndData[27] = POS12 >> 8
		SndData[28] = POS12
		SndData[29] = POS13 >> 8
		SndData[30] = POS13
		SndData[31] = POS14 >> 8
		SndData[32] = POS14
		SndData[33] = POS15 >> 8
		SndData[34] = POS15
		SndData[35] = POS16 >> 8
		SndData[36] = POS16
		SndData[37] = POS17 >> 8
		SndData[38] = POS17
		SndData[39] = POS18 >> 8
		SndData[40] = POS18
		SndData[41] = POS19 >> 8
		SndData[42] = POS19
		SndData[43] = POS20 >> 8
		SndData[44] = POS20
		SndData[45] = POS21 >> 8
		SndData[46] = POS21
		SndData[47] = POS22 >> 8
		SndData[48] = POS22
		SndData[49] = POS23 >> 8
		SndData[50] = POS23
		SndData[51] = POS24 >> 8
		SndData[52] = POS24
		SndData[53] = POS25 >> 8
		SndData[54] = POS25
		SndData[55] = POS26 >> 8
		SndData[56] = POS26
		SndData[57] = POS27 >> 8
		SndData[58] = POS27
		SndData[59] = POS28 >> 8
		SndData[60] = POS28
		SndData[61] = POS29 >> 8
		SndData[62] = POS29
		SndData[63] = POS30 >> 8
		SndData[64] = POS30
		SndData[65] = 0
		SndData[66] = 0

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)
		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	def AERO_Snd_SpeedSet(self,POS1,POS2,POS3,POS4,POS5,POS6,POS7,POS8,POS9,POS10,
POS11,POS12,POS13,POS14,POS15,POS16,POS17,POS18,POS19,POS20,
POS21,POS22,POS23,POS24,POS25,POS26,POS27,POS28,POS29,POS30):

		bCheckSum = 0
		bCount = 0
		bLength = 68

		SndData = bLength*[0]

		SndData[0] = 0xFD
		SndData[1] = 0xDF
		SndData[2] = 64				#Data Length
		SndData[3] = 0x02			#Command
		SndData[4] = 0x00			#ID
		SndData[5] = POS1 >> 8
		SndData[6] = POS1
		SndData[7] = POS2 >> 8
		SndData[8] = POS2
		SndData[9] = POS3 >> 8
		SndData[10] = POS3
		SndData[11] = POS4 >> 8
		SndData[12] = POS4
		SndData[13] = POS5 >> 8
		SndData[14] = POS5
		SndData[15] = POS6 >> 8
		SndData[16] = POS6
		SndData[17] = POS7 >> 8
		SndData[18] = POS7
		SndData[19] = POS8 >> 8
		SndData[20] = POS8
		SndData[21] = POS9 >> 8
		SndData[22] = POS9
		SndData[23] = POS10 >> 8
		SndData[24] = POS10
		SndData[25] = POS11 >> 8
		SndData[26] = POS11
		SndData[27] = POS12 >> 8
		SndData[28] = POS12
		SndData[29] = POS13 >> 8
		SndData[30] = POS13
		SndData[31] = POS14 >> 8
		SndData[32] = POS14
		SndData[33] = POS15 >> 8
		SndData[34] = POS15
		SndData[35] = POS16 >> 8
		SndData[36] = POS16
		SndData[37] = POS17 >> 8
		SndData[38] = POS17
		SndData[39] = POS18 >> 8
		SndData[40] = POS18
		SndData[41] = POS19 >> 8
		SndData[42] = POS19
		SndData[43] = POS20 >> 8
		SndData[44] = POS20
		SndData[45] = POS21 >> 8
		SndData[46] = POS21
		SndData[47] = POS22 >> 8
		SndData[48] = POS22
		SndData[49] = POS23 >> 8
		SndData[50] = POS23
		SndData[51] = POS24 >> 8
		SndData[52] = POS24
		SndData[53] = POS25 >> 8
		SndData[54] = POS25
		SndData[55] = POS26 >> 8
		SndData[56] = POS26
		SndData[57] = POS27 >> 8
		SndData[58] = POS27
		SndData[59] = POS28 >> 8
		SndData[60] = POS28
		SndData[61] = POS29 >> 8
		SndData[62] = POS29
		SndData[63] = POS30 >> 8
		SndData[64] = POS30
		SndData[65] = 0
		SndData[66] = 0

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)
		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	#####################################################################################################

	#####################################################################################################
	########################## Dynamixel Function  ##################################
	#####################################################################################################

	#Aero Data Read
	def SEED_DX_Read(self):

		while(self.ser.inWaiting() < 4):
			pass

		headder1 = ord(self.ser.read(1))

		if (headder1 == 0xFF):
			pass
		else:
			print "Read Error. Headder is %d" % headder1
			self.ser.flushInput()
			self.ser.flushOutput()
			return None

		headder2 = ord(self.ser.read(1))
		id_data = ord(self.ser.read(1))
		data_length = ord(self.ser.read(1))

		timeout = time.time() + 1

		while(self.ser.inWaiting() < data_length):
			if(time.time() > timeout):
				print "Read Error. data_length is %d" % data_length
				self.ser.flushInput()
				self.ser.flushOutput()
				return None
			else:
				pass

		RcvData = self.num2str(headder1) + self.num2str(headder2) + self.num2hex(id_data) +  self.num2hex(data_length)\
				+ ''.join("{:02X}".format(ord(c)) for c in self.ser.read(data_length))

		print "Receive Data \t:%s" % RcvData

		#return RcvData

	def SEED_DX_Get(self,ID):
		bCheckSum = 0
		bCount = 0
		bLength = 8

		RS_TX= bLength*[0]

		RS_TX[0] = 0xFF
		RS_TX[1] = 0xFF
		RS_TX[2] = ID
		RS_TX[3] = 0x04		# Data Length
		RS_TX[4] = 0x02		# Instruction
		RS_TX[5] = 0x24		# Address
		RS_TX[6] = 0x08		# Count

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(RS_TX[bCount]),16)
		RS_TX[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			RS_TX[i] = int(self.num2hex(RS_TX[i]),16)

		self.SEED_UART_Snd(RS_TX)

	def SEED_DX_Get_POS(self,ID):
		bCheckSum = 0
		bCount = 0
		bLength = 8

		RS_TX= bLength*[0]

		RS_TX[0] = 0xFF
		RS_TX[1] = 0xFF
		RS_TX[2] = ID
		RS_TX[3] = 0x04
		RS_TX[4] = 0x02
		RS_TX[5] = 0x24
		RS_TX[6] = 0x02

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(RS_TX[bCount]),16)
		RS_TX[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			RS_TX[i] = int(self.num2hex(RS_TX[i]),16)

		self.SEED_UART_Snd(RS_TX)

	def SEED_DX_SRV(self,ID,SW):
		bCheckSum = 0
		bCount = 0
		bLength = 9

		RS_TX= bLength*[0]

		RS_TX[0] = 0xFF
		RS_TX[1] = 0xFF
		RS_TX[2] = ID
		RS_TX[3] = 0x05
		RS_TX[4] = 0x03
		RS_TX[5] = 0x18
		RS_TX[6] = SW
		RS_TX[7] = SW

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(RS_TX[bCount]),16)
		RS_TX[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			RS_TX[i] = int(self.num2hex(RS_TX[i]),16)

		self.SEED_UART_Snd(RS_TX)

	def SEED_DX_TURN(self,ID,SPD):

		bCheckSum = 0
		bCount = 0
		bLength = 9

		SndData = bLength*[0]

		SndData[0] = 0xFF
		SndData[1] = 0xFF
		SndData[2] = ID
		SndData[3] = 0x05	#LENGTH
		SndData[4] = 0x03	#Instruction
		SndData[5] = 0x20	#CMD

		SndData[6] = SPD
		SndData[7] = SPD >> 8

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	def SEED_DX_MOV(self,ID,SPD,TRG):

		bCheckSum = 0
		bCount = 0
		bLength = 11

		SndData = bLength*[0]

		SndData[0] = 0xFF
		SndData[1] = 0xFF
		SndData[2] = ID
		SndData[3] = 0x07	#LENGTH
		SndData[4] = 0x03	#Instruction
		SndData[5] = 0x1E	#CMD
		SndData[6] = TRG
		SndData[7] = TRG >> 8
		SndData[8] = SPD
		SndData[9] = SPD >> 8

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)

	def SEED_DX_MODE(self,ID,CW_Limit,CCW_Limit):
		bCheckSum = 0
		bCount = 0
		bLength = 11

		SndData = bLength*[0]

		SndData[0] = 0xFF
		SndData[1] = 0xFF
		SndData[2] = ID
		SndData[3] = 0x07	#LENGTH
		SndData[4] = 0x03	#Instruction
		SndData[5] = 0x06	#CMD
		SndData[6] = CW_Limit
		SndData[7] = CW_Limit >> 8
		SndData[8] = CCW_Limit
		SndData[9] = CCW_Limit >> 8

		#check sum
		for bCount in range(2,bLength-1,1):
			bCheckSum += int(self.num2hex(SndData[bCount]),16)

		SndData[bLength - 1] =  int(self.num2hex(~bCheckSum),16)

		#num to hex
		for i in range(0,bLength-1):
			SndData[i] = int(self.num2hex(SndData[i]),16)

		self.SEED_UART_Snd(SndData)


	#####################################################################################################

	#####################################################################################################
	########################## Debug Function  ##################################
	#####################################################################################################
	def Debug_Read(self):

		Data = []

		while(self.ser.inWaiting() < 1):
			pass

		now = (time.time() % 1.0 * 1000)

		print "time is %3d [ms]" % now

		sys.stdout.write("Receive Data \t:")

		RcvData = ''.join("{:02X}".format(ord(c)) for c in self.ser.read(1))

		sys.stdout.write(str(RcvData))

		timeout = time.time() + 0.002
		return_flag = 0

		while (return_flag == 0):
			timeout = time.time() + 0.002
			while(self.ser.inWaiting() < 1):
				if(time.time() > timeout):
					print ""
					return_flag = 1
					break
				else :
					pass
			if (return_flag == 0):
				#Data = Data + RcvData
				Data.append(RcvData)
				RcvData = ''.join("{:02X}".format(ord(c)) for c in self.ser.read(1))
				sys.stdout.write(str(RcvData))
			else :
				pass

		Data.append(RcvData)
		Data.insert(0,int(now))

		#return [int(now),Data]

		return Data

