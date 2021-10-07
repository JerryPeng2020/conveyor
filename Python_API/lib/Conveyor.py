#!/usr/bin/python3 

# Project: Python Library for Conveyor  ( Version 0.1 -- prototype! ) 
# Author: Jerry Peng
# Date:   Sep. 27th, 2020

import serial 


BAUD_RATE = 115200      # baud rate of robot serial port

# Register ID 
# ROM
DEVICE_TYPE_ID = 0
VERSION_ID = 1
MAC_ID = 2
#EEPROM
# EEPROM_LEN = 9
# EEPROM_ID = 11
DEVICE_ID = 11
# BAUDRATE_ID = 12
# OFFSET_ID = 13
#RAM 
SPEED_ID = 30
# EEPROM_LOCK_ID = 28
# MOTOR_STATUS_ID = 29
# EFFECTOR_ID = 30
# VACUUM_ID = 31
# SPEED_ID = 32
# TIME_ID = 39
# ANGLE_ID = 46
# END_LENGTH_ID = 53
# IK_ID = 54
# IK_DATA_LENGTH = 9
# IK7_DATA_LENGTH = 12
# ANGLE_FEEDBACK_FREQ_ID = 82
# ANGLE_FEEDBACK_ID = 83
# LOAD_FEEDBACK_FREQ_ID = 90
# LOAD_FEEDBACK_ID = 91

class Conveyor: 
	""" The interface on host machine with 7Bot robot """

	def __init__(self, port): 
		self.ser = serial.Serial(port, BAUD_RATE, timeout = 0.2) 

	def readReg(self, addr, num): 
		# asking for register information 
		buf = [ 0x03, addr & 0xff, num & 0xff ] 
		self.writeSerial(buf) 
		
		ret_pack = self.readSerial()
		if (ret_pack.pop(0) != 0x03):  
			raise serial.SerialException("mismatching pack type") 
		return ret_pack[2:] 
		
	def writeReg(self, addr: int, data: list): 
		buf = [ 0x04, addr & 0xff, len(data) & 0xff ] 
		for d in data: 
			buf.append(d & 0xff) 
		self.writeSerial(buf) 
		# information feedback from robot is not required so far 
	

	
	# Get Functions #

	# get decice code
	def getDeviceCode(self):
		return self.readReg(DEVICE_TYPE_ID, 1)[0]

	# get version
	def getVersion(self):
		return self.readReg(VERSION_ID, 1)[0]/10
	
	# get MAC address(lenmgth: 6 bytes)
	def getMAC(self):
		 return self.listToString(self.readReg(MAC_ID, 6))
	# Function to convert   
	def listToString(self, l: list):  
		# initialize an empty string 
		str1 = ""  
		# traverse in the string   
		for ele in l:  
			str1 += hex(ele)[2:]
		# return string   
		return str1  

	# get ID
	def getID(self):
		return self.readReg(DEVICE_ID, 1)[0]



	# Set Functions #

	# set decice code
	def setID(self, ID: int):
		self.writeReg(DEVICE_ID, [ID])

	# set motion speed
    # Speed: range [-127, 127]
    # if Speed = 0, conveyor stop motion
	# if Speed > 0, conveyor move in positive direction; if Speed < 0, move in negative direction.
	def setSpeed(self, speed: int):

		self.writeReg(SPEED_ID, [speed+127])



	def readSerial(self): 
		cnt = 0 
		# read pack head 
		while (True): 
			tmp = self.ser.read() 
			#print(tmp) 
			if (tmp == b'\xaa'): 
				tmp = self.ser.read() 
				if (tmp == b'\x77'): 
					break 
			cnt += 1 
			if (50 == cnt): 
				raise serial.SerialTimeoutException() 
		
		# from here data is to be returned 
		tmp = self.ser.read(3) 
		ret = [ 0xaa, 0x77, tmp[0], tmp[1], tmp[2] ]
		tmp = self.ser.read(ret[4] + 2) 
		for d in tmp: ret.append(d) 
		crc = self.CRC16_MODBUS(ret[0:-2]) 
		if ((crc & 0xff == ret[-2]) and ((crc >> 8) & 0xff == ret[-1])): 
			return ret[2:-2] 
		else: 
			# print(ret)
			raise serial.SerialException("data corrupted") 

	def writeSerial(self, data: list): 
		buf = [ 0xaa, 0x77 ] 
		for tmp in data: buf.append(tmp) 

		crc = self.CRC16_MODBUS(buf) 
		buf.append(crc & 0xff) 
		buf.append((crc >> 8) & 0xff) 

		return self.ser.write(bytes(buf)) == len(buf) 

	def invert8(self, val): 
		ret = i = 0 
		while (i < 8): 
			ret |= ((val >> i) & 0x01) << (7 - i) 
			i += 1 
		return ret & 0xff 

	def invert16(self, val): 
		ret = i = 0 
		while (i < 16): 
			ret |= ((val >> i) & 0x01) << (15 - i) 
			i += 1 
		return ret & 0xffff 

	def CRC16_MODBUS(self, data: list): 
		wCRCin = 0xffff 
		wCPoly = 0x8005

		for tmp in data: 
			tmp = self.invert8(tmp) 
			wCRCin ^= (tmp << 8) & 0xffff
			i = 0 
			while (i < 8): 
				i += 1 
				if ((wCRCin & 0x8000) != 0): 
					wCRCin = 0xffff & ((wCRCin << 1) & 0xffff) ^ wCPoly 
				else: 
					wCRCin = (wCRCin << 1) & 0xffff 
		wCRCin = self.invert16(wCRCin) 
		return wCRCin 

