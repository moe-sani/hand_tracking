#!/usr/bin/env python
#This publishes up to 4 sources dynamicly

# license removed for brevity
import rospy
from std_msgs.msg import String
import serial
from geometry_msgs.msg import Point

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=10)


def twos_comp(val, bits):
	"""compute the 2's complement of int value val"""
	if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
		val = val - (1 << bits)        # compute negative value
	return val                         # return positive value as is


def lowbyte_highbyte_to_float(highbyte,lowbyte):
	# print("high:{} low:{}".format(bin(ord(highbyte)),bin(ord(lowbyte))))
	result = twos_comp((int(ord(highbyte)) << 8) + int(ord(lowbyte)),16)
	return result



def talker():
	pub = rospy.Publisher('/imu_pub', Point, queue_size=1)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(200) # 10hz
	# ser = serial.Serial('/dev/ttyUSB0')  # open serial port
	# print(ser.name)  # check which port was really used
	ser.close()
	ser.open()

	while not rospy.is_shutdown():


		hello_str = "hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)


		# with serial.Serial('/dev/ttyUSB0', 115200, timeout=2) as ser:

		# rospy.loginfo("open prt")
		# x = ser.read()  # read one byte
		# s = ser.read(10)  # read up to ten bytes (timeout)
		line = ser.readline()  # read a '\n' terminated line
		# rospy.loginfo(line)
		# print("rawdata:".format(line))

		for i in range(len(line)) :
			print("num {}: {}".format(i,hex(ord(line[i]))))

		# print("ch0:".format(hex(ord(line[1]))))

		# TODO check length first
		if len(line)>13:
			if ord(line[0])==160:
				S0AX = lowbyte_highbyte_to_float(line[1],line[2])
				S0AY = lowbyte_highbyte_to_float(line[3],line[4])
				S0AZ = lowbyte_highbyte_to_float(line[5],line[6])
				S0BX = lowbyte_highbyte_to_float(line[7],line[8])
				S0BY = lowbyte_highbyte_to_float(line[9],line[10])
				S0BZ = lowbyte_highbyte_to_float(line[11],line[12])
				# print("S0AX: {}, Y: {} ,Z: {}; S0BX: {}, Y:{}, Z:{}".format(S0AX,S0AY,S0AZ,S0BX,S0BY,S0BZ))
				sensorA_point=Point()
				sensorA_point.x =float(S0AX)/10
				sensorA_point.y =float(S0AY)/10
				sensorA_point.z =float(S0AZ)/10
				pub.publish(sensorA_point)
			elif ord(line[0])==176:
				# print("rawdata:".format(line))
				rospy.loginfo(line)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

	finally:
		print ('closing port')
		ser.close()
		# sock.close()
