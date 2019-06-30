#!/usr/bin/env python
#This publishes up to 4 sources dynamicly

# license removed for brevity
import rospy
from std_msgs.msg import String
import serial
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Header
import json
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=10)		#ttyUSB0

temp_pose=Pose()

lst=[temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose]


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
	pub_S0 = rospy.Publisher('/imu_S0', Point, queue_size=1)
	pub_S1 = rospy.Publisher('/imu_S1', Point, queue_size=1)
	pub_S2 = rospy.Publisher('/imu_S2', Point, queue_size=1)
	pub_S3 = rospy.Publisher('/imu_S3', Point, queue_size=1)
	pub_S4 = rospy.Publisher('/imu_S4', Point, queue_size=1)
	pub_S5 = rospy.Publisher('/imu_S5', Point, queue_size=1)
	pub_S6 = rospy.Publisher('/imu_S6', Point, queue_size=1)
	pub_S7 = rospy.Publisher('/imu_S7', Point, queue_size=1)
	pub_S8 = rospy.Publisher('/imu_S8', Point, queue_size=1)
	pub_S9 = rospy.Publisher('/imu_S9', Point, queue_size=1)
	pub_S10= rospy.Publisher('/imu_S10', Point, queue_size=1)
	pub_S11 = rospy.Publisher('/imu_S11', Point, queue_size=1)
	pub_S12 = rospy.Publisher('/imu_S12', Point, queue_size=1)
	pub_S13 = rospy.Publisher('/imu_S13', Point, queue_size=1)
	pubArray = rospy.Publisher('/imu_pub_array', PoseArray, queue_size=1)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(1000) # 10hz
	# ser = serial.Serial('/dev/ttyUSB0')  # open serial port
	# print(ser.name)  # check which port was really used
	ser.close()
	ser.open()
	imu_array=PoseArray()
	myheader = Header()
	while not rospy.is_shutdown():


		hello_str = "Serial_Publisher Node spinning... %s" % rospy.get_time()
		rospy.loginfo(hello_str)


		# with serial.Serial('/dev/ttyUSB0', 115200, timeout=2) as ser:

		# rospy.loginfo("open prt")
		# x = ser.read()  # read one byte
		# s = ser.read(10)  # read up to ten bytes (timeout)
		line = ser.readline()  # read a '\n' terminated line
		print("raw data:{}".format(line))
		if len(line)>13:
			dataj = json.loads(line)
			print(dataj)
			print(dataj['S0'])

		# print type(dataj)
		# rospy.loginfo(line)
		# print("rawdata:".format(line))

		# for i in range(len(line)) :
		# 	print("num {}: {}".format(i,hex(ord(line[i]))))

		# print("ch0:".format(hex(ord(line[1]))))

		# TODO check length first
		# if len(line)>13:
		#
		# 	S0AZ = lowbyte_highbyte_to_float(line[1], line[2])
		# 	S0AY = lowbyte_highbyte_to_float(line[3], line[4])
		# 	S0AX = lowbyte_highbyte_to_float(line[5], line[6])
		# 	S0BZ = lowbyte_highbyte_to_float(line[7], line[8])
		# 	S0BY = lowbyte_highbyte_to_float(line[9], line[10])
		# 	S0BX = lowbyte_highbyte_to_float(line[11], line[12])
		# 	# print("S0AX: {}, Y: {} ,Z: {}; S0BX: {}, Y:{}, Z:{}".format(S0AX,S0AY,S0AZ,S0BX,S0BY,S0BZ))
		# 	Sensor_poseA = Pose()
		# 	Sensor_poseA.position.x = float(S0AX) / 10
		# 	Sensor_poseA.position.y = float(S0AY) / 10
		# 	Sensor_poseA.position.z = float(S0AZ) / 10
		#
		# 	Sensor_poseB = Pose()
		# 	Sensor_poseB.position.x = float(S0BX) / 10
		# 	Sensor_poseB.position.y = float(S0BY) / 10
		# 	Sensor_poseB.position.z = float(S0BZ) / 10
		#
		# 	# sensorA_point=Point()
		# 	# sensorA_point.x =float(S0AX)/10
		# 	# sensorA_point.y =float(S0AY)/10
		# 	# sensorA_point.z =float(S0AZ)/10
		# 	# pub.publish(sensorA_point)
		#
		# 	if ord(line[0])==176:
		# 		# print("rawdata:".format(line))
		# 		rospy.loginfo(line)
		# 	elif ord(line[0])==160:
		# 		lst[0] = Sensor_poseA
		# 		lst[1] = Sensor_poseB
		# 		pub_S0.publish(Sensor_poseA.position)
		# 		pub_S1.publish(Sensor_poseB.position)
		# 	elif ord(line[0])==161:
		# 		lst[2] = Sensor_poseA
		# 		lst[3] = Sensor_poseB
		# 		pub_S2.publish(Sensor_poseA.position)
		# 		pub_S3.publish(Sensor_poseB.position)
		# 	elif ord(line[0]) == 162:
		# 		lst[4] = Sensor_poseA
		# 		lst[5] = Sensor_poseB
		# 		pub_S4.publish(Sensor_poseA.position)
		# 		pub_S5.publish(Sensor_poseB.position)
		# 	elif ord(line[0]) == 163:
		# 		lst[6] = Sensor_poseA
		# 		lst[7] = Sensor_poseB
		# 		pub_S6.publish(Sensor_poseA.position)
		# 		pub_S7.publish(Sensor_poseB.position)
		# 	elif ord(line[0]) == 164:
		# 		lst[8] = Sensor_poseA
		# 		lst[9] = Sensor_poseB
		# 		pub_S8.publish(Sensor_poseA.position)
		# 		pub_S9.publish(Sensor_poseB.position)
		# 	elif ord(line[0]) == 165:
		# 		lst[10] = Sensor_poseA
		# 		lst[11] = Sensor_poseB
		# 		pub_S10.publish(Sensor_poseA.position)
		# 		pub_S11.publish(Sensor_poseB.position)
		# 	elif ord(line[0]) == 166:
		# 		lst[12] = Sensor_poseA
		# 		lst[13] = Sensor_poseB
		# 		pub_S12.publish(Sensor_poseA.position)
		# 		pub_S13.publish(Sensor_poseB.position)
		#
		# 	# else :
		# 	# 	line_index=ord(line[0])-160
		# 	# 	print("line index:{}".format(line_index))
		# 	# 	lst[line_index]=Sensor_pose
		#
		# 	# myheader.stamp=
		# 	imu_array.header.stamp=rospy.get_rostime()
		# 	imu_array.header.frame_id='/world'
		# 	imu_array.poses=lst
		# 	pubArray.publish(imu_array)
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
