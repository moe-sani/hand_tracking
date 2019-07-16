#!/usr/bin/env python
#This publishes up to 4 sources dynamicly

# license removed for brevity
import rospy
from std_msgs.msg import String
import serial
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64
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
	pub_S0 = rospy.Publisher('/imu_S0', PoseStamped, queue_size=1)
	pub_S1 = rospy.Publisher('/imu_S1', PoseStamped, queue_size=1)
	pub_S2 = rospy.Publisher('/imu_S2', PoseStamped, queue_size=1)
	pub_S3 = rospy.Publisher('/imu_S3', PoseStamped, queue_size=1)
	pub_S4 = rospy.Publisher('/imu_S4', PoseStamped, queue_size=1)
	pub_S5 = rospy.Publisher('/imu_S5', PoseStamped, queue_size=1)
	pub_S6 = rospy.Publisher('/imu_S6', PoseStamped, queue_size=1)
	pub_S7 = rospy.Publisher('/imu_S7', PoseStamped, queue_size=1)
	pub_S8 = rospy.Publisher('/imu_S8', PoseStamped, queue_size=1)
	pub_S9 = rospy.Publisher('/imu_S9', PoseStamped, queue_size=1)
	pub_S10= rospy.Publisher('/imu_S10', PoseStamped, queue_size=1)
	pub_S11 = rospy.Publisher('/imu_S11', PoseStamped, queue_size=1)
	pub_SC = rospy.Publisher('/scale', PoseStamped, queue_size=1)
	# pub_S12 = rospy.Publisher('/imu_S12', Point, queue_size=1)
	# pub_S13 = rospy.Publisher('/imu_S13', Point, queue_size=1)
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

		line = ser.readline()  # read a '\n' terminated line
		print("raw data:{}".format(line))
		if len(line)>13:
			dataj = json.loads(line)
			# print(dataj)
			# print("SC:")
			# print(dataj['SC'][0])
			Sensor_pose0 = Pose()
			Sensor_pose0.position.x = float(dataj['S0'][0]) / 10
			Sensor_pose0.position.y = float(dataj['S0'][1]) / 10
			Sensor_pose0.position.z = float(dataj['S0'][2]) / 10
			Sensor_posest0=PoseStamped()
			Sensor_posest0.pose=Sensor_pose0
			Sensor_posest0.header.stamp = rospy.get_rostime()
			Sensor_posest0.header.frame_id = '/world'



			Sensor_pose1 = Pose()
			# TODO: change all to orientations
			Sensor_pose1.position.x = float(dataj['S1'][0]) / 10
			Sensor_pose1.position.y = float(dataj['S1'][1]) / 10
			Sensor_pose1.position.z = float(dataj['S1'][2]) / 10
			Sensor_posest1=PoseStamped()
			Sensor_posest1.pose=Sensor_pose1
			Sensor_posest1.header.stamp = rospy.get_rostime()
			Sensor_posest1.header.frame_id = '/world'

			Sensor_pose2 = Pose()
			Sensor_pose2.position.x = float(dataj['S2'][0]) / 10
			Sensor_pose2.position.y = float(dataj['S2'][1]) / 10
			Sensor_pose2.position.z = float(dataj['S2'][2]) / 10
			Sensor_posest2=PoseStamped()
			Sensor_posest2.pose=Sensor_pose2
			Sensor_posest2.header.stamp = rospy.get_rostime()
			Sensor_posest2.header.frame_id = '/world'

			Sensor_pose3 = Pose()
			Sensor_pose3.position.x = float(dataj['S3'][0]) / 10
			Sensor_pose3.position.y = float(dataj['S3'][1]) / 10
			Sensor_pose3.position.z = float(dataj['S3'][2]) / 10
			Sensor_posest3=PoseStamped()
			Sensor_posest3.pose=Sensor_pose3
			Sensor_posest3.header.stamp = rospy.get_rostime()
			Sensor_posest3.header.frame_id = '/world'

			Sensor_pose4 = Pose()
			Sensor_pose4.position.x = float(dataj['S4'][0]) / 10
			Sensor_pose4.position.y = float(dataj['S4'][1]) / 10
			Sensor_pose4.position.z = float(dataj['S4'][2]) / 10
			Sensor_posest4=PoseStamped()
			Sensor_posest4.pose=Sensor_pose4
			Sensor_posest4.header.stamp = rospy.get_rostime()
			Sensor_posest4.header.frame_id = '/world'

			Sensor_pose5 = Pose()
			Sensor_pose5.position.x = float(dataj['S5'][0]) / 10
			Sensor_pose5.position.y = float(dataj['S5'][1]) / 10
			Sensor_pose5.position.z = float(dataj['S5'][2]) / 10
			Sensor_posest5=PoseStamped()
			Sensor_posest5.pose=Sensor_pose5
			Sensor_posest5.header.stamp = rospy.get_rostime()
			Sensor_posest5.header.frame_id = '/world'

			Sensor_pose6 = Pose()
			Sensor_pose6.position.x = float(dataj['S6'][0]) / 10
			Sensor_pose6.position.y = float(dataj['S6'][1]) / 10
			Sensor_pose6.position.z = float(dataj['S6'][2]) / 10
			Sensor_posest6=PoseStamped()
			Sensor_posest6.pose=Sensor_pose6
			Sensor_posest6.header.stamp = rospy.get_rostime()
			Sensor_posest6.header.frame_id = '/world'

			Sensor_pose7 = Pose()
			Sensor_pose7.position.x = float(dataj['S7'][0]) / 10
			Sensor_pose7.position.y = float(dataj['S7'][1]) / 10
			Sensor_pose7.position.z = float(dataj['S7'][2]) / 10
			Sensor_posest7=PoseStamped()
			Sensor_posest7.pose=Sensor_pose7
			Sensor_posest7.header.stamp = rospy.get_rostime()
			Sensor_posest7.header.frame_id = '/world'

			Sensor_pose8 = Pose()
			Sensor_pose8.position.x = float(dataj['S8'][0]) / 10
			Sensor_pose8.position.y = float(dataj['S8'][1]) / 10
			Sensor_pose8.position.z = float(dataj['S8'][2]) / 10
			Sensor_posest8=PoseStamped()
			Sensor_posest8.pose=Sensor_pose8
			Sensor_posest8.header.stamp = rospy.get_rostime()
			Sensor_posest8.header.frame_id = '/world'

			Sensor_pose9 = Pose()
			Sensor_pose9.position.x = float(dataj['S9'][0]) / 10
			Sensor_pose9.position.y = float(dataj['S9'][1]) / 10
			Sensor_pose9.position.z = float(dataj['S9'][2]) / 10
			Sensor_posest9=PoseStamped()
			Sensor_posest9.pose=Sensor_pose9
			Sensor_posest9.header.stamp = rospy.get_rostime()
			Sensor_posest9.header.frame_id = '/world'

			Sensor_pose10 = Pose()
			Sensor_pose10.position.x = float(dataj['S10'][0]) / 10
			Sensor_pose10.position.y = float(dataj['S10'][1]) / 10
			Sensor_pose10.position.z = float(dataj['S10'][2]) / 10
			Sensor_posest10=PoseStamped()
			Sensor_posest10.pose=Sensor_pose10
			Sensor_posest10.header.stamp = rospy.get_rostime()
			Sensor_posest10.header.frame_id = '/world'

			Sensor_pose11 = Pose()
			Sensor_pose11.position.x = float(dataj['S11'][0]) / 10
			Sensor_pose11.position.y = float(dataj['S11'][1]) / 10
			Sensor_pose11.position.z = float(dataj['S11'][2]) / 10
			Sensor_posest11=PoseStamped()
			Sensor_posest11.pose=Sensor_pose11
			Sensor_posest11.header.stamp = rospy.get_rostime()
			Sensor_posest11.header.frame_id = '/world'

			Sensor_poseSC = Pose()
			Sensor_poseSC.position.x = float(dataj['SC'][0]) / 10
			Sensor_posestSC=PoseStamped()
			Sensor_posestSC.pose=Sensor_poseSC
			Sensor_posestSC.header.stamp = rospy.get_rostime()
			Sensor_posestSC.header.frame_id = '/world'

			pub_S0.publish(Sensor_posest0)
			pub_S1.publish(Sensor_posest1)
			pub_S2.publish(Sensor_posest2)
			pub_S3.publish(Sensor_posest3)
			pub_S4.publish(Sensor_posest4)
			pub_S5.publish(Sensor_posest5)
			pub_S6.publish(Sensor_posest6)
			pub_S7.publish(Sensor_posest7)
			pub_S8.publish(Sensor_posest8)
			pub_S9.publish(Sensor_posest9)
			pub_S10.publish(Sensor_posest10)
			pub_S11.publish(Sensor_posest11)
			pub_SC.publish(Sensor_posestSC)

			lst[0] =Sensor_pose0
			lst[1] =Sensor_pose1
			lst[2] =Sensor_pose2
			lst[3] =Sensor_pose3
			lst[4] =Sensor_pose4
			lst[5] =Sensor_pose5
			lst[6] =Sensor_pose6
			lst[7] =Sensor_pose7
			lst[8] =Sensor_pose8
			lst[9] =Sensor_pose9
			lst[10] =Sensor_pose10
			lst[11] =Sensor_pose11
			lst[12] =Sensor_poseSC

			imu_array.header.stamp=rospy.get_rostime()
			imu_array.header.frame_id='/world'
			imu_array.poses=lst
			pubArray.publish(imu_array)
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
