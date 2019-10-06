#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import serial
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64
from std_msgs.msg import Header
import json
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import math
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped


ser = serial.Serial('/dev/ttyACM0', 115200, timeout=10)		#ttyUSB0

# temp_pose=Pose()

# lst=[temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose,temp_pose]



def twos_comp(val, bits):
	"""compute the 2's complement of int value val"""
	if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
		val = val - (1 << bits)        # compute negative value
	return val                         # return positive value as is

def lowbyte_highbyte_to_float(highbyte,lowbyte):
	# print("high:{} low:{}".format(bin(ord(highbyte)),bin(ord(lowbyte))))
	result = twos_comp((int(ord(highbyte)) << 8) + int(ord(lowbyte)),16)
	return result


def extract_quaternion(data):
	# print(data)
	Sensor_quat = Quaternion()
	Sensor_quat.x = float(data[0]) / 10000
	Sensor_quat.y = float(data[1]) / 10000
	Sensor_quat.z = float(data[2]) / 10000
	Sensor_quat.w = float(data[3]) / 10000
	return Sensor_quat

def extract_pose_list(jdata,jason_stream_lables):
	pose_list = []
	for i in range(0, len(jdata)-1):
		temp_pose = Pose()
		temp_pose.orientation = extract_quaternion(jdata[jason_stream_lables[i]])
		euler = Point()
		[euler.x, euler.y, euler.z] = euler_from_quaternion([temp_pose.orientation.x, temp_pose.orientation.y, temp_pose.orientation.z, temp_pose.orientation.w])
		temp_pose.position=euler
		pose_list.append(temp_pose)

	return pose_list


def serial_parser():
	rospy.init_node('Serial_Publisher', anonymous=True)
	pub_SC = rospy.Publisher('/tool_status', Float64, queue_size=1)
	pubArray = rospy.Publisher('/imu_pub_array', PoseArray, queue_size=1)
	rate = rospy.Rate(1000) # 10hz
	jason_stream_lables = rospy.get_param('/jason_stream_lables')
	ser.close()
	ser.open()
	while not rospy.is_shutdown():
		# hello_str = "Serial_Publisher Node spinning... %s" % rospy.get_time()
		# rospy.loginfo(hello_str)
		line = ser.readline()  # read a '\n' terminated line
		# print("raw data:{}".format(line))
		if len(line)>13:
			dataj = json.loads(line)
			# print(dataj)
			if 'SC'in dataj:
				# print("SC:")
				# print(dataj['SC'][0])
				sc_data = Float64()
				sc_data = float(dataj['SC'][0])
				pub_SC.publish(sc_data)

			if 'S0' in dataj:
				print('data:')
				print(dataj)
				imu_array=PoseArray()
				imu_array.header.stamp = rospy.get_rostime()
				imu_array.header.frame_id='/world'
				imu_array.poses=extract_pose_list(dataj,jason_stream_lables)
				pubArray.publish(imu_array)
				# print("extract_pose_list(dataj):=======================================")
				# print(extract_pose_list(dataj))
			elif 'C8' in dataj:
				print("calibration", dataj)
				pass
		rate.sleep()

if __name__ == '__main__':
	try:
		serial_parser()
	except rospy.ROSInterruptException:
		pass

	finally:
		print ('closing port')
		ser.close()
		# sock.close()
