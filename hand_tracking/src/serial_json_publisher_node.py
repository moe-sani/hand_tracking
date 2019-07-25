#!/usr/bin/env python
#This publishes up to 4 sources dynamicly

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
		# print("iterate number:======================{}=====================:".format(i))
		temp_pose = Pose()
		temp_pose.orientation=extract_quaternion(jdata[jason_stream_lables[i]])
		# print ("temp pose:")
		# print(temp_pose)
		pose_list.append(temp_pose)
		# pose_list.append(i)
		# print("pose_list:")
		# print(pose_list)

	# pub_S0 = rospy.Publisher('/imu_S0', Quaternion, queue_size=1)
	# pub_S1 = rospy.Publisher('/imu_S1', Quaternion, queue_size=1)
	# pub_S0_euler = rospy.Publisher('/imu_S0_euler', Point, queue_size=1)
	# pub_S1_euler = rospy.Publisher('/imu_S1_euler', Point, queue_size=1)
	# pub_eu_rel = rospy.Publisher('/pub_eu_rel', Point, queue_size=1)

	# Sensor0=pose_list[0]
	# Sensor1=pose_list[1]
	# pub_S0.publish(Sensor0.orientation)
	# pub_S1.publish(Sensor1.orientation)


	# euler1=Point()
	# euler2=Point()
	# [euler1.x,euler1.y ,euler1.z ] = euler_from_quaternion([Sensor0.orientation.x, Sensor0.orientation.y, Sensor0.orientation.z, Sensor0.orientation.w])
	# [euler2.x,euler2.y ,euler2.z ] = euler_from_quaternion([Sensor1.orientation.x, Sensor1.orientation.y, Sensor1.orientation.z, Sensor1.orientation.w])
	# pub_S0_euler.publish(euler1)
	# pub_S1_euler.publish(euler2)

	# output = Point()
	# output.x=euler1.x-euler2.x
	# output.y=euler1.y-euler2.y
	# output.z=euler1.z-euler2.z
	# pub_eu_rel.publish(output)
	# print(pose_list)
	return pose_list



def talker():
	rospy.init_node('Serial_Publisher', anonymous=True)


	pub_S0 = rospy.Publisher('/imu_S0', Quaternion, queue_size=1)
	# pub_S0o = rospy.Publisher('/imu_S0o', Point, queue_size=1)
	pub_S1 = rospy.Publisher('/imu_S1', Quaternion, queue_size=1)
	pub_S2 = rospy.Publisher('/imu_S2', Quaternion, queue_size=1)
	pub_S3 = rospy.Publisher('/imu_S3', Quaternion, queue_size=1)
	pub_S4 = rospy.Publisher('/imu_S4', Quaternion, queue_size=1)
	pub_S5 = rospy.Publisher('/imu_S5', Quaternion, queue_size=1)
	pub_S6 = rospy.Publisher('/imu_S6', Quaternion, queue_size=1)
	pub_S7 = rospy.Publisher('/imu_S7', Quaternion, queue_size=1)
	pub_S8 = rospy.Publisher('/imu_S8', Quaternion, queue_size=1)
	pub_S9 = rospy.Publisher('/imu_S9', Quaternion, queue_size=1)
	pub_S10= rospy.Publisher('/imu_S10', Quaternion, queue_size=1)
	pub_S11 = rospy.Publisher('/imu_S11', Quaternion, queue_size=1)
	pub_SC = rospy.Publisher('/scale', Point, queue_size=1)
	# pub_S12 = rospy.Publisher('/imu_S12', Point, queue_size=1)
	# pub_S13 = rospy.Publisher('/imu_S13', Point, queue_size=1)
	pubArray = rospy.Publisher('/imu_pub_array', PoseArray, queue_size=1)
	rate = rospy.Rate(1000) # 10hz
	# ser = serial.Serial('/dev/ttyUSB0')  # open serial port
	# print(ser.name)  # check which port was really used

	jason_stream_lables = rospy.get_param('/jason_stream_lables')


	ser.close()
	ser.open()
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

			imu_array=PoseArray()
			imu_array.header.stamp = rospy.get_rostime()
			imu_array.header.frame_id='/world'
			imu_array.poses=extract_pose_list(dataj,jason_stream_lables)

			# print("extract_pose_list(dataj):=======================================")
			# print(extract_pose_list(dataj))
			pubArray.publish(imu_array)


			# For debugging ================

			# orientation_list = [Sensor_point0.x, Sensor_point0.y, Sensor_point0.z, Sensor_point0.w]
			# (Sensor_point0o.x, Sensor_point0o.y, Sensor_point0o.z) = euler_from_quaternion(orientation_list)
			# Sensor_point0o.x=math.degrees(Sensor_point0o.x)
			# Sensor_point0o.y=math.degrees(Sensor_point0o.y)
			# Sensor_point0o.z=math.degrees(Sensor_point0o.z)
			# roll, pitch, yaw



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

# # TODO: change them to function
# Sensor0_quat = Quaternion()
# Sensor1_quat = Quaternion()
# Sensor2_quat = Quaternion()
# Sensor3_quat = Quaternion()
# Sensor4_quat = Quaternion()
# Sensor5_quat = Quaternion()
# Sensor6_quat = Quaternion()
# Sensor7_quat = Quaternion()
# Sensor8_quat = Quaternion()
# Sensor9_quat = Quaternion()
# Sensor10_quat = Quaternion()
# Sensor11_quat = Quaternion()
#
# Sensor0_quat.x = float(dataj['S0'][0])/10000
# Sensor0_quat.y = float(dataj['S0'][1])/10000
# Sensor0_quat.z = float(dataj['S0'][2])/10000
# Sensor0_quat.w = float(dataj['S0'][3])/10000
# Sensor1_quat.x = float(dataj['S1'][0]) / 10000
# Sensor1_quat.y = float(dataj['S1'][1]) / 10000
# Sensor1_quat.z = float(dataj['S1'][2]) / 10000
# Sensor1_quat.w = float(dataj['S1'][3]) / 10000
# Sensor2_quat.x = float(dataj['S2'][0]) / 10000
# Sensor2_quat.y = float(dataj['S2'][1]) / 10000
# Sensor2_quat.z = float(dataj['S2'][2]) / 10000
# Sensor2_quat.w = float(dataj['S2'][3]) / 10000
# Sensor3_quat.x = float(dataj['S3'][0]) / 10000
# Sensor3_quat.y = float(dataj['S3'][1]) / 10000
# Sensor3_quat.z = float(dataj['S3'][2]) / 10000
# Sensor3_quat.w = float(dataj['S3'][3]) / 10000
# Sensor4_quat.x = float(dataj['S4'][0]) / 10000
# Sensor4_quat.y = float(dataj['S4'][1]) / 10000
# Sensor4_quat.z = float(dataj['S4'][2]) / 10000
# Sensor4_quat.w = float(dataj['S4'][3]) / 10000
# Sensor5_quat.x = float(dataj['S5'][0]) / 10000
# Sensor5_quat.y = float(dataj['S5'][1]) / 10000
# Sensor5_quat.z = float(dataj['S5'][2]) / 10000
# Sensor5_quat.w = float(dataj['S5'][3]) / 10000
# Sensor6_quat.x = float(dataj['S6'][0]) / 10000
# Sensor6_quat.y = float(dataj['S6'][1]) / 10000
# Sensor6_quat.z = float(dataj['S6'][2]) / 10000
# Sensor6_quat.w = float(dataj['S6'][3]) / 10000
# Sensor7_quat.x = float(dataj['S7'][0]) / 10000
# Sensor7_quat.y = float(dataj['S7'][1]) / 10000
# Sensor7_quat.z = float(dataj['S7'][2]) / 10000
# Sensor7_quat.w = float(dataj['S7'][3]) / 10000
# Sensor8_quat.x = float(dataj['S8'][0]) / 10000
# Sensor8_quat.y = float(dataj['S8'][1]) / 10000
# Sensor8_quat.z = float(dataj['S8'][2]) / 10000
# Sensor8_quat.w = float(dataj['S8'][3]) / 10000
# Sensor9_quat.x = float(dataj['S9'][0]) / 10000
# Sensor9_quat.y = float(dataj['S9'][1]) / 10000
# Sensor9_quat.z = float(dataj['S9'][2]) / 10000
# Sensor9_quat.w = float(dataj['S9'][3]) / 10000
# Sensor10_quat.x = float(dataj['S10'][0]) / 10000
# Sensor10_quat.y = float(dataj['S10'][1]) / 10000
# Sensor10_quat.z = float(dataj['S10'][2]) / 10000
# Sensor10_quat.w = float(dataj['S10'][3]) / 10000
# Sensor11_quat.x = float(dataj['S11'][0]) / 10000
# Sensor11_quat.y = float(dataj['S11'][1]) / 10000
# Sensor11_quat.z = float(dataj['S11'][2]) / 10000
# Sensor11_quat.w = float(dataj['S11'][3]) / 10000
