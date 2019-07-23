#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from odas_msgs.msg import Odas, OdasList
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64
from hand_tracking.msg import Float64ArrayStamped
from geometry_msgs.msg import TwistStamped
import numpy
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import math
import tf


offset_buff=[]

def brodcast_tf_now(br,traslation,orientaion,frame_id,refrence_frame):
    br.sendTransform(traslation,
					 orientaion,
					 rospy.Time.now(),
					 frame_id,
					 refrence_frame)

def tf_brodcaster(pose_list):

    tf_prodcaster_list=[]
    for i in range(len(pose_list)):
        tf_prodcaster_list.append(tf.TransformBroadcaster())

    sensor_n_list = rospy.get_param('/sensor_n_list')
    sensor_f_list = rospy.get_param('/sensor_f_list')
    sensor_rf_list = rospy.get_param('/sensor_rf_list')
    hft_tf_translations = rospy.get_param('/hft_tf')


    for i,pose in enumerate(pose_list):
        temp_tr=hft_tf_translations[sensor_f_list[i]]
        # print("temp_tr",temp_tr)
        brodcast_tf_now(tf_prodcaster_list[i],
						(temp_tr[0],temp_tr[1],temp_tr[2]),(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
						sensor_f_list[i],sensor_rf_list[i])
    return 0


def imu_array_store(imu_pose_list):
    global offset_buff
    #store 100 values
    quaternion_buff=[]
    for idx,pose in enumerate(imu_pose_list):
        # store quaternion values
        temp_quaternion=[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        quaternion_buff.append(temp_quaternion)
    offset_buff.append(quaternion_buff)


def calc_mean_of_buff(offset_buff):
    # offset_buff
    #calculate and return mean
    return quat_offset_list
    #substract imu pose list from offset values

def subtract_offset(imu_pose_list,imu_offset_list):
    imu_pose_array_no_offset=[]
    for pose, pose_offset in zip(imu_pose_list,imu_offset_list):
        q1=pose.orientation
        q2=pose_offset.orientation
        q = quaternion_multiply([q1.x, q1.y, q1.z, q1.w], [q2.x, q2.y, q2.z, -q2.w])

        new_pose=Pose()
        new_pose.orientation.x=q[0]
        new_pose.orientation.y=q[1]
        new_pose.orientation.z=q[2]
        new_pose.orientation.w=q[3]
        imu_pose_array_no_offset.append(new_pose)

    return imu_pose_array_no_offset







def imu_array_callback(imu_pose_array):
    global offset_buff
    imu_pose_list=imu_pose_array.poses
    if len(offset_buff)<1 :
        # imu_array_store(imu_pose_list)
        offset_buff=imu_pose_list
    else:
        # quat_offset_list=calc_mean_of_buff(offset_buff)

        imu_pose_array_no_offset=subtract_offset(imu_pose_list,offset_buff)
        tf_brodcaster(imu_pose_array_no_offset)

    # for sensor_id, sensor_values in enumerate(data.poses):
    #     print("sensor{}:{}".format(sensor_id, sensor_values))
    rospy.loginfo("test")



def main():
    rospy.init_node('hand_joint_mapping', anonymous=True)
    rospy.Subscriber("/imu_pub_array", PoseArray, imu_array_callback, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    main()

