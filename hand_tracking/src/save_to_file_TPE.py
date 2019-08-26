#!/usr/bin/env python
# safe to file for Tool Pose estimation

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from odas_msgs.msg import Odas, OdasList
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from hand_tracking.msg import Float64ArrayStamped
from geometry_msgs.msg import QuaternionStamped

from geometry_msgs.msg import TwistStamped
import numpy
from geometry_msgs.msg import Quaternion
import math
import message_filters
import tf
import datetime
from tf.transformations import *
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped

zero_pose=Pose()
zero_pose.position.x=0
zero_pose.position.y=0
zero_pose.position.z=0
zero_pose.orientation.x=0
zero_pose.orientation.y=0
zero_pose.orientation.z=0
zero_pose.orientation.w=0
# imu_array = [zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose]

lst_q_joints=[zero_pose.orientation for i in range(0,12)]
lst_p_joints=[zero_pose.orientation for ii in range(0,12)]
lst_q_imu=[zero_pose.orientation for j in range(0,12)]
wrist_pose=zero_pose
tool_pose=zero_pose
listener=0
tool_status=0


now = datetime.datetime.now()
str_filename=now.strftime("/home/moe/smartsurg_logfiles/TPE/%Y-%m-%d_%H-%M-%S.csv")


def conv_quat_to_degree(quat):
    euler_rad=Quaternion()
    orientation_euler=Quaternion()
    [euler_rad.x, euler_rad.y, euler_rad.z] = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    orientation_euler.x=math.degrees(euler_rad.x)
    orientation_euler.y=math.degrees(euler_rad.y)
    orientation_euler.z=math.degrees(euler_rad.z)
    return orientation_euler

def conv_data(data):
    output_r=Point()
    output_d=Point()
    [output_r.x, output_r.y, output_r.z] = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    output_d.x=math.degrees(output_r.x)
    output_d.y=math.degrees(output_r.y)
    output_d.z=math.degrees(output_r.z)
    return output_d

def imu_serial_callback(data):
    global lst_q_imu
    # global S5_4_quat
    sensor_f_list = rospy.get_param('/sensor_f_list')
    jason_stream_lables = rospy.get_param('/jason_stream_lables')
    # lst_q_imu=[]
    # print(data.poses)
    for i, pose in enumerate(data.poses):
        lst_q_imu[i]=pose.orientation
    # rearrange to the organized way
    # q_imu[0] = imu_array[jason_stream_lables.index('S0')].orientation
    # q_imu[1] = imu_array[jason_stream_lables.index('S1')].orientation
    # q_imu[2] = imu_array[jason_stream_lables.index('S2')].orientation
    # q_imu[3] = imu_array[jason_stream_lables.index('S3')].orientation
    # q_imu[4] = imu_array[jason_stream_lables.index('S4')].orientation
    # q_imu[5] = imu_array[jason_stream_lables.index('S5')].orientation
    # q_imu[6] = imu_array[jason_stream_lables.index('S6')].orientation
    # q_imu[7] = imu_array[jason_stream_lables.index('S7')].orientation
    # q_imu[8] = imu_array[jason_stream_lables.index('S8')].orientation
    # q_imu[9] = imu_array[jason_stream_lables.index('S9')].orientation
    # q_imu[10] = imu_array[jason_stream_lables.index('S10')].orientation
    # q_imu[11] = imu_array[jason_stream_lables.index('S11')].orientation

    # publish_all_wrt_world(imu_array)
    # quat_list_cf, euler_list_cf = transform_all_to_cf(imu_array)
    # S5_4_quat = quat_list_cf[sensor_f_list.index('S5')]


def joints_array_callback(joints_array):
    sensor_f_list = rospy.get_param('/sensor_f_list')
    global lst_q_joints
    global lst_p_joints
    for i, pose in enumerate(joints_array.poses):
        lst_q_joints[i]=pose.orientation
        lst_p_joints[i]=pose.position
    # Create a zip object from two lists
    zipbObj_q = zip(sensor_f_list, lst_q_joints)
    zipbObj_p = zip(sensor_f_list, lst_p_joints)
    # Create a dictionary from zip object
    dict_q_joints = dict(zipbObj_q)
    dict_p_joints = dict(zipbObj_p)


def Wrist_callback(data):
    global wrist_pose
    wrist_pose.position = data.pose.position
    # wrist_pose.orientation=conv_quat_to_degree(data.pose.orientation)
    wrist_pose.orientation=data.pose.orientation

def tool_callback(data):
    global tool_pose
    tool_pose = data.pose
    tool_pose.position = data.pose.position
    # tool_pose.orientation=conv_quat_to_degree(data.pose.orientation)
    tool_pose.orientation=data.pose.orientation

def tool_status_callback(data):
    global tool_status
    tool_status=data.data


def return_lst(lst_q):
    '''

    :param lst_q: array of quaternion
    :return: expanded list of that
    '''
    lst=[]
    for q in lst_q:
        lst.append(q.x)
        lst.append(q.y)
        lst.append(q.z)
        lst.append(q.w)
    return lst


def return_str(lst):
    "returns comma separated string of array"
    lst_str = [str(elem) for elem in lst]
    # print(lst_str)
    fullstr = ','.join(lst_str)
    # print(fullstr)
    return fullstr +','

def creat_lables(lst_names,lst_suffix):
    '''
    creats lables for quaternion or point list
    :param lst:
    :return:
    '''
    lables=[]
    for name in lst_names:
        for suffix in lst_suffix:
            lables.append(name+suffix)
    return lables

def save_to_csv(f):
    global lst_q_joints
    global lst_q_imu
    global wrist_pose
    global tool_pose
    global tool_status
    global str_filename

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("---save_to_csv---")
            # print("imu_array:{}".format(imu_array))
            seconds = rospy.get_time()
            f = open(str_filename, "a+")
            f.write('{0},{1},'.format(seconds,tool_status))
            f.write(return_str(return_lst(lst_q_imu)))
            # print(lst_q_imu)
            # print(return_str(return_lst(lst_q_imu)))
            f.write(return_str(return_lst(lst_q_joints)))
            f.write('{0},{1},{2},{3},{4},{5},{6},'
                    .format(wrist_pose.position.x,wrist_pose.position.y,wrist_pose.position.z,
                            wrist_pose.orientation.x,wrist_pose.orientation.y,wrist_pose.orientation.z,
                            wrist_pose.orientation.w))
            f.write('{0},{1},{2},{3},{4},{5},{6},'
                    .format(tool_pose.position.x, tool_pose.position.y, tool_pose.position.z,
                            tool_pose.orientation.x, tool_pose.orientation.y, tool_pose.orientation.z,
                            tool_pose.orientation.w))
            f.write(', \r\n')
            f.close()
        except rospy.ServiceException, e:
            print("save_to_csv  failed: %s" % e)

        rate.sleep()
    rospy.spin()

def main():
    global str_filename
    global listener

    rospy.init_node('smartsurg_savedata', anonymous=True)
    rospy.loginfo("---WELCOME TO Save to file---")
    # rospy.Subscriber("/hand_joints_array", Float64ArrayStamped, hand_joints_callback, queue_size=10)
    rospy.Subscriber("/imu_pub_array", PoseArray, imu_serial_callback, queue_size=10)
    rospy.Subscriber("/joints_array", PoseArray, joints_array_callback, queue_size=10)
    rospy.Subscriber("/tool_status", Float64, tool_status_callback, queue_size=10)
    rospy.Subscriber("/ndi/Tool/position_cartesian_current", PoseStamped, tool_callback, queue_size=10)
    # rospy.Subscriber("/ndi/IndexMarkerM/position_cartesian_current", PoseStamped, Index_callback, queue_size=10)
    # rospy.Subscriber("/ndi/MiddleMarkerM/position_cartesian_current", PoseStamped, Middle_callback, queue_size=10)
    rospy.Subscriber("/ndi/WristMarkerM/position_cartesian_current", PoseStamped, Wrist_callback, queue_size=10)
    # rospy.Subscriber("/ndi/ThumbMarkerM/position_cartesian_current", PoseStamped, Thumb_callback, queue_size=10)
    listener = tf.TransformListener()

    jason_stream_lables = rospy.get_param('/jason_stream_lables')
    sensor_f_list = rospy.get_param('/sensor_f_list')
    f = open(str_filename, "a+")
    f.write('seconds,tool_status,')

    q_imu_lables = creat_lables(jason_stream_lables,['.x','.y','.z','.w'])
    f.write(return_str(q_imu_lables))

    joints_lables = creat_lables(sensor_f_list,['.x','.y','.z','.w'])
    f.write(return_str(joints_lables))

    f.write('wrist_pose.position.x, wrist_pose.position.y, wrist_pose.position.z,'
            'wrist_pose.orientation.x, wrist_pose.orientation.y, wrist_pose.orientation.z,'
            'wrist_pose.orientation.w,')
    f.write('tool_pose.position.x, tool_pose.position.y, tool_pose.position.z,'
            'tool_pose.orientation.x, tool_pose.orientation.y, tool_pose.orientation.z,'
            'tool_pose.orientation.w,')
    f.write('\r\n')
    f.close()

    save_to_csv(f)

    # rospy.spin()

if __name__ == '__main__':
    main()



