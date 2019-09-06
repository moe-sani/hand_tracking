#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from odas_msgs.msg import Odas, OdasList
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

from tf.msg import tfMessage
from std_msgs.msg import Float64
from hand_tracking.msg import Float64ArrayStamped
from geometry_msgs.msg import TwistStamped
import numpy
from tf.transformations import *
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
import math
import tf


offset_buff=[]
listener=0
change=0

def transform_to_cf(quat,target_frame):
    global listener
    quat_st_wf = QuaternionStamped()
    quat_st_wf.header.frame_id = '/world'
    quat_st_wf.quaternion = quat

    quat_st_cf = listener.transformQuaternion(target_frame, quat_st_wf)
    return quat_st_cf.quaternion

def append_new_frame(translation,rotation,frame_id,frame_target):
    pub_tf = rospy.Publisher("/tf", tfMessage, queue_size=1)

    t = TransformStamped()
    t.header.frame_id = frame_id
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = frame_target
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]

    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]
    tfm = tfMessage([t])
    pub_tf.publish(tfm)

def build_tf_tree(imu_pose_list,imu_offset_list):
    sensor_f_list = rospy.get_param('/sensor_f_list')
    sensor_rf_list = rospy.get_param('/sensor_rf_list')
    sensor_rf_raw_list = rospy.get_param('/sensor_rf_raw_list')
    hft_tf_translations = rospy.get_param('/hft_tf')
    for idx,(pose, pose_offset) in enumerate(zip(imu_pose_list, imu_offset_list)):
        # STEP3: transform pose_offset to [frame]
        quat_offset_cf = transform_to_cf(pose_offset.orientation, sensor_rf_list[idx])
        # STEP4: append new frame
        temp_tr = hft_tf_translations[sensor_f_list[idx]]
        append_new_frame((temp_tr[0], temp_tr[1], temp_tr[2]), (quat_offset_cf.x, quat_offset_cf.y, quat_offset_cf.z, -quat_offset_cf.w),
                         sensor_rf_list[idx], sensor_rf_raw_list[idx])
        # STEP1: tranform the quaternions to correct frame:
        quat_cf=transform_to_cf(pose.orientation, sensor_rf_list[idx])
        # STEP2: append new frame: which is called frame-raw; (it has trans +rot but not offsets)
        append_new_frame((0, 0, 0), (quat_cf.x, quat_cf.y, quat_cf.z, quat_cf.w),
                         sensor_rf_raw_list[idx], sensor_f_list[idx])

def publish_all_wrt_world(imu_pose_list):
    sensor_f_list = rospy.get_param('/sensor_f_list')
    hft_tf_translations = rospy.get_param('/hft_tfw')
    for idx, pose in enumerate(imu_pose_list):
        temp_tr = hft_tf_translations[sensor_f_list[idx]]
        append_new_frame((temp_tr[0], temp_tr[1], temp_tr[2]), (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                         '/world', sensor_f_list[idx])


def transform_all_to_cf(imu_pose_list):
    sensor_rf_list = rospy.get_param('/sensor_rf_list')
    quat_list_cf=[]
    euler_list_cf=[]
    for idx, pose in enumerate(imu_pose_list):
        quat_cf = transform_to_cf(pose.orientation, sensor_rf_list[idx])
        quat_list_cf.append(quat_cf)
        euler = Point()
        [euler.x, euler.y, euler.z] = euler_from_quaternion([quat_cf.x, quat_cf.y, quat_cf.z, quat_cf.w])
        euler_list_cf.append(euler)

    return quat_list_cf,euler_list_cf
#=====================================================================================

def rad_to_degree(p_rad):
    p_deg=Point()
    p_deg.x=math.degrees(p_rad.x)
    p_deg.y=math.degrees(p_rad.y)
    p_deg.z=math.degrees(p_rad.z)
    return p_deg

def joints_array_publisher(lst_q_joints,lst_p_joints):
    '''
    General convention: pose.position will contain euler orientation in radians
                        pose.orientation will contain quaternion orientation
    :param euler_list_cf:
    :return:
    '''

    pub_joints=rospy.Publisher('/joints_array',PoseArray,queue_size=1)
    joints_array = PoseArray()
    joints_array.header.stamp = rospy.get_rostime()
    joints_array.header.frame_id = '/world'
    lst_pose=[]
    for (q, p) in zip(lst_q_joints, lst_p_joints):
        pose=Pose()
        pose.orientation=q
        pose.position=p
        lst_pose.append(pose)
    joints_array.poses=lst_pose
    pub_joints.publish(joints_array)

def joints_publisher(lst_p_joints):
    sensor_f_list = rospy.get_param('/sensor_f_list')
    for i, point in enumerate(lst_p_joints):
        pub=rospy.Publisher('/joint_'+sensor_f_list[i], Point, queue_size=1)
        pub.publish(rad_to_degree(point))

def raw_data_publisher(imu_pose_list):
    jason_stream_lables = rospy.get_param('/jason_stream_lables')
    # publisher_list = []
    for i, pose in enumerate(imu_pose_list):
        pub=rospy.Publisher('/imu_'+jason_stream_lables[i], Point, queue_size=1)
        pub.publish(rad_to_degree(pose.position))

def imu_array_callback(imu_pose_array):
    imu_pose_list=imu_pose_array.poses
    raw_data_publisher(imu_pose_list)
    publish_all_wrt_world(imu_pose_list)
    quat_list_cf, euler_list_cf = transform_all_to_cf(imu_pose_list)
    rospy.loginfo('publishing joint angles ...')
    joints_array_publisher(quat_list_cf,euler_list_cf)
    joints_publisher(euler_list_cf)

def main():
    global listener
    rospy.init_node('HFT_joint_angle_publisher_node', anonymous=True)
    rospy.Subscriber("/imu_pub_array", PoseArray, imu_array_callback, queue_size=10)
    listener = tf.TransformListener()

    rospy.spin()


if __name__ == '__main__':
    main()

