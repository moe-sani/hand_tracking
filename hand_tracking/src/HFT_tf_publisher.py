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
from tf.msg import tfMessage
from std_msgs.msg import Float64
from hand_tracking.msg import Float64ArrayStamped
from geometry_msgs.msg import TwistStamped
import numpy
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import math
import tf


offset_buff=[]
global listener
change=0

def transform_to_cf(quat,target_frame):
    global listener
    # br = tf.TransformBroadcaster()
    # br.sendTransform((1, 1, 0),
    #                  (sensor6_pose.orientation.x, sensor6_pose.orientation.y, sensor6_pose.orientation.z,
    #                   sensor6_pose.orientation.w),
    #                  rospy.Time.now(),
    #                  '/sensor6',
    #                  '/world')



    quat_st_wf = QuaternionStamped()
    quat_st_wf.header.frame_id = '/world'
    quat_st_wf.quaternion = quat

    quat_st_cf = listener.transformQuaternion(target_frame, quat_st_wf)
    return quat_st_cf.quaternion



    # br1 = tf.TransformBroadcaster()
    # br1.sendTransform((0.5, 0, 0),
    #                   (sensor1_sf.quaternion.x, sensor1_sf.quaternion.y, sensor1_sf.quaternion.z,
    #                    sensor1_sf.quaternion.w),
    #                   rospy.Time.now(),
    #                   '/sensor1',
    #                   '/sensor6')
    #
    # sensor0_wf = QuaternionStamped()
    # sensor0_wf.header.frame_id = '/world'
    # sensor0_wf.quaternion = sensor0_pose.orientation
    #
    # sensor0_sf = listener.transformQuaternion('/sensor1', sensor0_wf)
    # br2 = tf.TransformBroadcaster()
    # br2.sendTransform((0.5, 0, 0),
    #                   (sensor0_sf.quaternion.x, sensor0_sf.quaternion.y, sensor0_sf.quaternion.z,
    #                    sensor0_sf.quaternion.w),
    #                   rospy.Time.now(),
    #                   '/sensor0',
    #                   '/sensor1')





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

    sensor_f_list = rospy.get_param('/sensor_f_list')
    sensor_rf_list = rospy.get_param('/sensor_rf_list')
    sensor_rf_raw_list = rospy.get_param('/sensor_rf_raw_list')
    hft_tf_translations = rospy.get_param('/hft_tf')


    for i,pose in enumerate(pose_list):
        temp_tr=hft_tf_translations[sensor_f_list[i]]

        # print("temp_tr",temp_tr)
        quat_cf=transform_to_cf(pose.orientation,sensor_rf_list[i])
        brodcast_tf_now(tf_prodcaster_list[i],
						(temp_tr[0],temp_tr[1],temp_tr[2]),(quat_cf.x, quat_cf.y, quat_cf.z, quat_cf.w),
						sensor_rf_raw_list[i],sensor_rf_list[i])
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


def tf_tree_offset_publisher(offset_list):
    # TODO: transforms from refrence frame raw to refrence frame
    # tf_prodcaster_list=[]
    # for i in range(len(pose_list)):
    #     tf_prodcaster_list.append(tf.TransformBroadcaster())

    pub_tf = rospy.Publisher("/tf", tfMessage)

    sensor_f_list = rospy.get_param('/sensor_f_list')
    sensor_rf_list = rospy.get_param('/sensor_rf_list')
    sensor_rf_raw_list = rospy.get_param('/sensor_rf_raw_list')
    hft_tf_translations = rospy.get_param('/hft_tf')

    lst=[]
    for i,pose in enumerate(offset_list):
        # temp_tr=hft_tf_translations[sensor_f_list[i]]

        # print("temp_tr",temp_tr)
        quat_cf=transform_to_cf(pose.orientation,sensor_rf_raw_list[i])

        t = TransformStamped()
        t.header.frame_id = sensor_rf_raw_list[i]
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = sensor_f_list[i]
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0

        t.transform.rotation.x = quat_cf.x
        t.transform.rotation.y = quat_cf.y
        t.transform.rotation.z = quat_cf.z
        t.transform.rotation.w = -quat_cf.w
        lst.append(t)
    tfm = tfMessage(lst)
    pub_tf.publish(tfm)



    return 0



def imu_array_callback(imu_pose_array):
    global offset_buff
    imu_pose_list=imu_pose_array.poses
    if len(offset_buff)<1 :
        # imu_array_store(imu_pose_list)
        offset_buff=imu_pose_list
    else:
        # quat_offset_list=calc_mean_of_buff(offset_buff)
        # imu_pose_array_no_offset=subtract_offset(imu_pose_list,offset_buff)
        # tf_brodcaster(imu_pose_array_no_offset)
        tf_brodcaster(imu_pose_list)
        tf_tree_offset_publisher(offset_buff)



    # pose8_off=offset_buff[0]
    # pose8=imu_pose_list[0]
    # pose11=offset_buff[11]
    rospy.loginfo("test")

    # rospy.loginfo("test")
    # pub_tf = rospy.Publisher("/tf", tfMessage)
    # t = TransformStamped()
    # t.header.frame_id = "world"
    # t.header.stamp = rospy.Time.now()
    # t.child_frame_id = "carrot_offset"
    # t.transform.translation.x = 1.0
    # t.transform.translation.y = 1.0
    # t.transform.translation.z = 0.0
    #
    # t.transform.rotation.x = pose8_off.orientation.x
    # t.transform.rotation.y = pose8_off.orientation.y
    # t.transform.rotation.z = pose8_off.orientation.z
    # t.transform.rotation.w = -pose8_off.orientation.w
    #
    # t1 = TransformStamped()
    # t1.header.frame_id = "carrot_offset"
    # t1.header.stamp = rospy.Time.now()
    # t1.child_frame_id = "carrot"
    # t1.transform.translation.x = 0.0
    # t1.transform.translation.y = 0.0
    # t1.transform.translation.z = 0.0
    #
    # t1.transform.rotation.x = pose8.orientation.x
    # t1.transform.rotation.y = pose8.orientation.y
    # t1.transform.rotation.z = pose8.orientation.z
    # t1.transform.rotation.w = pose8.orientation.w
    # tfm = tfMessage([t, t1])
    # pub_tf.publish(tfm)




def main():
    global listener
    rospy.init_node('hand_joint_mapping', anonymous=True)
    rospy.Subscriber("/imu_pub_array", PoseArray, imu_array_callback, queue_size=10)
    listener = tf.TransformListener()

    rospy.spin()


if __name__ == '__main__':
    main()

