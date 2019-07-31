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
global listener
change=0

def transform_to_cf(quat,target_frame):
    global listener
    quat_st_wf = QuaternionStamped()
    quat_st_wf.header.frame_id = '/world'
    quat_st_wf.quaternion = quat

    quat_st_cf = listener.transformQuaternion(target_frame, quat_st_wf)
    return quat_st_cf.quaternion

def append_new_frame(translation,rotation,frame_id,frame_target):
    pub_tf = rospy.Publisher("/tf", tfMessage)

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
    for idx, pose in enumerate(imu_pose_list):
        quat_cf = transform_to_cf(pose.orientation, sensor_rf_list[idx])
        quat_list_cf.append(quat_cf)
    return quat_list_cf
#=====================================================================================


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


class Active_Margining:
    def __init__(self, rom,name):
        print("new margining")
        self.low_margin=1000
        self.high_margin=-1000
        self.rom=rom
        self.name=name
    def update_margins(self,new_angle):

        if new_angle > self.high_margin:
            self.high_margin = new_angle
            self.low_margin = self.high_margin - self.rom

        if new_angle < self.low_margin:
            self.low_margin = new_angle
            self.high_margin = self.low_margin + self.rom

    def normalize(self,new_angle):
        self.update_margins(new_angle)
        normalized_angle = abs((new_angle - self.low_margin) / self.rom)  # normalize between 0-1
        print("sensor rom: {}, high_margin: {}, low margin: {},normalized_angle:{}".format(self.rom,self.high_margin,self.low_margin,normalized_angle))
        return normalized_angle
    def visulize_margins(self):
        publisher = rospy.Publisher(self.name, Marker)
        cube = Marker()
        cube.header.frame_id = "world"
        cube.header.stamp = rospy.Time.now()
        cube.ns = "self.name"
        cube.action = 0
        cube.type = cube.CUBE
        cube.id = 2

        cube.scale.x = self.rom/50
        cube.scale.y = 0.1
        cube.scale.z = 0.1
        #

        cube.color.a = 0.5
        cube.color.r = 0
        cube.color.g = 1
        cube.color.b = 0
        cube.pose.position.x = self.low_margin/100
        cube.pose.position.y = 0
        cube.pose.position.z = 0
        # vel.pose.orientation=Wrist_quat
        publisher.publish(cube)
    def visualize_new_angle(self,new_angle):
        publisher = rospy.Publisher(self.name+'_angle', Marker)
        cube = Marker()
        cube.header.frame_id = "world"
        cube.header.stamp = rospy.Time.now()
        cube.ns = self.name+'_angle'
        cube.action = 0
        cube.type = cube.CUBE
        cube.id = 2

        cube.scale.x = 0.1
        cube.scale.y = 0.1
        cube.scale.z = 0.1
        #

        cube.color.a = 1
        cube.color.r = 1
        cube.color.g = 0
        cube.color.b = 0
        cube.pose.position.x = new_angle/100
        cube.pose.position.y = 0
        cube.pose.position.z = 0
        # vel.pose.orientation=Wrist_quat
        publisher.publish(cube)

def publish_to_davinci(outer_wrist_pitch,outer_wrist_yaw,jaw):
    pub_joints = rospy.Publisher('/dvrk/PSM2/joint_states', JointState, queue_size=1)
    joint_state=JointState()

    # joint_state.name.append("outer_yaw")
    joint_state.name = ["outer_yaw", "outer_pitch", "outer_pitch_1", "outer_pitch_2", "outer_pitch_3", "outer_pitch_4",
  "outer_pitch_5", "outer_insertion", "outer_roll", "outer_wrist_pitch", "outer_wrist_yaw",
  "jaw", "jaw_mimic_1", "jaw_mimic_2"]
    # jaw: 0-1.57
    # jaw_mimic_1 + jaw_mimic_2 should be equal to jaw
    joint_state.position=[0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,outer_wrist_pitch,outer_wrist_yaw,jaw,jaw/2,jaw/2]
    # joint_state.header.frame_id = "world"
    joint_state.header.stamp = rospy.Time.now()
    # print('joint_state',joint_state)
    pub_joints.publish(joint_state)


wrist_ab_margining=Active_Margining(40,'wrist_ab')
wrist_fl_margining=Active_Margining(60,'wrist_fl')
index_fl_margining=Active_Margining(90,'index_fl')

def imu_array_callback(imu_pose_array):
    global wrist_ab_margining
    global wrist_fl_margining
    global index_fl_margining
    # rospy.loginfo("HFT_joint_mapping")
    sensor_f_list = rospy.get_param('/sensor_f_list')

    imu_pose_list=imu_pose_array.poses
    publish_all_wrt_world(imu_pose_list)
    quat_list_cf=transform_all_to_cf(imu_pose_list)

    # now if you want for example Wrist joint, do as follows:
    Wrist_quat=quat_list_cf[sensor_f_list.index('Wrist')]
    Index_MIP_quat=quat_list_cf[sensor_f_list.index('Index_MIP')]
    # print("Wrist_quat", Wrist_quat)

    # publish euler angle
    pub_eu_rel = rospy.Publisher('/pub_eu_rel', Point, queue_size=1)
    euler1 = Point()
    [euler1.x, euler1.y, euler1.z] = euler_from_quaternion([Wrist_quat.x, Wrist_quat.y, Wrist_quat.z, Wrist_quat.w])
    euler1.x=math.degrees(euler1.x)
    euler1.y=math.degrees(euler1.y)
    euler1.z=math.degrees(euler1.z)
    pub_eu_rel.publish(euler1)
    print("eulers1: x: {}, y: {}, z: {}".format(euler1.x,euler1.y,euler1.z))

    wrist_fl=euler1.x
    wrist_ab=euler1.z


    euler2 = Point()
    [euler2.x, euler2.y, euler2.z] = euler_from_quaternion([Index_MIP_quat.x, Index_MIP_quat.y, Index_MIP_quat.z, Index_MIP_quat.w])
    euler2.x=math.degrees(euler2.x)
    euler2.y=math.degrees(euler2.y)
    euler2.z=math.degrees(euler2.z)
    print("eulers2: x: {}, y: {}, z: {}".format(euler2.x,euler2.y,euler2.z))

    index_fl=euler2.x




    wrist_ab_n=wrist_ab_margining.normalize(wrist_ab)
    wrist_fl_n=wrist_fl_margining.normalize(wrist_fl)
    index_fl_n=index_fl_margining.normalize(index_fl)

    publish_to_davinci(wrist_ab_n*math.pi-math.pi/2,wrist_fl_n*math.pi-math.pi/2,index_fl_n*math.pi/2)

    index_fl_margining.visulize_margins()
    index_fl_margining.visualize_new_angle(index_fl)

    # visualize

    # publisher = rospy.Publisher('Wrist_marker', Marker)
    # vel = Marker()
    # vel.header.frame_id = "world"
    # vel.header.stamp = rospy.Time.now()
    # vel.ns = "wrist"
    # vel.action = 0
    # vel.type = vel.ARROW
    # vel.id = 1
    #
    # vel.scale.x = 0.5
    # vel.scale.y = 0.1
    # vel.scale.z = 0.1
    # #
    #
    # vel.color.a = 1
    # vel.color.r = 0
    # vel.color.g = 0
    # vel.color.b = 1
    # vel.pose.position.x=1
    # vel.pose.position.y=1
    # vel.pose.position.z=1
    # vel.pose.orientation=Wrist_quat
    # publisher.publish(vel)




    # publisher1 = rospy.Publisher('wrist_fl_n', Marker)
    # cube2 = Marker()
    # cube2.header.frame_id = "world"
    # cube2.header.stamp = rospy.Time.now()
    # cube2.ns = "wrist_fl_n"
    # cube2.action = 0
    # cube2.type = vel.CUBE
    # cube2.id = 2
    #
    # cube2.scale.x = 0.5
    # cube2.scale.y = 0.1
    # cube2.scale.z = 0.1
    # #
    #
    # cube2.color.a = 1
    # cube2.color.r = 1
    # cube2.color.g = 0
    # cube2.color.b = 0
    # cube2.pose.position.x=-1.5
    # cube2.pose.position.y=0.5
    # cube2.pose.position.z=wrist_fl_n
    # # vel.pose.orientation=Wrist_quat
    # publisher1.publish(cube2)

def main():
    global listener
    rospy.init_node('hand_joint_mapping', anonymous=True)
    rospy.Subscriber("/imu_pub_array", PoseArray, imu_array_callback, queue_size=10)
    listener = tf.TransformListener()
    # rate = rospy.Rate(100)  # 10hz
    # while not rospy.is_shutdown():
    #     publish_to_davinci()
    #     rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()

