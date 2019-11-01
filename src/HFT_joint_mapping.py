#!/usr/bin/env python
# license removed for brevity
'''
author: Mohammad Fattahi Sani
email: fattahi.m91@gmail.com
this code is provided only for SMARTsurg's Internal refrence.
author's permission is required for any kind of partial or whole usage or distribution.
'''

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

from tf.msg import tfMessage
from std_msgs.msg import Int32
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




#=====================================================================================

def rad_to_degree(p_rad):
    p_deg=Point()
    p_deg.x=math.degrees(p_rad.x)
    p_deg.y=math.degrees(p_rad.y)
    p_deg.z=math.degrees(p_rad.z)
    return p_deg


class Active_Margining:
    def __init__(self,name,uniqe_id):
        '''

        :param rom: this is not used for now as we have calibration stage to find the ROM
        :param name: dedicating a specific name for each joint
        :param uniqe_id:
        '''
        print("new margining")
        self.low_margin = 100
        self.high_margin = -100
        self.rom=0
        self.name=name
        self.id=uniqe_id
        self.publisher_margin=rospy.Publisher(self.name, Marker, queue_size=1)
        self.publisher_angles=rospy.Publisher(self.name + '_angle', Marker, queue_size=1)
        self.buffer_index=0
        self.buffer_size=20
        self.buffer=[0 for i in range(self.buffer_size)]
        self.buffer_margins=[]
        self.buffer_margins_index=0
        self.buffer_margins_size=20
        self.margin_stiffness=math.radians(0.014)
        self.first_round= True
        self.energy_tank=0
        self.coeff=0.01
        self.previous_angle=0
        speed_limit = rospy.get_param('/speed_limit')
        self.max_step=speed_limit
        CalLength = rospy.get_param('/CalLength')
        self.calibration_counter=0
        self.calibration_length=CalLength[self.name]
        desired_ROMs = rospy.get_param('/desired_ROMs')
        self.desired_ROM=desired_ROMs[self.name]*math.pi

    def calibration(self, new_angle):
        '''
        this func will run to find min and max of the motion, find the range and set the initial angle to its middle.
        :return:
        '''
        if self.calibration_counter < self.calibration_length:
            self.calibration_counter=self.calibration_counter+1
            print('Please move {} to their limits...(0 to {}) now:{}'.format(self.name,self.calibration_length,self.calibration_counter))
            if new_angle > self.high_margin:
                self.high_margin = new_angle
            if new_angle < self.low_margin:
                self.low_margin = new_angle

            self.rom = abs(self.high_margin - self.low_margin)
            return False
        else:
            return True


    def normalize(self,new_angle):
        # self.update_margins(new_angle)
        if new_angle<self.low_margin :
            normalized_angle=0
        elif new_angle > self.high_margin:
            normalized_angle=1
        else:
            normalized_angle = abs((new_angle - self.low_margin) / self.rom)  # normalize between 0-1
        # print("sensor : {}, high_margin: {}, low margin: {},new_angle:{}, Angle_n:{}".format(self.name,self.high_margin,self.low_margin,new_angle,normalized_angle))
        return normalized_angle

    def map_to_margines(self,new_angle):
        normalized_angle = self.normalize(new_angle) # normalize between 0-1
        mapped_angled=normalized_angle*self.desired_ROM - self.desired_ROM/2
        return mapped_angled

    def visulize_margins(self):
        cube = Marker()
        cube.header.frame_id = "world"
        cube.header.stamp = rospy.Time.now()
        cube.ns = "self.name"
        cube.action = 0
        cube.type = cube.CUBE
        cube.id = self.id

        cube.scale.x = self.rom
        cube.scale.y = 0.1
        # print("sef.id:",float(self.id) / 10)
        cube.scale.z = 0.1
        #

        cube.color.a = 0.5
        cube.color.r = 0
        cube.color.g = 1
        cube.color.b = 0
        cube.pose.position.x = self.low_margin+self.rom/2
        cube.pose.position.y = 0
        cube.pose.position.z = float(self.id)/10
        # vel.pose.orientation=Wrist_quat
        self.publisher_margin.publish(cube)

    def visualize_new_angle(self,new_angle):

        cube = Marker()
        cube.header.frame_id = "world"
        cube.header.stamp = rospy.Time.now()
        cube.ns = self.name+'_angle'
        cube.action = 0
        cube.type = cube.CUBE
        cube.id = self.id

        cube.scale.x = 0.04
        cube.scale.y = 0.1
        cube.scale.z = 0.1

        cube.color.a = 1
        cube.color.r = 1
        cube.color.g = 0
        cube.color.b = 0
        cube.pose.position.x = new_angle
        cube.pose.position.y = 0
        cube.pose.position.z = float(self.id)/10
        # vel.pose.orientation=Wrist_quat
        self.publisher_angles.publish(cube)

    def speed_limit(self, new_anle):
        if abs(new_anle - self.previous_angle) > self.max_step :
            if new_anle > self.previous_angle:
                output_angle=self.previous_angle + self.max_step
            else:
                output_angle=self.previous_angle - self.max_step
            # print("sensor : {}, new_anle:{}, step:{} ".format(self.name, new_anle, (new_anle-self.previous_angle)))
        else:
            output_angle=new_anle
        self.previous_angle=output_angle
        return output_angle



def publish_to_davinci(elbow_roll,outer_wrist_pitch,outer_wrist_yaw,jaw):
    global pedal
    joint_state = JointState()
    joint_state.header.frame_id = "world"
    joint_state.header.stamp = rospy.Time.now()

    simulation_param=rospy.get_param('/simulation_param')
    if simulation_param is True:
        pub_joints = rospy.Publisher('/dvrk/PSM2/joint_states', JointState, queue_size=1)
        joint_state.name = ["outer_yaw", "outer_pitch", "outer_pitch_1", "outer_pitch_2", "outer_pitch_3",
                            "outer_pitch_4",
                            "outer_pitch_5", "outer_insertion", "outer_roll", "outer_wrist_pitch", "outer_wrist_yaw",
                            "jaw", "jaw_mimic_1", "jaw_mimic_2"]
        # jaw: 0-1.57
        # jaw_mimic_1 + jaw_mimic_2 should be equal to jaw
        joint_state.position=[0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,outer_wrist_pitch,outer_wrist_yaw,jaw,jaw/2,jaw/2]
    else:
        pub_joints = rospy.Publisher('/davinci_joint_states', JointState, queue_size=1)
        joint_state.name = ["roll","pitch","yaw","jaw"]
        joint_state.position=[elbow_roll,-outer_wrist_pitch,-outer_wrist_yaw,jaw]

    desired_pedal = rospy.get_param('/desired_pedal')
    if(pedal==desired_pedal):
        print('roll:{} ,pitch:{} ,yaw:{} ,jaw:{} '.format(elbow_roll,outer_wrist_pitch,outer_wrist_yaw,jaw))
        pub_joints.publish(joint_state)


# Initializing active margining for each joint.
elbow_roll_margining=Active_Margining('elbow_roll',1)
wrist_ab_margining=Active_Margining('wrist_ab',2)   # this means wrist abduction, which is equal to yaw
wrist_fl_margining=Active_Margining('wrist_fl',3)   # this means wrist flexsion-extension, which is equal to pitch
index_fl_margining=Active_Margining('index_fl',4)   #this means index finger flexsion, which is equal to jaw open close


bCalibrationIsFinished=False

def joints_array_callback(joints_array):
    global elbow_roll_margining
    global wrist_ab_margining
    global wrist_fl_margining
    global index_fl_margining
    global bCalibrationIsFinished
    # rospy.loginfo("HFT_joint_mapping")
    sensor_f_list = rospy.get_param('/sensor_f_list')

    joints_list=joints_array.poses

    # now if you want for example Wrist joint, do as follows:
    Elbow_euler = joints_list[sensor_f_list.index('Elbow')].position
    Wrist_euler=joints_list[sensor_f_list.index('Wrist')].position
    index_thumb_euler=joints_list[sensor_f_list.index('Index_MIP')].position
    # print(Wrist_euler)
    elbow_roll=Elbow_euler.y
    wrist_fl=Wrist_euler.x      # this means wrist flexsion-extension, which is equal to pitch
    wrist_ab=Wrist_euler.z      # this means wrist abduction, which is equal to yaw
    index_fl=index_thumb_euler.z   #this means index finger flexsion, which is equal to jaw open close

    if bCalibrationIsFinished:
        #first we normalize the angles between 0~1
        elbow_roll_n = elbow_roll_margining.map_to_margines(elbow_roll)
        wrist_ab_n = wrist_ab_margining.map_to_margines(wrist_ab)
        wrist_fl_n = wrist_fl_margining.map_to_margines(wrist_fl)
        index_fl_n = index_fl_margining.map_to_margines(index_fl)
        # //TODO:speed limit >> is better to go to davinci drive
        elbow_roll_n_p = elbow_roll_margining.speed_limit(elbow_roll_n)
        wrist_ab_n_p = wrist_ab_margining.speed_limit(wrist_ab_n)
        wrist_fl_n_p = wrist_fl_margining.speed_limit(wrist_fl_n)
        index_fl_n_p = index_fl_margining.speed_limit(index_fl_n)

        # publish_to_davinci(elbow_roll, outer_wrist_pitch, outer_wrist_yaw, jaw)
        publish_to_davinci(elbow_roll_n_p,  wrist_fl_n_p, wrist_ab_n_p, index_fl_n_p)
        # publish_to_davinci(elbow_roll_n_p * math.pi - math.pi / 2, -(wrist_ab_n_p * math.pi - math.pi / 2),
        #                    -(wrist_fl_n_p * math.pi - math.pi / 2), index_ab_n_p * math.pi / 2)
        # publish_to_davinci(wrist_ab_n*math.pi-math.pi/2,wrist_fl_n*math.pi-math.pi/2,index_ab_n*math.pi/2)
    else:
        if elbow_roll_margining.calibration(elbow_roll) is True:
            if wrist_ab_margining.calibration(wrist_ab) is True:
                if wrist_fl_margining.calibration(wrist_fl) is True:
                    if index_fl_margining.calibration(index_fl) is True:
                        bCalibrationIsFinished = True

    elbow_roll_margining.visulize_margins()
    wrist_ab_margining.visulize_margins()
    wrist_fl_margining.visulize_margins()
    index_fl_margining.visulize_margins()

    elbow_roll_margining.visualize_new_angle(elbow_roll)
    wrist_ab_margining.visualize_new_angle(wrist_ab)
    wrist_fl_margining.visualize_new_angle(wrist_fl)
    index_fl_margining.visualize_new_angle(index_fl)


pedal=Int32()
def pedal_read_callback(data):
    global pedal
    pedal=data.data

def main():
    global listener
    rospy.init_node('HFT_joint_mapping', anonymous=True)
    rospy.Subscriber("/joints_array", PoseArray, joints_array_callback, queue_size=10)
    rospy.Subscriber("/pedals", Int32, pedal_read_callback, queue_size=10)
    listener = tf.TransformListener()
    rospy.spin()

if __name__ == '__main__':
    main()

