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
        self.desired_ROM_upper=desired_ROMs[self.name+'_upper']
        self.desired_ROM_lower=desired_ROMs[self.name+'_lower']
        self.desired_ROM=self.desired_ROM_upper-self.desired_ROM_lower


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
        mapped_angled=normalized_angle*self.desired_ROM + self.desired_ROM_lower
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
        cube.pose.position.y = -1
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
        cube.pose.position.y = -1
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



def publish_to_3fingertool(index_mip_n_p, middle_mip_n_p, thumb_mip_n_p, index_pip_n_p, middle_pip_n_p, thumb_dip_n_p):
    global pedal
    joint_state = JointState()
    joint_state.header.frame_id = "world"
    joint_state.header.stamp = rospy.Time.now()

    pub_joints = rospy.Publisher('/ss_3f_model/ss_tool/joint_states', JointState, queue_size=1)
    joint_state.name = ["joint_index_MIP", "joint_middle_MIP", "joint_thumb_MIP",
                        "joint_index_PIP", "joint_middle_PIP", "joint_thumb_DIP"]
    # jaw: 0-1.57
    # jaw_mimic_1 + jaw_mimic_2 should be equal to jaw
    joint_state.position = [index_mip_n_p, middle_mip_n_p, thumb_mip_n_p,
                            index_pip_n_p, middle_pip_n_p, thumb_dip_n_p]
    joint_state.velocity = [0,0,0,0,0,0]
    joint_state.effort = [0,0,0,0,0,0]

    desired_pedal = rospy.get_param('/desired_pedal')
    # if(pedal==desired_pedal):
        # print('roll:{} ,pitch:{} ,yaw:{} ,jaw:{} '.format(elbow_roll,outer_wrist_pitch,outer_wrist_yaw,jaw))
    pub_joints.publish(joint_state)


def publish_to_davinci(elbow_roll,outer_wrist_pitch,outer_wrist_yaw,jaw):
    global pedal
    joint_state = JointState()
    joint_state.header.frame_id = "world"
    joint_state.header.stamp = rospy.Time.now()
    simulation_param=rospy.get_param('/simulation_param')
    if simulation_param is True:
        pub_joints = rospy.Publisher('/dvrk/ss_davinci/joint_states', JointState, queue_size=1)
        joint_state.name = [ "outer_roll", "outer_wrist_pitch", "outer_wrist_yaw",
                                "jaw", "jaw_mimic_1", "jaw_mimic_2"]
            # jaw: 0-1.57
            # jaw_mimic_1 + jaw_mimic_2 should be equal to jaw
        joint_state.position=[elbow_roll,outer_wrist_pitch,outer_wrist_yaw,jaw,jaw/2,jaw/2]
        joint_state.velocity=[0.0,0,0,0,0,0]
        joint_state.effort=[0.0,0,0,0,0,0]
    else:
        pub_joints = rospy.Publisher('/davinci_joint_states', JointState, queue_size=1)
        joint_state.name = ["roll","pitch","yaw","jaw"]
        joint_state.position=[elbow_roll,-outer_wrist_pitch,-outer_wrist_yaw,jaw]

    desired_pedal = rospy.get_param('/desired_pedal')
    pub_joints.publish(joint_state)
    print('roll:{} ,pitch:{} ,yaw:{} ,jaw:{} '.format(elbow_roll,outer_wrist_pitch,outer_wrist_yaw,jaw))
    if(pedal==desired_pedal):
        pass


# Initializing active margining for each joint.
index_mip_margining  = Active_Margining('index_mip',1)
index_pip_margining  = Active_Margining('index_pip',2)
middle_mip_margining = Active_Margining('middle_mip',3)
middle_pip_margining  = Active_Margining('middle_pip',4)
thumb_mip_margining  = Active_Margining('thumb_mip',5)
thumb_dip_margining  = Active_Margining('thumb_dip',6)


bCalibrationIsFinished=False

def joints_array_callback(joints_array):
    global index_mip_margining
    global index_pip_margining
    global middle_mip_margining
    global middle_pip_margining
    global thumb_mip_margining
    global thumb_dip_margining
    global bCalibrationIsFinished
    # rospy.loginfo("HFT_joint_mapping")
    sensor_f_list = rospy.get_param('/sensor_f_list')

    joints_list=joints_array.poses

    # now if you want for example Wrist joint, do as follows:
    index_mip_euler = joints_list[sensor_f_list.index('Index_MIP')].position    #must be same as in params
    index_pip_euler = joints_list[sensor_f_list.index('Index_PIP')].position    #must be same as in params
    middle_mip_euler = joints_list[sensor_f_list.index('Middle_MIP')].position    #must be same as in params
    middle_pip_euler = joints_list[sensor_f_list.index('Middle_PIP')].position    #must be same as in params
    thumb_mip_euler = joints_list[sensor_f_list.index('Thumb_MIP')].position    #must be same as in params
    thumb_dip_euler = joints_list[sensor_f_list.index('Thumb_DIP')].position    #must be same as in params

    index_mip=-index_mip_euler.x
    index_pip=-index_pip_euler.x
    middle_mip=-middle_mip_euler.x
    middle_pip=-middle_pip_euler.x
    thumb_mip=thumb_mip_euler.x
    thumb_dip=thumb_dip_euler.x
    # print(index_mip,middle_mip,thumb_mip)
    publish_to_3fingertool(0,0,0,0,0,0)
    if bCalibrationIsFinished:
        #first we normalize the angles between 0~1
        index_mip_n =  index_mip_margining.map_to_margines(index_mip)
        index_pip_n =  index_pip_margining.map_to_margines(index_pip)
        middle_mip_n = middle_mip_margining.map_to_margines(middle_mip)
        middle_pip_n = middle_pip_margining.map_to_margines(middle_pip)
        thumb_mip_n =  thumb_mip_margining.map_to_margines(thumb_mip)
        thumb_dip_n =  thumb_dip_margining.map_to_margines(thumb_dip)

        # //TODO:speed limit >> is better to go to davinci drive

        index_mip_n_p =  index_mip_margining.speed_limit(index_mip_n)
        index_pip_n_p =  index_pip_margining.speed_limit(index_pip_n)
        middle_mip_n_p = middle_mip_margining.speed_limit(middle_mip_n)
        middle_pip_n_p = middle_pip_margining.speed_limit(middle_pip_n)
        thumb_mip_n_p =  thumb_mip_margining.speed_limit(thumb_mip_n)
        thumb_dip_n_p =  thumb_dip_margining.speed_limit(thumb_dip_n)

        publish_to_3fingertool(index_mip_n_p, middle_mip_n_p, thumb_mip_n_p, index_pip_n_p, middle_pip_n_p, thumb_dip_n_p)
        # publish_to_davinci(elbow_roll, outer_wrist_pitch, outer_wrist_yaw, jaw)
        # publish_to_davinci(elbow_roll_n_p,  wrist_fl_n_p, wrist_ab_n_p, index_fl_n_p)
        # publish_to_davinci(elbow_roll_n_p * math.pi - math.pi / 2, -(wrist_ab_n_p * math.pi - math.pi / 2),
        #                    -(wrist_fl_n_p * math.pi - math.pi / 2), index_ab_n_p * math.pi / 2)
        # publish_to_davinci(wrist_ab_n*math.pi-math.pi/2,wrist_fl_n*math.pi-math.pi/2,index_ab_n*math.pi/2)
    else:
        if index_mip_margining.calibration(index_mip) is True:
            if index_pip_margining.calibration(index_pip) is True:
                if middle_mip_margining.calibration(middle_mip) is True:
                    if middle_pip_margining.calibration(middle_pip) is True:
                        if thumb_mip_margining.calibration(thumb_mip) is True:
                            if thumb_dip_margining.calibration(thumb_dip) is True:
                                bCalibrationIsFinished = True

    index_mip_margining.visulize_margins()
    index_pip_margining.visulize_margins()
    middle_mip_margining.visulize_margins()
    middle_pip_margining.visulize_margins()
    thumb_mip_margining.visulize_margins()
    thumb_dip_margining.visulize_margins()

    index_mip_margining.visualize_new_angle(index_mip)
    index_pip_margining.visualize_new_angle(index_pip)
    middle_mip_margining.visualize_new_angle(middle_mip)
    middle_pip_margining.visualize_new_angle(middle_pip)
    thumb_mip_margining.visualize_new_angle(thumb_mip)
    thumb_dip_margining.visualize_new_angle(thumb_dip)



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

