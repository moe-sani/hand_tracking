#!/usr/bin/env python
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




#=====================================================================================

def rad_to_degree(p_rad):
    p_deg=Point()
    p_deg.x=math.degrees(p_rad.x)
    p_deg.y=math.degrees(p_rad.y)
    p_deg.z=math.degrees(p_rad.z)
    return p_deg


class Active_Margining:
    def __init__(self, rom,name,uniqe_id):
        print("new margining")
        self.low_margin = 100
        self.high_margin = -100
        self.rom=rom
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
        self.max_step=0.02
    def update_margins(self,new_angle):

        if self.first_round is True:
            self.high_margin=new_angle+self.rom/2
            self.low_margin=self.high_margin-self.rom
            self.first_round=False


        if new_angle > self.high_margin:
            self.high_margin = self.high_margin+self.margin_stiffness
            self.low_margin = self.high_margin - self.rom

        if new_angle < self.low_margin:
            self.low_margin = self.low_margin-self.margin_stiffness
            self.high_margin = self.low_margin + self.rom

    def calibration(self, new_angle):
        '''
        this func will run to find min and max of the motion, find the range and set the initial angle to its middle.
        :return:
        '''
        if new_angle > self.high_margin:
            self.high_margin = new_angle
        if new_angle < self.low_margin:
            self.low_margin = new_angle

        self.rom = abs(self.high_margin - self.low_margin)

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
        #

        cube.color.a = 1
        cube.color.r = 1
        cube.color.g = 0
        cube.color.b = 0
        cube.pose.position.x = new_angle
        cube.pose.position.y = 0
        cube.pose.position.z = float(self.id)/10
        # vel.pose.orientation=Wrist_quat
        self.publisher_angles.publish(cube)
    def running_average(self,new_angle):
        self.buffer[self.buffer_index]=new_angle
        self.buffer_index=self.buffer_index+1
        if self.buffer_index>=self.buffer_size:
            self.buffer_index=0

        print("buffer:",self.buffer)
        print("new_angle: {}, average: {}".format(new_angle,numpy.mean(self.buffer)))
        return numpy.mean(self.buffer)

    def running_average_margins(self,new_data):
        self.buffer_margins[self.buffer_margins_index]=new_data
        self.buffer_margins_index=self.buffer_margins_index+1
        if self.buffer_margins_index>self.buffer_margins_size:
            self.buffer_margins_index=0

        return numpy.mean(self.buffer_margins)

    def passivity(self,normalized_angle):
        angle=normalized_angle-0.5

        self.energy_tank=self.energy_tank+self.coeff*angle

        if self.energy_tank > 1:
            self.energy_tank=1

        if self.energy_tank < 0:
            self.energy_tank = 0

        print("sensor : {}, angle:{},  normalized_angle:{}, energy_tank:{} ".format(self.name, angle, normalized_angle, self.energy_tank))
        return self.energy_tank

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
    # pub_joints = rospy.Publisher('/dvrk/PSM2/joint_states', JointState, queue_size=1)
    pub_joints = rospy.Publisher('/davinci_joint_states', JointState, queue_size=1)
    joint_state=JointState()


    # joint_state.name = ["outer_yaw", "outer_pitch", "outer_pitch_1", "outer_pitch_2", "outer_pitch_3", "outer_pitch_4",
  # "outer_pitch_5", "outer_insertion", "outer_roll", "outer_wrist_pitch", "outer_wrist_yaw",
  # "jaw", "jaw_mimic_1", "jaw_mimic_2"]
    # jaw: 0-1.57
    # jaw_mimic_1 + jaw_mimic_2 should be equal to jaw
    # joint_state.position=[0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,outer_wrist_pitch,outer_wrist_yaw,jaw,jaw/2,jaw/2]
    joint_state.header.frame_id = "world"
    joint_state.name = ["roll","pitch","yaw","jaw"]
    joint_state.position=[elbow_roll,-outer_wrist_yaw,-outer_wrist_pitch,jaw]
    joint_state.header.stamp = rospy.Time.now()
    print('roll:{} ,pitch:{} ,yaw:{} ,jaw:{} '.format(elbow_roll,outer_wrist_pitch,outer_wrist_yaw,jaw))
    pub_joints.publish(joint_state)

elbow_roll_margining=Active_Margining(math.radians(120), 'elbow_roll',1)
wrist_ab_margining=Active_Margining(math.radians(40), 'wrist_ab',2)
wrist_fl_margining=Active_Margining(math.radians(110), 'wrist_fl',3)
index_fl_margining=Active_Margining(math.radians(90), 'index_fl',4)
counter_calib=0

def joints_array_callback(joints_array):
    global elbow_roll_margining
    global wrist_ab_margining
    global wrist_fl_margining
    global index_fl_margining
    global counter_calib
    # rospy.loginfo("HFT_joint_mapping")
    sensor_f_list = rospy.get_param('/sensor_f_list')

    joints_list=joints_array.poses


    # now if you want for example Wrist joint, do as follows:
    Elbow_euler = joints_list[sensor_f_list.index('Elbow')].position
    Wrist_euler=joints_list[sensor_f_list.index('Wrist')].position
    index_middle_euler=joints_list[sensor_f_list.index('Index_MIP')].position
    # print(Wrist_euler)
    elbow_roll=Elbow_euler.x
    wrist_fl=Wrist_euler.y
    wrist_ab=Wrist_euler.z
    index_ab=index_middle_euler.z

    # wrist_ab_n=wrist_ab_margining.running_average(wrist_ab)
    # wrist_fl_n=wrist_fl_margining.running_average(wrist_fl)
    # index_fl_n=index_fl_margining.running_average(index_fl)
    if counter_calib > 2000:
        elbow_roll_n=elbow_roll_margining.normalize(elbow_roll)
        wrist_ab_n=wrist_ab_margining.normalize(wrist_ab)
        wrist_fl_n=wrist_fl_margining.normalize(wrist_fl)
        index_ab_n=index_fl_margining.normalize(index_ab)

        elbow_roll_n_p=elbow_roll_margining.speed_limit(elbow_roll_n)
        wrist_ab_n_p=wrist_ab_margining.speed_limit(wrist_ab_n)
        wrist_fl_n_p=wrist_fl_margining.speed_limit(wrist_fl_n)
        index_ab_n_p=index_fl_margining.speed_limit(index_ab_n)

        publish_to_davinci(elbow_roll_n_p*math.pi-math.pi/2,-(wrist_ab_n_p*math.pi-math.pi/2),-(wrist_fl_n_p*math.pi-math.pi/2), index_ab_n_p*math.pi/2)
        # publish_to_davinci(wrist_ab_n*math.pi-math.pi/2,wrist_fl_n*math.pi-math.pi/2,index_ab_n*math.pi/2)
    else:
        counter_calib = counter_calib+1
        print('Please move angles to their limits...(0 to 1000) now:{}'.format(counter_calib))
        elbow_roll_margining.calibration(elbow_roll)
        wrist_ab_margining.calibration(wrist_ab)
        wrist_fl_margining.calibration(wrist_fl)
        index_fl_margining.calibration(index_ab)

    elbow_roll_margining.visulize_margins()
    wrist_ab_margining.visulize_margins()
    wrist_fl_margining.visulize_margins()
    index_fl_margining.visulize_margins()

    elbow_roll_margining.visualize_new_angle(elbow_roll)
    wrist_ab_margining.visualize_new_angle(wrist_ab)
    wrist_fl_margining.visualize_new_angle(wrist_fl)
    index_fl_margining.visualize_new_angle(index_ab)

def main():
    global listener
    rospy.init_node('hand_joint_mapping', anonymous=True)
    rospy.Subscriber("/joints_array", PoseArray, joints_array_callback, queue_size=10)
    listener = tf.TransformListener()
    # rate = rospy.Rate(100)  # 10hz
    # while not rospy.is_shutdown():
    #     publish_to_davinci()
    #     rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()

