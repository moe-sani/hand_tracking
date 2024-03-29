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
from std_msgs.msg import Float64MultiArray
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

class Joints:
    def __init__(self,angleMin, angleMax, stepPulseMin, stepPulseMax):
        self.angle = 0
        self.angleMin = angleMin
        self.angleMax = angleMax
        self.angle_range = angleMax-angleMin
        self.stepPulse = 0
        self.stepPulseMin = stepPulseMin
        self.stepPulseMax = stepPulseMax
        self.stepPulse_range = stepPulseMax-stepPulseMin

    def joint_limits(self, value):
        if value > self.angleMax:
            value = self.angleMax
        if value < self.angleMin:
            value = self.angleMin
        return value

    def return_encoder_pulse(self, value):
        value_normalized = abs((value-self.angleMin)/self.angle_range)
        step_pulse = value_normalized*self.stepPulse_range+self.stepPulseMin
        return step_pulse



class davinci_tool_joints:
    def __init__(self):
        # print("new ")
        joint_limits_QC = rospy.get_param('/joint_limits_QC')
        joint_ROM = rospy.get_param('/joint_ROM')
        self.roll=Joints(-joint_ROM['roll_ROM']*math.pi / 2, joint_ROM['roll_ROM']*math.pi / 2, joint_limits_QC['roll_min_QC'], joint_limits_QC['roll_max_QC'])
        self.pitch=Joints(-joint_ROM['pitch_ROM']*math.pi / 2, joint_ROM['pitch_ROM']*math.pi / 2, joint_limits_QC['pitch_min_QC'], joint_limits_QC['pitch_max_QC'])
        self.leftjaw=Joints(-joint_ROM['leftjaw_ROM']*math.pi / 2, joint_ROM['leftjaw_ROM']*math.pi / 2, joint_limits_QC['leftjaw_min_QC'], joint_limits_QC['leftjaw_max_QC'])
        self.rightjaw=Joints(-joint_ROM['rightjaw_ROM']*math.pi / 2, joint_ROM['rightjaw_ROM']*math.pi / 2, joint_limits_QC['rightjaw_min_QC'], joint_limits_QC['rightjaw_max_QC'])

        self.pitch_low_margin=self.pitch.angleMin
        self.pitch_rom=self.pitch.angle_range
        self.jaw_offset=joint_limits_QC['jaw_offset']

    def pitch_yaw_mapping(self,yaw,pitch):
        pitch_normalized=abs((pitch - self.pitch_low_margin) / self.pitch_rom)  # normalize between 0-1
        yaw_corrected=yaw+0.5*pitch_normalized
        return yaw_corrected

    def calculate_jaw(self,center_of_jaw,jaw):
        left_jaw=center_of_jaw+jaw/2
        right_jaw=center_of_jaw-jaw/2+self.jaw_offset
        return [left_jaw,right_jaw]

    def calculate_joints(self, roll, pitch, yaw, jaw):

        yaw_corrected = self.pitch_yaw_mapping(yaw, pitch)
        [left_jaw, right_jaw] = self.calculate_jaw(yaw_corrected, jaw)

        roll_limited = self.roll.joint_limits(roll)
        pitch_limited = self.roll.joint_limits(pitch)
        left_jaw_limited = self.leftjaw.joint_limits(left_jaw)
        right_jaw_limited = self.rightjaw.joint_limits(right_jaw)

        da_roll = self.roll.return_encoder_pulse(roll_limited)
        da_pitch = self.pitch.return_encoder_pulse(pitch_limited)
        da_leftjaw = self.leftjaw.return_encoder_pulse(left_jaw_limited)
        da_rightjaw = self.rightjaw.return_encoder_pulse(right_jaw_limited)

        return [da_roll, da_pitch, da_leftjaw, da_rightjaw]




joints=davinci_tool_joints()

def davinci_joint_states_callback(joint_state):
    global joints
    # print(joint_state)
    joint_dict=dict(zip(joint_state.name,joint_state.position))

    roll=joint_dict['roll']
    pitch=joint_dict['pitch']
    yaw=joint_dict['yaw']
    jaw=joint_dict['jaw']

    [da_roll, da_pitch, da_leftjaw, da_rightjaw]=joints.calculate_joints(roll, pitch, yaw, jaw)
    pub = rospy.Publisher('exoskeleton', Float64MultiArray, queue_size=10)

    dataPub = [da_roll, da_pitch, da_leftjaw, da_rightjaw, 1]  # data to publish to ROS
    #print(dataPub)
    array_pub = Float64MultiArray(data=dataPub)  # Convert to std_msg type
    pub.publish(array_pub)  # Publish data to ROS


def main():
    # global listener
    rospy.init_node('davinci_drive', anonymous=True)
    rospy.Subscriber("/davinci_joint_states", JointState, davinci_joint_states_callback, queue_size=10)
    # listener = tf.TransformListener()

    rospy.spin()


if __name__ == '__main__':
    main()

