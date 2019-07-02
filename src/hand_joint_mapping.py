#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from odas_msgs.msg import Odas, OdasList
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64

from geometry_msgs.msg import TwistStamped
import numpy

import math


pub_rh_ffj0=0
pub_rh_ffj3=0
pub_rh_ffj4=0
pub_rh_mfj0=0
pub_rh_mfj3=0
pub_rh_mfj4=0
pub_rh_thj1=0
pub_rh_thj2=0
pub_rh_thj3=0
pub_rh_thj4=0
pub_rh_wrj1=0
pub_rh_wrj2=0

calibration_counter=0


Index_DIP_fl_buff=[]
Index_PIP_fl_buff=[]
Index_MIP_fl_buff=[]
Index_MIP_ab_buff=[]
Middle_DIP_fl_buff=[]
Middle_PIP_fl_buff=[]
Middle_MIP_fl_buff=[]
Middle_MIP_ab_buff=[]
Thumb_DIP_fl_buff=[]
Thumb_MIP_fl_buff=[]
Thumb_CMC_fl_buff=[]
Thumb_CMC_ab_buff=[]
Wrist_JOINT_fl_buff=[]
Wrist_JOINT_ab_buff=[]

Index_DIP_fl_offset=0
Index_PIP_fl_offset=0
Index_MIP_fl_offset=0
Index_MIP_ab_offset=0
Middle_DIP_fl_offset=0
Middle_PIP_fl_offset=0
Middle_MIP_fl_offset=0
Middle_MIP_ab_offset=0
Thumb_DIP_fl_offset=0
Thumb_MIP_fl_offset=0
Thumb_CMC_fl_offset=0
Thumb_CMC_ab_offset=0
Wrist_JOINT_fl_offset=0
Wrist_JOINT_ab_offset=0


def calc_joints(data1,data2):
    output=Point()
    output.x=data2.position.x-data1.position.x
    output.y=data2.position.y-data1.position.y
    output.z=data2.position.z-data1.position.z
    return output



def imu_serial_callback(data):
    global pub_rh_ffj0
    global pub_rh_ffj3
    global pub_rh_ffj4
    global pub_rh_mfj0
    global pub_rh_mfj3
    global pub_rh_mfj4
    global pub_rh_thj1
    global pub_rh_thj2
    global pub_rh_thj3
    global pub_rh_thj4
    global pub_rh_wrj1
    global pub_rh_wrj2

    global calibration_counter

    global Index_DIP_fl_buff
    global Index_PIP_fl_buff
    global Index_MIP_fl_buff
    global Index_MIP_ab_buff
    global Middle_DIP_fl_buff
    global Middle_PIP_fl_buff
    global Middle_MIP_fl_buff
    global Middle_MIP_ab_buff
    global Thumb_DIP_fl_buff
    global Thumb_MIP_fl_buff
    global Thumb_CMC_fl_buff
    global Thumb_CMC_ab_buff
    global Wrist_JOINT_fl_buff
    global Wrist_JOINT_ab_buff

    global Index_DIP_fl_offset
    global Index_PIP_fl_offset
    global Index_MIP_fl_offset
    global Index_MIP_ab_offset
    global Middle_DIP_fl_offset
    global Middle_PIP_fl_offset
    global Middle_MIP_fl_offset
    global Middle_MIP_ab_offset
    global Thumb_DIP_fl_offset
    global Thumb_MIP_fl_offset
    global Thumb_CMC_fl_offset
    global Thumb_CMC_ab_offset
    global Wrist_JOINT_fl_offset
    global Wrist_JOINT_ab_offset

    # for sensor_id, sensor_values in enumerate(data.poses):
    #     print("sensor{}:{}".format(sensor_id, sensor_values))
    rospy.loginfo("test")
    # % Index                                           #full sensor model
    Index_DIP = calc_joints(data.poses[0],data.poses[1])#3-2
    Index_PIP = calc_joints(data.poses[0],data.poses[1])#2-8
    Index_MIP = calc_joints(data.poses[7],data.poses[0])#8-7

    Index_DIP_fl =Index_DIP.x
    Index_PIP_fl =Index_PIP.x
    Index_MIP_fl =Index_MIP.x
    Index_MIP_ab =Index_MIP.z
    print("Index_MIP:{}".format(Index_MIP))

    # % Middle
    Middle_DIP = calc_joints(data.poses[3],data.poses[2])#5-4
    Middle_PIP = calc_joints(data.poses[3],data.poses[2])#4-10
    Middle_MIP = calc_joints(data.poses[7],data.poses[3])#10-7

    Middle_DIP_fl=Middle_DIP.x
    Middle_PIP_fl=Middle_PIP.x
    Middle_MIP_fl=Middle_MIP.x
    Middle_MIP_ab=Middle_MIP.z
    print("Middle_DIP:{}".format(Middle_DIP))
    print("Middle_MIP:{}".format(Middle_MIP))

    # % Thumb
    Thumb_DIP = calc_joints(data.poses[5],data.poses[4])#1-0
    Thumb_MIP = calc_joints(data.poses[5],data.poses[4])#0-7
    Thumb_CMC = calc_joints(data.poses[7],data.poses[5])#7-6

    Thumb_DIP_fl = Thumb_DIP.x
    Thumb_MIP_fl = Thumb_MIP.x
    Thumb_CMC_fl = Thumb_CMC.x
    Thumb_CMC_ab = Thumb_CMC.z

    # % wrist
    Wrist_JOINT = calc_joints(data.poses[6],data.poses[7])#9-10
    Wrist_JOINT_fl=Wrist_JOINT.x
    Wrist_JOINT_ab=Wrist_JOINT.z


    if calibration_counter<100:
        Index_DIP_fl_buff.append(Index_DIP_fl)
        Index_PIP_fl_buff.append(Index_PIP_fl)
        Index_MIP_fl_buff.append(Index_MIP_fl)
        Index_MIP_ab_buff.append(Index_MIP_ab)

        Middle_DIP_fl_buff.append(Middle_DIP_fl)
        Middle_PIP_fl_buff.append(Middle_PIP_fl)
        Middle_MIP_fl_buff.append(Middle_MIP_fl)
        Middle_MIP_ab_buff.append(Middle_MIP_ab)

        Thumb_DIP_fl_buff.append(Thumb_DIP_fl)
        Thumb_MIP_fl_buff.append(Thumb_MIP_fl)
        Thumb_CMC_fl_buff.append(Thumb_CMC_fl)
        Thumb_CMC_ab_buff.append(Thumb_CMC_ab)

        Wrist_JOINT_fl_buff.append(Wrist_JOINT_fl)
        Wrist_JOINT_ab_buff.append(Wrist_JOINT_ab)

        Index_DIP_fl_offset=numpy.mean(Index_DIP_fl_buff)
        Index_PIP_fl_offset=numpy.mean(Index_PIP_fl_buff)
        Index_MIP_fl_offset=numpy.mean(Index_MIP_fl_buff)
        Index_MIP_ab_offset=numpy.mean(Index_MIP_ab_buff)

        Middle_DIP_fl_offset=numpy.mean(Middle_DIP_fl_buff)
        Middle_PIP_fl_offset=numpy.mean(Middle_PIP_fl_buff)
        Middle_MIP_fl_offset=numpy.mean(Middle_MIP_fl_buff)
        Middle_MIP_ab_offset=numpy.mean(Middle_MIP_ab_buff)

        Thumb_DIP_fl_offset=numpy.mean(Thumb_DIP_fl_buff)
        Thumb_MIP_fl_offset=numpy.mean(Thumb_MIP_fl_buff)
        Thumb_CMC_fl_offset=numpy.mean(Thumb_CMC_fl_buff)
        Thumb_CMC_ab_offset=numpy.mean(Thumb_CMC_ab_buff)

        Wrist_JOINT_fl_offset=numpy.mean(Wrist_JOINT_fl_buff)
        Wrist_JOINT_ab_offset=numpy.mean(Wrist_JOINT_ab_buff)

        calibration_counter=calibration_counter+1


    else :
        pub_rh_ffj0.publish(math.radians(Index_PIP_fl-Index_PIP_fl_offset))
        pub_rh_ffj3.publish(math.radians(Index_MIP_fl-Index_MIP_fl_offset))
        pub_rh_ffj4.publish(math.radians(Index_MIP_ab-Index_MIP_ab_offset))
        print("Index_PIP_fl:{},Index_MIP_fl:{},Index_MIP_ab:{}".format(Index_PIP_fl,Index_MIP_fl,Index_MIP_ab))
        # print("Index_PIP_fl:{},Index_MIP_fl:{},Index_MIP_ab:{}".format(Index_PIP_fl-Index_PIP_fl_offset,
        #                                                                Index_MIP_fl-Index_MIP_fl_offset,
        #                                                                Index_MIP_ab-Index_MIP_ab_offset))

        pub_rh_mfj0.publish(math.radians(Middle_PIP_fl - Middle_PIP_fl_offset))
        pub_rh_mfj3.publish(math.radians(Middle_MIP_fl - Middle_MIP_fl_offset))
        pub_rh_mfj4.publish(math.radians(Middle_MIP_ab - Middle_MIP_ab_offset))
        print("Middle_PIP_fl:{},Middle_MIP_fl:{},Middle_MIP_ab:{}".format(Middle_PIP_fl, Middle_MIP_fl, Middle_MIP_ab))
        # print("Middle_PIP_fl:{},Middle_MIP_fl:{},Middle_MIP_ab:{}".format(Middle_PIP_fl - Middle_PIP_fl_offset,
        #                                                                Middle_MIP_fl - Middle_MIP_fl_offset,
        #                                                                Middle_MIP_ab - Middle_MIP_ab_offset))

        pub_rh_thj1.publish(math.radians(Thumb_DIP_fl - Thumb_DIP_fl_offset))
        pub_rh_thj2.publish(math.radians(Thumb_MIP_fl - Thumb_MIP_fl_offset))
        pub_rh_thj4.publish(math.radians(Thumb_CMC_ab - Thumb_CMC_ab_offset))
        # print("Thumb_PIP_fl:{},Thumb_MIP_fl:{},Thumb_MIP_ab:{}".format(Thumb_DIP_fl - Thumb_DIP_fl_offset,
        #                                                                Thumb_MIP_fl - Thumb_MIP_fl_offset,
        #                                                                Thumb_CMC_ab - Thumb_CMC_ab_offset))

        pub_rh_wrj1.publish(math.radians(Wrist_JOINT_fl-Wrist_JOINT_fl_offset))
        pub_rh_wrj2.publish(math.radians(Wrist_JOINT_ab-Wrist_JOINT_ab_offset))
        # print("Wrist_JOINT_fl_buff:{},Wrist_JOINT_ab_buff:{}".format(Wrist_JOINT_fl-Wrist_JOINT_fl_offset,
        #                                         Wrist_JOINT_ab-Wrist_JOINT_ab_offset))

def main():

    rospy.init_node('hand_joint_mapping', anonymous=True)

    # rospy.Subscriber("/odas_sst_raw", Odas, odas_snobber_Callback, queue_size=10)
    rospy.Subscriber("/imu_pub_array", PoseArray, imu_serial_callback, queue_size=10)
    # global listener
    global pub_rh_ffj0
    global pub_rh_ffj3
    global pub_rh_ffj4
    global pub_rh_mfj0
    global pub_rh_mfj3
    global pub_rh_mfj4
    global pub_rh_thj1
    global pub_rh_thj2
    global pub_rh_thj3
    global pub_rh_thj4
    global pub_rh_wrj1
    global pub_rh_wrj2


    # listener = tf.TransformListener()
    pub_rh_ffj0 = rospy.Publisher("/sh_rh_ffj0_position_controller/command", Float64, queue_size=10) #index pip
    pub_rh_ffj3 = rospy.Publisher("/sh_rh_ffj3_position_controller/command", Float64, queue_size=10)#index mix flex
    pub_rh_ffj4 = rospy.Publisher("/sh_rh_ffj4_position_controller/command", Float64, queue_size=10)#index mix abd

    pub_rh_mfj0 = rospy.Publisher("/sh_rh_mfj0_position_controller/command", Float64, queue_size=10) #middle pip
    pub_rh_mfj3 = rospy.Publisher("/sh_rh_mfj3_position_controller/command", Float64, queue_size=10)#middle mix flex
    pub_rh_mfj4 = rospy.Publisher("/sh_rh_mfj4_position_controller/command", Float64, queue_size=10)#middle mix abd

    pub_rh_thj1 = rospy.Publisher("/sh_rh_thj1_position_controller/command", Float64, queue_size=10) #thumb dip
    pub_rh_thj2 = rospy.Publisher("/sh_rh_thj2_position_controller/command", Float64, queue_size=10)#thumb mip
    pub_rh_thj3 = rospy.Publisher("/sh_rh_thj3_position_controller/command", Float64, queue_size=10)#thumb cmc flex
    pub_rh_thj4 = rospy.Publisher("/sh_rh_thj4_position_controller/command", Float64, queue_size=10)#thumb cmc abd

    pub_rh_wrj1 = rospy.Publisher("/sh_rh_wrj1_position_controller/command", Float64, queue_size=10) #wrist flex
    pub_rh_wrj2 = rospy.Publisher("/sh_rh_wrj2_position_controller/command", Float64, queue_size=10)#wrist abd

    rospy.spin()


if __name__ == '__main__':
    main()



# shadow robot's joint limits:
# sh_rh_ffj0=  0 > 180
# sh_rh_ffj3=  0 > 90
# sh_rh_ffj4= -20 > 20
# sh_rh_mfj0= 0 > 180
# sh_rh_mfj3= 0 > 90
# sh_rh_mfj4= -20 > 20
# sh_rh_thj1=  0 > 90
# sh_rh_thj2=  -40 > 40
# sh_rh_thj3=  -12 > 12
# sh_rh_thj4= 0 > 70
# sh_rh_thj5= -60 > 60
# sh_rh_wrj1=  -40 > 28
# sh_rh_wrj2=  -30 > 10