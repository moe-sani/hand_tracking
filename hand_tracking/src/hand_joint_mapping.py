#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from odas_msgs.msg import Odas, OdasList
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64
from hand_tracking.msg import Float64ArrayStamped
from geometry_msgs.msg import TwistStamped
import numpy
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import math


pub_rh_ffj0=0
pub_rh_ffj1=0
pub_rh_ffj2=0
pub_rh_ffj3=0
pub_rh_ffj4=0
pub_rh_mfj0=0
pub_rh_mfj1=0
pub_rh_mfj2=0
pub_rh_mfj3=0
pub_rh_mfj4=0
pub_rh_thj1=0
pub_rh_thj2=0
pub_rh_thj3=0
pub_rh_thj4=0
pub_rh_thj5=0
pub_rh_wrj1=0
pub_rh_wrj2=0

calibration_counter=0

ffj1_buff=[]
ffj2_buff=[]
ffj3_buff=[]
ffj4_buff=[]
mfj1_buff=[]
mfj2_buff=[]
mfj3_buff=[]
mfj4_buff=[]
thj1_buff=[]
thj2_buff=[]
thj3_buff=[]
thj4_buff=[]
thj5_buff=[]
wrj1_buff=[]
wrj2_buff=[]
Tool_status_buff=[]
ffj1_offset=0
ffj2_offset=0
ffj3_offset=0
ffj4_offset=0
mfj1_offset=0
mfj2_offset=0
mfj3_offset=0
mfj4_offset=0
thj1_offset=0
thj2_offset=0
thj3_offset=0
thj4_offset=0
thj5_offset=0
wrj1_offset=0
wrj2_offset=0
Tool_status_offset=0


lst=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
def inverse_quat(q):
    q.w=-q.w
    return q





def calc_joints(pose1,pose2):
    output=Point()
    # output.x=data2.position.x-data1.position.x
    # output.y=data2.position.y-data1.position.y
    # output.z=data2.position.z-data1.position.z

    # relative quaternion!
    q1=pose1.orientation
    q1_inverse=inverse_quat(q1)
    q2=pose2.orientation

    # print("q1:", q1)
    # print("q2:", q2)

    # print("=============direct euler representation of each quaternion")
    # euler1=euler_from_quaternion([q1.x, q1.y, q1.z, q1.w])
    # euler2=euler_from_quaternion([q2.x, q2.y, q2.z, q2.w])
    # print("euler1.x", math.degrees(euler1[0]))
    # print("euler1.y", math.degrees(euler1[1]))
    # print("euler1.z", math.degrees(euler1[2]))
    # print("euler2.x", math.degrees(euler2[0]))
    # print("euler2.y", math.degrees(euler2[1]))
    # print("euler2.z", math.degrees(euler2[2]))
    # print("==================convert to euler =======================")
    # pub_eu_rel=rospy.Publisher('/pub_eu_rel',Point,queue_size=1)




    print("===================multiplication of two===================")
    # qr=q2*q1_inverse
    # q=quaternion_multiply([q2.x, q2.y, q2.z, q2.w], [q1_inverse.x,q1_inverse.y ,q1_inverse.z ,q1_inverse.w ])
    q=quaternion_multiply([q2.x, q2.y, q2.z, q2.w], [q1.x,q1.y ,q1.z ,-q1.w ])
    euler=euler_from_quaternion(q)
    print("euler.x", math.degrees(euler[0]))
    print("euler.y", math.degrees(euler[1]))
    print("euler.z", math.degrees(euler[2]))

    # publish relative quaternion

    pub_qr=rospy.Publisher('/q21_relative',Quaternion,queue_size=1)
    pub_qr_eu=rospy.Publisher('/q21_relative_euler',Point,queue_size=1)

    qr=Quaternion(q[0],q[1] ,q[2] ,q[3] )
    pub_qr.publish(qr)

    euler_point=Point()
    euler_point.x=euler[0]
    euler_point.y=euler[1]
    euler_point.z=euler[2]
    pub_qr_eu.publish(euler_point)



    # print("===================method of paper===================")
    # alpha
    # output.x=math.atan2(2*(qr.z*qr.w - qr.y*qr.x),qr.w*qr.w + qr.y*qr.y - qr.x*qr.x - qr.z*qr.z)
    # beta
    # output.y=math.asin(2*(qr.x*qr.w + qr.y*qr.z))
    # gama
    # output.z=math.atan2(2*(qr.y*qr.w - qr.x*qr.z),qr.w*qr.y - qr.y*qr.y - qr.x*qr.x + qr.z*qr.z)
    # print("Index_DIP.x:", math.degrees(output.x))
    # print("Index_DIP.y:", math.degrees(output.y))
    # print("Index_DIP.z", math.degrees(output.z))




    return output



def imu_serial_callback(data):
    global pub_rh_ffj0
    global pub_rh_ffj1
    global pub_rh_ffj2
    global pub_rh_ffj3
    global pub_rh_ffj4
    global pub_rh_mfj0
    global pub_rh_mfj1
    global pub_rh_mfj2
    global pub_rh_mfj3
    global pub_rh_mfj4
    global pub_rh_thj1
    global pub_rh_thj2
    global pub_rh_thj3
    global pub_rh_thj4
    global pub_rh_thj5
    global pub_rh_wrj1
    global pub_rh_wrj2
    global pub_hand_joints_array

    global calibration_counter

    global ffj1_buff
    global ffj2_buff
    global ffj3_buff
    global ffj4_buff
    global mfj1_buff
    global mfj2_buff
    global mfj3_buff
    global mfj4_buff
    global thj1_buff
    global thj2_buff
    global thj3_buff
    global thj4_buff
    global thj5_buff
    global wrj1_buff
    global wrj2_buff
    global Tool_status_buff

    global ffj1_offset
    global ffj2_offset
    global ffj3_offset
    global ffj4_offset
    global mfj1_offset
    global mfj2_offset
    global mfj3_offset
    global mfj4_offset
    global thj1_offset
    global thj2_offset
    global thj3_offset
    global thj4_offset
    global thj5_offset
    global wrj1_offset
    global wrj2_offset
    global Tool_status_offset

    # for sensor_id, sensor_values in enumerate(data.poses):
    #     print("sensor{}:{}".format(sensor_id, sensor_values))
    rospy.loginfo("test")

    #full sensor model
    Index_DIP = calc_joints(data.poses[0],data.poses[1])


    Index_PIP = 0#calc_joints(data.poses[6],data.poses[0])
    Index_MIP = 0#calc_joints(data.poses[9],data.poses[6])
    Middle_DIP = 0#calc_joints(data.poses[3],data.poses[2])
    Middle_PIP = 0#calc_joints(data.poses[7],data.poses[3])
    Middle_MIP = 0#calc_joints(data.poses[9],data.poses[7])
    Thumb_DIP = 0#calc_joints(data.poses[5],data.poses[4])
    Thumb_MIP = 0#calc_joints(data.poses[11],data.poses[5])
    Thumb_CMC = 0#calc_joints(data.poses[10],data.poses[11])
    Wrist_JOINT = 0#calc_joints(data.poses[8],data.poses[9])

    ffj1=Index_DIP.x
    ffj2=Index_PIP.x
    ffj3=Index_MIP.x
    ffj4=Index_MIP.z
    mfj1=Middle_DIP.x
    mfj2=Middle_PIP.x
    mfj3=Middle_MIP.x
    mfj4=Middle_MIP.z
    thj1=Thumb_DIP.x
    thj2=Thumb_MIP.x
    thj3=Thumb_MIP.z
    thj4=Thumb_CMC.z
    thj5=Thumb_CMC.y
    wrj1=Wrist_JOINT.x
    wrj2=Wrist_JOINT.z

    Tool_pose=data.poses[11]
    Tool_status=Tool_pose.position.x

    # print("Index_MIP:{}".format(Index_MIP))
    # print("Middle_DIP:{}".format(Middle_DIP))
    # print("Middle_MIP:{}".format(Middle_MIP))



    if calibration_counter<100:

        ffj1_buff.append(ffj1)
        ffj2_buff.append(ffj2)
        ffj3_buff.append(ffj3)
        ffj4_buff.append(ffj4)
        mfj1_buff.append(mfj1)
        mfj2_buff.append(mfj2)
        mfj3_buff.append(mfj3)
        mfj4_buff.append(mfj4)
        thj1_buff.append(thj1)
        thj2_buff.append(thj2)
        thj3_buff.append(thj3)
        thj4_buff.append(thj4)
        thj5_buff.append(thj5)
        wrj1_buff.append(wrj1)
        wrj2_buff.append(wrj2)
        Tool_status_buff.append(Tool_status)

        ffj1_offset=numpy.mean(ffj1_buff)
        ffj2_offset=numpy.mean(ffj2_buff)
        ffj3_offset=numpy.mean(ffj3_buff)
        ffj4_offset=numpy.mean(ffj4_buff)
        mfj1_offset=numpy.mean(mfj1_buff)
        mfj2_offset=numpy.mean(mfj2_buff)
        mfj3_offset=numpy.mean(mfj3_buff)
        mfj4_offset=numpy.mean(mfj4_buff)
        thj1_offset=numpy.mean(thj1_buff)
        thj2_offset=numpy.mean(thj2_buff)
        thj3_offset=numpy.mean(thj3_buff)
        thj4_offset=numpy.mean(thj4_buff)
        thj5_offset=numpy.mean(thj5_buff)
        wrj1_offset=numpy.mean(wrj1_buff)
        wrj2_offset=numpy.mean(wrj2_buff)
        Tool_status_offset=numpy.mean(Tool_status_buff)


        calibration_counter=calibration_counter+1

    else :
        # Publish all the final calibrated joint values
        lst[0] =    ffj1-ffj1_offset
        lst[1] =    ffj2-ffj2_offset
        lst[2] =    ffj3-ffj3_offset
        lst[3] =    ffj4-ffj4_offset
        lst[4] =    mfj1-mfj1_offset
        lst[5] =    mfj2-mfj2_offset
        lst[6] =    mfj3-mfj3_offset
        lst[7] =    mfj4-mfj4_offset
        lst[8] =    thj1-thj1_offset
        lst[9] =    thj2-thj2_offset
        lst[10] =   thj3-thj3_offset
        lst[11] =   thj4-thj4_offset
        lst[12] =   thj5-thj5_offset
        lst[13] =   wrj1-wrj1_offset
        lst[14] =   wrj2-wrj2_offset
        lst[15] =   Tool_status-Tool_status_offset

        joints_array = Float64ArrayStamped()
        joints_array.header.stamp = rospy.get_rostime()
        joints_array.header.frame_id = '/world'
        joints_array.data = lst
        pub_hand_joints_array.publish(joints_array)



        pub_rh_ffj0.publish(math.radians(ffj1-ffj1_offset))
        pub_rh_ffj1.publish(math.radians(ffj1-ffj1_offset))
        pub_rh_ffj2.publish(math.radians(ffj2-ffj2_offset))
        pub_rh_ffj3.publish(math.radians(ffj3-ffj3_offset))
        pub_rh_ffj4.publish(math.radians(ffj4-ffj4_offset))
        pub_rh_mfj0.publish(math.radians(mfj1-mfj1_offset))
        pub_rh_mfj1.publish(math.radians(mfj1-mfj1_offset))
        pub_rh_mfj2.publish(math.radians(mfj2-mfj2_offset))
        pub_rh_mfj3.publish(math.radians(mfj3-mfj3_offset))
        pub_rh_mfj4.publish(math.radians(mfj4-mfj4_offset))
        pub_rh_thj1.publish(math.radians(thj1-thj1_offset))
        pub_rh_thj2.publish(math.radians(thj2-thj2_offset))
        pub_rh_thj3.publish(math.radians(thj3-thj3_offset))
        pub_rh_thj4.publish(math.radians(thj4-thj4_offset))
        pub_rh_thj5.publish(math.radians(thj5-thj5_offset))
        pub_rh_wrj1.publish(math.radians(wrj1-wrj1_offset))
        pub_rh_wrj2.publish(math.radians(wrj2-wrj2_offset))


        # print("Index_PIP_fl:{},Index_MIP_fl:{},Index_MIP_ab:{}".format(Index_PIP_fl,Index_MIP_fl,Index_MIP_ab))
        # print("Index_PIP_fl:{},Index_MIP_fl:{},Index_MIP_ab:{}".format(Index_PIP_fl-Index_PIP_fl_offset,
        #                                                                Index_MIP_fl-Index_MIP_fl_offset,
        #                                                                Index_MIP_ab-Index_MIP_ab_offset))


        # print("Middle_PIP_fl:{},Middle_MIP_fl:{},Middle_MIP_ab:{}".format(Middle_PIP_fl, Middle_MIP_fl, Middle_MIP_ab))
        # print("Middle_PIP_fl:{},Middle_MIP_fl:{},Middle_MIP_ab:{}".format(Middle_PIP_fl - Middle_PIP_fl_offset,
        #                                                                Middle_MIP_fl - Middle_MIP_fl_offset,
        #                                                                Middle_MIP_ab - Middle_MIP_ab_offset))


        # print("Thumb_PIP_fl:{},Thumb_MIP_fl:{},Thumb_MIP_ab:{}".format(Thumb_DIP_fl - Thumb_DIP_fl_offset,
        #                                                                Thumb_MIP_fl - Thumb_MIP_fl_offset,
        #                                                                Thumb_CMC_ab - Thumb_CMC_ab_offset))


        # print("Wrist_JOINT_fl_buff:{},Wrist_JOINT_ab_buff:{}".format(Wrist_JOINT_fl-Wrist_JOINT_fl_offset,
        #                                         Wrist_JOINT_ab-Wrist_JOINT_ab_offset))





def main():

    rospy.init_node('hand_joint_mapping', anonymous=True)

    # rospy.Subscriber("/odas_sst_raw", Odas, odas_snobber_Callback, queue_size=10)
    rospy.Subscriber("/imu_pub_array", PoseArray, imu_serial_callback, queue_size=10)
    # global listener
    global pub_rh_ffj0
    global pub_rh_ffj1
    global pub_rh_ffj2
    global pub_rh_ffj3
    global pub_rh_ffj4
    global pub_rh_mfj0
    global pub_rh_mfj1
    global pub_rh_mfj2
    global pub_rh_mfj3
    global pub_rh_mfj4
    global pub_rh_thj1
    global pub_rh_thj2
    global pub_rh_thj3
    global pub_rh_thj4
    global pub_rh_thj5
    global pub_rh_wrj1
    global pub_rh_wrj2
    global pub_hand_joints_array


    pub_hand_joints_array=rospy.Publisher('/hand_joints_array',Float64ArrayStamped , queue_size=1)



    # listener = tf.TransformListener()
    pub_rh_ffj0 = rospy.Publisher("/sh_rh_ffj0_position_controller/command", Float64, queue_size=10) #index pip
    pub_rh_ffj1 = rospy.Publisher("/sh_rh_ffj1_position_controller/command", Float64, queue_size=10) #index pip
    pub_rh_ffj2 = rospy.Publisher("/sh_rh_ffj2_position_controller/command", Float64, queue_size=10) #index pip
    pub_rh_ffj3 = rospy.Publisher("/sh_rh_ffj3_position_controller/command", Float64, queue_size=10)#index mix flex
    pub_rh_ffj4 = rospy.Publisher("/sh_rh_ffj4_position_controller/command", Float64, queue_size=10)#index mix abd

    pub_rh_mfj0 = rospy.Publisher("/sh_rh_mfj0_position_controller/command", Float64, queue_size=10) #middle pip
    pub_rh_mfj1 = rospy.Publisher("/sh_rh_mfj1_position_controller/command", Float64, queue_size=10) #middle pip
    pub_rh_mfj2 = rospy.Publisher("/sh_rh_mfj2_position_controller/command", Float64, queue_size=10) #middle pip
    pub_rh_mfj3 = rospy.Publisher("/sh_rh_mfj3_position_controller/command", Float64, queue_size=10)#middle mix flex
    pub_rh_mfj4 = rospy.Publisher("/sh_rh_mfj4_position_controller/command", Float64, queue_size=10)#middle mix abd

    pub_rh_thj1 = rospy.Publisher("/sh_rh_thj1_position_controller/command", Float64, queue_size=10) #thumb dip
    pub_rh_thj2 = rospy.Publisher("/sh_rh_thj2_position_controller/command", Float64, queue_size=10)#thumb mip
    pub_rh_thj3 = rospy.Publisher("/sh_rh_thj3_position_controller/command", Float64, queue_size=10)#thumb mip flex
    pub_rh_thj4 = rospy.Publisher("/sh_rh_thj4_position_controller/command", Float64, queue_size=10)#thumb cmc abd
    pub_rh_thj5 = rospy.Publisher("/sh_rh_thj5_position_controller/command", Float64, queue_size=10)#cmc turning around it self!
    # the final one seems tricky a bit. needs to be studied in realtime.

    pub_rh_wrj1 = rospy.Publisher("/sh_rh_wrj1_position_controller/command", Float64, queue_size=10) #wrist flex
    pub_rh_wrj2 = rospy.Publisher("/sh_rh_wrj2_position_controller/command", Float64, queue_size=10)#wrist abd

    rospy.spin()


if __name__ == '__main__':
    main()



# shadow robot's joint limits:
# sh_rh_ffj0=  0 > 180      : 0 means open
# sh_rh_ffj3=  0 > 90       : 0 means open straight
# sh_rh_ffj4= -20 > 20      : 0 means open stright

# sh_rh_mfj0= 0 > 180
# sh_rh_mfj3= 0 > 90
# sh_rh_mfj4= -20 > 20


# sh_rh_thj1=  0 > 90       :0 mens open straght
# sh_rh_thj2=  -40 > 40     : 0 means open straight
# sh_rh_thj3=  -12 > 12     : 0 means straight
# sh_rh_thj4= 0 > 70
# sh_rh_thj5= -60 > 60

# sh_rh_wrj1=  -40 > 28
# sh_rh_wrj2=  -30 > 10