#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from odas_msgs.msg import Odas, OdasList
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

from geometry_msgs.msg import TwistStamped
import numpy

import math
import message_filters
listener=0
import tf
import datetime
from tf.transformations import *

# I should only record data when both frames are available , both tool and wrist.
# I should read tool frame converted to wrist frame.


now = datetime.datetime.now()
str_filename=now.strftime("/home/moe/smartsurg_logfiles/Raimondo5Jul/exp/%Y-%m-%d_%H-%M.csv")



def conv_data(data):
    output=Point()
    output.x=data.position.x
    output.y=data.position.y
    output.z=data.position.z
    return output

def calc_joints(data1,data2):
    output=Point()
    output.x=data2.position.x-data1.position.x
    output.y=data2.position.y-data1.position.y
    output.z=data2.position.z-data1.position.z
    return output

def callback(imu_sub, tool_sub):
    global listener
    rospy.loginfo("Saveing Data ...")
    print("tool_sub:",tool_sub)
    tool_cf = listener.transformPose('Wrist', tool_sub)
    print('tool_cf',tool_cf)
    data=imu_sub
    #full sensor model
    Index_DIP = calc_joints(data.poses[0],data.poses[1])
    Index_PIP = calc_joints(data.poses[6],data.poses[0])
    Index_MIP = calc_joints(data.poses[9],data.poses[6])
    Middle_DIP = calc_joints(data.poses[3],data.poses[2])
    Middle_PIP = calc_joints(data.poses[7],data.poses[3])
    Middle_MIP = calc_joints(data.poses[9],data.poses[7])
    Thumb_DIP = calc_joints(data.poses[5],data.poses[4])
    Thumb_MIP = calc_joints(data.poses[11],data.poses[5])
    Thumb_CMC = calc_joints(data.poses[10],data.poses[11])
    Wrist_JOINT = calc_joints(data.poses[8],data.poses[9])


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

    Tool_pose=data.poses[12]
    Tool_status=Tool_pose.position.x

    imu_array=imu_sub.poses
    S0 = conv_data(imu_array[0])
    S1 = conv_data(imu_array[1])
    S2 = conv_data(imu_array[2])
    S3 = conv_data(imu_array[3])
    S4 = conv_data(imu_array[4])
    S5 = conv_data(imu_array[5])
    S6 = conv_data(imu_array[6])
    S7 = conv_data(imu_array[7])
    S8 = conv_data(imu_array[8])
    S9 = conv_data(imu_array[9])
    S10 = conv_data(imu_array[10])
    S11 = conv_data(imu_array[11])
    SC = conv_data(imu_array[12])
    tool_pose=tool_sub.pose
    seconds = rospy.get_time()
    f = open(str_filename, "a+")
    f.write('{0},{1},'.format(seconds,Tool_status))
    f.write('{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},'
            .format(ffj1, ffj2, ffj3, ffj4, mfj1, mfj2, mfj3, mfj4,
                    thj1, thj2, thj3, thj4, thj5, wrj1, wrj2))
    f.write('{0},{1},{2},{3},{4},{5},{6},'
        .format(tool_pose.position.x, tool_pose.position.y, tool_pose.position.z,
                tool_pose.orientation.x, tool_pose.orientation.y, tool_pose.orientation.z,tool_pose.orientation.w))

    f.write('\r\n')
    f.close()



def main():
    global str_filename
    global listener
    rospy.init_node('smartsurg_savedata', anonymous=True)
    rospy.loginfo("---WELCOME TO Save to file---")
    # rospy.Subscriber("/imu_pub_array", PoseArray, imu_serial_callback, queue_size=10)
    imu_sub=message_filters.Subscriber('/imu_pub_array', PoseArray)
    tool_sub=message_filters.Subscriber('/tool_wf', PoseStamped)

    ts = message_filters.ApproximateTimeSynchronizer([imu_sub, tool_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    listener = tf.TransformListener()
    #The API is the same as TimeSynchronizer except for an extra slop parameter in the constructor that defines the delay (in seconds) with which messages can be synchronized
    f = open(str_filename, "a+")
    f.write('seconds,Tool_status,')
    f.write('ffj1, ffj2, ffj3, ffj4, mfj1, mfj2, mfj3, mfj4,'
            'thj1, thj2, thj3, thj4, thj5, wrj1, wrj2,')
    f.write('tool_pose.position.x, tool_pose.position.y, tool_pose.position.z,'
            'tool_pose.orientation.x, tool_pose.orientation.y, tool_pose.orientation.z,tool_pose.orientation.w,')
    f.write('\r\n')
    f.close()

    rospy.spin()

if __name__ == '__main__':
    main()

