#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from odas_msgs.msg import Odas, OdasList
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from hand_tracking.msg import Float64ArrayStamped

from geometry_msgs.msg import TwistStamped
import numpy
from geometry_msgs.msg import Quaternion
import math
import message_filters
import tf
import datetime
from tf.transformations import *

joints_array=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
index_pose=Pose()
middle_pose=Pose()
wrist_pose=Pose()
thumb_pose=Pose()

zero_pose=Pose()
zero_pose.position.x=0
zero_pose.position.y=0
zero_pose.position.z=0
zero_pose.orientation.x=0
zero_pose.orientation.y=0
zero_pose.orientation.z=0
imu_array = [zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose]



now = datetime.datetime.now()
str_filename=now.strftime("/home/moe/smartsurg_logfiles/mario/%Y-%m-%d_%H-%M.csv")


def conv_quat_to_degree(quat):
    euler_rad=Quaternion()
    orientation_euler=Quaternion()
    [euler_rad.x, euler_rad.y, euler_rad.z] = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    orientation_euler.x=math.degrees(euler_rad.x)
    orientation_euler.y=math.degrees(euler_rad.y)
    orientation_euler.z=math.degrees(euler_rad.z)
    return orientation_euler

def conv_data(data):
    output_r=Point()
    output_d=Point()
    [output_r.x, output_r.y, output_r.z] = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
    output_d.x=math.degrees(output_r.x)
    output_d.y=math.degrees(output_r.y)
    output_d.z=math.degrees(output_r.z)
    return output_d

def imu_serial_callback(data):
    global imu_array
    imu_array=data.poses

def hand_joints_callback(data):
    global joints_array
    joints_array=data.data
    # print("imu_array:{}".format(imu_array))

def Middle_callback(data):
    global middle_pose
    middle_pose.position = data.pose.position
    middle_pose.orientation=conv_quat_to_degree(data.pose.orientation)

def Index_callback(data):
    global index_pose
    index_pose.position = data.pose.position
    index_pose.orientation=conv_quat_to_degree(data.pose.orientation)

def Wrist_callback(data):
    global wrist_pose
    wrist_pose.position = data.pose.position
    wrist_pose.orientation=conv_quat_to_degree(data.pose.orientation)

def Thumb_callback(data):
    global thumb_pose
    thumb_pose.position = data.pose.position
    thumb_pose.orientation=conv_quat_to_degree(data.pose.orientation)





def save_to_csv(f):
    global joints_array
    global imu_array
    global index_pose
    global middle_pose
    global wrist_pose
    global thumb_pose

    global str_filename

    rate = rospy.Rate(50)
    jason_stream_lables = rospy.get_param('/jason_stream_lables')

    while not rospy.is_shutdown():
        try:

            rospy.loginfo("---save_to_csv---")
            # print("imu_array:{}".format(imu_array))

            S0 = conv_data(imu_array[jason_stream_lables.index('S0')])
            S1 = conv_data(imu_array[jason_stream_lables.index('S1')])
            S2 = conv_data(imu_array[jason_stream_lables.index('S2')])
            S3 = conv_data(imu_array[jason_stream_lables.index('S3')])
            S4 = conv_data(imu_array[jason_stream_lables.index('S4')])
            S5 = conv_data(imu_array[jason_stream_lables.index('S5')])
            S6 = conv_data(imu_array[jason_stream_lables.index('S6')])
            S7 = conv_data(imu_array[jason_stream_lables.index('S7')])
            S8 = conv_data(imu_array[jason_stream_lables.index('S8')])
            S9 = conv_data(imu_array[jason_stream_lables.index('S9')])
            S10 = conv_data(imu_array[jason_stream_lables.index('S10')])
            S11 = conv_data(imu_array[jason_stream_lables.index('S11')])
            # SC = conv_data(imu_array[12])

            seconds = rospy.get_time()
            f = open(str_filename, "a+")
            # f.write("az=0,el=0,%d,%d,%d,%d,\r\n" % (i))
            f.write('{0},'
                    '{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},'
                    '{13},{14},{15},{16},'
                    '{17},{18},{19},{20},{21},{22},{23},{24},{25},{26},{27},{28},'
                    '{29},{30},{31},{32},{33},{34},{35},{36},\r\n'
                    .format(seconds,

                            S8.x, S8.y, S8.z,
                            S9.x, S9.y, S9.z,
                            S10.x, S10.y, S10.z,
                            S11.x, S11.y, S11.z,

                            middle_pose.position.x,middle_pose.position.y,middle_pose.position.z,
                            middle_pose.orientation.x,middle_pose.orientation.y,middle_pose.orientation.z,
                            index_pose.position.x, index_pose.position.y, index_pose.position.z,
                            index_pose.orientation.x, index_pose.orientation.y, index_pose.orientation.z,
                            thumb_pose.position.x, thumb_pose.position.y, thumb_pose.position.z,
                            thumb_pose.orientation.x, thumb_pose.orientation.y, thumb_pose.orientation.z,
                            wrist_pose.position.x,wrist_pose.position.y,wrist_pose.position.z,
                            wrist_pose.orientation.x,wrist_pose.orientation.y,wrist_pose.orientation.z))
            f.close()
        except rospy.ServiceException, e:
            print("save_to_csv  failed: %s" % e)

        rate.sleep()
    rospy.spin()

def main():
    global str_filename

    rospy.init_node('smartsurg_savedata', anonymous=True)
    rospy.loginfo("---WELCOME TO Save to file---")
    rospy.Subscriber("/hand_joints_array", Float64ArrayStamped, hand_joints_callback, queue_size=10)
    rospy.Subscriber("/imu_pub_array", PoseArray, imu_serial_callback, queue_size=10)
    rospy.Subscriber("/ndi/IndexMarker/position_cartesian_current", PoseStamped, Index_callback, queue_size=10)
    rospy.Subscriber("/ndi/MiddleMarker/position_cartesian_current", PoseStamped, Middle_callback, queue_size=10)
    rospy.Subscriber("/ndi/WristMarker/position_cartesian_current", PoseStamped, Wrist_callback, queue_size=10)
    rospy.Subscriber("/ndi/ThumbMarker/position_cartesian_current", PoseStamped, Thumb_callback, queue_size=10)




    f = open(str_filename, "a+")
    f.write('seconds,'
                    'S8.x,S8.y,S8.z,'
                    'S9.x,S9.y,S9.z,'
                    'S10.x,S10.y,S10.z,'
                    'S11.x,S11.y,S11.z,'
                    'middle_pose.position.x,middle_pose.position.y,middle_pose.position.z,'
                    'middle_pose.orientation.x,middle_pose.orientation.y,middle_pose.orientation.z,'
                    'index_pose.position.x, index_pose.position.y, index_pose.position.z,'
                    'index_pose.orientation.x, index_pose.orientation.y, index_pose.orientation.z,'
                    'thumb_pose.position.x, thumb_pose.position.y, thumb_pose.position.z,'
                    'thumb_pose.orientation.x, thumb_pose.orientation.y, thumb_pose.orientation.z,'
                    'wrist_pose.position.x,wrist_pose.position.y,wrist_pose.position.z,'
                    'wrist_pose.orientation.x,wrist_pose.orientation.y,wrist_pose.orientation.z, \r\n')

    # print('seconds,robot_x,robot_y,robot_z,landmark_count,source_az,source_el,source_sim_az,source_sim_el,covar_mat1_det,covar_mat1l_det,covar_mat2_det,covar_mat2l_det,ss_pos_x,ss_pos_y,ss_pos_z,\r\n')
    f.close()

    save_to_csv(f)

    # rospy.spin()

if __name__ == '__main__':
    main()
