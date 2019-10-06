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
box_pose=Pose()
tool_pose=Pose()
wrist_pose=Pose()

zero_pose=Pose()
zero_pose.position.x=0
zero_pose.position.y=0
zero_pose.position.z=0
zero_pose.orientation.x=0
zero_pose.orientation.y=0
zero_pose.orientation.z=0
imu_array = [zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose,zero_pose]



now = datetime.datetime.now()
str_filename=now.strftime("/home/moe/smartsurg_logfiles/imu_compare/%Y-%m-%d_%H-%M.csv")


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

def box_callback(data):
    global box_pose
    box_pose.position = data.pose.position
    box_pose.orientation=conv_quat_to_degree(data.pose.orientation)
def tool_callback(data):
    global tool_pose
    tool_pose = data.pose
    tool_pose.position = data.pose.position
    tool_pose.orientation=conv_quat_to_degree(data.pose.orientation)

def wrist_callback(data):
    global wrist_pose
    wrist_pose=data.pose
    wrist_pose.position = data.pose.position
    wrist_pose.orientation=conv_quat_to_degree(data.pose.orientation)





def save_to_csv(f):
    global joints_array
    global imu_array
    global box_pose
    global tool_pose
    global wrist_pose
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
                    '{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},'
                    ','
                    '{17},{18},{19},{20},{21},{22},{23},{24},{25},{26},{27},{28},'
                    '{29},{30},{31},{32},{33},{34},{35},{36},{37},{38},{39},{40},'
                    '{41},{42},{43},{44},{45},{46},{47},{48},{49},{50},{51},{52},'
                    ','
                    '{53},{54},{55},{56},{57},{58},{59},'
                    '{60},{61},{62},{63},{64},{65},'
                    '{66},{67},{68},{69},{70},\r\n'
                    .format(seconds,
                            joints_array[0], joints_array[1], joints_array[2], joints_array[3], joints_array[4],
                            joints_array[5], joints_array[6], joints_array[7], joints_array[8], joints_array[9],
                            joints_array[10], joints_array[11], joints_array[12], joints_array[13], joints_array[14],
                            joints_array[15],

                            S0.x, S0.y, S0.z,
                            S1.x, S1.y, S1.z,
                            S2.x, S2.y, S2.z,
                            S3.x, S3.y, S3.z,
                            S4.x, S4.y, S4.z,
                            S5.x, S5.y, S5.z,
                            S6.x, S6.y, S6.z,
                            S7.x, S7.y, S7.z,
                            S8.x, S8.y, S8.z,
                            S9.x, S9.y, S9.z,
                            S10.x, S10.y, S10.z,
                            S11.x, S11.y, S11.z,

                            tool_pose.position.x,tool_pose.position.y,tool_pose.position.z,
                            tool_pose.orientation.x,tool_pose.orientation.y,tool_pose.orientation.z,
                            wrist_pose.position.x,wrist_pose.position.y,wrist_pose.position.z,
                            wrist_pose.orientation.x,wrist_pose.orientation.y,wrist_pose.orientation.z,
                            box_pose.position.x,box_pose.position.y,box_pose.position.z,
                            box_pose.orientation.x,box_pose.orientation.y,box_pose.orientation.z))
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
    rospy.Subscriber("/ndi/Box/position_cartesian_current", PoseStamped, box_callback, queue_size=10)
    rospy.Subscriber("/ndi/Tool/position_cartesian_current", PoseStamped, tool_callback, queue_size=10)
    rospy.Subscriber("/ndi/Wrist/position_cartesian_current", PoseStamped, wrist_callback, queue_size=10)




    f = open(str_filename, "a+")
    f.write('seconds,'
                    'ffj1,ffj2, ffj3, ffj4, mfj1, mfj2, mfj3, mfj4, thj1, thj2,'
                    'thj3, thj4, thj5, wrj1, wrj2, Tool,'
                    ','
                    'S0.x,S0.y,S0.z,'
                    'S1.x,S1.y,S1.z,'
                    'S2.x,S2.y,S2.z,'
                    'S3.x,S3.y,S3.z,'
                    'S4.x,S4.y,S4.z,'
                    'S5.x,S5.y,S5.z,'
                    'S6.x,S6.y,S6.z,'
                    'S7.x,S7.y,S7.z,'
                    'S8.x,S8.y,S8.z,'
                    'S9.x,S9.y,S9.z,'
                    'S10.x,S10.y,S10.z,'
                    'S11.x,S11.y,S11.z,'
                    ','
                    'tool_pose.position.x,tool_pose.position.y,tool_pose.position.z,'
                    'tool_pose.orientation.x,tool_pose.orientation.y,tool_pose.orientation.z,'
                    'wrist_pose.position.x,wrist_pose.position.y,wrist_pose.position.z,'
                    'wrist_pose.orientation.x,wrist_pose.orientation.y,wrist_pose.orientation.z,'
                    'box_pose.position.x,box_pose.position.y,box_pose.position.z,'
                    'box_pose.orientation.x,box_pose.orientation.y,box_pose.orientation.z, \r\n')

    # print('seconds,robot_x,robot_y,robot_z,landmark_count,source_az,source_el,source_sim_az,source_sim_el,covar_mat1_det,covar_mat1l_det,covar_mat2_det,covar_mat2l_det,ss_pos_x,ss_pos_y,ss_pos_z,\r\n')
    f.close()

    save_to_csv(f)

    # rospy.spin()

if __name__ == '__main__':
    main()
