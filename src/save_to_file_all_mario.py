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
from geometry_msgs.msg import QuaternionStamped

from geometry_msgs.msg import TwistStamped
import numpy
from geometry_msgs.msg import Quaternion
import math
import message_filters
import tf
import datetime
from tf.transformations import *
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped

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
S5_4_quat=Quaternion()
global listener


now = datetime.datetime.now()
str_filename=now.strftime("/home/moe/smartsurg_logfiles/mario_MJA/%Y-%m-%d_%H-%M-%S.csv")


def transform_to_cf(quat, target_frame):
    global listener
    quat_st_wf = QuaternionStamped()
    quat_st_wf.header.frame_id = '/world'
    quat_st_wf.quaternion = quat
    quat_st_cf = listener.transformQuaternion(target_frame, quat_st_wf)
    return quat_st_cf.quaternion


def append_new_frame(translation, rotation, frame_id, frame_target):
    pub_tf = rospy.Publisher("/tf", tfMessage, queue_size=1)

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


def publish_all_wrt_world(imu_pose_list):
    sensor_f_list = rospy.get_param('/sensor_f_list')
    hft_tf_translations = rospy.get_param('/hft_tfw')
    for idx, pose in enumerate(imu_pose_list):
        temp_tr = hft_tf_translations[sensor_f_list[idx]]
        append_new_frame((temp_tr[0], temp_tr[1], temp_tr[2]),
                         (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                         '/world', sensor_f_list[idx])


def transform_all_to_cf(imu_pose_list):
    sensor_rf_list = rospy.get_param('/sensor_rf_list')
    quat_list_cf = []
    euler_list_cf = []
    for idx, pose in enumerate(imu_pose_list):
        quat_cf = transform_to_cf(pose.orientation, sensor_rf_list[idx])
        quat_list_cf.append(quat_cf)

        euler = Point()
        [euler.x, euler.y, euler.z] = euler_from_quaternion([quat_cf.x, quat_cf.y, quat_cf.z, quat_cf.w])
        euler_list_cf.append(euler)

    return quat_list_cf, euler_list_cf

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
    global S5_4_quat
    sensor_f_list = rospy.get_param('/sensor_f_list')
    imu_array=data.poses
    publish_all_wrt_world(imu_array)
    quat_list_cf, euler_list_cf = transform_all_to_cf(imu_array)
    S5_4_quat = quat_list_cf[sensor_f_list.index('S5')]

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
    global S5_4_quat

    global str_filename

    rate = rospy.Rate(50)
    jason_stream_lables = rospy.get_param('/jason_stream_lables')

    while not rospy.is_shutdown():
        try:

            rospy.loginfo("---save_to_csv---")
            # print("imu_array:{}".format(imu_array))

            S0 = imu_array[jason_stream_lables.index('S0')].orientation
            S1 = imu_array[jason_stream_lables.index('S1')].orientation
            S2 = imu_array[jason_stream_lables.index('S2')].orientation
            S3 = imu_array[jason_stream_lables.index('S3')].orientation
            S4 = imu_array[jason_stream_lables.index('S4')].orientation
            S5 = imu_array[jason_stream_lables.index('S5')].orientation
            S6 = imu_array[jason_stream_lables.index('S6')].orientation
            S7 = imu_array[jason_stream_lables.index('S7')].orientation
            S8 = imu_array[jason_stream_lables.index('S8')].orientation
            S9 = imu_array[jason_stream_lables.index('S9')].orientation
            S10 = imu_array[jason_stream_lables.index('S10')].orientation
            S11 = imu_array[jason_stream_lables.index('S11')].orientation

            S0_e = conv_data(imu_array[jason_stream_lables.index('S0')])
            S1_e = conv_data(imu_array[jason_stream_lables.index('S1')])
            S2_e = conv_data(imu_array[jason_stream_lables.index('S2')])
            S3_e = conv_data(imu_array[jason_stream_lables.index('S3')])
            S4_e = conv_data(imu_array[jason_stream_lables.index('S4')])
            S5_e = conv_data(imu_array[jason_stream_lables.index('S5')])
            S6_e = conv_data(imu_array[jason_stream_lables.index('S6')])
            S7_e = conv_data(imu_array[jason_stream_lables.index('S7')])
            S8_e = conv_data(imu_array[jason_stream_lables.index('S8')])
            S9_e = conv_data(imu_array[jason_stream_lables.index('S9')])
            S10_e = conv_data(imu_array[jason_stream_lables.index('S10')])
            S11_e = conv_data(imu_array[jason_stream_lables.index('S11')])
            # SC = conv_data(imu_array[12])
            S5_4_quat_pose=Pose()
            S5_4_quat_pose.orientation=S5_4_quat
            S5_4_quat_e=conv_data(S5_4_quat_pose)
            # print(S5_4_quat_e)

            # find relative quaternion

            seconds = rospy.get_time()
            f = open(str_filename, "a+")
            # f.write("az=0,el=0,%d,%d,%d,%d,\r\n" % (i))
            f.write('{0},'
                    '{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17},{18},{19},{20},{21},{22},{23},{24},{25},{26},{27},{28},{29},{30},{31},{32},'
                    '{33},{34},{35},{36},{37},{38},{39},{40},{41},{42},{43},{44},{45},{46},{47},{48},{49},{50},{51},{52},{53},{54},{55},{56},'
                    '{57},{58},{59},{60},{61},{62},{63},{64},{65},{66},{67},{68},{69},{70},{71},{72},{73},{74},{75},{76},{77},{78},{79},{80},'
                    '{81},{82},{83},{84},{85},{86},{87}'
                    '\r\n'
                    .format(seconds,
                            S4.x, S4.y, S4.z,S4.w,
                            S5.x, S5.y, S5.z,S5.w,
                            S6.x, S6.y, S6.z,S6.w,
                            S7.x, S7.y, S7.z,S7.w,
                            S8.x, S8.y, S8.z,S8.w,
                            S9.x, S9.y, S9.z,S9.w,
                            S10.x, S10.y, S10.z,S10.w,
                            S11.x, S11.y, S11.z,S11.w,

                            S4_e.x, S4_e.y, S4_e.z,
                            S5_e.x, S5_e.y, S5_e.z,
                            S6_e.x, S6_e.y, S6_e.z,
                            S7_e.x, S7_e.y, S7_e.z,
                            S8_e.x, S8_e.y, S8_e.z,
                            S9_e.x, S9_e.y, S9_e.z,
                            S10_e.x, S10_e.y, S10_e.z,
                            S11_e.x, S11_e.y, S11_e.z,

                            middle_pose.position.x,middle_pose.position.y,middle_pose.position.z,
                            middle_pose.orientation.x,middle_pose.orientation.y,middle_pose.orientation.z,
                            index_pose.position.x, index_pose.position.y, index_pose.position.z,
                            index_pose.orientation.x, index_pose.orientation.y, index_pose.orientation.z,
                            thumb_pose.position.x, thumb_pose.position.y, thumb_pose.position.z,
                            thumb_pose.orientation.x, thumb_pose.orientation.y, thumb_pose.orientation.z,
                            wrist_pose.position.x,wrist_pose.position.y,wrist_pose.position.z,
                            wrist_pose.orientation.x,wrist_pose.orientation.y,wrist_pose.orientation.z,

                            S5_4_quat.x,S5_4_quat.y,S5_4_quat.z,S5_4_quat.w,
                            S5_4_quat_e.x,S5_4_quat_e.y,S5_4_quat_e.z
                            ))
            f.close()
        except rospy.ServiceException, e:
            print("save_to_csv  failed: %s" % e)

        rate.sleep()
    rospy.spin()

def main():
    global str_filename
    global listener

    rospy.init_node('smartsurg_savedata', anonymous=True)
    rospy.loginfo("---WELCOME TO Save to file---")
    rospy.Subscriber("/hand_joints_array", Float64ArrayStamped, hand_joints_callback, queue_size=10)
    rospy.Subscriber("/imu_pub_array", PoseArray, imu_serial_callback, queue_size=10)
    rospy.Subscriber("/ndi/IndexMarkerM/position_cartesian_current", PoseStamped, Index_callback, queue_size=10)
    rospy.Subscriber("/ndi/MiddleMarkerM/position_cartesian_current", PoseStamped, Middle_callback, queue_size=10)
    rospy.Subscriber("/ndi/WristMarkerM/position_cartesian_current", PoseStamped, Wrist_callback, queue_size=10)
    rospy.Subscriber("/ndi/ThumbMarkerM/position_cartesian_current", PoseStamped, Thumb_callback, queue_size=10)
    listener = tf.TransformListener()



    f = open(str_filename, "a+")
    f.write('seconds,'
                    'S4.x, S4.y, S4.z,S4.w,'
                    'S5.x, S5.y, S5.z,S5.w,'
                    'S6.x, S6.y, S6.z,S6.w,'
                    'S7.x, S7.y, S7.z,S7.w,'
                    'S8.x, S8.y, S8.z,S8.w,'
                    'S9.x, S9.y, S9.z,S9.w,'
                    'S10.x, S10.y, S10.z,S10.w,'
                    'S11.x, S11.y, S11.z,S11.w,'
                    'S4_e.x, S4_e.y, S4_e.z,'
                    'S5_e.x, S5_e.y, S5_e.z,'
                    'S6_e.x, S6_e.y, S6_e.z,'
                    'S7_e.x, S7_e.y, S7_e.z,'
                    'S8_e.x, S8_e.y, S8_e.z,'
                    'S9_e.x, S9_e.y, S9_e.z,'
                    'S10_e.x, S10_e.y, S10_e.z,'
                    'S11_e.x, S11_e.y, S11_e.z,'
                    'middle_pose.position.x,middle_pose.position.y,middle_pose.position.z,'
                    'middle_pose.orientation.x,middle_pose.orientation.y,middle_pose.orientation.z,'
                    'index_pose.position.x, index_pose.position.y, index_pose.position.z,'
                    'index_pose.orientation.x, index_pose.orientation.y, index_pose.orientation.z,'
                    'thumb_pose.position.x, thumb_pose.position.y, thumb_pose.position.z,'
                    'thumb_pose.orientation.x, thumb_pose.orientation.y, thumb_pose.orientation.z,'
                    'wrist_pose.position.x,wrist_pose.position.y,wrist_pose.position.z,'
                    'wrist_pose.orientation.x,wrist_pose.orientation.y,wrist_pose.orientation.z,'
                    'S5_4_quat.x,S5_4_quat.y,S5_4_quat.z,S5_4_quat.w,'
                    'S5_4_quat_e.x,S5_4_quat_e.y,S5_4_quat_e.z, \r\n')

    # print('seconds,robot_x,robot_y,robot_z,landmark_count,source_az,source_el,source_sim_az,source_sim_el,covar_mat1_det,covar_mat1l_det,covar_mat2_det,covar_mat2l_det,ss_pos_x,ss_pos_y,ss_pos_z,\r\n')
    f.close()

    save_to_csv(f)

    # rospy.spin()

if __name__ == '__main__':
    main()



