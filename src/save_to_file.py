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


import datetime



now = datetime.datetime.now()
str_filename=now.strftime("/home/moe/smartsurg_logfiles/%Y-%m-%d_%H-%M.csv")

def conv_data(data):
    output=Point()
    output.x=data.position.x
    output.y=data.position.y
    output.z=data.position.z
    return output


def imu_serial_callback(data):
    rospy.loginfo("Saveing Data ...")

    S0 = conv_data(data.poses[0])
    S1 = conv_data(data.poses[1])
    S2 = conv_data(data.poses[2])
    S3 = conv_data(data.poses[3])
    S4 = conv_data(data.poses[4])
    S5 = conv_data(data.poses[5])
    S6 = conv_data(data.poses[6])
    S7 = conv_data(data.poses[7])
    S8 = conv_data(data.poses[8])
    S9 = conv_data(data.poses[9])
    S10 = conv_data(data.poses[10])
    S11 = conv_data(data.poses[11])
    SC = conv_data(data.poses[12])


    seconds = rospy.get_time()
    f = open(str_filename, "a+")
    # f.write("az=0,el=0,%d,%d,%d,%d,\r\n" % (i))
    f.write('{0},{1},{2},{3},'
            '{4},{5},{6},{7},'
            '{8},{9},{10},{11},'
            '{12},{13},{14},{15},'
            '{16},{17},{18},{19},'
            '{20},{21},{22},{23},'
            '{24},{25},{26},{27},'
            '{28},{29},{30},{31},'
            '{32},{33},{34},{35},{36},{37}\r\n'
            .format(seconds,SC.x,
                    S0.x,S0.y,S0.z,
                    S1.x,S1.y,S1.z,
                    S2.x,S2.y,S2.z,
                    S3.x,S3.y,S3.z,
                    S4.x,S4.y,S4.z,
                    S5.x,S5.y,S5.z,
                    S6.x,S6.y,S6.z,
                    S7.x,S7.y,S7.z,
                    S8.x,S8.y,S8.z,
                    S9.x,S9.y,S9.z,
                    S10.x,S10.y,S10.z,
                    S11.x,S11.y,S11.z))
    f.close()




def main():
	global str_filename

	rospy.init_node('smartsurg_savedata', anonymous=True)
	rospy.loginfo("---WELCOME TO Save to file---")
	rospy.Subscriber("/imu_pub_array", PoseArray, imu_serial_callback, queue_size=10)



	f = open(str_filename, "a+")
	f.write('seconds,SC.x,'
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
                    'S11.x,S11.y,S11.z,\r\n')

	# print('seconds,robot_x,robot_y,robot_z,landmark_count,source_az,source_el,source_sim_az,source_sim_el,covar_mat1_det,covar_mat1l_det,covar_mat2_det,covar_mat2l_det,ss_pos_x,ss_pos_y,ss_pos_z,\r\n')
	f.close()

	# save_to_csv(f)

	rospy.spin()

if __name__ == '__main__':
	main()
