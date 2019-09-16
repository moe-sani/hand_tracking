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



def tool_callback(tool_pose):
    global listener
    rospy.loginfo("converting ...")
    print("tool_pose:",tool_pose)
    tool_cf = listener.transformPose('Wrist', tool_pose)
    print('tool_cf',tool_cf)
    pub=rospy.Publisher('/tool_wf', PoseStamped, queue_size=1)
    pub.publish(tool_cf)



def main():
    global str_filename
    global listener
    rospy.init_node('transform_to_wrist', anonymous=True)
    rospy.loginfo("---WELCOME TO transform---")

    rospy.Subscriber('/ndi/Tool/position_cartesian_current', PoseStamped,tool_callback, queue_size=10)
    listener = tf.TransformListener()
    rospy.spin()

if __name__ == '__main__':
    main()

