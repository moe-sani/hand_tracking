#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from odas_msgs.msg import Odas, OdasList
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
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

def transform_to_cf(quat,target_frame):
    global listener
    quat_st_wf = QuaternionStamped()
    quat_st_wf.header.frame_id = '/world'
    quat_st_wf.quaternion = quat

    quat_st_cf = listener.transformQuaternion(target_frame, quat_st_wf)
    return quat_st_cf.quaternion

def append_new_frame(translation,rotation,frame_id,frame_target):
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

def ndi_box_callback(data):
    box_pose=data.pose
    print(box_pose)
    append_new_frame((box_pose.position.x, box_pose.position.y, box_pose.position.z),
                     (box_pose.orientation.x, box_pose.orientation.y, box_pose.orientation.z, box_pose.orientation.w),
                     '/world', '/box')

def main():
    global listener
    rospy.init_node('NDI_tf_publisher', anonymous=True)
    rospy.Subscriber("/ndi/Box/position_cartesian_current", PoseStamped, ndi_box_callback, queue_size=10)
    listener = tf.TransformListener()
    # rate = rospy.Rate(100)  # 10hz
    # while not rospy.is_shutdown():
    #     publish_to_davinci()
    #     rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()

