#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64

pub = rospy.Publisher('topic_name', String, queue_size=10)
rospy.init_node('acc_vel_ratio')
r = rospy.Rate(0.1) # 10hz


ratio = 0.20
velocity_ratio_pub = rospy.Publisher("/dvrk/PSM3/set_joint_velocity_ratio",Float64,latch=True, queue_size=1)
acceleration_ratio_pub = rospy.Publisher("/dvrk/PSM3/set_joint_acceleration_ratio",Float64,latch=True, queue_size=1)	


velocity_ratio_pub.publish(Float64(ratio))
acceleration_ratio_pub.publish(Float64(ratio)) 
while not rospy.is_shutdown():
   # pub.publish("hello world")
   r.sleep()