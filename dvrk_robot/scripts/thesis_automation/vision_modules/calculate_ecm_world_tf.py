#!/usr/bin/env python
import json
import numpy as np 
#Ros libraries
import rospy
from cv_bridge import CvBridge, CvBridgeError
import tf_conversions.posemath as pm
import PyKDL
#Ros messages
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Joy
from std_msgs.msg import String as rosString
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
#Python libraries
import cv2 
import time
import datetime
import numpy as np 
import sys
import random 
import argparse
import json
import os
from os.path import join, exists
#Custom class
from dvrk_tf_module import dvrk_tf_module


def main():
	rospy.init_node('recording_node')
	time.sleep(0.2)
	

	dst_path = "/home/isat/juanantonio/davinci_catkin_ws_1.7/src/dvrk-ros/dvrk_robot/scripts/thesis_automation/vision_modules/test_videos"
	cm = dvrk_tf_module(userId = "test", dst_path = dst_path, rig_name="pu_dvrk_cam")

	#Sleep until the subscribers are ready.
	time.sleep(0.2)

	ans = raw_input("Would you like to recalculate the transformation between the ECM and the wordl? (y/n) ")
	if ans == 'y':
		if not cm.get_cam_world_transform():
			print("Chessboard was not detected")
			exit(0)

	try:
		while not rospy.core.is_shutdown():
			rospy.rostime.wallsleep(0.25)

	except KeyboardInterrupt:
		print("Shutting down")

	finally:
		print("exit")


if __name__ == "__main__":
	main()

