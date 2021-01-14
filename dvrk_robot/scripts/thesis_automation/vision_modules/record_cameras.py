#!/usr/bin/env python
import json
import numpy as np 
#Ros libraries
import rospy
from cv_bridge import CvBridge, CvBridgeError
import tf_conversions.posemath as pm
import PyKDL
#Ros messages
from std_msgs.msg import Int32, Int32MultiArray
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
import socket
import os
from os.path import join, exists


class recording_module:


	def __init__(self, dst_path = None):
		rig_name = "pu_dvrk_cam"
		self.dst_path = dst_path

		#Video files
		self.bridge = CvBridge()
		fourcc = cv2.VideoWriter_fourcc(*'XVID')
		self.out_right = cv2.VideoWriter(join(self.dst_path,"video_right_color.avi"),fourcc, 30.0, (640,480))
		self.out_right_aug = cv2.VideoWriter(join(self.dst_path,"video_right_aug_color.avi"),fourcc, 30.0, (640,480))
		
		#############
		#Subscribers#
		#############

		##Video
		self.image_sub_right  = rospy.Subscriber("/"+rig_name+"/right/inverted", Image, self.right_callback)
		self.image_sub_right_aug = rospy.Subscriber("/"+rig_name+"/modified_display_right", Image, self.right_aug_callback)

	###################
	#IMAGE CALLBACKS ##
	###################
	def right_callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			self.left_frame = cv_image
		except CvBridgeError as e:
			print(e)

		self.out_right.write(cv_image)

	def right_aug_callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			self.right_frame = cv_image
		except CvBridgeError as e:
			print(e)

		self.out_right_aug.write(cv_image)

		
	def close_files(self):
		pass


def main():
	
	rospy.init_node('recording_cameras')

	cm = recording_module(dst_path = "./test_videos")

	#Sleep until the subscribers are ready.
	time.sleep(0.20)

	try:
		while not rospy.core.is_shutdown():
			rospy.rostime.wallsleep(0.25)
	except KeyboardInterrupt:
		print("Shutting down")
	finally:
		cm.close_files()
		print("Shutting down")


if __name__ == '__main__':
	main()