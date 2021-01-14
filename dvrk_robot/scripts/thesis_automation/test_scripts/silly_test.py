#!/usr/bin/env python

import dvrk

#Ros libraries
import rospy
from cv_bridge import CvBridge, CvBridgeError
import dvrk
import tf_conversions.posemath as pm
import PyKDL 
from PyKDL import Vector, Rotation

#Ros messages
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Joy
from std_msgs.msg import String as rosString
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

#Python libraries
import torch
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


def pykdl_frame2str(frame):
	(rx, ry, rz, rw) = frame.M.GetQuaternion()

	
	message1 = "Position (x,y,z): {: 0.8f}, {: 0.8f}, {: 0.8f}  Orientation(rx,ry,rz,rw): {: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}"\
							.format(frame.p.x(), frame.p.y(), frame.p.z(), rx,ry,rz,rw)
	return message1

class silly_class:

	def __init__(self,):

		self.api_arm = dvrk.arm('MTML')
		
		#tf 
		self.tf_mtmlb_mtml = None
		#subscribers
		self.tf_subs = rospy.Subscriber("/dvrk/MTML/position_cartesian_current", PoseStamped, self.tf_callback)

	###########
	#Callbacks#
	###########
	def tf_callback(self,data):
		self.tf_mtmlb_mtml = pm.fromMsg(data.pose)
		self.tf_2 = self.api_arm.get_current_position()
		


def main():
	print("Test pytorch")
	x = torch.rand(5, 3)
	print(x)

	silly_test = silly_class()
	# rospy.init_node('silly_node', anonymous=True)
	#Sleep until the subscribers are ready.
	time.sleep(0.40)

	try:
		while not rospy.core.is_shutdown():
			rospy.rostime.wallsleep(2)
			print(pykdl_frame2str(silly_test.tf_mtmlb_mtml))
			print(pykdl_frame2str(silly_test.tf_2))

	except KeyboardInterrupt:
		print("Shutting down")


if __name__ == "__main__":
	main()