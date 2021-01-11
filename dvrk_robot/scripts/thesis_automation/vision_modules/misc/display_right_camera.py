#!/usr/bin/env python

#Ros libraries
import rospy
from cv_bridge import CvBridge, CvBridgeError
import dvrk

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
import socket
import os
from os.path import join, exists


class CameraSubscriber:

	def __init__(self,):
		rospy.init_node('camera_display', anonymous=True)
		self.bridge = CvBridge()
		self.right_frame = None
		self.camera_subs = rospy.Subscriber("/pu_dvrk_cam/modified_display_right", Image, self.camera_callback)		

		self.wind_name = "img"
		cv2.namedWindow(self.wind_name)
		cv2.setMouseCallback(self.wind_name,self.get_coordinates)
		
	
	def camera_callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			self.right_frame = cv_image
		except CvBridgeError as e:
			print(e)
	
		cv2.imshow(self.wind_name, cv_image)
		cv2.waitKey(1)

	def get_coordinates(self, event,x,y,flags,param):
		if event == cv2.EVENT_LBUTTONDOWN:
		    print(x,y)

def main():
	cam = CameraSubscriber()
	#Sleep until the subscribers are ready.
	time.sleep(0.40)


	try:
		while not rospy.core.is_shutdown():
			rospy.rostime.wallsleep(0.25)

	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()


if __name__ == "__main__":
	main()