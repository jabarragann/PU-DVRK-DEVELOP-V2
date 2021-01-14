#!/usr/bin/env python

#Ros libraries
import rospy
from cv_bridge import CvBridge, CvBridgeError
import dvrk

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


class PSM3Arm:

	class ArmKinematic:
		def __init__(self, arm_name):
			self.arm_name = arm_name
			self.x = 0
			self.y = 0
			self.z = 0
			self.rx = 0
			self.ry = 0
			self.rz = 0
			self.rw = 0

		def set_pose(self, data):
			self.x = data.pose.position.x
			self.y = data.pose.position.y
			self.z = data.pose.position.z

			self.rx = data.pose.orientation.x
			self.ry = data.pose.orientation.y
			self.rz = data.pose.orientation.z
			self.rw = data.pose.orientation.w

		def create_str_repr(self,):
			#Position: x,y,z Rotation: rx,ry,rz,rw
			message1 = "Position (x,y,z): {: 0.8f}, {: 0.8f}, {: 0.8f} Orientation(rx,ry,rz,rw): {: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}, "\
						.format(self.x, self.y,self.z,self.rx,self.ry,self.rz,self.rw)
	
			return message1 


	def __init__(self,):

		self.arm_kinematic = self.ArmKinematic('PSM3')
		#python api
		self.api_psm3_arm = dvrk.arm('PSM3')
		

		#subscribers
		self.psm3_cartesian_subs = rospy.Subscriber("/dvrk/PSM3/position_cartesian_current", PoseStamped, self.psm3_cartesian_callback)
		self.psm1_suj_subs = rospy.Subscriber("/dvrk/PSM1/io/suj_clutch", Joy, self.setup_button_psm1_callback)

		self.tf_world_psm3b_pub = rospy.Publisher("/pu_dvrk_tf/tf_world_psm3b", PoseStamped, queue_size=5)

		#publisher
		self.multi_arr_pub = rospy.Publisher("/pu_dvrk_tf/centroid_coordinates",Int32MultiArray,queue_size=5)

		#subs
		# self.multi_arr_pub = rospy.Publisher("/pu_dvrk_tf/centroid_coordinates",UInt8MultiArray,self.centroids_callback)
		
	###########
	#Callbacks#
	###########
	def psm3_cartesian_callback(self, data):
		self.arm_kinematic.set_pose(data)

	def setup_button_psm1_callback(self,data):
		if data.buttons[0]:
			#Get Python API pose
			pose = self.api_psm3_arm.get_current_position()

			print("Ros topic pose")
			print(self.arm_kinematic.create_str_repr())
			print("Python api pose")
			print(self.pykdl_frame2str(pose))

	##########
	#Other ###
	##########
	def pykdl_frame2str(self,frame):
		(rx, ry, rz, rw) = frame.M.GetQuaternion()

		
		message1 = "Position (x,y,z): {: 0.8f}, {: 0.8f}, {: 0.8f}  Orientation(rx,ry,rz,rw): {: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}"\
								.format(frame.p.x(), frame.p.y(), frame.p.z(), rx,ry,rz,rw)
		return message1
	def print_current_position(self,):
		print(self.arm_kinematic.create_str_repr())

	def home(self,):
		self.api_psm3_arm.home()



def main():
	psm3 = PSM3Arm()

	#Sleep until the subscribers are ready.
	time.sleep(0.20)

	psm3.home()

	#multi array test
	arr = np.array([[40,50], [580,5]]).astype(np.int32)
	print(arr)
	arr = arr.reshape(-1).tolist()
	arr_to_send = Int32MultiArray()
	arr_to_send.data = arr

	try:
		while not rospy.core.is_shutdown():
			psm3.multi_arr_pub.publish(arr_to_send)
			rospy.rostime.wallsleep(2)
			
	except KeyboardInterrupt:
		print("Shutting down")


if __name__ == "__main__":
	main()