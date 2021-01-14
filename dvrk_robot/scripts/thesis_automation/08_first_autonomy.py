#!/usr/bin/env python

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
import json


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
		
		#tf 
		self.tf_world_psm3b = None

		#subscribers
		self.psm3_cartesian_subs = rospy.Subscriber("/dvrk/PSM3/position_cartesian_current", PoseStamped, self.psm3_cartesian_callback)
		self.psm1_suj_subs = rospy.Subscriber("/dvrk/PSM1/io/suj_clutch", Joy, self.setup_button_psm1_callback)

		#tf 
		self.tf_world_psm3b_subs = rospy.Subscriber("/pu_dvrk_tf/tf_world_psm3b", PoseStamped, self.tf_world_psm3b_callback)
		
		#psm offset
		self.fix_orientation = PyKDL.Rotation.Quaternion(0.20611444, -0.10502740,  0.60974223,  0.75809003)

		#Upload homography between pixels and robot x,y
		self.homography = None
		with open("./vision_modules/misc/homography_data/homography.json","r") as f1:
			self.homography = np.array(json.load(f1))

		print("Retrieved homography")
		print(self.homography)

	def move_using_homography(self, pixel, height):
		pixel_h = np.ones((3,1))
		pixel_h[:2,:] = np.array(pixel).reshape((2,1))
		coord = np.dot(self.homography,pixel_h).T
		coord = coord / coord[:,2]
	
		new_position = PyKDL.Vector(coord[0,0], coord[0,1],height)

		print(new_position)

		self.move_psm3_to(new_position)

	def move_psm3_to(self,new_position):
		'''
		new position vector is given with respect to world coordinates
		'''
		if self.tf_world_psm3b is None:
			print("no transformation to move the robot")
			exit(0)

		#add the offset 
		new_position = new_position 
		#Transform to psmb
		new_position = self.tf_world_psm3b.Inverse() * new_position
		
		# print("moving to new location:")
		# self.print_vect(new_position)

		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, new_position))

	###########
	#Callbacks#
	###########
	def tf_world_psm3b_callback(self,data):
		self.tf_world_psm3b = pm.fromMsg(data.pose)

	def psm3_cartesian_callback(self, data):
		self.arm_kinematic.set_pose(data)

	def setup_button_psm1_callback(self,data):
		if data.buttons[0]:
			#Get Python API pose
			pose_in_psm3b = self.api_psm3_arm.get_current_position()
			pose_in_world = self.tf_world_psm3b * pose_in_psm3b.p
			offset = PyKDL.Vector(-0.007,-0.026,0.082)
			scale_fact = 100/1.065
			normalized = (pose_in_world - offset)

			print("Python api pose")
			self.print_vect(pose_in_psm3b.p)
			print("psm3 origin in world coordinates")
			self.print_vect(pose_in_world)
			print("normalized to chessboard")
			self.print_vect(normalized)
			print("\n")
			# print(self.tf_world_psm3b.Inverse()*pose_in_world)

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
	def print_vect(self, vect):
		str_rep = "position ({:+0.3f},{:+0.3f},{:+0.3f})".format(vect.x(), vect.y(),vect.z())
		print(str_rep)



def main():
	psm3 = PSM3Arm()

	#Sleep until the subscribers are ready.
	time.sleep(0.20)
	sleep_time = 0.8

	#Centroid coordinates
	centroids = np.array([[379,154],[168,228], [222,421], [409, 351]]).astype(np.int32)
	# centroids = np.array([[379,154],[168,228]]).astype(np.int32)
	
 
	base_position = Vector(*[+0.033,-0.012,+0.010]) ##Fill with the appropriate value

	answer = raw_input("have you check if the homography is up to date? if yes write 'Y' to continue, else don't move the robot.")
	if answer != 'Y':
		print("Exiting the program")
		exit(0)
	else:
		print("Move to base position")
		psm3.move_psm3_to(base_position)
		time.sleep(7)

		print("Start moving arm 5 times")
		for _ in range(1):
		
			for i in range(centroids.shape[0]):
				print("Movement {:d}".format(i))
				
				pix = centroids[i]
				print(pix)
				height_low = 0.090
				height_high = height_low - 0.05
				psm3.move_using_homography(pix,height_high)
				psm3.move_using_homography(pix,height_low)
				time.sleep(4.5)
				psm3.move_using_homography(pix,height_high - 0.025)

			psm3.move_psm3_to(base_position)
			time.sleep(sleep_time)
	try:
		while not rospy.core.is_shutdown():
			rospy.rostime.wallsleep(0.25)

	except KeyboardInterrupt:
		print("Shutting down")


if __name__ == "__main__":
	main()