#!/usr/bin/env python

import sys
sys.path.append("/home/isat/juanantonio/davinci_catkin_ws_1.7/src/dvrk-ros/dvrk_robot/scripts/thesis_automation/vision_modules/misc")
from fcn import VGGNet, FCNs
import segmentation_utils as utils

#Ros libraries
import rospy
from cv_bridge import CvBridge, CvBridgeError
import dvrk
import tf_conversions.posemath as pm
import PyKDL 
from PyKDL import Vector, Rotation

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
import json

import argparse


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

		#frames
		self.bridge = CvBridge()
		self.right_frame = None

		###############
		##subscribers##
		###############
		self.psm3_cartesian_subs = rospy.Subscriber("/dvrk/PSM3/position_cartesian_current", PoseStamped, self.psm3_cartesian_callback)
		self.psm1_suj_subs = rospy.Subscriber("/dvrk/PSM1/io/suj_clutch", Joy, self.setup_button_psm1_callback)
		self.camera_subs = rospy.Subscriber("/pu_dvrk_cam/right/inverted", Image, self.camera_callback)	
		#tf 
		self.tf_world_psm3b_subs = rospy.Subscriber("/pu_dvrk_tf/tf_world_psm3b", PoseStamped, self.tf_world_psm3b_callback)
		
		##############
		##Publishers##
		##############
		self.multi_arr_pub = rospy.Publisher("/pu_dvrk_tf/centroid_coordinates",Int32MultiArray,queue_size=5)

		#psm offset
		self.fix_orientation = PyKDL.Rotation.Quaternion(0.20611444, -0.10502740,  0.60974223,  0.75809003)

		#Upload homography between pixels and robot x,y
		self.homography = None
		with open("./vision_modules/misc/homography_data/homography.json","r") as f1:
			self.homography = np.array(json.load(f1))

		print("Retrieved homography")
		print(self.homography)

	def move_using_homography(self, pixel, height):
		'''
		Transforms the pixels obtained from computer vision into coordinates
		in the x,y plane of the world coordinate system.
		'''

		pixel_h = np.ones((3,1))
		pixel_h[:2,:] = np.array(pixel).reshape((2,1))
		coord = np.dot(self.homography,pixel_h).T
		coord = coord / coord[:,2]
	
		new_position = PyKDL.Vector(coord[0,0], coord[0,1],height)

		print(new_position)

		self.move_psm3_to(new_position)

	def move_psm3_to(self,new_position):
		'''
		new position vector should be given with respect to world coordinates!
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

		if not rospy.core.is_shutdown():
			self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, new_position))

	###########
	#Callbacks#
	###########
	def camera_callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			self.right_frame = cv_image
		except CvBridgeError as e:
			print(e)

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


def create_model():
	best_params_path =  "./vision_modules/misc/model/model.pickle"
	model = utils.create_model(best_params_path)
	labels_path = "./vision_modules/misc/model/label_colors.json"
	labels_dict = utils.labels_colors_dicts(labels_path)

	return model, labels_dict

def calculate_centroids(mask,pub):
	centroids = utils.calculate_centroids(mask)
	# print(centroids)
	#use only the biggest centroid
	centroids = centroids[0,:].reshape((1,2))
	arr_to_send = Int32MultiArray()
	arr_to_send.data = centroids.reshape(-1).tolist()
	pub.publish(arr_to_send)

	return centroids 

def main(args):

	psm3 = PSM3Arm()
	model, labels_dict = create_model()

	#Sleep until the subscribers are ready.
	time.sleep(0.20)
	sleep_time = 0.8

	#Centroid coordinates
 	# centroids = np.array([[168,228]]).astype(np.int32)
	
	#Fix landmarks
	base_position = Vector(*[+0.02774402,-0.02132568,+0.03622959]) 
	height_low = +0.086
	height_medium = +0.0687
	height_high = height_low - 0.05

	answer = raw_input("Did set velocity? have you check if the homography is up to date? if yes write 'Y' to continue, else don't move the robot.")
	if answer != 'Y':
		print("Exiting the program")
		exit(0)
	else:
		time.sleep(2)
		print("Move to base position")
		psm3.move_psm3_to(base_position)
		time.sleep(1)

		print("Move arm 5 times")
		try:
			while not rospy.core.is_shutdown():
			# for _ in range(1):

				#Calculate centroids
				final_frame, mask = utils.test_img(psm3.right_frame,model, labels_dict)
				centroids_coord = calculate_centroids(mask, psm3.multi_arr_pub)
				#Ask if you want to move to centroid
				print("centroids", centroids_coord)

				if centroids_coord.size > 0:
					answer = 'Y' # raw_input("Do you want to move?")
					if answer != 'Y':
						print("Exiting the program")
						exit(0)
					else:
						print("Move to base position")
						psm3.move_psm3_to(base_position)
						time.sleep(2)
				
						# for i in range(centroids_coord.shape[0]):
						# 	print("Movement {:d}".format(i))
							
						pix = centroids_coord[0]
						print(pix)
						
						psm3.move_using_homography(pix,height_high)

						psm3.move_using_homography(pix,height_medium); time.sleep(sleep_time);
						psm3.move_using_homography(pix,height_low);time.sleep(sleep_time);
						psm3.move_using_homography(pix,height_medium); time.sleep(sleep_time);
						psm3.move_using_homography(pix,height_low);time.sleep(sleep_time);
						psm3.move_using_homography(pix,height_medium); time.sleep(sleep_time);
						psm3.move_using_homography(pix,height_low);time.sleep(sleep_time);
						#time.sleep(4.5)
						psm3.move_using_homography(pix,height_high)

						psm3.move_psm3_to(base_position)
						# time.sleep(sleep_time)
				else:
					print("No pools of blood detected")

		# try:
			while not rospy.core.is_shutdown():
				rospy.rostime.wallsleep(0.25)

		except KeyboardInterrupt:
			psm3.move_psm3_to(base_position)
			print("Shutting down")


def script_config_arg():
	import argparse

	parser = argparse.ArgumentParser()

	##Visualization arguments -- Control what is display in the screens of the console
	parser.add_argument('--ar', action='store_true', default=False, help='activate AR visualization (default: disabled)')
	parser.add_argument('-d', '--debug-visualisation', action='store_true', default=False, help='display valid boundary, and saved chessboard corners. (default: disabled)')

	##

	# parser.add_argument('--foo', action='store_const', const=True, default=False,
	# 					metavar='FOO!')
	# parser.add_argument('--foo2', metavar='YYY')
	# parser.add_argument('--foo3', action='store_true', default=False, help='activate foo power')
	# # parser.add_argument('--bee', const=True, default=False)
	# parser.add_argument('bar', nargs=1)
	# parser.add_argument('bar2', nargs=1)

	args = parser.parse_args()
	return args

args = None
if __name__ == "__main__":
	args =  script_config_arg()
	main(args)