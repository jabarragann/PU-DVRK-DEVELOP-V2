#!/usr/bin/env python

import sys
sys.path.append("/home/isat/juanantonio/davinci_catkin_ws_1.7/src/dvrk-ros/dvrk_robot/scripts/thesis_automation/vision_modules/misc")
sys.path.append("/home/isat/juanantonio/davinci_catkin_ws_1.7/src/dvrk-ros/dvrk_robot/scripts/thesis_automation/vision_modules/")

#My modules
from dvrk_tf_module import dvrk_tf_module
from fcn import VGGNet, FCNs
import segmentation_utils as utils
# from recording_module import createTimeStamp, recording_module
# from socket_client import socket_client

#Ros libraries
import rospy
from cv_bridge import CvBridge, CvBridgeError
import dvrk
import tf_conversions.posemath as pm
import PyKDL 
from PyKDL import Vector, Rotation

#Ros messages
from std_msgs.msg import Int32, Int32MultiArray, Bool
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


class AutonomyModule:

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


	def __init__(self, activate_ar=False, activate_debug_visuals=False):

		#python api -- This code initializes the ros node
		self.api_psm3_arm = dvrk.arm('PSM3')
		
		#visual controller
		self.dvrk_controller = dvrk_tf_module(userId = "test", dst_path = './videos', rig_name="pu_dvrk_cam",
												 activate_ar=activate_ar, activate_debug_visuals=activate_debug_visuals)

		#Ar config
		self.ar_orientation  = PyKDL.Rotation.Quaternion(0.68175651, -0.27654879,  0.56096521,  0.37953506)
		self.dvrk_controller.ar_orientation = self.ar_orientation
		

		#
		self.arm_kinematic = self.ArmKinematic('PSM3')
		
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
		self.fix_orientation = PyKDL.Rotation.Quaternion(0.35131810, -0.12377625,  0.53801977,  0.75616781)

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

	##########
	## AR  ###
	##########
	def set_activate_ar(self, new_pos):
		self.dvrk_controller.ar_pose = new_pos
		self.dvrk_controller.activate_ar = True	

	def get_position_homography(self, pixel, height):
		'''
		Transforms the pixels obtained from computer vision into coordinates
		in the x,y plane of the world coordinate system.
		'''

		pixel_h = np.ones((3,1))
		pixel_h[:2,:] = np.array(pixel).reshape((2,1))
		coord = np.dot(self.homography,pixel_h).T
		coord = coord / coord[:,2]
	
		new_position = PyKDL.Vector(coord[0,0], coord[0,1],height)

		return new_position

	def generate_trajectory(self,init_pt, end_pt, n=20):
		init_pt = init_pt.reshape(3,1)
		end_pt = end_pt.reshape(3,1)
		t = np.linspace(0, 1, n).reshape(1,n)
		pt_on_line = (end_pt - init_pt) * t + init_pt
		return pt_on_line

	def ar_animation(self, init_pt, end_pt, n=40, animation_sleep=0.040):
		timeout = 14
		init_time = time.time()
		
		init_pt = self.tf_world_psm3b.Inverse() * init_pt
		end_pt = self.tf_world_psm3b.Inverse() * end_pt

		trajectory = self.generate_trajectory(self.vect2np(init_pt), self.vect2np(end_pt),n=n)
		
		for i in range(n):

			new_position = Vector(*list(trajectory[:,i]))
			self.set_activate_ar(PyKDL.Frame(self.fix_orientation, new_position))
			time.sleep(animation_sleep)

			if time.time() - init_time > timeout:
				break

		print("Ar time", self.dvrk_controller.activate_ar)

	def vect2np(self, vect):
		return np.array([vect.x(),vect.y(), vect.z()])

def create_model():
	best_params_path =  "./vision_modules/misc/model/model_v2.pickle"
	model = utils.create_model(best_params_path)
	labels_path = "./vision_modules/misc/model/label_colors.json"
	labels_dict = utils.labels_colors_dicts(labels_path)

	return model, labels_dict

def calculate_centroids(mask,pub,centroid_module):
	centroids = centroid_module.calculate_centroids(mask)
	# print(centroids)
    if centroids.size > 0:
        #use only the biggest centroid
        centroids = centroids[0,:].reshape((1,2))
        arr_to_send = Int32MultiArray()
        arr_to_send.data = centroids.reshape(-1).tolist()
        pub.publish(arr_to_send)

	return centroids 



class MainModule:

	def __init__(self, args):

		self.args = args 
		self.psm3 = AutonomyModule(activate_ar=args.activate_ar, activate_debug_visuals=args.activate_debug_visuals)
		self.model, self.labels_dict = create_model()
		self.centroid_module = utils.centroid_module()

		#Sleep until the subscribers are ready.
		time.sleep(0.20)
		self.sleep_time = 1.5

		#Fix landmarks
		self.base_position = Vector(*[+0.04527744,-0.02624656,-0.02356771]) 
		self.height_low = +0.07217540
		self.height_medium = +0.06053191
		self.height_high = self.height_low - 0.05

		#Flags 
		self.autonomy_on = False

		#############
        #Subscribers#
        #############
		self.autonomy_flag_subs = rospy.Subscriber("/recording_module/autonomy_active", Bool, self.autonomy_callback)

		##We are missing a suscriber to identify when the user wants to directly control the dvrk arm
		##This is the final piece of the puzzle!
		
	def autonomy_callback(self,data):
		print("autonomy received")
		print(data)
		self.autonomy_on = data.data

	def run(self):
		#Make sure everything is ready to go
		answer = raw_input("Did set velocity? have you check if the homography is up to date? if yes write 'Y' to continue, else don't move the robot. ")
		if answer != 'Y':
			print("Exiting the program")
			exit(0)
		else:
			time.sleep(2)
			print("Move to base position")
			self.psm3.move_psm3_to(self.base_position)
			time.sleep(1)

			try:
				while not rospy.core.is_shutdown():
					if self.autonomy_on:
						#Autonomy loop
						#Calculate centroids of pools of blood
						final_frame, mask = utils.test_img(self.psm3.right_frame,self.model, self.labels_dict)
						centroids_coord = calculate_centroids(mask, self.psm3.multi_arr_pub,self.centroid_module)
						print("centroids", centroids_coord)

						#Move to centroids
						if centroids_coord.size > 0:
							print("Move to base position")
							self.psm3.move_psm3_to(self.base_position)
							time.sleep(0.2)
								
							pix = centroids_coord[0]
							print(pix)

							#AR animations
							if self.args.activate_ar:
								self.psm3.ar_animation(self.base_position, self.psm3.get_position_homography(pix,self.height_medium))
							time.sleep(1.0)
					
							#Moving routine
							self.move_routine(pix)
						else:
							print("No pools of blood detected")

					rospy.rostime.wallsleep(0.25)

			# try:
				while not rospy.core.is_shutdown():
					rospy.rostime.wallsleep(0.25)

			except KeyboardInterrupt:
				psm3.move_psm3_to(base_position)
				print("Shutting down")

	def move_routine(self, pix):
		# self.psm3.move_using_homography(pix,self.height_high)
		# self.deactivate_ar()
		# self.psm3.move_using_homography(pix,self.height_medium); 
		# time.sleep(self.sleep_time-0.3);
		# self.psm3.move_using_homography(pix,self.height_low);
		# time.sleep(self.sleep_time+0.3);
		# self.psm3.move_using_homography(pix,self.height_medium); 
		# time.sleep(self.sleep_time-0.3);
		# self.psm3.move_using_homography(pix,self.height_low);
		# time.sleep(self.sleep_time+0.3);
		# self.psm3.move_using_homography(pix,self.height_high)
		# self.psm3.move_psm3_to(self.base_position)
		"""
		move sequence contains a list of functions an its parameters that form the complete movement of the arm.
		This was representation was chosen to be able to stop the arm at any part of the sequence
		"""
		move_sequence = [[self.psm3.move_using_homography,(pix,self.height_high)],
						 [self.deactivate_ar,()],
						 [self.psm3.move_using_homography,(pix,self.height_medium)], 
						 [time.sleep,(self.sleep_time-0.6,)],
						 [self.psm3.move_using_homography,(pix,self.height_low)],
						 [time.sleep,(self.sleep_time+0.6,)],
						 [self.psm3.move_using_homography,(pix,self.height_medium)],
						 [time.sleep,(self.sleep_time-0.6,)],
						 [self.psm3.move_using_homography,(pix,self.height_low)],
						 [time.sleep,(self.sleep_time+0.6,)],
						 [self.psm3.move_using_homography,(pix,self.height_high)]]

		count = 0
		print("mov rutine")
		for i in range(len(move_sequence)):
			print("mov ", count)
			count += 1
			func, attributes = move_sequence[i]
			if self.autonomy_on:
				func(*attributes) 	#do the command
			else:
				break

		self.psm3.move_psm3_to(self.base_position)

	# def move_and_sleep(self,pix,height,s_time):
	# 	self.psm3.move_using_homography(pix,self.height_medium); 
	# 	time.sleep(self.sleep_time-0.3);

	def deactivate_ar(self,):
		self.psm3.dvrk_controller.activate_ar = False

def script_config_arg():
	import argparse

	parser = argparse.ArgumentParser()

	##Visualization arguments -- Control what is display in the screens of the console
	parser.add_argument('--activate_ar', action='store_true', default=False, help='activate AR visualization (default: disabled)')
	parser.add_argument('-d', '--activate_debug_visuals', action='store_true', default=False, help='display valid boundary, and saved chessboard corners. (default: disabled)')
	
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


if __name__ == "__main__":
	args = None
	args =  script_config_arg()
	controller = MainModule(args)
	controller.run()
