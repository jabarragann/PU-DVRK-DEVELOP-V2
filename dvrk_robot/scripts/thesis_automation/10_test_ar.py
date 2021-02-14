#!/usr/bin/env python

import sys
sys.path.append("/home/isat/juanantonio/davinci_catkin_ws_1.7/src/dvrk-ros/dvrk_robot/scripts/thesis_automation/vision_modules/")

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

from dvrk_tf_module import dvrk_tf_module

class PSM3Arm:

	def __init__(self,):
		#python api
		self.api_psm3_arm = dvrk.arm('PSM3')
		self.dvrk_controller = dvrk_tf_module(userId = "test", dst_path = './videos', rig_name="pu_dvrk_cam")

		#tf 
		self.tf_world_psm3b = PyKDL.Frame()

		#subscribers
		self.psm1_suj_subs = rospy.Subscriber("/dvrk/PSM1/io/suj_clutch", Joy, self.setup_button_psm1_callback)

		#tf 
		self.tf_world_psm3b_subs = rospy.Subscriber("/pu_dvrk_tf/tf_world_psm3b", PoseStamped, self.tf_world_psm3b_callback)
		
		self.ar_orientation  = PyKDL.Rotation.Quaternion(0.60939205, -0.44821878,  0.41774239,  0.50322217)
		self.dvrk_controller.ar_orientation = self.ar_orientation

		self.fix_orientation = PyKDL.Rotation.Quaternion(0.20611444, -0.10502740,  0.60974223,  0.75809003)
		self.base = Vector(*[ 0.19825322, -0.12229110,  0.05486537]) 
		self.position1 = Vector(*[0.21956419, -0.14727539,  0.11189935]) 
		self.position2 = Vector(*[0.23203641, -0.18565913,  0.06422155]) 
		self.position3 = Vector(*[0.17023624, -0.19693916,  0.06869363]) 
		self.position4 = Vector(*[0.16812289, -0.14461934,  0.11209671]) 

	###########
	#Callbacks#
	###########
	def tf_world_psm3b_callback(self,data):
		self.tf_world_psm3b = pm.fromMsg(data.pose)

	def setup_button_psm1_callback(self,data):
		if data.buttons[0]:

			# if self.position1 is None:
			# 	self.position1 = self.api_psm3_arm.get_current_position().p
			# elif self.position2 is None:
			# 	self.position2 = self.api_psm3_arm.get_current_position().p
			# elif self.position3 is None:
			# 	pass
			# elif self.position4 is None:
			# 	pass

			#Get Python API pose
			pose_in_psm3b = self.api_psm3_arm.get_current_position()
			pose_in_world = self.tf_world_psm3b * pose_in_psm3b.p
			offset = PyKDL.Vector(-0.007,-0.026,0.082)
			scale_fact = 100/1.065
			normalized = (pose_in_world - offset)

			# print('Print position 1 ')
			# print(self.pykdl_frame2str(self.position1))
			# print('Print position 2 ')
			# print(self.pykdl_frame2str(self.position2))

			print("Python api pose")
			print(self.pykdl_frame2str(pose_in_psm3b))
			# self.print_vect(pose_in_psm3b.p)
			print("psm3 origin in world coordinates")
			self.print_vect(pose_in_world)
			print("normalized to chessboard")
			self.print_vect(normalized)
			print("\n")
			# print(self.tf_world_psm3b.Inverse()*pose_in_world)

			#Turn on AR
			# self.dvrk_controller.ar_pose = self.position1
			# self.dvrk_controller.activate_ar = not self.dvrk_controller.activate_ar

	def move_rutine(self,):
		sleep_time = 2.5 

		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.base)) 
		self.set_activate_ar(PyKDL.Frame(self.fix_orientation, self.position1))
		time.sleep(sleep_time)
		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.position1))
		time.sleep(sleep_time)
		self.dvrk_controller.activate_ar = False

		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.base))
		self.set_activate_ar(PyKDL.Frame(self.fix_orientation, self.position2)) 
		time.sleep(sleep_time)
		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.position2)) 
		time.sleep(sleep_time)
		self.dvrk_controller.activate_ar = False

		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.base)) 
		self.set_activate_ar(PyKDL.Frame(self.fix_orientation, self.position3))
		time.sleep(sleep_time)
		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.position3)) 
		time.sleep(sleep_time)
		self.dvrk_controller.activate_ar = False

		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.base)) 
		self.set_activate_ar(PyKDL.Frame(self.fix_orientation, self.position4))
		time.sleep(sleep_time)
		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.position4)) 
		time.sleep(sleep_time)
		self.dvrk_controller.activate_ar = False

	def move_rutine_with_animation(self,):
		sleep_time = 1.0

		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.base)) 
		self.ar_animation(self.vect2np(self.base), self.vect2np(self.position1))
		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.position1))
		time.sleep(sleep_time)
		self.dvrk_controller.activate_ar = False

		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.base)) 
		self.ar_animation(self.vect2np(self.base), self.vect2np(self.position2))
		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.position2))
		time.sleep(sleep_time)
		self.dvrk_controller.activate_ar = False

		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.base)) 
		self.ar_animation(self.vect2np(self.base), self.vect2np(self.position3))
		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.position3))
		time.sleep(sleep_time)
		self.dvrk_controller.activate_ar = False

		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.base)) 
		self.ar_animation(self.vect2np(self.base), self.vect2np(self.position4))
		self.api_psm3_arm.move(PyKDL.Frame(self.fix_orientation, self.position4))
		time.sleep(sleep_time)
		self.dvrk_controller.activate_ar = False




	def set_activate_ar(self, new_pos):
		self.dvrk_controller.ar_pose = new_pos
		self.dvrk_controller.activate_ar = True		

	##########
	#Other ###
	##########
	def pykdl_frame2str(self,frame):
		if frame is None:
			return "None"

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

	def generate_trajectory(self,init_pt, end_pt, n=20):
		init_pt = init_pt.reshape(3,1)
		end_pt = end_pt.reshape(3,1)
		t = np.linspace(0, 1, n).reshape(1,n)
		pt_on_line = (end_pt - init_pt) * t + init_pt
		return pt_on_line

	def ar_animation(self, init_pt, end_pt, n=40, animation_sleep=0.040):
		timeout = 14
		init_time = time.time()
		
		trajectory = self.generate_trajectory(init_pt, end_pt,n=n)

		for i in range(n):

			new_position = Vector(*list(trajectory[:,i]))
			self.set_activate_ar(PyKDL.Frame(self.fix_orientation, new_position))
			time.sleep(animation_sleep)

			if time.time() - init_time > timeout:
				break

	def vect2np(self,vect):
		return np.array([vect.x(),vect.y(), vect.z()])


def main():

	psm3 = PSM3Arm()

	#Sleep until the subscribers are ready.
	time.sleep(0.20)

	# while any([psm3.position1 is None, psm3.position2 is None]) and not rospy.core.is_shutdown():
	# 	pass
	# time.sleep(1)
	print("Positions for this script should be specified with respect to the psm3_base coordinates system!")
	answer = raw_input("Did you checked the four positions where the robot is moving? Ready to move robot? (Y/n) ")

	if answer == 'Y' and not rospy.core.is_shutdown():
		try:
			for _ in range(2):
				# psm3.move_rutine()
				psm3.move_rutine_with_animation()
		
			while not rospy.core.is_shutdown():

				rospy.rostime.wallsleep(0.25)

		except KeyboardInterrupt:
			print("Shutting down")


if __name__ == "__main__":
	main()