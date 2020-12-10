#!/usr/bin/env python
import json
import numpy as np 
#Ros libraries
import rospy
from cv_bridge import CvBridge, CvBridgeError
import tf_conversions.posemath as pm
import PyKDL
from PyKDL import Vector, Rotation, Frame
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
import dvrk

def generate_trajectory(init_pt, end_pt, n=20):
	init_pt = init_pt.reshape(3,1)
	end_pt = end_pt.reshape(3,1)
	t = np.linspace(0, 1, n).reshape(1,n)
	pt_on_line = (end_pt - init_pt) * t + init_pt # Shape (3,n) where each column represent a different point on the line.

	return pt_on_line

def main():
	rospy.init_node('dvrk_tf_node')
	time.sleep(0.2)
	
	dst_path = "/home/isat/juanantonio/davinci_catkin_ws_1.7/src/dvrk-ros/dvrk_robot/scripts/thesis_automation/vision_modules/test_videos"
	cm = dvrk_tf_module(userId = "test", dst_path = dst_path, rig_name="pu_dvrk_cam")

	#Sleep until the subscribers are ready.
	time.sleep(0.2)

	ans = raw_input("Would you like to recalculate the transformation between the world and the psmb (Requires chessboard)? (y/n) ")
	if ans == 'y':
		if not cm.get_cam_world_transform():
			print("Chessboard was not detected")
			exit(0)

		print("Re calculating trans_psmb_world")
		cm.calculate_psmb_world_transform()
	
	# print("psmb_world ")
	# print(cm.trans_to_matrix(cm.trans_psmb_world_l))
	# print("psm3b_psm")
	# print(cm.trans_to_matrix(cm.trans_psm3b_psm3))
	psm3_arm = dvrk.arm('PSM3')
	time.sleep(0.2)

	targets = cm.modified_corners
	trans_psmb_world_l = cm.trans_psmb_world_l
	
	for idx in range(targets.shape[0]):
		target_world  = Vector(*targets[idx])
		target_psmb = trans_psmb_world_l * target_world

		psm3_position1 = psm3_arm.get_current_position().p
		psm3_position2 = cm.trans_psm3b_psm3.p 
		print("Target (world frame)")
		print(target_world)
		print("Init Psm3 location (psm3b frame)")
		print(psm3_position1, psm3_position2)
		print("Target (psm3b frame)")
		print(target_psmb)

		ans = raw_input("Move to next point(y/n)")
		if ans == 'n':
			exit(0)

		trajectory = generate_trajectory(np.array([psm3_position1.x(),psm3_position1.y(),psm3_position1.z()]), 
										 np.array([target_psmb.x(),target_psmb.y(),target_psmb.z()]), 
										 n=5)
		for idx, c in enumerate(range(trajectory.shape[1])):
			next_position = PyKDL.Vector(trajectory[0,c], trajectory[1,c], trajectory[2,c],)
			print("position {:}".format(idx), next_position)
			
			# ans = raw_input("Move to next point. (y/n). ")
			# if ans == 'n':
			# 	exit(0)
			
			# psm3_arm.move(PyKDL.Vector(next_position)) 

		print("After motion")
		print("Psm3 location (psm3b frame)")
		print(psm3_arm.get_current_position().p, cm.trans_psm3b_psm3.p )
		print("Target (psm3b frame)")
		print(target_psmb)
		print("\n\n")
	try:
		while not rospy.core.is_shutdown():
			rospy.rostime.wallsleep(0.25)

	except KeyboardInterrupt:
		print("Shutting down")

	finally:
		print("exit")


if __name__ == "__main__":
	main()

