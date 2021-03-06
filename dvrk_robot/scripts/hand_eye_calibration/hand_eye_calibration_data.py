#!/usr/bin/env python

#Ros libraries
import rospy
from cv_bridge import CvBridge, CvBridgeError
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

#Inconsistency between files headers and the order in which data was written.
#It should be already been fix but keep this in case you run into errors.

#Version 3
'''
Newest features
PSM3 recordings
kinematic data at each image frame will be saved in a independent file.
'''

class hand_eye_collection_module:

	class ArmKinematic:
		def __init__(self, arm_name, number_of_joints, has_gripper, file_path, file_header):
			self.arm_name = arm_name
			self.x = 0
			self.y = 0
			self.z = 0
			self.rx = 0
			self.ry = 0
			self.rz = 0
			self.rw = 0
			self.joints = np.zeros(number_of_joints) #NOt counting the gripper.

			self.file = open(file_path,'w')
			self.file.write(file_header)

			if has_gripper:
				self.gripper = 0

		def set_joints(self,joints):
			self.joints[:] = joints 

		def set_pose(self, data):
			self.x = data.pose.position.x
			self.y = data.pose.position.y
			self.z = data.pose.position.z

			self.rx = data.pose.orientation.x
			self.ry = data.pose.orientation.y
			self.rz = data.pose.orientation.z
			self.rw = data.pose.orientation.w

		def save_to_file(self, idx):
			
			message1 = self.create_str_repr(rospy.Time.now(), idx)
			message1 += "\n"
			self.file.write(message1)
			self.file.flush()

		def create_str_repr(self,ts,idx,):
			#File header ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,j4,j5,j6 --> (ts, translation, rotation, joints)
			message1 =  "{:},".format(ts)
			message1 += "{:d},".format(idx)
			message1 += "{: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}, "\
						.format(self.x, self.y,self.z,self.rx,
							self.ry,self.rz,self.rw)
			joints = ",".join(["{: 0.8f}".format(joint_i) for joint_i in self.joints])
			message1 += joints
			# message1 += ",\n"

			return message1 

		def close_file(self,):
			self.file.flush()
			self.file.close()

	def __init__(self, userId = None, dst_path = None, rig_name=None):


		##############
		##Init files##
		##############
		self.dst_path = dst_path + '/' + self.createTimeStamp() + userId + "/"
		os.mkdir(self.dst_path)
		os.mkdir(self.dst_path + 'ecm_snapshots')
		os.mkdir(self.dst_path + 'psm1_snapshots')
		os.mkdir(self.dst_path + 'psm2_snapshots')
		os.mkdir(self.dst_path + 'psm3_snapshots')

		#Video files
		fourcc = cv2.VideoWriter_fourcc(*'XVID')
		self.out_left = cv2.VideoWriter(join(self.dst_path,"video_left_color.avi"),fourcc, 30.0, (640,480))
		self.out_right = cv2.VideoWriter(join(self.dst_path,"video_right_color.avi"),fourcc, 30.0, (640,480))
		
		#kinematic Files
		self.psm3_kin_file = open(join(self.dst_path,"psm3_kin_file"),'w')
		self.psm3_kin_file.write("ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,j4,j5\n")

		###################################
		#Create kinematic data structures##
		###################################
		#Create ECM arms
		header = "ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,\n"
		self.ecm_cartesian_kin = self.ArmKinematic(arm_name='ecm_cart', number_of_joints=4, has_gripper=False, 
											file_path= self.dst_path + "ecm_snapshots/ecm_cartesian.txt", 
											file_header=header)
		
		self.ecm_cartesian_local_kin = self.ArmKinematic(arm_name='ecm_cart_local', number_of_joints=4, has_gripper=False, 
											file_path= self.dst_path + "ecm_snapshots/ecm_cartesian_local.txt",
											file_header=header)
		#Create PSM arms
		header = "ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,j4,j5\n"
		self.psm1_cartesian_kin = self.ArmKinematic(arm_name='psm1_cart', number_of_joints=6, has_gripper=False, 
											file_path= self.dst_path + "psm1_snapshots/psm1_cartesian.txt", 
											file_header=header)
		
		self.psm2_cartesian_kin = self.ArmKinematic(arm_name='psm2_cart', number_of_joints=6, has_gripper=False, 
											file_path= self.dst_path + "psm2_snapshots/psm2_cartesian.txt",
											file_header=header)

		self.psm3_cartesian_kin = self.ArmKinematic(arm_name='psm3_cart', number_of_joints=6, has_gripper=False, 
											file_path= self.dst_path + "psm3_snapshots/psm3_cartesian.txt",
											file_header=header)

		#Write video ts and ECM kinematics

		#Header of video_left/right has to change to include ecm and psm1/2
		header2 = "ecm_ts,ecm_idx,ecm_x,ecm_y,ecm_z,ecm_rx,ecm_ry,ecm_rz,ecm_rw,ecm_j0,ecm_j1,ecm_j2,ecm_j3,"  \
					"psm1_ts,psm1_idx,psm1_x,psm1_y,psm1_z,psm1_rx,psm1_ry,psm1_rz,psm1_rw,psm1_j0,psm1_j1,psm1_j2,psm1_j3,psm1_j4,psm1_j5," \
					"psm2_ts,psm2_idx,psm2_x,psm2_y,psm2_z,psm2_rx,psm2_ry,psm2_rz,psm2_rw,psm2_j0,psm2_j1,psm2_j2,psm2_j3,psm2_j4,psm2_j5, \n"
		self.out_left_ts = open(join(self.dst_path, "video_left_color_ts.txt"),'w')
		self.out_left_ts.write(header2)
		self.out_right_ts = open(join(self.dst_path,"video_right_color_ts.txt"),'w')
		self.out_right_ts.write(header2)

		#Init important variables
		self.save_frames_and_kinematics = False
		self.bicoag_count = 0
		self.left_frame = None
		self.left_frame_count = 0
		self.right_frame = None
		self.right_frame_count = 0
		self.bridge = CvBridge()

		#Font variables
		self.font = cv2.FONT_HERSHEY_SIMPLEX 
		self.fontSize = 1.0
		self.color = (0,255,0)
		self.thickness = 1
		self.alpha = 0.8

		#PSM buttons
		self.psm1_snapshot = False
		self.psm2_snapshot = False
		self.psm3_snapshot = False
		self.suj_button_psm1_count = 0
		self.suj_button_psm2_count = 0
		self.suj_button_psm3_count = 0
		
		#############
		#Subscribers#
		#############

		##Pedal 
		self.camera_pedal_sub = rospy.Subscriber("/dvrk/footpedals/bicoag", Joy, self.pedal_callback)

		##ECM Kinematics
		self.ecm_cartesian_subs = rospy.Subscriber("/dvrk/ECM/position_cartesian_current", PoseStamped, self.ecm_cartesian_callback)
		self.ecm_cartesian_local_subs = rospy.Subscriber("/dvrk/ECM/position_cartesian_local_current", PoseStamped, self.ecm_cartesian_local_callback)
		self.ecm_joints_subs = rospy.Subscriber("/dvrk/ECM/state_joint_current", JointState, self.ecm_joints_callback)

		##PSM1/2 kinematics
		self.psm1_cartesian_subs = rospy.Subscriber("/dvrk/PSM1/position_cartesian_current", PoseStamped, self.psm1_cartesian_callback)
		self.psm2_cartesian_subs = rospy.Subscriber("/dvrk/PSM2/position_cartesian_current", PoseStamped, self.psm2_cartesian_callback)
		self.psm3_cartesian_subs = rospy.Subscriber("/dvrk/PSM3/position_cartesian_current", PoseStamped, self.psm3_cartesian_callback)

		self.psm1_joints_subs = rospy.Subscriber("/dvrk/PSM1/state_joint_current", JointState, self.psm1_joints_callback)
		self.psm2_joints_subs = rospy.Subscriber("/dvrk/PSM2/state_joint_current", JointState, self.psm2_joints_callback)
		self.psm3_joints_subs = rospy.Subscriber("/dvrk/PSM3/state_joint_current", JointState, self.psm3_joints_callback)

		self.psm1_suj_subs = rospy.Subscriber("/dvrk/PSM1/io/suj_clutch", Joy, self.setup_button_psm1_callback)
		self.psm3_suj_subs = rospy.Subscriber("/dvrk/PSM3/io/suj_clutch", Joy, self.setup_button_psm3_callback)


		##Video
		self.image_sub_left  = rospy.Subscriber("/"+rig_name+"/left/inverted", Image, self.left_callback)
		self.image_sub_right = rospy.Subscriber("/"+rig_name+"/right/inverted", Image, self.right_callback)

		############
		#Publishers#
		############

		##Modified displays + compressed
		self.image_pub1 = rospy.Publisher("/"+rig_name+"/modified_display_left",Image, queue_size=5)
		self.image_pub2 = rospy.Publisher("/"+rig_name+"/modified_display_right",Image, queue_size=5)
		self.image_pub1_compressed = rospy.Publisher("/"+rig_name+"/modified_display_left/compressed" ,CompressedImage, queue_size=5)
		self.image_pub2_compressed = rospy.Publisher("/"+rig_name+"/modified_display_right/compressed",CompressedImage, queue_size=5)


	def modifyImageAndPublish(self,cv_image_orig, publisherId=1):

		cv_image = cv_image_orig.copy()
		publisher = self.image_pub1 if publisherId == 1 else self.image_pub2
		compressedPublisher = self.image_pub1_compressed if publisherId == 1 else self.image_pub2_compressed

		#Modify Image
		if True:
			#########################
			#Add recording indicator#
			#########################
			overlay = cv_image.copy()
			overlay = cv2.putText(overlay, '{:03d}'.format(self.bicoag_count), (0,25), self.font, self.fontSize, self.color, self.thickness, cv2.LINE_AA)
			cv2.rectangle(cv_image, (600, 0), (640,40), self.color, -1)
			cv2.addWeighted(overlay, self.alpha, cv_image, 1 - self.alpha, 0, cv_image)

			############################
			##Add chess board corners###
			############################

			# gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
			# # Find the chess board corners
			# ret, corners = cv2.findChessboardCorners(gray, (8,6),None)
			# # If found, add object points, image points (after refining them)
			# if ret == True:
			# 	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

			# 	# self.objpoints.append(objp) # Object points
			# 	corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
			# 	# imgpoints.append(corners2)
			# 	# Draw and display the corners
			# 	cv_image = cv2.drawChessboardCorners(cv_image, (8,6), corners2,ret)

		#Publish modified Image
		try:
			publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)

		#### Create and Publish Compressed Image ####
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
		msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
		compressedPublisher.publish(msg)

	###################
	#IMAGE CALLBACKS ##
	###################
	def left_callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			self.left_frame = cv_image
		except CvBridgeError as e:
			print(e)

		self.left_frame_count += 1

		#Save kinematics and frame to video
		ecm_kinematics  = self.ecm_cartesian_kin.create_str_repr(ts=rospy.Time.now(),idx=self.left_frame_count)
		psm1_kinematics = self.psm1_cartesian_kin.create_str_repr(ts=rospy.Time.now(),idx=self.left_frame_count)
		psm2_kinematics = self.psm2_cartesian_kin.create_str_repr(ts=rospy.Time.now(),idx=self.left_frame_count)
		psm3_kinematics = self.psm3_cartesian_kin.create_str_repr(ts=rospy.Time.now(),idx=self.left_frame_count)
		message = ecm_kinematics+','+psm1_kinematics+','+psm2_kinematics+',' +'\n'

		#Write to files
		self.out_left.write(cv_image)
		self.out_left_ts.write(message)
		self.psm3_kin_file.write(psm3_kinematics+'\n')
		#Modify image for display
		self.modifyImageAndPublish(cv_image, publisherId=1)

	def right_callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			self.right_frame = cv_image
		except CvBridgeError as e:
			print(e)

		self.right_frame_count += 1

		#Save kinematics and frame to video
		ecm_kinematics  = self.ecm_cartesian_kin.create_str_repr(ts=rospy.Time.now(),idx=self.right_frame_count)
		psm1_kinematics = self.psm1_cartesian_kin.create_str_repr(ts=rospy.Time.now(),idx=self.right_frame_count)
		psm2_kinematics = self.psm2_cartesian_kin.create_str_repr(ts=rospy.Time.now(),idx=self.right_frame_count)
		message = ecm_kinematics+','+psm1_kinematics+','+psm2_kinematics+',' +'\n'
		self.out_right.write(cv_image)
		self.out_right_ts.write(message)

		#Modify image for display
		self.modifyImageAndPublish(cv_image, publisherId=2)


	###################
	#PEDALS CALLBACKS##
	###################
	def pedal_callback(self,data):
		if data.buttons[0]:
			self.save_frames_and_kinematics = True
			self.bicoag_count += 1
			print("bicoag pressed count: %i, Recording..." % self.bicoag_count)

	###############
	#ECM callbacks#
	###############
	def ecm_cartesian_callback(self, data):
		self.ecm_cartesian_kin.set_pose(data)

		if self.save_frames_and_kinematics:
			self.saving_frame_kinematic_bicoag()
			self.save_frames_and_kinematics = False

	def ecm_cartesian_local_callback(self, data):
		self.ecm_cartesian_local_kin.set_pose(data)

	def ecm_joints_callback(self, data):
		self.ecm_cartesian_local_kin.set_joints(data.position)
		self.ecm_cartesian_kin.set_joints(data.position)

	###############
	#PSM1 callback#
	###############
	def psm1_cartesian_callback(self, data):
		self.psm1_cartesian_kin.set_pose(data)

		if self.psm1_snapshot:
			self.saving_frame_kinmetic_suj_button_psm1()
			self.psm1_snapshot = False

	def psm1_joints_callback(self, data):
		self.psm1_cartesian_kin.set_joints(data.position)

	def setup_button_psm1_callback(self,data):
		if data.buttons[0]:
			self.psm1_snapshot = True
			self.suj_button_psm1_count += 1
			print("suj_button_psm1_count pressed count: %i, Recording..." % self.suj_button_psm1_count)

	################
	#PSM2 callbacks#
	################
	def psm2_cartesian_callback(self, data):
		self.psm2_cartesian_kin.set_pose(data)

	def psm2_joints_callback(self, data):
		self.psm2_cartesian_kin.set_joints(data.position)

	################
	#PSM3 callbacks#
	################
	def psm3_cartesian_callback(self, data):
		self.psm3_cartesian_kin.set_pose(data)
		if self.psm3_snapshot:
			self.saving_frame_kinmetic_suj_button_psm3()
			self.psm3_snapshot = False

	def psm3_joints_callback(self, data):
		self.psm3_cartesian_kin.set_joints(data.position)

	def setup_button_psm3_callback(self,data):
		if data.buttons[0]:
			self.psm3_snapshot = True
			self.suj_button_psm3_count += 1
			print("suj_button_psm3_count pressed count: %i, Recording..." % self.suj_button_psm3_count)

	####################
	#HELPER FUNCTIONS###
	####################

	def saving_frame_kinematic_bicoag(self):
		#Save kinematics
		self.ecm_cartesian_kin.save_to_file(idx=self.bicoag_count)
		self.ecm_cartesian_local_kin.save_to_file (idx=self.bicoag_count)
		#Save frames
		cv2.imwrite(self.dst_path + "ecm_snapshots/left_{:03d}.png".format(self.bicoag_count), self.left_frame) #Left
		cv2.imwrite(self.dst_path + "ecm_snapshots/right_{:03d}.png".format(self.bicoag_count), self.right_frame) #Right
	
	def saving_frame_kinmetic_suj_button_psm1(self):
		#Save kinematics
		self.psm1_cartesian_kin.save_to_file(idx=self.suj_button_psm1_count)
		
		#Save frames
		cv2.imwrite(self.dst_path + "psm1_snapshots/left_{:03d}.png".format(self.suj_button_psm1_count), self.left_frame) #Left
		cv2.imwrite(self.dst_path + "psm1_snapshots/right_{:03d}.png".format(self.suj_button_psm1_count), self.right_frame) #Right

	def saving_frame_kinmetic_suj_button_psm3(self):
		#Save kinematics
		self.psm3_cartesian_kin.save_to_file(idx=self.suj_button_psm3_count)
		
		#Save frames
		cv2.imwrite(self.dst_path + "psm3_snapshots/left_{:03d}.png".format(self.suj_button_psm3_count), self.left_frame) #Left
		cv2.imwrite(self.dst_path + "psm3_snapshots/right_{:03d}.png".format(self.suj_button_psm3_count), self.right_frame) #Right


	def createTimeStamp(self):
		ts = time.time()
		return datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%Hh.%Mm.%Ss_')

	def close_files(self):
		print("Flushing info and closing files.")
		self.ecm_cartesian_kin.close_file()
		self.ecm_cartesian_local_kin.close_file()
		


def main():
	#Get rospy parameters
	if rospy.has_param('/image_inverter/rig_name'):
		rig_name   = rospy.get_param('/image_inverter/rig_name')
	else:
		print("rig_name was not specified")
		rig_name = "default_cam"


	# allParams = rospy.get_param_names()
	# for i in allParams:
	# 	print(i)

	# myargv = rospy.myargv(argv=sys.argv)



	##############################
	###     PARSE ARGS         ###
	##############################
	parser = argparse.ArgumentParser()
	parser.add_argument('-i', action="store", dest="collection_id", required=True, help="Collection id, example: S3")

	args = parser.parse_args()
	collection_id = args.collection_id


	#Temp 
	rig_name = "pu_dvrk_cam"
	subject_id = "test"

	dst_path = "/home/isat/juanantonio/da_vinci_video_recordings/hand-eye_calibration"

	print("Starting Da vinci video Operation...")

	cm = hand_eye_collection_module(userId = collection_id, dst_path = dst_path, rig_name=rig_name)

	#Sleep until the subscribers are ready.
	time.sleep(0.10)

	try:
		while not rospy.core.is_shutdown():
			rospy.rostime.wallsleep(0.25)

	except KeyboardInterrupt:
		print("Shutting down")

	finally:
		cm.close_files()
		print("Shutting down")


if __name__ == '__main__':

	rospy.init_node('recording_node')
	main()