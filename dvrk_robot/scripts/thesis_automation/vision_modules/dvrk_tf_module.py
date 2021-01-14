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


class dvrk_tf_module:

	"""
	dvrk_tf_module

	This module will be incharge of managing all the transformations of the Da vinci robot
	"""

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

			# self.file = open(file_path,'w')
			# self.file.write(file_header)

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

		self.dst_path = dst_path
		# #Video files
		# fourcc = cv2.VideoWriter_fourcc(*'XVID')
		# self.out_left = cv2.VideoWriter(join(self.dst_path,"video_left_color.avi"),fourcc, 30.0, (640,480))
		# self.out_right = cv2.VideoWriter(join(self.dst_path,"video_right_color.avi"),fourcc, 30.0, (640,480))
		
		# #kinematic Files
		# self.psm3_kin_file = open(join(self.dst_path,"psm3_kin_file"),'w')
		# self.psm3_kin_file.write("ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,j4,j5\n")

		
		with open('./calib_parameters/left_camera_params.json','r') as f:
			left_params = json.load(f)
		with open('./calib_parameters/right_camera_params.json','r') as f:
			right_params = json.load(f)

		#########################
		## Camera parameters ####
		#########################

		#Chess board coordinates - Base on the dimension of my setup. square side length = 1.065cm, size = (8,6)
		self.chessboard_scale = 1.065 / 100
		objp = np.zeros((8 * 6, 3), np.float32)  
		objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)
		self.chessboard_coord = objp * self.chessboard_scale  # Convert to meters

		self.modified_corners = self.chessboard_coord[0:17:2,:] + np.array([[0,0,-2*self.chessboard_scale]])

		self.centroid_coordinates = np.array([[5,5]])
		# print(self.modified_corners)
		
		self.left_mtx = np.array(left_params['mtx'])
		self.left_dist = np.array(left_params['dist']).reshape((1,5))
		self.right_mtx = np.array(right_params['mtx'])
		self.right_dist = np.array(right_params['dist']).reshape((1,5))

		#########################
		## TF transformations####
		#########################
		self.trans_camleft_world = None
		self.trans_camright_world = None

		self.trans_ecmb_ecm = None   #ECM cartesian pose
		self.trans_psm3b_psm3 = None   #PSM3 cartesian pose

		with open('./calib_parameters/x_estimation_C5.json','r') as f:
			X = json.load(f)
		self.trans_camleft_ecm = pm.fromMatrix(np.array(X['T_left'])) #X matrix
		self.trans_camright_ecm = pm.fromMatrix(np.array(X['T_right']))

		with open('./calib_parameters/w_estimation_C6.json','r') as f:
			W = json.load(f)
		self.trans_ecmb_psmb_l = pm.fromMatrix(np.array(W['T']))

		with open('./calib_parameters/y_estimation.json','r') as f:
			Y = json.load(f)
		self.trans_ecmb_world_l = pm.fromMatrix(np.array(Y['T_from_left']))
		self.trans_ecmb_world_r = pm.fromMatrix(np.array(Y['T_from_right']))

		with open('./calib_parameters/z_estimation.json','r') as f:
			Z = json.load(f)
		self.trans_psmb_world_l = pm.fromMatrix(np.array(Z['T_from_left']))
		self.trans_psmb_world_r = pm.fromMatrix(np.array(Z['T_from_right']))
		
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
		##ECM Kinematics
		self.ecm_cartesian_subs = rospy.Subscriber("/dvrk/ECM/position_cartesian_current", PoseStamped, self.ecm_cartesian_callback)
		self.ecm_cartesian_local_subs = rospy.Subscriber("/dvrk/ECM/position_cartesian_local_current", PoseStamped, self.ecm_cartesian_local_callback)
		self.ecm_joints_subs = rospy.Subscriber("/dvrk/ECM/state_joint_current", JointState, self.ecm_joints_callback)

		##PSM3 kinematics
		self.psm3_cartesian_subs = rospy.Subscriber("/dvrk/PSM3/position_cartesian_current", PoseStamped, self.psm3_cartesian_callback)
		self.psm3_joints_subs = rospy.Subscriber("/dvrk/PSM3/state_joint_current", JointState, self.psm3_joints_callback)
		self.psm3_suj_subs = rospy.Subscriber("/dvrk/PSM3/io/suj_clutch", Joy, self.setup_button_psm3_callback)

		##Video
		self.image_sub_left  = rospy.Subscriber("/"+rig_name+"/left/inverted", Image, self.left_callback)
		self.image_sub_right = rospy.Subscriber("/"+rig_name+"/right/inverted", Image, self.right_callback)

		#Centroids
		self.multi_arr_sub = rospy.Subscriber("/pu_dvrk_tf/centroid_coordinates",Int32MultiArray, self.centroids_callback)

		############
		#Publishers#
		############

		##TF publishers - published everytime in the left camera callback
		self.tf_world_psm3b_pub = rospy.Publisher("/pu_dvrk_tf/tf_world_psm3b", PoseStamped, queue_size=5)

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
			# overlay = cv_image.copy()
			# overlay = cv2.putText(overlay, '{:03d}'.format(self.bicoag_count), (0,25), self.font, self.fontSize, self.color, self.thickness, cv2.LINE_AA)
			# cv2.rectangle(cv_image, (600, 0), (640,40), self.color, -1)
			# cv2.addWeighted(overlay, self.alpha, cv_image, 1 - self.alpha, 0, cv_image)

			#Draw centroids
			# self.draw_points_2(cv_image,self.centroid_coordinates, color= (0,255,0))


			############################
			##Add chess board corners###
			############################
			# if cam_frame is not None:
			# 	trans_matrix = pm.toMatrix(cam_frame)
			# 	tvecs, rvecs = trans_matrix[:3,3],trans_matrix[:3,:3]
			# 	imgpts, jac = cv2.projectPoints(self.chessboard_coord, rvecs, tvecs, mtx, dist)
			# 	img = self.draw_corners(cv_image, imgpts, (0,0,255))

			if self.trans_ecmb_ecm is not None:
				# trans_matrix = pm.toMatrix(cam_ecm*self.trans_ecmb_ecm.Inverse()*self.trans_ecmb_world_l)
				# tvecs, rvecs = trans_matrix[:3,3],trans_matrix[:3,:3]

				#Project corners to chess board
				# cv_image = self.draw_chess_corners(cv_image, self.chessboard_coord,publisherId, color =(255,0,0) )
				# cv_image = self.draw_chess_corners(cv_image, self.modified_corners,publisherId, color =(0,255,0) )
				#Project PSM3 pose
				cv_image = self.draw_psm_axis(cv_image,publisherId)

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
	#DRAW  Methods   ##
	###################
	def draw_chess_corners(self, img, corners, publisherId, color = (255,255,255)):
		# trans_cam_world = self.trans_camleft_world if publisherId == 1 else self.trans_camright_world
		trans_cam_ecm   = self.trans_camleft_ecm if publisherId == 1 else self.trans_camright_ecm
		mtx = self.left_mtx if publisherId == 1 else self.right_mtx
		dist = self.left_dist if publisherId == 1 else self.right_dist

		tvecs, rvecs = self.convert_to_rvec_tvec(trans_cam_ecm*self.trans_ecmb_ecm.Inverse()*self.trans_ecmb_world_l)
		imgpts, jac = cv2.projectPoints(corners, rvecs, tvecs, mtx, dist)
		img = self.draw_points(img, imgpts, color)

		return img

	def draw_psm_axis(self, img, publisherId):
		cam_frame = self.trans_camleft_world if publisherId == 1 else self.trans_camright_world
		cam_ecm   = self.trans_camleft_ecm if publisherId == 1 else self.trans_camright_ecm
		mtx = self.left_mtx if publisherId == 1 else self.right_mtx
		dist = self.left_dist if publisherId == 1 else self.right_dist

		psm3_vect = self.trans_psm3b_psm3.p
		psm_pose = pm.toMatrix(self.trans_psm3b_psm3)

		psm_pose = psm_pose[:3,:3]
		psm_x_unit = psm_pose[:,0] / 100
		psm_y_unit = psm_pose[:,1] / 100
		psm_z_unit = psm_pose[:,2] / 100
		psm_pts = np.array([[psm3_vect.x(),psm3_vect.y(),psm3_vect.z()]])

		pts_to_convert = np.vstack((psm_x_unit+psm_pts,psm_y_unit+psm_pts, psm_z_unit+psm_pts, psm_pts))

		trans_l, trans_r = self.calculate_trans_cam_psmb()
		trans = trans_l if publisherId == 1 else trans_r
		tvecs, rvecs = self.convert_to_rvec_tvec(trans)

		imgpts1, jac = cv2.projectPoints(pts_to_convert, rvecs, tvecs, mtx, dist)
		imgpts1 = imgpts1.squeeze().astype(np.int)
		img = self.draw_coordinate_axis(imgpts1[0, :], imgpts1[1, :], imgpts1[2, :], imgpts1[3, :], img)

		#Project cornerst to chess board simple
		# psm3_vect = self.trans_psm3b_psm3.p
		# trans_l, trans_r = self.calculate_trans_cam_psmb()
		# trans = trans_l if publisherId == 1 else trans_r
		# tvecs, rvecs = self.convert_to_rvec_tvec(trans)
		# imgpts, jac = cv2.projectPoints(np.array([[psm3_vect.x(),psm3_vect.y(),psm3_vect.z()]]), rvecs, tvecs, mtx, dist)
		# img = self.draw_corners(cv_image, imgpts, (0,0,255))

		return img


	def draw_points(self, img,pts, color):
		pts = pts.astype(np.float32) #cv2.circle only accepts float32
		
		for i in range(pts.shape[0]):
			center_coordinates = (pts[i,0,0], pts[i,0,1])
			image = cv2.circle(img, center_coordinates, radius=3, color=color, thickness=-1)
		
		return image
	def draw_points_2(self, img,pts, color):
		pts = pts.astype(np.float32) #cv2.circle only accepts float32
		
		for i in range(pts.shape[0]):
			center_coordinates = (pts[i,0], pts[i,1])
			image = cv2.circle(img, center_coordinates, radius=3, color=color, thickness=-1)
		
		return image

	def draw_coordinate_axis(self, x_unit,y_unit,z_unit,orig,img):

	    img = cv2.line(img, tuple(x_unit), tuple(orig), (0,0,255), 4)
	    img = cv2.line(img, tuple(y_unit), tuple(orig), (0,0,255), 4)
	    img = cv2.line(img, tuple(z_unit), tuple(orig), (0,0,0), 4)
	    return img

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

	
		self.modifyImageAndPublish(cv_image, publisherId=1)

		#TF publishers
		tf_world_psmrb = self.trans_ecmb_world_l.Inverse() * self.trans_ecmb_psmb_l
		self.publish_tf(tf_world_psmrb, self.tf_world_psm3b_pub)

	def right_callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			self.right_frame = cv_image
		except CvBridgeError as e:
			print(e)

		self.right_frame_count += 1


		#Modify image for display
		self.modifyImageAndPublish(cv_image, publisherId=2)

	def centroids_callback(self, data):
		arr = np.array(data.data).reshape(-1,2)
		self.centroid_coordinates = arr

		# print(self.centroid_coordinates)

	###############
	#ECM callbacks#
	###############
	def ecm_cartesian_callback(self, data):
		self.ecm_cartesian_kin.set_pose(data)

		self.trans_ecmb_ecm = pm.fromMsg(data.pose)

		if self.save_frames_and_kinematics:
			self.saving_frame_kinematic_bicoag()
			self.save_frames_and_kinematics = False

	def ecm_cartesian_local_callback(self, data):
		self.ecm_cartesian_local_kin.set_pose(data)

	def ecm_joints_callback(self, data):
		self.ecm_cartesian_local_kin.set_joints(data.position)
		self.ecm_cartesian_kin.set_joints(data.position)

	################
	#PSM3 callbacks#
	################
	def psm3_cartesian_callback(self, data):
		self.psm3_cartesian_kin.set_pose(data)

		self.trans_psm3b_psm3 = pm.fromMsg(data.pose)

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

	def get_cam_world_transform(self):

		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

		cam_frames =  [("left", self.left_frame, self.left_mtx, self.left_dist), ("right",self.right_frame, self.right_mtx, self.right_dist)]
		frames = {"left":{"success":False,"frame":None}, "right":{"success":False,"frame":None}}
		for cam_id, img, mtx, dist in cam_frames:	

			gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
			ret, corners = cv2.findChessboardCorners(gray, (8,6),None)
			
			frames[cam_id]['success'] = ret
			if ret == True:
				corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
				# Find the rotation and translation vectors.
				ret,rvecs, tvecs = cv2.solvePnP(self.chessboard_coord, corners2, mtx, dist)

				rvecs,_ = cv2.Rodrigues(rvecs)
				
				
				vec = PyKDL.Vector(tvecs[0],tvecs[1],tvecs[2])
				rot = PyKDL.Rotation(*list(rvecs.flatten()))
				frame = PyKDL.Frame(rot,vec)
				frames[cam_id]['frame'] = frame
				# print(cam_id)
				# print(tvecs)
				# print(rvecs)
				# print(type(rvecs))
				# print(rvecs.shape)
				# print(dist)
				# print(frame)

		self.trans_camleft_world = frames['left']['frame']
		self.trans_camright_world = frames['right']['frame']

		print(self.chessboard_coord.shape)

		success = all([frames['left']['success'],frames['right']['success']])
		print("Operation successful? ", success)
		if not success:
			exit(0)
		self.calculate_ecmb_world_transform()
		
		return success

	def calculate_ecmb_world_transform(self):
		#ECM matrix
		self.trans_ecmb_world_l = self.trans_ecmb_ecm * self.trans_camleft_ecm.Inverse() * self.trans_camleft_world
		self.trans_ecmb_world_r = self.trans_ecmb_ecm * self.trans_camright_ecm.Inverse() * self.trans_camright_world

		# print(self.trans_ecmb_world_l)
		# print(self.trans_ecmb_world_r)
		

		Y = {"T_from_left": pm.toMatrix(self.trans_ecmb_world_l).tolist(), "T_from_right": pm.toMatrix(self.trans_ecmb_world_r).tolist()}
		with open("./calib_parameters/y_estimation.json","w") as f:
			Y = json.dumps(Y, indent=4)
			f.write(Y)


	def calculate_psmb_world_transform(self):
		self.trans_psmb_world_l = self.trans_ecmb_psmb_l.Inverse() *self.trans_ecmb_ecm * self.trans_camleft_ecm.Inverse() * self.trans_camleft_world
		self.trans_psmb_world_r = self.trans_ecmb_psmb_l.Inverse() *self.trans_ecmb_ecm * self.trans_camright_ecm.Inverse() * self.trans_camright_world
		
		z = {"T_from_left": pm.toMatrix(self.trans_psmb_world_l).tolist(), "T_from_right": pm.toMatrix(self.trans_psmb_world_r).tolist()}
		with open("./calib_parameters/z_estimation.json","w") as f:
			z = json.dumps(z, indent=4)
			f.write(z)

	def trans_to_matrix(self,trans):
		return pm.toMatrix(trans)

	def calculate_trans_cam_psmb(self):
		# trans_cam_psmb_l = self.trans_camleft_ecm.Inverse() * self.trans_ecmb_ecm.Inverse() * self.trans_ecmb_psmb_l 
		# trans_cam_psmb_r = self.trans_camright_ecm.Inverse() *  self.trans_ecmb_ecm.Inverse() * self.trans_ecmb_psmb_l
		trans_cam_psmb_l = self.trans_camleft_ecm * self.trans_ecmb_ecm.Inverse() * self.trans_ecmb_psmb_l 
		trans_cam_psmb_r = self.trans_camright_ecm *  self.trans_ecmb_ecm.Inverse() * self.trans_ecmb_psmb_l
		return trans_cam_psmb_l, trans_cam_psmb_r

	def convert_to_rvec_tvec(self, trans):
		trans_matrix = pm.toMatrix(trans)
		tvecs, rvecs = trans_matrix[:3,3],trans_matrix[:3,:3]

		return tvecs, rvecs

	def publish_tf(self,tf, pub):
		goal = PoseStamped()

		#vect
		vect = tf.p
		rot = tf.M.GetQuaternion() 
		goal.header.seq = 1
		goal.header.stamp = rospy.Time.now()
		goal.header.frame_id = "map"

		goal.pose.position.x = vect.x()
		goal.pose.position.y = vect.y()
		goal.pose.position.z = vect.z()

		goal.pose.orientation.x = rot[0]
		goal.pose.orientation.y = rot[1]
		goal.pose.orientation.z = rot[2]
		goal.pose.orientation.w = rot[3]

		pub.publish(goal)