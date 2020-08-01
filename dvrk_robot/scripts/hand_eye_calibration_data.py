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

#Inconsistency between files headers and the order in which data was written.

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
			
			message1 = self.create_str_repr(idx)
			self.file.write(message1)
			self.file.flush()

		def create_str_repr(self,idx,):
			#File header ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,j4,j5,j6 --> (ts, translation, rotation, joints)
			message1 =  "{:},".format(rospy.Time.now())
			message1 += "{:d},".format(idx)
			message1 += "{: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}, "\
						.format(self.x, self.y,self.z,self.rx,
							self.ry,self.rz,self.rw)
			joints = ",".join(["{: 0.8f}".format(joint_i) for joint_i in self.joints])
			message1 += joints
			message1 += ",\n"

			return message1 

		def close_file(self,):
			self.file.flush()
			self.file.close()

	def __init__(self, userId = None, dst_path = None, rig_name=None):

		##Init files
		self.dst_path = dst_path + '/' + self.createTimeStamp() + userId + "/"
		os.mkdir(self.dst_path)

		#Create kinematic data structures
		header = "ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,\n"
		self.ecm_cartesian_kin = ArmKinematic(arm_name='ecm_cart', number_of_joints=4, has_gripper=False, 
											file_path= open(self.dst_path + "cartesian.txt","w"), 
											file_header=header)
		
		self.ecm_cartesian_local_kin = ArmKinematic(arm_name='ecm_cart_local', number_of_joints=4, has_gripper=False, 
											file_path= open(self.dst_path + "cartesian_local.txt","w"), 
											file_header=header)

		#Video files
		self.out_left = cv2.VideoWriter(join(self.dst_path,"video_left_color.avi"),fourcc, 30.0, (640,480))
		self.out_right = cv2.VideoWriter(join(self.dst_path,"video_right_color.avi"),fourcc, 30.0, (640,480))
		#Write video ts and ECM kinematics
		self.out_left_ts = open(join(self.dst_path, "video_left_color_ts.txt"),'w')
		self.out_left_ts.write(header)
		self.out_right_ts = open(join(self.dst_path,"video_right_color_ts.txt"),'w')
		self.out_right_ts.write(header)

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

		#############
		#Subscribers#
		#############

		##Pedal 
		self.camera_pedal_sub = rospy.Subscriber("/dvrk/footpedals/bicoag", Joy, self.pedal_callback)

		##ECM Kinematics
		self.ecm_cartesian_subs = rospy.Subscriber("/dvrk/ECM/position_cartesian_current", PoseStamped, self.ecm_cartesian_callback)
		self.ecm_cartesian_subs = rospy.Subscriber("/dvrk/ECM/position_cartesian_local_current", PoseStamped, self.ecm_cartesian_local_callback)
		self.ecm_joints_subs = rospy.Subscriber("/dvrk/ECM/state_joint_current", JointState, self.ecm_joints_callback)

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

	def left_callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			self.left_frame = cv_image
		except CvBridgeError as e:
			print(e)

		self.left_frame_count += 1

		#Save kinematics and frame to video
		message = self.ecm_cartesian_kin.create_str_repr()
		self.out_left.write(cv_image)
		self.out_left_ts.write(message)

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
		message = self.ecm_cartesian_kin.create_str_repr()
		self.out_right.write(cv_image)
		self.out_right_ts.write(message)

		#Modify image for display
		self.modifyImageAndPublish(cv_image, publisherId=2)

	def pedal_callback(self,data):
		if data.buttons[0]:
			self.save_frames_and_kinematics = True
			self.bicoag_count += 1
			print("bicoag pressed count: %i, Recording..." % self.bicoag_count)


	#ECM callbacks
	def ecm_cartesian_callback(self, data):

		self.ecm_cartesian_kin.x = data.pose.position.x
		self.ecm_cartesian_kin.y = data.pose.position.y
		self.ecm_cartesian_kin.z = data.pose.position.z

		self.ecm_cartesian_kin.rx = data.pose.orientation.x
		self.ecm_cartesian_kin.ry = data.pose.orientation.y
		self.ecm_cartesian_kin.rz = data.pose.orientation.z
		self.ecm_cartesian_kin.rw = data.pose.orientation.w

		if self.save_frames_and_kinematics:
			self.saving_frame_kinematic()
			self.save_frames_and_kinematics = False


	def ecm_cartesian_local_callback(self, data):
		
		self.ecm_cartesian_local_kin.x = data.pose.position.x
		self.ecm_cartesian_local_kin.y = data.pose.position.y
		self.ecm_cartesian_local_kin.z = data.pose.position.z

		self.ecm_cartesian_local_kin.rx = data.pose.orientation.x
		self.ecm_cartesian_local_kin.ry = data.pose.orientation.y
		self.ecm_cartesian_local_kin.rz = data.pose.orientation.z
		self.ecm_cartesian_local_kin.rw = data.pose.orientation.w

	def ecm_joints_callback(self, data):
		self.joints_ecm = data.position

	def saving_frame_kinematic(self):

		#Save kinematics
		self.ecm_cartesian_kin.save_to_file(idx=self.bicoag_count)
		self.ecm_cartesian_local_kin.save_to_file (idx=self.bicoag_count)
		#Save frames
		cv2.imwrite(self.dst_path + "left_{:03d}.png".format(self.bicoag_count), self.left_frame) #Left
		cv2.imwrite(self.dst_path + "right_{:03d}.png".format(self.bicoag_count), self.right_frame) #Right

	def createTimeStamp(self):
		ts = time.time()
		return datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%Hh.%Mm.%Ss_')

	def close_files(self):
		print("Flushing info and closing files.")
		self.cartesian_local_file.flush()
		self.cartesian_local_file.close()
		self.cartesian_file.flush()
		self.cartesian_file.close()



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