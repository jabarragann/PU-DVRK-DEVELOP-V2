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
from os import mkdir, makedirs
from os.path import join, exists
from itertools import chain


class collection_module:

	"""
	All kinematic data will be syncronized w.r.t. the left camera.
	Every time a new frame arrives to the callback the kinematic information of all the arms
	will be saved.
	"""

	class ArmKinematic:
		def __init__(self, arm_name, number_of_joints, has_gripper, file):
			self.arm_name = arm_name
			self.x = 0
			self.y = 0
			self.z = 0
			self.rx = 0
			self.ry = 0
			self.rz = 0
			self.rw = 0
			self.joints = np.zeros(number_of_joints) #NOt counting the gripper.

			self.file = file
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

		def save_to_file(self,ts, idx):
			#File header ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,j4,j5,j6 --> (ts, translation, rotation, joints)
			message1 =  "{:},".format(rospy.Time.now())
			message1 += "{:d},".format(idx)
			message1 += "{: 0.7f}, {: 0.7f}, {: 0.7f}, {: 0.7f}, {: 0.7f}, {: 0.7f}, {: 0.7f}, "\
						.format(self.x, self.y,self.z,self.rx,
							self.ry,self.rz,self.rw)

			joints = ",".join(["{: 0.8f}".format(joint_i) for joint_i in self.joints])
			message1 += joints
			message1 += ",\n"
			self.file.write(message1)
			self.file.flush()

		def close_file(self,):
			self.file.flush()
			self.file.close()


	def __init__(self, trial = None, dst_path = None, rig_name=None, message= None,invert_img= False):

		
		self.dst_path = dst_path 
		self.message = message

		#Init video and timestamp files
		self.init_videos_and_files()

		#Init important variables
		self.turn_on_square = False
		self.record = False
		self.bicoag_count = 0
		self.coag_count = 0
		self.frame_counter_left = 0
		self.frame_counter_right = 0
		self.bridge = CvBridge()

		#Font variables
		self.font = cv2.FONT_HERSHEY_SIMPLEX 
		self.fontSize = 1.0
		self.color = (0,255,0)
		self.thickness = 1
		self.alpha = 0.8

		#Arm kinematic information
		self.psm1_kinematic = self.ArmKinematic('psm1', 6, has_gripper=True, file=self.out_PSM1)
		self.psm2_kinematic = self.ArmKinematic('psm2', 6, has_gripper=True, file=self.out_PSM2)
		self.mtml_kinematic = self.ArmKinematic('mtml', 7, has_gripper=False, file=self.out_MTML)
		self.mtmr_kinematic = self.ArmKinematic('mtmr', 7, has_gripper=False, file=self.out_MTMR)
		self.kinematic_dict ={'psm1':self.psm1_kinematic,'psm2':self.psm2_kinematic,'mtml':self.mtml_kinematic,'mtmr':self.mtmr_kinematic}

		self.psm1_kinematic_local = self.ArmKinematic('psm1', 6, has_gripper=True, file=self.out_PSM1_local)
		self.psm2_kinematic_local = self.ArmKinematic('psm2', 6, has_gripper=True, file=self.out_PSM2_local)
		self.mtml_kinematic_local = self.ArmKinematic('mtml', 7, has_gripper=False, file=self.out_MTML_local)
		self.mtmr_kinematic_local = self.ArmKinematic('mtmr', 7, has_gripper=False, file=self.out_MTMR_local)
		self.local_kinematic_dict ={'psm1':self.psm1_kinematic_local,'psm2':self.psm2_kinematic_local,'mtml':self.mtml_kinematic_local,'mtmr':self.mtmr_kinematic_local}


		#############
		#Subscribers#
		#############

		##Pedal 
		self.bicoag_pedal_sub = rospy.Subscriber("/dvrk/footpedals/bicoag", Joy, self.bicoag_callback)
		self.coag_pedal_sub   = rospy.Subscriber("/dvrk/footpedals/coag", Joy, self.coag_callback)

		##PSM1 Kinematics
		self.psm1_cartesian_subs = rospy.Subscriber("/dvrk/PSM1/position_cartesian_current", PoseStamped, self.psm1_cartesian_callback)
		self.psm1_cartesian_local_subs = rospy.Subscriber("/dvrk/PSM1/position_cartesian_local_current", PoseStamped, self.psm1_cartesian_local_callback)
		self.psm1_joints_subs = rospy.Subscriber("/dvrk/PSM1/state_joint_current", JointState, self.psm1_joints_callback)
		self.psm1_gripper_subs = rospy.Subscriber("/dvrk/PSM1/state_jaw_current", JointState, self.psm1_gripper_callback)

		##PSM2 Kinematics
		self.psm2_cartesian_subs = rospy.Subscriber("/dvrk/PSM2/position_cartesian_current", PoseStamped, self.psm2_cartesian_callback)
		self.psm2_cartesian_local_subs = rospy.Subscriber("/dvrk/PSM2/position_cartesian_local_current", PoseStamped, self.psm2_cartesian_local_callback)
		self.psm2_joints_subs = rospy.Subscriber("/dvrk/PSM2/state_joint_current", JointState, self.psm2_joints_callback)
		self.psm2_gripper_subs = rospy.Subscriber("/dvrk/PSM2/state_jaw_current", JointState, self.psm2_gripper_callback)

		##MTML Kinematics
		self.mtml_cartesian_subs = rospy.Subscriber("/dvrk/MTML/position_cartesian_current", PoseStamped, self.mtml_cartesian_callback)
		self.mtml_cartesian_local_subs = rospy.Subscriber("/dvrk/MTML/position_cartesian_local_current", PoseStamped, self.mtml_cartesian_local_callback)
		self.mtml_joints_subs = rospy.Subscriber("/dvrk/MTML/state_joint_current", JointState, self.mtml_joints_callback)

		##MTMR Kinematics
		self.mtmr_cartesian_subs = rospy.Subscriber("/dvrk/MTMR/position_cartesian_current", PoseStamped, self.mtmr_cartesian_callback)
		self.mtmr_cartesian_local_subs = rospy.Subscriber("/dvrk/MTMR/position_cartesian_local_current", PoseStamped, self.mtmr_cartesian_local_callback)
		self.mtmr_joints_subs = rospy.Subscriber("/dvrk/MTMR/state_joint_current", JointState, self.mtmr_joints_callback)

		##Video
		self.invert_img = invert_img


		#Had to this weird if statement to fix a weird bug of ROS
		#Apparently using two rospy.Subscriber statements to the same topic is not good, even if they are inside a if statement.
		if not invert_img:
			print("Normal view")
			left_topic ="/"+rig_name+"/left/inverted"
			left_callback = self.left_callback
			right_topic = "/"+rig_name+"/right/inverted"
			right_callback = self.right_callback
		else:
			print("Inverted view")
			left_topic ="/video0/image_raw"
			left_callback = self.right_callback
			right_topic = "/video1/image_raw"
			right_callback = self.left_callback

		self.image_sub_left  = rospy.Subscriber(left_topic ,Image,left_callback)
		self.image_sub_right = rospy.Subscriber(right_topic,Image,right_callback)

		############
		#Publishers#
		############

		##Modified displays + compressed
		self.image_pub1 = rospy.Publisher("/"+rig_name+"/modified_display_left",Image, queue_size=5)
		self.image_pub2 = rospy.Publisher("/"+rig_name+"/modified_display_right",Image, queue_size=5)
		self.image_pub1_compressed = rospy.Publisher("/"+rig_name+"/modified_display_left/compressed" ,CompressedImage, queue_size=5)
		self.image_pub2_compressed = rospy.Publisher("/"+rig_name+"/modified_display_right/compressed",CompressedImage, queue_size=5)

	def init_videos_and_files(self):
		fourcc = cv2.VideoWriter_fourcc(*'XVID')
		self.out_left = cv2.VideoWriter(join(self.dst_path,"video_left_color.avi"),fourcc, 30.0, (640,480))
		self.out_right = cv2.VideoWriter(join(self.dst_path,"video_right_color.avi"),fourcc, 30.0, (640,480))
		self.out_left_ts = open(join(self.dst_path, "video_left_color_ts.txt"),'w')
		self.out_right_ts = open(join(self.dst_path,"video_right_color_ts.txt"),'w')
		self.out_PSM1 = open(join(self.dst_path,"PSM1_kinematics.txt"),'w')
		self.out_PSM1_local = open(join(self.dst_path,"PSM1_local_kinematics.txt"),'w')
		self.out_PSM2 = open(join(self.dst_path,"PSM2_kinematics.txt"),'w')
		self.out_PSM2_local = open(join(self.dst_path,"PSM2_local_kinematics.txt"),'w')
		self.out_MTML = open(join(self.dst_path,"MTML_kinematics.txt"),'w')
		self.out_MTML_local = open(join(self.dst_path,"MTML_local_kinematics.txt"),'w')
		self.out_MTMR = open(join(self.dst_path,"MTMR__kinematics.txt"),'w')
		self.out_MTMR_local = open(join(self.dst_path,"MTMR_local_kinematics.txt"),'w')

		self.out_left_ts.write("ts,idx\n")
		self.out_right_ts.write("ts,idx\n")
		self.out_PSM1.write("ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,j4,j5,j6\n")        
		self.out_PSM1_local.write("ts,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,j4,j5,j6\n")
		self.out_PSM2.write("ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,j4,j5,j6\n")        
		self.out_PSM2_local.write("ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,j4,j5,j6\n")
		self.out_MTML.write("ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,j4,j5,j6\n")        
		self.out_MTML_local.write("ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,j4,j5,j6\n")
		self.out_MTMR.write("ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,j4,j5,j6\n")        
		self.out_MTMR_local.write("ts,idx,x,y,z,rx,ry,rz,rw,j0,j1,j2,j3,j4,j5,j6\n")


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
			overlay = cv2.putText(overlay, '{:}'.format(self.message), (0,25), self.font, self.fontSize, self.color, self.thickness, cv2.LINE_AA)
			rect_color = (0,255,0) if self.turn_on_square else (0,0,255)
			cv2.rectangle(cv_image, (600, 0), (640,40), rect_color, -1)
			cv2.addWeighted(overlay, self.alpha, cv_image, 1 - self.alpha, 0, cv_image)

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

		return cv_image

	
	###########################################
	#########Subscriber callbacks##############
	###########################################

	#Video callbacks
	def left_callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			self.left_frame = cv_image
		except CvBridgeError as e:
			print(e)

		modified_frame = self.modifyImageAndPublish(cv_image, publisherId=1)

		self.out_left.write(modified_frame)
		self.frame_counter_left += 1
		ts = rospy.Time.now()
		self.out_left_ts.write("{:},{:d},\n".format(str(ts), self.frame_counter_left))

		self.save_all(str(ts), self.frame_counter_left)

	def right_callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		modified_frame = self.modifyImageAndPublish(cv_image, publisherId=2)
		
		self.out_right.write(modified_frame)
		self.frame_counter_right += 1
		self.out_right_ts.write("{:},{:d},\n".format(str(rospy.Time.now()), self.frame_counter_right))

	#Pedal callbacks
	def bicoag_callback(self,data):
		if data.buttons[0]:
			self.turn_on_square = not self.turn_on_square 
			self.bicoag_count += 1
			print("bicoag pressed count: %i, Recording..." % self.bicoag_count)

	def coag_callback(self,data):
		# if the button is pressed, record data
		self.coag_count +=1
		print("coag pressed count: %i, Recording..." % self.coag_count)
		if data.buttons[0]:
		    self.record = True
		else:
		    self.record = False

	#PSM1 callback
	def psm1_cartesian_callback(self, data):
		self.psm1_kinematic.set_pose(data)
		# if self.record:
		# 	self.psm1_kinematic.save_to_file()

	def psm1_cartesian_local_callback(self, data):
		self.psm1_kinematic_local.set_pose(data)
		# if self.record:
		# 	self.psm1_kinematic_local.save_to_file()

	def psm1_joints_callback(self, data):
		self.psm1_kinematic.set_joints(data.position)
		self.psm1_kinematic_local.set_joints(data.position)

	def psm1_gripper_callback(self, data):
		self.psm1_kinematic.gripper = data.position[0]
		self.psm1_kinematic_local.gripper = data.position[0]

	#PSM2 callbacks
	def psm2_cartesian_callback(self, data):
		self.psm2_kinematic.set_pose(data)
		# if self.record:
		# 	self.psm2_kinematic.save_to_file()

	def psm2_cartesian_local_callback(self, data):
		self.psm2_kinematic_local.set_pose(data)
		# if self.record:
		# 	self.psm2_kinematic_local.save_to_file()


	def psm2_joints_callback(self, data):
		self.psm2_kinematic.set_joints(data.position)
		self.psm2_kinematic_local.set_joints(data.position)

	def psm2_gripper_callback(self, data):
		self.psm2_kinematic.gripper = data.position[0]
		self.psm2_kinematic_local.gripper = data.position[0]

	#MTML callbacks
	def mtml_cartesian_callback(self, data):
		self.mtml_kinematic.set_pose(data)
		# if self.record:
		# 	self.mtml_kinematic.save_to_file()

	def mtml_cartesian_local_callback(self, data):
		self.mtml_kinematic_local.set_pose(data)
		# if self.record:
		# 	self.mtml_kinematic_local.save_to_file()

	def mtml_joints_callback(self, data):
		self.mtml_kinematic.set_joints(data.position)
		self.mtml_kinematic_local.set_joints(data.position)


	#MTMR callbacks
	def mtmr_cartesian_callback(self, data):
		self.mtmr_kinematic.set_pose(data)
		# if self.record:
		# 	self.mtmr_kinematic.save_to_file()

	def mtmr_cartesian_local_callback(self, data):
		self.mtmr_kinematic_local.set_pose(data)
		# if self.record:
		# 	self.mtmr_kinematic_local.save_to_file()

	def mtmr_joints_callback(self, data):
		self.mtmr_kinematic.set_joints(data.position)
		self.mtmr_kinematic_local.set_joints(data.position)


	################################################################

	def createTimeStamp(self):
		ts = time.time()
		return datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%Hh.%Mm.%Ss_')

	def save_all(self,ts, idx):
		#Save all kinematic information
		for obj in chain(self.kinematic_dict.values(),self.local_kinematic_dict.values()):
			obj.save_to_file(ts,idx)
	
	def close_files(self):

		self.out_left.release()
		self.out_right.release()
		self.out_left_ts.close()
		self.out_right_ts.close()

		print("Flushing info and closing files.")
		for obj in self.kinematic_dict.values():
			obj.close_file()
		for obj in self.local_kinematic_dict.values():
			obj.close_file()
				


def main():
	#Get rospy parameters
	if rospy.has_param('/image_inverter/rig_name'):
		rig_name   = rospy.get_param('/image_inverter/rig_name')
	else:
		print("rig_name was not specified")
		rig_name = "default_cam"

	##############################
	###     PARSE ARGS         ###
	##############################
	parser = argparse.ArgumentParser()
	parser.add_argument('--user_id', action="store", dest="user_id", required=True, help="Example: U03")
	parser.add_argument('--session', action="store", dest="session", required=True, help="Example: S03")
	parser.add_argument('--task', action="store", dest="task", required=True, help="Either suture or knot")
	parser.add_argument('--condition', action="store", dest="condition", required=True, help="Either nback or normal")
	parser.add_argument('--trial', action="store", dest="trial", required=True, help="Example: 1")

	args = parser.parse_args()
	user_id = args.user_id
	session = args.session
	task = args.task
	condition= args.condition
	trial = args.trial
	invert_img = False


	#Validate parameters
	if task not in ['peg', 'suture','knot']:
		print("Task needs to be either the word suture, peg or knot")
		exit()
	if condition not in ['normal','nback', 'inversion']:
		print("Condition need to be either the work normal, nback or inversion")
		exit()
	try: 
		int(trial) # Check if trial is an integer
	except ValueError:
		print("Trial needs to be a integer")
		exit()

	if condition == 'inversion':
		invert_img = True

	#create path
	task_condition = task + "_" + condition
	message = task_condition + "_" + trial 

	dst_path = "/home/isat/juanantonio/da_vinci_video_recordings/realtime_project_experiments/"
	dst_path = join(dst_path , user_id + "/" + session + "/" + task_condition + "/" + trial) 

	if not exists(dst_path):
		makedirs(dst_path)
	else:
		print("path: ", dst_path)
		print("Already exists")
		exit()


	print("Starting Da vinci video Operation...")
	cm = collection_module(trial = trial, dst_path = dst_path, rig_name=rig_name, message = message, invert_img= invert_img)

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