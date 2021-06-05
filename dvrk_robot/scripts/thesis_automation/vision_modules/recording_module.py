#!/usr/bin/env python
import json
import numpy as np 
#Ros libraries
import rospy
from cv_bridge import CvBridge, CvBridgeError
import tf_conversions.posemath as pm
import PyKDL
#Ros messages
from diagnostic_msgs.msg import KeyValue 
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


class recording_module:

	def __init__(self, dst_path, root_path = None):
		
		if root_path is None:
			self.root_path = "/home/isat/juanantonio/da_vinci_video_recordings/thesis_recordings/"
		else:
			self.root_path = root_path

		self.dst_path = os.path.join(self.root_path,dst_path+'/')

	
		if not os.path.exists(self.dst_path):
			os.mkdir(self.dst_path)
			

		#Recording
		self.recording_counter = 0 
		self.is_recording_active = False
		self.arms_to_record = ['psm1','psm2','psm3','mtml','mtmr']

		self.right_frame = None
		self.right_frame_id = 0

		#Video files
		self.bridge = CvBridge()
		
		# Arms current cartesian positions
		self.psm1_current = None
		self.psm2_current = None
		self.psm3_current = None
		self.mtml_current = None
		self.mtmr_current = None

		self.arms_files_dict = {}


		#Create files
		self.create_files()
		# self.is_recording_active = True

		#############
		#Subscribers#
		#############

		##Video  
		rig_name = "pu_dvrk_cam"
		# self.image_right_sub  = rospy.Subscriber("/"+rig_name+"/right/inverted", Image, self.right_callback)
		self.image_right_sub  = rospy.Subscriber("/pu_dvrk_cam/modified_display_right", Image, self.right_callback)

		##Pedals
		self.bicoag_sub = rospy.Subscriber("/dvrk/footpedals/bicoag", Joy, self.bicoag_callback)
		self.clutch_sub = rospy.Subscriber("/dvrk/footpedals/clutch", Joy, self.clutch_callback)
		self.teleop_events_sub = rospy.Subscriber("/dvrk/console/teleop/teleop_psm_selected", KeyValue, self.teleop_events_callback)
		##Arms kinematics
		self.psm1_cartesian_sub = rospy.Subscriber("/dvrk/PSM1/position_cartesian_current", PoseStamped, self.psm1_cartesian_callback)
		self.psm2_cartesian_sub = rospy.Subscriber("/dvrk/PSM2/position_cartesian_current", PoseStamped, self.psm2_cartesian_callback)
		self.psm3_cartesian_sub = rospy.Subscriber("/dvrk/PSM3/position_cartesian_current", PoseStamped, self.psm3_cartesian_callback)
		self.mtml_cartesian_sub = rospy.Subscriber("/dvrk/MTML/position_cartesian_current", PoseStamped, self.mtml_cartesian_callback)
		self.mtmr_cartesian_sub = rospy.Subscriber("/dvrk/MTMR/position_cartesian_current", PoseStamped, self.mtmr_cartesian_callback)

		#Recording module
		self.recording_subs = rospy.Subscriber("/recording_module/recording_active", Bool, self.recording_callback)

		#############
		#Publishers #
		#############
		# self.is_recording_active_pub = rospy.Publisher("/recording_module/recording_active",Bool,queue_size=5) ###CHECK SYNTAX
		

	###################
	#IMAGE CALLBACKS ##
	###################
	def right_callback(self,data):

		self.right_frame_id += 1
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			self.right_frame = cv_image
		except CvBridgeError as e:
			print(e)

		##Synchronize all recordings w.r.t right frames
		if self.is_recording_active:
			self.save_information()
		
		##publish the recording signal
		# self.is_recording_active_pub.publish(self.is_recording_active) ###CHECK SYNTAX

	###################
	#ARM   CALLBACKS ##
	###################
	def psm1_cartesian_callback(self, data):
		self.psm1_current = pm.fromMsg(data.pose)
	def psm2_cartesian_callback(self, data):
		self.psm2_current = pm.fromMsg(data.pose)
	def psm3_cartesian_callback(self, data):
		self.psm3_current = pm.fromMsg(data.pose)
	def mtml_cartesian_callback(self, data):
		self.mtml_current = pm.fromMsg(data.pose)
	def mtmr_cartesian_callback(self, data):
		self.mtmr_current = pm.fromMsg(data.pose)
	
		
	###############
	## PEDALS #####
	###############
	def clutch_callback(self, data):
		if self.is_recording_active:
			if  data.buttons[0]:
				self.clutch_file.write("{:0.04f},pressed\n".format(time.time()))
			else:
				self.clutch_file.write("{:0.04f},released\n".format(time.time()))
				
	def teleop_events_callback(self, data):
		if self.is_recording_active:
			#Header --> "ts, key, value,\n"
			self.telop_events_file.write("{:0.04f},{:},{:}\n".format(time.time(), data.key,data.value))

	def bicoag_callback(self, data):
		if  data.buttons[0]:
			pass
			# if self.is_recording_active:
			# 	print("Stop recording")
			# 	self.recording_counter += 1
			# 	self.is_recording_active = False
			# 	self.close_files()
			# else:
			# 	print("recording counter", self.recording_counter)
			# 	print("Start recording")
			# 	self.create_files()
			# 	self.is_recording_active = True
			
	def recording_callback(self,data):
		self.is_recording_active = data.data

	def create_files(self,):
		print('files created at {:}'.format(self.dst_path))
		#Create txt
		for key in self.arms_to_record:
			self.arms_files_dict[key] = open(join(self.dst_path,"{:02d}_arms_{:}_cartesian.txt".format(self.recording_counter,key)),"w")
			self.arms_files_dict[key].write("frame_idx,ts,x,y,z,rx,ry,rz,rw\n")

		self.clutch_file = open(join(self.dst_path,"{:02d}_clutch_events.txt".format(self.recording_counter)),"w")
		self.clutch_file.write("ts,state\n")
		self.video_ts_file = open(join(self.dst_path,"{:02d}_video_ts.txt".format(self.recording_counter)),"w")
		self.video_ts_file.write("frame_idx,ts\n")

		self.telop_events_file = open(join(self.dst_path,"{:02d}_teleop_events.txt".format(self.recording_counter)),"w")
		self.telop_events_file.write("ts,key,value\n")

		#Create video
		fourcc = cv2.VideoWriter_fourcc(*'XVID')
		self.video_out_right = cv2.VideoWriter(join(self.dst_path,"{:02d}_video_right_color.avi".format(self.recording_counter)),fourcc, 30.0, (640,480))

	def save_information(self,):
		ts_str = "{:d},{:0.4f}".format(self.right_frame_id,time.time())
		self.arms_files_dict['psm1'].write(ts_str+self.pykdl2str(self.psm1_current))
		self.arms_files_dict['psm2'].write(ts_str+self.pykdl2str(self.psm2_current))
		self.arms_files_dict['psm3'].write(ts_str+self.pykdl2str(self.psm3_current))
		self.arms_files_dict['mtml'].write(ts_str+self.pykdl2str(self.mtml_current))
		self.arms_files_dict['mtmr'].write(ts_str+self.pykdl2str(self.mtmr_current))

		self.video_ts_file.write(ts_str+"\n")
		self.video_out_right.write(self.right_frame)

	def close_files(self):
		for key in self.arms_to_record:
			self.arms_files_dict[key].close()

		self.clutch_file.close()
		self.video_ts_file.close()
		self.video_out_right.release()


	def pykdl2str(self, frame):
		(rx, ry, rz, rw) = frame.M.GetQuaternion()
		message1 = ",{: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}\n".format(frame.p.x(), frame.p.y(), frame.p.z(), rx,ry,rz,rw)
		return message1

def createTimeStamp():
		ts = time.time()
		return datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%Hh.%Mm.%Ss_')

def main():
	import argparse
	parser = argparse.ArgumentParser()
	parser.add_argument('-d', '--dir_name',required=True, help='name of directory, format "Name_Session"')
	args = parser.parse_args()

	rospy.init_node('recording_cameras')

	ts = createTimeStamp()
	cm = recording_module(dst_path = ts+args.dir_name)

	#Sleep until the subscribers are ready.
	time.sleep(0.20)
	cm.is_recording_active = True

	try:
		while not rospy.core.is_shutdown():
			rospy.rostime.wallsleep(0.25)
	except KeyboardInterrupt:
		print("Shutting down")
	finally:
		# cm.close_files()
		print("Shutting down")


if __name__ == '__main__':
	main()