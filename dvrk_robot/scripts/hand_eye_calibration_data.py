#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import cv2 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import datetime
import numpy as np 
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Joy
from std_msgs.msg import String as rosString
import random 
import argparse
import json
import socket
import os


class secondary_task_module:

	def __init__(self,totalTime =1, secondaryTime = 5, file= None, videoFileName = None, userId = None, trialId= None, dst_path = None, rig_name=None):

		#Important variables
		self.initTime = time.time()
		self.secondaryTime = secondaryTime
		self.turnSecondaryTask = True
		self.lastActivation = -1
		self.startProcedure = False
		self.stopProcedure = False
		self.totalTime = totalTime*60 #In seconds
		self.client = None
		self.maxPossibleScore = (secondaryTime % 10)*2 * totalTime/2
		self.score = 0
		self.correct = 0
		self.incorrect = 0 

		#Initialize recording variables
		self.recording = False
		self.dst_path = dst_path + '/' + createTimeStamp() + userId 
		os.mkdir(self.dst_path)


		self.left_recording_name = ""
		self.right_recording_name = ""
		self.left_video = None
		self.right_video = None

		self.is_recording_time = False
		self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
		self.frame = None

		#Create file to write timestamp of each frame
		self.frames_time_file = open(dst_path + "/test.avi"+".txt",'w')
		self.frame_counter = 0
		self.frames_time_file.write("frame,timestamp\n")
		
		#New Published topics
		self.image_pub1 = rospy.Publisher("/"+rig_name+"/modified_display_left",Image, queue_size=5)
		self.image_pub2 = rospy.Publisher("/"+rig_name+"/modified_display_right",Image, queue_size=5)
		self.score_pub = rospy.Publisher("score_correctly", Joy, queue_size=5)
		
		self.image_pub1_compressed = rospy.Publisher("/"+rig_name+"/modified_display_left/compressed" ,CompressedImage, queue_size=5)
		self.image_pub2_compressed = rospy.Publisher("/"+rig_name+"/modified_display_right/compressed",CompressedImage, queue_size=5)
		self.bridge = CvBridge()
		
		#Subscribed topics
		#Topic to Record Video
		self.camera_left_stream_subs = rospy.Subscriber("/"+rig_name+"/modified_display_left/", Image, self.left_video_recording_callback,  queue_size = 1)
		self.camera_right_stream_subs = rospy.Subscriber("/"+rig_name+"/modified_display_right/", Image, self.right_video_recording_callback,  queue_size = 1)

		self.image_sub_left  = rospy.Subscriber("/"+rig_name+"/left/inverted", Image, self.left_callback)
		self.image_sub_right = rospy.Subscriber("/"+rig_name+"/right/inverted", Image, self.right_callback)
		self.camera_pedal_sub = rospy.Subscriber("/dvrk/footpedals/bicoag", Joy, self.pedal_callback)
		self.score_sub = rospy.Subscriber("score_correctly",Joy, self.score_callback)

		self.misalignment = 75
		self.fontSize = 1.2
		self.message =  ""
		self.timerStr = ""
		self.scoreStr = ""
		self.alpha = 0.8
		self.numberOfTargets = 2
		self.target = random.sample(range(min(secondaryTime,10)), self.numberOfTargets)


		#Blink a green/red rectangle on screen to indicate the user the secondary task is starting
		self.notifyUser= False
		self.notificationColor = (0,0,0)

		#Kernel used to blurr images when secondary task is active
		self.smoothingKernel = np.ones((5,5),np.float32)/25

		#File to write timestamps
		self.file = file


	def update(self):
		
		if self.startProcedure and not self.stopProcedure:
			currentTime = time.time()
			secondsCounter = int((currentTime - self.initTime))
			seconds = secondsCounter % 60
			minutes = int(secondsCounter / 60)
			self.timerStr = "Timer: {:02d}:{:02d}".format(minutes,seconds)

			#Check if procedure is already over.
			if time.time() - self.initTime > self.totalTime:
				secondaryTaskStatus = "finished"

				#Writing to file
				self.file.write("{:.9f} {}\n".format(time.time(), secondaryTaskStatus))
				self.file.write("##DATA##\n")
				self.file.write("{} {}\n".format("Score", self.score))
				self.file.write("{} {:.3f}\n".format("Max possible Score", float(self.maxPossibleScore) ))
				#self.file.write("{} {:.3f}\n".format("Accuracy", float(self.score/self.maxPossibleScore) ))
				self.file.write("{} {}\n".format("Incorrect", self.incorrect))
				self.file.write("{} {}\n".format("Correct", self.correct))
				self.file.flush()

				self.stopProcedure = True
				self.alpha = 0.9

				#Stop recording
				self.is_recording_time = False
				self.out.release()

				self.message = "Procedure finished"
			#If the procedure have not finished, check if the status of the secondary task have to change
			elif secondsCounter % self.secondaryTime == 0:
				if secondsCounter != self.lastActivation:
					self.turnSecondaryTask = not self.turnSecondaryTask
					self.target = random.sample(range(1,min(self.secondaryTime,10)), self.numberOfTargets)
					
					self.lastActivation = secondsCounter
					secondaryTaskStatus = "active" if self.turnSecondaryTask else "not_active"

					#Writing to file
					self.file.write("{:.9f} {}\n".format(time.time(), secondaryTaskStatus))
					self.file.flush()

					if self.turnSecondaryTask:
						temp = " ".join(map(str,self.target))
						self.message  = "Do secondary, Targets: {:s}".format(temp)
						self.scoreStr = "Score: {:3d}".format(self.score)
						self.alpha = 0.20
						
					else:
						self.message  = "Do only primary task"
						self.scoreStr = "Score: {:3d}".format(self.score)
						self.alpha = 0.20
					

	def modifyImageAndPublish(self,cv_image, misalignment=0, publisherId=1):

		publisher = self.image_pub1 if publisherId == 1 else self.image_pub2
		compressedPublisher = self.image_pub1_compressed if publisherId == 1 else self.image_pub2_compressed
		
		
		if self.recording:
			color = (0,255,0)
		else:
			color = (0,0,255)

		#Modify Image
		if True:
			#########################
			#Add recording indicator#
			#########################
			overlay = cv_image.copy()
						
			cv2.rectangle(cv_image, (600, 0), (640,40), color, -1)
			cv2.addWeighted(overlay, self.alpha, cv_image, 1 - self.alpha, 0, cv_image)

			############################
			##Add chess board corners###
			############################
			gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
			# Find the chess board corners
			ret, corners = cv2.findChessboardCorners(gray, (8,6),None)
			# If found, add object points, image points (after refining them)
			if ret == True:
				criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

				# self.objpoints.append(objp) # Object points
				corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
				# imgpoints.append(corners2)
				# Draw and display the corners
				cv_image = cv2.drawChessboardCorners(cv_image, (8,6), corners2,ret)

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
		except CvBridgeError as e:
			print(e)

		self.modifyImageAndPublish(cv_image, misalignment=0, publisherId=1)

	def right_callback(self,data):

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		self.modifyImageAndPublish(cv_image, misalignment=self.misalignment, publisherId=2)
	
	def pedal_callback(self,data):
		
		if data.buttons[0]: 
			#If recording, stop recording
			if self.recording:
				print("Stop recording")
				self.recording = False

				time.sleep(0.2)
				self.left_video.release()
				self.right_video.release()
			#If not recording start recording
			else:
				print("Start recording")
				
				timestamp =  createTimeStamp()
				self.left_video = cv2.VideoWriter(self.dst_path + "/" + timestamp + "_left.avi", self.fourcc, 30.0, (640,480))
				self.right_video = cv2.VideoWriter(self.dst_path + "/" + timestamp + "_right.avi", self.fourcc, 30.0, (640,480))

				self.frames_time_file_left  = open(self.dst_path + "/" + timestamp + "_left_timestamps.txt","w")
				self.frames_time_file_right = open(self.dst_path + "/" + timestamp + "_right_timestamps.txt","w")
				self.frame_counter_left = 0
				self.frame_counter_right = 0

				self.recording = True
				

	def score_callback(self,data):
		
		self.notificationColor = (0,255,0) if data.header.frame_id == "right" else (0,0,255)
		
		if data.buttons[0]:
			self.notifyUser = True
			time.sleep(0.4)
			self.notifyUser = False

	def left_video_recording_callback(self, videoFrame):
		tempFrame = None
		cv_image = self.bridge.imgmsg_to_cv2(videoFrame,"bgr8")
		if self.recording:	
			
			self.left_video.write(cv_image)

			self.frame_counter_left += 1
			self.frames_time_file_left.write("{:d},{:}\n".format(self.frame_counter_left,str(rospy.Time.now())))
	
	def right_video_recording_callback(self, videoFrame):
		tempFrame = None
		cv_image = self.bridge.imgmsg_to_cv2(videoFrame,"bgr8")
		if self.recording:	
			
			self.right_video.write(cv_image)

			self.frame_counter_right += 1
			self.frames_time_file_right.write("{:d},{:}\n".format(self.frame_counter_right,str(rospy.Time.now())))

def main():

	#Get rospy parameters
	if rospy.has_param('/recording_node/subject_id') and  rospy.has_param('/recording_node/rig_name'):
		subject_id = rospy.get_param('/recording_node/subject_id')
		rig_name   = rospy.get_param('/recording_node/rig_name')
	else:
		print("Subject Id or trial was not specified")
		subject_id = "test"
		rig_name = "default_cam"

	dst_path = "/home/isat/juanantonio/da_vinci_video_recordings/hand-eye_calibration"

	print("Starting Da vinci video Operation...")
	
	ic = secondary_task_module(file=file, secondaryTime = 30, totalTime = 6, userId = subject_id, dst_path = dst_path, rig_name=rig_name)
	
	#Sleep until the subscribers are ready.
	time.sleep(0.10)
	
	try:
		while not rospy.core.is_shutdown():
			# ic.update()
			rospy.rostime.wallsleep(0.25)

	except KeyboardInterrupt:
		print("Shutting down")

	finally:
		print("Shutting down")


def createTimeStamp():
	ts = time.time()
	return datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%Hh.%Mm.%Ss_')


if __name__ == '__main__':
	
	rospy.init_node('recording_node')
	main()