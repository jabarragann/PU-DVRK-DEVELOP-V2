#!/usr/bin/env python

#segmentation libraries
from fcn import VGGNet, FCNs
import segmentation_utils as utils
import pickle
import torch
from torch import nn
#Ros libraries
import rospy
from cv_bridge import CvBridge, CvBridgeError
import dvrk

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


class CameraSubscriber:

	def __init__(self,):
		rospy.init_node('camera_display', anonymous=True)
		self.bridge = CvBridge()
		self.right_frame = None
		self.camera_subs = rospy.Subscriber("/pu_dvrk_cam/right/inverted", Image, self.camera_callback)		
		#/pu_dvrk_cam/modified_display_right
		self.wind_name = "img"
		# cv2.namedWindow(self.wind_name)
		# cv2.setMouseCallback(self.wind_name,self.get_coordinates)

		#publisher
		self.multi_arr_pub = rospy.Publisher("/pu_dvrk_tf/centroid_coordinates",Int32MultiArray,queue_size=5)
		
	
	def camera_callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			self.right_frame = cv_image
		except CvBridgeError as e:
			print(e)
		
		# print(data.width)
		# cv2.imshow("d", cv_image)
		# cv2.waitKey(1)

	def get_coordinates(self, event,x,y,flags,param):
		if event == cv2.EVENT_LBUTTONDOWN:
		    print(x,y)

def create_model(path):
	n_class =5 
	python2_state_dict = pickle.load(open(path,"rb"))

	vgg_model = VGGNet(requires_grad=True, remove_fc=True)
	model = FCNs(pretrained_net=vgg_model, n_class=n_class)
	model = nn.DataParallel(model, device_ids=[0])
	model.load_state_dict(python2_state_dict)

	return model

def main():
	cam = CameraSubscriber()
	
	best_params_path =  "model/model.pickle"
	model = create_model(best_params_path)
	labels_path = "model/label_colors.json"
	labels_dict = utils.labels_colors_dicts(labels_path)

	#Sleep until the subscribers are ready.
	time.sleep(0.40)

	# frame = cam.right_frame
	# cv2.imshow("d", frame)
	# cv2.waitKey(0)
	count = 0 
	try:
		while not rospy.core.is_shutdown():
			# rospy.rostime.wallsleep(0.25)
			frame = cam.right_frame
			final_frame, mask = utils.test_img(frame,model, labels_dict)
			print("prediction {:}".format(count))
			count += 1
			
			# cv2.imshow("img", final_frame)
			# k = cv2.waitKey(0) & 0xFF

			
			# if k == 27:
			# 	cv2.imwrite('./masks/mask1.jpg', mask)
			# 	break

			centroids = utils.calculate_centroids(mask)
			# print(centroids)
			centroids = centroids.reshape(-1).tolist()
			arr_to_send = Int32MultiArray()
			arr_to_send.data = centroids
			cam.multi_arr_pub.publish(arr_to_send)
	
			break
			# time.sleep(4)
			# cv2.waitKey(1)

	except KeyboardInterrupt:
		
		print("Shutting down")
		cv2.destroyAllWindows()


if __name__ == "__main__":
	main()