#!/usr/bin/env python

#Real-time Modules
import real_time_modules.counting_module as count_module
import real_time_modules.normal_inversion_module as normal_inversion_module
#Ros Modules
import rospy
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
from os import mkdir, makedirs
from os.path import join, exists
from itertools import chain
import re

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
	flip = False

	#Validate parameters
	if task not in ['peg', 'suture','knot']:
		print("Task needs to be either the word suture, peg or knot")
		exit()
	if condition not in ['count', 'normal','nback','inversion', 'flip']:
		print("Condition needs to be either 'count', 'normal','nback','inversion', or 'flip'")
		exit()
	try: 
		int(trial) # Check if trial is an integer
	except ValueError:
		print("Trial needs to be a integer")
		exit()

	if condition == 'inversion':
		invert_img = True
	elif condition == 'flip':
		flip=True


	#create path
	#If path already exists and the user wants to recollect data, the orginal path will renamed to old_n
	task_condition = task + "_" + condition
	message = task_condition + "_" + trial 

	dst_path = "/home/isat/juanantonio/da_vinci_video_recordings/realtime_project_experiments/"
	dst_path = join(dst_path , user_id + "/" + session + "/" )
	#Create session folder
	if not exists(dst_path):
		makedirs(dst_path )

	#Check if the trial folder hasn't been created
	dst_path_cp = dst_path
	dst_path = dst_path_cp + "T{:02d}".format(int(trial)) + "_{:}/".format(task_condition)

	dirs = os.listdir(dst_path_cp)
	trialAlreadyTaken = any([len(re.findall('T{:02d}'.format(int(trial)), di)) for di in dirs])

	if not trialAlreadyTaken:
		makedirs(dst_path )
	else:
		print("path: ", dst_path)
		print("Already exists")

		x = raw_input("Do you want to retake data from trial {:} again ({:})? (Y/n)\n".format(trial, task_condition))
		

		if x =='Y':
			old_count = 1
			new_dst_path = dst_path_cp +  "T{:02d}_old{:02d}".format(int(trial),int(old_count))

			dirs = os.listdir(dst_path_cp)
			re_pattern = "T{:02d}_old{:02d}".format(int(trial),int(old_count))
			trialAlreadyTaken = any([len(re.findall(re_pattern, di)) for di in dirs])

			while trialAlreadyTaken:
				old_count += 1
				
				re_pattern = "T{:02d}_old{:02d}".format(int(trial),int(old_count))
				trialAlreadyTaken = any([len(re.findall(re_pattern, di)) for di in dirs]) 

			new_dst_path = dst_path_cp + "T{:02d}_old{:02d}".format(int(trial),int(old_count))

			re.findall('T{:02d}(<)'.format(int(trial)), di)
			path_to_rename = [di for di in dirs if len(re.findall('T{:02d}(?!_old)'.format(int(trial)), di))][0]
			# print(path_to_rename, exists(path_to_rename), new_dst_path + path_to_rename[3:])
			
			os.rename(dst_path_cp + path_to_rename, new_dst_path + path_to_rename[3:]) 
			makedirs(dst_path)

		else:
			exit()


	print("Starting Da vinci video Operation...")
	if condition == 'count':
		cm = count_module.collection_module(trial = trial, dst_path = dst_path, rig_name=rig_name, message = message, invert_img= invert_img, flip=flip)
	else:
		cm = normal_inversion_module.collection_module(trial = trial, dst_path = dst_path, rig_name=rig_name, message = message, invert_img= invert_img, flip=flip)

	#Sleep until the subscribers are ready.
	time.sleep(0.20)

	try:
		while not rospy.core.is_shutdown():
			cm.update()
			rospy.rostime.wallsleep(1.0)

	except KeyboardInterrupt:
		print("Shutting down")

	finally:
		cm.close_files()
		print("Shutting down")


if __name__ == '__main__':

	rospy.init_node('recording_node')
	main()