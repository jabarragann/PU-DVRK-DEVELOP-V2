#!/usr/bin/env python

import numpy as np 

import cv2

def read_file(file):

	data = open(file,'r')
	data = data.readlines()[1:]

	pixel_coord = []
	robot_coord = []


	for line in data:
		
		line = line.strip().split(";")
		print(line)
		line = list(map(lambda x:x.replace("(","").replace(")",""), line))

		robot_coord.append(list(map(float,line[1].split(",")))[:-1] )
		pixel_coord.append(list(map(float,line[2].split(","))))
		
		# print(line[1].split(","))
	

	return np.array(robot_coord), np.array(pixel_coord)

if __name__ == "__main__":

	print("calculating homography")

	robot_coord, pixel_coord = read_file("./homography_idea_data1.txt")

	print(robot_coord)
	print(pixel_coord)

	h, status = cv2.findHomography(pixel_coord, robot_coord) #(src,dst)

	print(h)

	#Test homography 
	pixel_coord_h = np.ones((7,3))
	pixel_coord_h[:,:2] = pixel_coord

	new_robot_coord = np.dot(h,  pixel_coord_h.T).T

	new_robot_coord = new_robot_coord / new_robot_coord[:,2].reshape((-1,1))
	print(new_robot_coord)
	print(new_robot_coord.shape)

	print(new_robot_coord[:,:2]- robot_coord)


	#Test on second data
	print("test on data 2")
	robot_coord2, pixel_coord2 =  read_file("./homography_idea_data2.txt")
	pixel_coord_h2 = np.ones((3,3))
	pixel_coord_h2[:,:2] = pixel_coord2
	new_robot_coord2 = np.dot(h,  pixel_coord_h2.T).T
	new_robot_coord2 = new_robot_coord2 / new_robot_coord2[:,2].reshape((-1,1))
	print(new_robot_coord2[:,:2]- robot_coord2)


	#Transform specific point
	print("\nspecific point ")
	pix = np.array([[204.0, 188.0,1.0]]).reshape((3,1))
	coord = np.dot(h,pix).T
	coord = coord / coord[:,2]
	print(coord)