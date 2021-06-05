#!/usr/bin/env python

import numpy as np 
import cv2
import json 

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
	np.random.seed(5)
	print("calculating homography")

	robot_coord, pixel_coord = read_file("./homography_data/homography_idea_data.txt")
	print(robot_coord.shape, pixel_coord.shape)

	idx = np.random.permutation(6)

	robot_coord1, pixel_coord1 = robot_coord[idx[:-1]],pixel_coord[idx[:-1]]
	robot_coord2, pixel_coord2 = robot_coord[idx[-1:]],pixel_coord[idx[-1:]]

	print(idx)
	print("training")
	print(pixel_coord1)
	print("testing")
	print(pixel_coord2)

	h, status = cv2.findHomography(pixel_coord, robot_coord) #(src,dst)
	print("Homography")
	print(h)

	# #Test homography 
	print("Test homography on the training data. difference between projected and real")
	pixel_coord_h1 = np.ones((pixel_coord1.shape[0],3))
	pixel_coord_h1[:,:2] = pixel_coord1

	new_robot_coord = np.dot(h,  pixel_coord_h1.T).T

	new_robot_coord = new_robot_coord / new_robot_coord[:,2].reshape((-1,1))
	# print(new_robot_coord)
	# print(new_robot_coord.shape)

	print(new_robot_coord[:,:2]- robot_coord1)


	#Test on second data
	print("test on testing data. difference between projected and real")
	# robot_coord2, pixel_coord2 =  read_file("./homography_idea_data2.txt")
	pixel_coord_h2 = np.ones((pixel_coord2.shape[0],3))
	pixel_coord_h2[:,:2] = pixel_coord2
	new_robot_coord2 = np.dot(h,  pixel_coord_h2.T).T
	new_robot_coord2 = new_robot_coord2 / new_robot_coord2[:,2].reshape((-1,1))
	print(new_robot_coord2[:,:2]- robot_coord2)


	#Transform specific point
	print("\nspecific point ")
	pix = np.array([[218.0, 415.0,1.0]]).reshape((3,1))
	coord = np.dot(h,pix).T
	coord = coord / coord[:,2]
	print(coord)

	#Save homography matrix

	lists = h.tolist()
	json_str = json.dumps(lists, indent=4,)

	with open("./homography_data/homography.json","w") as f1:
		f1.write(json_str)

