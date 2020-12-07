#!/usr/bin/env python

import dvrk
import PyKDL
import rospy
import numpy as np
import time
import matplotlib.pyplot as plt
from std_msgs.msg import Float64



def generate_trajectory(init_pt, end_pt, n=20):
	init_pt = init_pt.reshape(3,1)
	end_pt = end_pt.reshape(3,1)
	t = np.linspace(0, 1, n).reshape(1,n)
	pt_on_line = (end_pt - init_pt) * t + init_pt # Shape (3,n) where each column represent a different point on the line.

	return pt_on_line

def main():
	
	sleep_time = 1
	psm3_arm = dvrk.arm('PSM3')
	psm3_arm.home()


	# #Set up the velocity and acceleration ratio of the psm3
	# ratio = 0.01
	# velocity_ratio_pub = rospy.Publisher("/dvrk/PSM3/set_joint_velocity_ratio",Float64,latch=True, queue_size=1)
	# acceleration_ratio_pub = rospy.Publisher("/dvrk/PSM3/set_joint_acceleration_ratio",Float64,latch=True, queue_size=1)	
	# velocity_ratio_pub.publish(Float64(ratio))
	# acceleration_ratio_pub.publish(Float64(ratio)) 

	answer = raw_input("Make sure the PSM3 will not crash with the programmed movements and write 'Y' to continue. ")
	if answer != 'Y':
		print("Exiting the program")
		exit(0)

	
	for i in range(2):	
		square_trajectory = [[[0.21134809, -0.12155905,  0.10276267], [0.26790356, -0.12201603,  0.10604522]],
							 [[0.26790356, -0.12201603,  0.10604522], [0.21134809, -0.12155905,  0.10276267]],
							 [[0.21134809, -0.12155905,  0.10276267], [0.27185202, -0.14776761,  0.09245916]],
							 [[0.27185202, -0.14776761,  0.09245916], [0.21329086, -0.14354401,  0.08516887]],
							 [[0.21329086, -0.14354401,  0.08516887], [0.21134809, -0.12155905,  0.10276267]] ]
		
		for init_pt, end_pt in square_trajectory:
			init_pt = np.array(init_pt)
			end_pt = np.array(end_pt)

			trajectory = generate_trajectory(init_pt, end_pt, n=2)

			for idx, c in enumerate(range(trajectory.shape[1])):
				next_position = PyKDL.Vector(trajectory[0,c], trajectory[1,c], trajectory[2,c],)
				print("position {:}".format(idx), next_position)
				
				psm3_arm.move(PyKDL.Vector(next_position)) 
			# time.sleep(sleep_time)


if __name__ == "__main__":    
	rospy.init_node('recording_node')
	time.sleep(0.2)
	main()
	
