#!/usr/bin/env python
import dvrk
import time
import math
import PyKDL


if __name__ == "__main__":

	sleep_time = 2
	
	psm3 = dvrk.psm('PSM3')
	time.sleep(0.2)
	psm3.home()

	answer = raw_input("position the psm3 in a safe place and write 'Y' to continue. ")
	if answer != 'Y':
		print("Exiting the program")
		exit(0)
	else:
		print("Start moving arm 5 times")
		for i in range(3):
			print("Movement {:d}".format(i))

			#Incremental Cartesian movements
			# print("Cartesian Movement")
			# psm3.dmove(PyKDL.Vector(0.0, 0.035, 0.0)) # 5 cm in Y direction 
			# time.sleep(sleep_time)
			# psm3.dmove(PyKDL.Vector(0.0, -0.035, 0.0)) # 5 cm in Y direction 
			# time.sleep(sleep_time)

			#Absolute Cartesian movements
			print("Cartesian Movement")
			psm3.move(PyKDL.Vector(0.21134809, -0.12155905,  0.10276267)) 
			time.sleep(sleep_time)
			psm3.move(PyKDL.Vector(0.26790356, -0.12201603,  0.10604522)) 
			time.sleep(sleep_time)		
			psm3.move(PyKDL.Vector(0.27185202, -0.14776761,  0.09245916)) 
			time.sleep(sleep_time)
			psm3.move(PyKDL.Vector(0.21329086, -0.14354401,  0.08516887)) 
			time.sleep(sleep_time)			

#Rotation movements didn't seem to work very well 

# print("Rotation Movements")
# old_pose = psm3.get_current_position()
# old_orientation = old_pose.M

# print(old_pose)
# r = PyKDL.Rotation()
# r.DoRotX(math.pi * 0.1)
# psm3.dmove(r)
# time.sleep(sleep_time)
# psm3.move(old_orientation)
# time.sleep(sleep_time)