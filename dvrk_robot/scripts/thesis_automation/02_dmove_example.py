#!/usr/bin/env python
import dvrk
import time
import math
import PyKDL
from PyKDL import Vector, Rotation


def pykdl_frame2str(frame):
		(rx, ry, rz, rw) = frame.M.GetQuaternion()

		
		message1 = "Position (x,y,z): {: 0.8f}, {: 0.8f}, {: 0.8f}  Orientation(rx,ry,rz,rw): {: 0.8f}, {: 0.8f}, {: 0.8f}, {: 0.8f}"\
								.format(frame.p.x(), frame.p.y(), frame.p.z(), rx,ry,rz,rw)
		return message1


if __name__ == "__main__":

	sleep_time = 1
	
	psm3 = dvrk.psm('PSM3')
	time.sleep(0.3)
	psm3.home()

	current_pose = psm3.get_current_position()
	initial_orientation = current_pose.M
	print("Psm3 current position")
	print(pykdl_frame2str(current_pose))

	answer = raw_input("position the psm3 in a safe place and write 'Y' to continue. ")
	if answer != 'Y':
		print("Exiting the program")
		exit(0)
	else:
		print("Start moving arm 5 times")
		for i in range(2):
			print("Movement {:d}".format(i))

			#Incremental Cartesian movements
			# print("Cartesian Movement")
			# psm3.dmove(PyKDL.Vector(0.0, 0.035, 0.0)) # 5 cm in Y direction 
			# time.sleep(sleep_time)
			# psm3.dmove(PyKDL.Vector(0.0, -0.035, 0.0)) # 5 cm in Y direction 
			# time.sleep(sleep_time)

			# #Absolute Cartesian movements
			# print("Cartesian Movement")
			# psm3.move(PyKDL.Vector(0.22385906, -0.11899318,  0.09143069)) 
			# time.sleep(sleep_time)
			# psm3.move(PyKDL.Vector(0.21433946, -0.15271287,  0.04622113)) 
			# time.sleep(sleep_time)		
			
			# # Rotation with Quaternions
			# print("Rotation Movements")
			# # psm3.move(PyKDL.Vector(0.21433946, -0.15271287,  0.04622113)) 
			# # time.sleep(sleep_time)
			# print(pykdl_frame2str(psm3.get_current_position()))
			# psm3.move(PyKDL.Rotation.Quaternion(0.33748458, -0.22095775,  0.54350179,  0.73613018))
			# time.sleep(sleep_time)
			# psm3.move(PyKDL.Rotation.Quaternion(0.65754307, -0.45546080,  0.40695337,  0.44111396))
			# time.sleep(sleep_time)
			# print(pykdl_frame2str(psm3.get_current_position()))

			## Movements with full frames (position + orientation)
			# print("Full movement (position + orientation)")
			# fix_orientation = Rotation.Quaternion(0.26282424, -0.12377510,  0.49116142,  0.82119644,)
			# #High point
			# psm3.move(PyKDL.Frame(fix_orientation, Vector(0.20207338, -0.08238340,  0.04522971))) 
			# time.sleep(sleep_time)

			# #Point 1
			# psm3.move(PyKDL.Frame(fix_orientation, Vector(0.25880920, -0.12602521,  0.09615081))) 
			# time.sleep(sleep_time)

			# #High point
			# psm3.move(PyKDL.Frame(fix_orientation, Vector(0.20207338, -0.08238340,  0.04522971))) 
			# time.sleep(sleep_time)

			# #Point 2
			# psm3.move(PyKDL.Frame(fix_orientation, Vector(0.16551262, -0.17279799,  0.05455973))) 
			# time.sleep(sleep_time)

			# #High point
			# psm3.move(PyKDL.Frame(fix_orientation, Vector(0.20207338, -0.08238340,  0.04522971))) 
			# time.sleep(sleep_time)

			# #Point 3
			# psm3.move(PyKDL.Frame(fix_orientation, Vector(0.25669699, -0.17719280,  0.05745484))) 
			# time.sleep(sleep_time)

			# #High point
			# psm3.move(PyKDL.Frame(fix_orientation, Vector(0.20207338, -0.08238340,  0.04522971))) 
			# time.sleep(sleep_time)


			## Movements with full frames (position + orientation)
			fix_orientation = Rotation.Quaternion(0.26282424, -0.12377510,  0.49116142,  0.82119644,)
			#Point 1
                        psm3.move(PyKDL.Frame(fix_orientation, Vector(0.22415668, -0.16345158,  0.04202124))) 
			time.sleep(sleep_time)
			# #Point 2
			# psm3.move(PyKDL.Frame(fix_orientation, Vector(0.21758554, -0.16898800,  0.04562245))) 
			# time.sleep(sleep_time)
			# #Point 3
			# psm3.move(PyKDL.Frame(fix_orientation, Vector(0.20161770, -0.09351339,  0.05061632))) 
			# time.sleep(sleep_time)
					

#dmove and Rotation movements didn't seem to work very well 

# print("Rotation Movements")
# psm3.move(PyKDL.Vector(0.22385906, -0.11899318,  0.09143069)) 
# time.sleep(sleep_time)
# print(pykdl_frame2str(psm3.get_current_position()))
# r = PyKDL.Rotation()
# r.DoRotY(math.pi * 0.1)
# psm3.dmove(r)
# time.sleep(sleep_time)
# psm3.move(initial_orientation)
# time.sleep(sleep_time)
# print(pykdl_frame2str(psm3.get_current_position()))
# print(r)
