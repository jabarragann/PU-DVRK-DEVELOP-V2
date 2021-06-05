#!/usr/bin/env python
import dvrk
import numpy as np
import time


#DVRK interface does not allow to instantiate multiple arms at the same time.


# # Move PSM3 to the initial position
psm3 = dvrk.psm('PSM3')
psm3_joint_pos =[0.22989031, -0.39741956,  0.11103761, -0.22835235, -0.01284383, -0.24452774]
print('psm3 current')
print(psm3.get_current_joint_position())
print('psm3 next')
print(psm3_joint_pos)

psm3.move_joint(np.array([psm3_joint_pos]))
