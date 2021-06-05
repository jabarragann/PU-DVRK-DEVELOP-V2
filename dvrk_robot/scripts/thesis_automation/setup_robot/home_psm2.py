#!/usr/bin/env python
import dvrk
import numpy as np
import time


#DVRK interface does not allow to instantiate multiple arms at the same time.


# # Move PSM2 to the initial position
psm2 = dvrk.psm('PSM2')
psm2_joint_pos =[-0.96013649, -0.26240302,  0.0786874,  -0.18452715, -0.01052101,  0.89814234] 
print('psm2 current')
print(psm2.get_current_joint_position())
print('psm2 next')
print(psm2_joint_pos)

psm2.move_joint(np.array([psm2_joint_pos]))
