#!/usr/bin/env python
import dvrk
import numpy as np
import time


#DVRK interface does not allow to instantiate multiple arms at the same time.


# # Move PSM2 to the initial position
psm2 = dvrk.psm('PSM2')
psm2_joint_pos = [-1.06577575, -0.14864716,  0.10963891,  3.2256604,   0.26739763, -1.1241071 ]
print('psm2 current')
print(psm2.get_current_joint_position())
print('psm2 next')
print(psm2_joint_pos)

psm2.move_joint(np.array([psm2_joint_pos]))
