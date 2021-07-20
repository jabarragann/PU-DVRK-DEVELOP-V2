#!/usr/bin/env python
import dvrk
import numpy as np
import time


#DVRK interface does not allow to instantiate multiple arms at the same time.


# Move PSM1 to the initial position
mtmr = dvrk.arm('MTMR')
mtmr.home()
mtmr_joint_pos =[ 0.03027136,  0.04115939, -0.08394668,  1.88627917,  1.39150009,  0.08585852, 4.84360442] 

print('mtmr current')
print(mtmr.get_current_joint_position())
print('mtmr next')
print(mtmr_joint_pos)

mtmr.move_joint(np.array([mtmr_joint_pos]))

