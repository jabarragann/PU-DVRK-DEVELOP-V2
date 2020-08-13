#!/usr/bin/env python
import dvrk
import numpy as np
import time


#DVRK interface does not allow to instantiate multiple arms at the same time.


# Move PSM1 to the initial position
mtmr = dvrk.arm('MTMR')
mtmr.home()
mtmr_joint_pos = [-0.14427202,  0.18954733, -0.15922543,  1.67041871,  1.68164266,  0.03256702, 0.95332556]

print('mtmr current')
print(mtmr.get_current_joint_position())
print('mtmr next')
print(mtmr_joint_pos)

mtmr.move_joint(np.array([mtmr_joint_pos]))

