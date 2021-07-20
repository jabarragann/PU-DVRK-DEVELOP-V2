#!/usr/bin/env python
import dvrk
import numpy as np
import time


#DVRK interface does not allow to instantiate multiple arms at the same time.


# Move PSM1 to the initial position
mtmr = dvrk.arm('MTMR')
mtmr.home()
mtmr_joint_pos =[-0.03240175,  0.15021445, -0.13135859,  1.82255991,  1.34412987,  0.10066171, 4.80807676]

print('mtmr current')
print(mtmr.get_current_joint_position())
print('mtmr next')
print(mtmr_joint_pos)

#mtmr.move_joint(np.array([mtmr_joint_pos]))

