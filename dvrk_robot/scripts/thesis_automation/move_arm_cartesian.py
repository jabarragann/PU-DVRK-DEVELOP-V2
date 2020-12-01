#!/usr/bin/env python


import dvrk
import numpy as np
import time


#DVRK interface does not allow to instantiate multiple arms at the same time.


# Move PSM1 to the initial position
mtml = dvrk.arm('MTML')
mtml.home()
mtml_joint_pos = [  0.2342934,   0.25712809, -0.13925924, -1.69204613,  1.72013096,  0.11842554, -1.02438089]

print('mtml current')
print(mtml.get_current_joint_position())
print('mtml next')
print(mtml_joint_pos)

mtml.move_joint(np.array([mtml_joint_pos]))
