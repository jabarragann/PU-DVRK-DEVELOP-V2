#!/usr/bin/env python
import dvrk
import numpy as np

# Change the camera position to the desired one:
ecm = dvrk.ecm('ECM')
ecm_joint_pos = [-0.39735257, -0.38272453,  0.14414322,  0.36241095]
#[-0.20915153,  0.13093896,  0.12079741,  0.03336274]

print('ecm current')
print(ecm.get_current_joint_position())
print('ecm next')
print(ecm_joint_pos)

ecm.move_joint(np.array([ecm_joint_pos]))
