#!/usr/bin/env python
import dvrk
import numpy as np

# Change the camera position to the desired one:
ecm = dvrk.ecm('ECM')
ecm_joint_pos = [-0.25492715,  0.14746505,  0.12288696, -0.00621463]


print('ecm current')
print(ecm.get_current_joint_position())
print('ecm next')
print(ecm_joint_pos)

ecm.move_joint(np.array([ecm_joint_pos]))
