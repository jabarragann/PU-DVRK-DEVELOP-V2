#!/usr/bin/env python
import dvrk
import numpy as np

# Change the camera position to the desired one:
ecm = dvrk.ecm('ECM')
ecm_joint_pos = [-0.26441084, 0.13176363,  0.1208117, -0.07621097]


print('ecm current')
print(ecm.get_current_joint_position())
print('ecm next')
print(ecm_joint_pos)

ecm.move_joint(np.array([ecm_joint_pos]))
