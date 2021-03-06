#!/usr/bin/env python
import dvrk
import numpy as np

# Change the camera position to the desired one:
ecm = dvrk.ecm('ECM')
ecm_joint_pos = [-0.5241092844045072, -0.41092686407423995, 0.11746841756, 0.4742742612271361]
#[-0.20915153,  0.13093896,  0.12079741,  0.03336274]

print('ecm current')
print(ecm.get_current_joint_position())
print('ecm next')
print(ecm_joint_pos)

ecm.move_joint(np.array([ecm_joint_pos]))
