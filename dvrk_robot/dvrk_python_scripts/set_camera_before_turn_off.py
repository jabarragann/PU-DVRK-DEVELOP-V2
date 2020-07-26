#!/usr/bin/env python
import dvrk
import numpy as np

# Change the camera position to the desired one:
p = dvrk.psm('ECM')
ecm_joint_pos = [-0.255175863287831, 0.23757639944303313, 0.08938986888, 1.482679466305247]
p.move_joint(np.array([ecm_joint_pos]))
