#!/usr/bin/env python
import dvrk
import numpy as np

# Change the camera position to the desired one:
p = dvrk.psm('ECM')
ecm_joint_pos = [-0.2532320028334223, 0.2244471601449058, 0.08884337124, 0.04121279787215114]
p.move_joint(np.array([ecm_joint_pos]))
