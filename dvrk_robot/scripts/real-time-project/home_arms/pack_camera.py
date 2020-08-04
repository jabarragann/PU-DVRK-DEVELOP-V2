#!/usr/bin/env python
import dvrk
import numpy as np

# Change the camera position to the desired one:
p = dvrk.psm('ECM')
ecm_joint_pos = [-0.015550883635269477, -0.7843509658462517, 0.17908691944, 0.3771298090999227]
p.move_joint(np.array([ecm_joint_pos]))