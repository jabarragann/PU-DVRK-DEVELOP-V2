#!/usr/bin/env python
import dvrk
import numpy as np

# Change the camera position to the desired one:
ecm = dvrk.ecm('ECM')

print('ecm current')
print(ecm.get_current_joint_position())
