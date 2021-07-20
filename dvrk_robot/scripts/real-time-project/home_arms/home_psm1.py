
#!/usr/bin/env python
import dvrk
import numpy as np
import time


#DVRK interface does not allow to instantiate multiple arms at the same time.


# Move PSM1 to the initial position
psm1 = dvrk.psm('PSM1')
psm1_joint_pos =[0.72062365, -0.24412331, 0.14545411, -0.13294342,  0.03552549, -0.6043137]
print('psm1 current')
print(psm1.get_current_joint_position())

print('psm1 next')
print(psm1_joint_pos)
psm1.move_joint(np.array([psm1_joint_pos]))


