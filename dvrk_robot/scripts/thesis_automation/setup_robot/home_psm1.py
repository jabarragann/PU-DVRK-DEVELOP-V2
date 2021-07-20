
#!/usr/bin/env python
import dvrk
import numpy as np
import time


#DVRK interface does not allow to instantiate multiple arms at the same time.


# Move PSM1 to the initial position
psm1 = dvrk.psm('PSM1')
psm1_joint_pos =[0.65529705, -0.16671063, 0.16067774, -0.01404923, -0.02295493, -0.44668725]
print('psm1 current')
print(psm1.get_current_joint_position())

print('psm1 next')
print(psm1_joint_pos)
psm1.move_joint(np.array([psm1_joint_pos]))


