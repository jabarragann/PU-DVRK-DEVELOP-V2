
#!/usr/bin/env python
import dvrk
import numpy as np
import time


#DVRK interface does not allow to instantiate multiple arms at the same time.


# Move PSM1 to the initial position
mtml = dvrk.arm('MTML')
mtml.home()
mtml_joint_pos = [  0.16094356,  0.18617774, -0.13542208, -1.70923487,  1.67276075, -0.0769766,-1.01845961]

print('mtml current')
print(mtml.get_current_joint_position())
print('mtml next')
print(mtml_joint_pos)

mtml.move_joint(np.array([mtml_joint_pos]))


