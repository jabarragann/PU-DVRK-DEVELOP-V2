
#!/usr/bin/env python
import dvrk
import numpy as np
import time


#DVRK interface does not allow to instantiate multiple arms at the same time.


# Move PSM1 to the initial position
mtml = dvrk.arm('MTML')
mtml.home()
mtml_joint_pos =[-0.16862288,  0.0511107,  -0.1271916,  -1.65313818,  1.4536735,  -0.33751279, 1.42110643]

print('mtml current')
print(mtml.get_current_joint_position())
print('mtml next')
print(mtml_joint_pos)

mtml.move_joint(np.array([mtml_joint_pos]))


