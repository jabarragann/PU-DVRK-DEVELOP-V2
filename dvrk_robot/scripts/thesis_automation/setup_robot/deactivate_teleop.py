
#!/usr/bin/env python
import dvrk
from dvrk import console
import numpy as np
import time

console_instance = console()
print("stop teleoperation")
console_instance.home()
console_instance.teleop_stop()

