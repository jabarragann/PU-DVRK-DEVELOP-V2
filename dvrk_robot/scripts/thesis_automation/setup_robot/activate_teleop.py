#!/usr/bin/env python
import dvrk
from dvrk import console
import numpy as np
import time

console_instance = console()
print("start teleoperation")
console_instance.teleop_start()

